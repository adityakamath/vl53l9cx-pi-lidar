#!/usr/bin/env python3
"""LiDAR Web Viewer -- pure standard library (no numpy/cv2), for headless Pis.

Reads RAW8 frames from a FIFO, decodes the distance channel in a background
thread, and serves a live web UI on the LAN:

    http://<pi-ip>:8080   ->  orbitable WebGL 3D point cloud (true Cartesian mm)
                              + depth heatmap + live stats

All rendering happens in the browser; the Pi only ships ~4.5 KB/frame of u16
distances plus a static per-zone calibration table, so it runs comfortably on
a headless RPi without OpenCV/numpy installed.

Endpoints:
    /            dashboard page (HTML + raw WebGL)
    /depth.bin   latest frame as little-endian u16 distances (N_ZONES*2 bytes;
                 row-major over the RAW zone grid, invalid zones = 0)
    /rays.bin    per-zone calibration coefficients (kx,ky,kz float32 LE), static
    /stats.json  fps, active-zone count, min/median/max mm, frame counter

Frame payload layout (de-strided to 100 bytes/line):
    [ distance u16 LE x N | amplitude u16 LE x N | ambient u16 LE x N | ... ]
Distance word: bit15 = VALID flag, low 15 bits = radial mm.

Usage: VL_BINNING=2|4 VL_PIPE=/tmp/tof_pipe VL_STRIDE=112 VL_PORT=8080 \
           python3 web_view.py
"""
import os
import time
import math
import json
import struct
import threading
import http.server
import socketserver
from urllib.parse import urlparse

BINNING = int(os.environ.get("VL_BINNING", "2"))
GEOM = {2: (54, 42, 148), 4: (24, 24, 38)}   # RAW_W, RAW_H, CSI capture height
RAW_W, RAW_H, CSI_H = GEOM[BINNING]
N_ZONES = RAW_W * RAW_H
CSI_W = 100
# RPi5 CFE csi2_ch0 pads width=100 to bytesperline=112 (Pi4 unicam used 128).
STRIDE = int(os.environ.get("VL_STRIDE", "112"))
FRAME_BYTES = STRIDE * CSI_H
DIST_MASK = 0x7FFF
PIPE = os.environ.get("VL_PIPE", "/tmp/tof_pipe")
PORT = int(os.environ.get("VL_PORT", "8080"))
MAX_MM = 4000


# ---- per-zone calibration rays (pure-stdlib port of lidar_calib.ray_coeffs) --
def ray_coeff_bytes(width, height, binning):
    """Per-zone (kx, ky, kz) float32 LE so point_mm = radial_mm * (kx,ky,kz).

    Faithful scalar port of web/lidar_calib.py (ST default params: efl 2428.16,
    alpha -0.00015, spad_size 10.17 um, max_spads 216x168). ky is negated so
    +Y points up in the browser's view space (same convention as lidar_web.py).
    """
    efl = 2428.16
    alpha = -0.00015
    mspads_x, mspads_y = 216 / 2.0, 168 / 2.0
    spad_size_um = 10.17
    mpix = binning * 2.0
    focal = efl / (spad_size_um * mpix)                    # binned-pixel units
    x_center = ((mspads_x - (mspads_x - binning * width)) / mpix) - 0.5
    y_center = ((mspads_y - (mspads_y - binning * height)) / mpix) - 0.5

    out = bytearray(12 * width * height)
    for r in range(height):
        dy = r - y_center
        for c in range(width):
            dx = c - x_center
            rsq = dx * dx + dy * dy
            ud = 1.0 + (binning * binning / 4.0) * (alpha * rsq)
            kz = 1.0 / math.sqrt(1.0 + rsq / (ud * focal) ** 2)
            k = kz / (ud * focal)
            struct.pack_into("<fff", out, 12 * (r * width + c), dx * k, -dy * k, kz)
    return bytes(out)


RAYS_BYTES = ray_coeff_bytes(RAW_W, RAW_H, BINNING)


def _selftest():
    # Constant-radial field sanity: centre kz ~= 1, corner kz clearly < 1.
    off_c = ((RAW_H // 2) * RAW_W + (RAW_W // 2)) * 12   # 3 x float32 per zone
    kc = struct.unpack_from("<fff", RAYS_BYTES, off_c)[2]
    k0 = struct.unpack_from("<fff", RAYS_BYTES, 0)[2]
    print(f"[wv] rays selftest: centre kz={kc:.4f} corner kz={k0:.4f}")
    assert 0.98 < kc <= 1.0 + 1e-6, "centre kz must be ~1"
    assert k0 < 0.95, "corner kz should fall off (fisheye)"


# ---- shared state: one reader thread, many HTTP threads ----------------------
class Studio:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = b"\x00" * (2 * N_ZONES)     # latest u16 LE distances
        self.frames = 0
        self.fps = 0.0
        self.active = 0
        self.min_mm, self.med_mm, self.max_mm = 0, 0, 0

    def push(self, frame_bytes):
        with self.lock:
            self.frame = frame_bytes
            self.frames += 1


STUDIO = Studio()


def read_exact(f, n):
    buf = b""
    while len(buf) < n:
        chunk = f.read(n - len(buf))
        if not chunk:
            raise EOFError("FIFO closed")
        buf += chunk
    return buf


def reader_loop():
    """Consume the FIFO forever; auto-reopens if the writer goes away."""
    print(f"[wv] opening {PIPE} (binning {BINNING}, {RAW_W}x{RAW_H}, "
          f"stride {STRIDE})", flush=True)
    n_win = 0
    t_last = time.time()

    while True:
        try:
            f = open(PIPE, "rb")
        except OSError as e:
            print(f"[wv] FIFO not ready ({e}); retrying...", flush=True)
            time.sleep(1.0)
            continue
        print("[wv] FIFO connected; waiting for frames...", flush=True)

        while True:
            try:
                raw = read_exact(f, FRAME_BYTES)
            except (EOFError, OSError):
                break                       # writer closed -> reopen below

            # De-stride (receiver pads each line to STRIDE): keep the first
            # CSI_W bytes of every line.  join() avoids bytearray slice-assign
            # resize side effects and yields exactly CSI_W*CSI_H bytes.
            payload = b"".join(raw[y * STRIDE:(y + 1) * STRIDE][:CSI_W]
                               for y in range(CSI_H))

            dist_raw = struct.unpack_from(f"<{N_ZONES}H", bytes(payload), 0)
            frame = bytearray(2 * N_ZONES)
            for i, v in enumerate(dist_raw):
                struct.pack_into("<H", frame, 2 * i,
                                 v & DIST_MASK if v & 0x8000 else 0)
            STUDIO.push(bytes(frame))

            n_win += 1
            now = time.time()
            if now - t_last >= 1.0:
                vals = sorted((v & DIST_MASK) for v in dist_raw if v & 0x8000)
                with STUDIO.lock:
                    STUDIO.fps = n_win / (now - t_last)
                    STUDIO.active = len(vals)
                    if vals:
                        STUDIO.min_mm = vals[0]
                        STUDIO.med_mm = vals[len(vals) // 2]
                        STUDIO.max_mm = vals[-1]
                print(f"[wv] {STUDIO.fps:5.1f} FPS frames={STUDIO.frames} "
                      f"valid={len(vals)}/{N_ZONES}"
                      + (f" min={vals[0]}mm med={vals[len(vals)//2]}mm "
                         f"max={vals[-1]}mm" if vals else ""), flush=True)
                t_last, n_win = now, 0

        try:
            f.close()
        except OSError:
            pass
        print("[wv] FIFO closed by writer; reopening...", flush=True)
        time.sleep(0.5)


# ---- HTTP --------------------------------------------------------------------
class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass   # quiet

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        try:
            if path == "/" or path == "/index.html":
                html = _PAGE.replace("__RAW_W__", str(RAW_W)) \
                            .replace("__RAW_H__", str(RAW_H)) \
                            .replace("__MAX_MM__", str(MAX_MM))
                self._send(200, "text/html; charset=utf-8", html.encode())
            elif path == "/depth.bin":
                with STUDIO.lock:
                    body = STUDIO.frame
                self._send(200, "application/octet-stream", body)
            elif path == "/rays.bin":
                self._send(200, "application/octet-stream", RAYS_BYTES)
            elif path == "/stats.json":
                with STUDIO.lock:
                    st = {"fps": round(STUDIO.fps, 1), "frames": STUDIO.frames,
                          "active": STUDIO.active, "min_mm": STUDIO.min_mm,
                          "med_mm": STUDIO.med_mm, "max_mm": STUDIO.max_mm}
                self._send(200, "application/json", json.dumps(st).encode())
            else:
                self._send(404, "text/plain", b"not found")
        except (BrokenPipeError, ConnectionResetError):
            pass


class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


# ---- front-end (HTML + raw WebGL, no external deps) ---------------------------
_PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>VL53L9CX LiDAR</title>
<style>
  :root{--bg:#0a0e14;--pan:#121822;--ln:#1e2836;--tx:#c8d4e0;--dim:#6b7a8d;--ac:#39d0ff;--hot:#ff6b3d;}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.4 system-ui,Segoe UI,Roboto,sans-serif}
  header{display:flex;align-items:center;gap:12px;padding:12px 18px;border-bottom:1px solid var(--ln);background:var(--pan)}
  header h1{font-size:16px;margin:0;letter-spacing:.5px}
  .dot{width:9px;height:9px;border-radius:50%;background:var(--hot);box-shadow:0 0 8px var(--hot)}
  .dot.live{background:#42e07a;box-shadow:0 0 8px #42e07a}
  .wrap{display:grid;grid-template-columns:1fr 1fr;gap:14px;padding:14px;max-width:1400px;margin:0 auto}
  @media(max-width:900px){.wrap{grid-template-columns:1fr}}
  .card{background:var(--pan);border:1px solid var(--ln);border-radius:10px;overflow:hidden}
  .card h2{font-size:12px;text-transform:uppercase;letter-spacing:1px;color:var(--dim);margin:0;padding:10px 14px;border-bottom:1px solid var(--ln)}
  .card .body{padding:12px 14px}
  #cloud{width:100%;height:420px;display:block;background:#05080d;cursor:grab;touch-action:none}
  #cloud:active{cursor:grabbing}
  #heat{width:100%;display:block;background:#05080d;border-radius:6px;image-rendering:pixelated}
  .stats{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:12px}
  .stat{background:#0d131c;border:1px solid var(--ln);border-radius:7px;padding:9px 11px}
  .stat .k{font-size:10px;text-transform:uppercase;letter-spacing:.8px;color:var(--dim)}
  .stat .v{font-size:19px;font-weight:600;margin-top:2px}
  .stat .v small{font-size:11px;color:var(--dim);font-weight:400}
  .row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-top:10px}
  button{background:#182231;color:var(--tx);border:1px solid var(--ln);border-radius:7px;padding:8px 13px;font-size:13px;cursor:pointer;transition:.12s}
  button:hover{border-color:var(--ac);color:#fff}
  .hint{color:var(--dim);font-size:11px;margin-top:8px}
  button.on{background:#24455e;border-color:var(--ac);color:#fff}
  #legend{width:100%;height:14px;display:block;background:#05080d;border-radius:6px;margin-top:10px}
  .ticks{position:relative;height:32px;font-size:10px;color:var(--dim)}
  .tick{position:absolute;transform:translateX(-50%);text-align:center;white-space:nowrap}
  .tick i{display:block;width:1px;height:6px;background:#4a5a70;margin:0 auto 2px;font-style:normal}
  #tip{position:absolute;z-index:9;pointer-events:none;background:rgba(8,12,18,.93);border:1px solid var(--ac);color:#fff;border-radius:6px;padding:4px 9px;font-size:12px;display:none;white-space:nowrap}
  .hwrap{position:relative}
  #marks{position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none;display:none;z-index:5}
  #marks.on{display:block}
  .mk{position:absolute;transform:translate(-50%,-100%);margin-top:-9px;background:rgba(8,12,18,.92);border:1px solid var(--a,#fff);color:#cfd8e3;border-radius:6px;padding:2px 7px;font-size:11px;line-height:1.5;white-space:nowrap;display:flex;gap:6px;align-items:center}
  .mk b{color:#fff;font-weight:700}
  .mk::after{content:'';position:absolute;left:50%;top:100%;transform:translateX(-50%);border:5px solid transparent;border-top-color:var(--a,#fff)}
  .mk.below{transform:translate(-50%,0);margin-top:9px}
  .mk.below::after{top:auto;bottom:100%;border-top-color:transparent;border-bottom-color:var(--a,#fff)}
</style></head>
<body>
<header>
  <span class="dot" id="dot"></span>
  <h1>VL53L9CX LiDAR &mdash; RPi5 CSI-2</h1>
  <span style="flex:1"></span>
  <span style="color:var(--dim);font-size:12px" id="fps">-- fps</span>
</header>
<div class="wrap">
  <div class="card">
    <h2>3D point cloud &mdash; drag to orbit, scroll to zoom</h2>
    <div style="position:relative">
      <canvas id="cloud"></canvas>
      <div id="tip"></div>
    </div>
    <div class="body">
      <div class="row" style="margin-top:0">
        <button id="viewbtn" onclick="cycleView()">&#9678; View: Points</button>
        <button id="calbtn" onclick="toggleCalib()">&#128207; Calibrated: on</button>
        <button id="fxbtn" onclick="toggleFlip('x')">&#8693; Flip H</button>
        <button id="fybtn" onclick="toggleFlip('y')">&#8597; Flip V</button>
        <button id="spin"></button>
      </div>
      <div class="hint">Probe: <span id="probev" style="color:#fff;font-weight:600">&mdash;</span></div>
      <div class="hint">Points are true Cartesian millimetres (ST radial&rarr;perpendicular pinhole calibration). Mesh = lit surface. Heatmap view orbits with the cloud. Flip H/V mirrors scene + flat heatmap. Hover to read a point's depth.</div>
    </div>
  </div>

  <div class="card">
    <h2>Depth heatmap &amp; live stats</h2>
    <div class="body">
      <div class="row" style="margin-top:0"><button id="tkbtn" class="on" onclick="toggleTrack()">&#128205; Track L/M/H</button></div>
      <div class="hwrap">
        <canvas id="heat" width="__RAW_W__" height="__RAW_H__"></canvas>
        <div id="marks"><div class="mk" id="mkL"><span>LOW</span><b id="mkLv">&ndash;</b></div><div class="mk" id="mkM"><span>MID</span><b id="mkMv">&ndash;</b></div><div class="mk" id="mkH"><span>HIGH</span><b id="mkHv">&ndash;</b></div></div>
      </div>
      <canvas id="legend" width="256" height="14"></canvas>
      <div class="ticks" id="lticks"></div>
      <div class="hint">Depth scale &mdash; LOW = near, HIGH = far (colour matches the 3D views). Track L/M/H pins the live closest / median / farthest zone on this preview.</div>
      <div class="stats">
        <div class="stat"><div class="k">FPS</div><div class="v" id="s_fps">--</div></div>
        <div class="stat"><div class="k">Closest</div><div class="v" id="s_min">--<small> mm</small></div></div>
        <div class="stat"><div class="k">Median</div><div class="v" id="s_med">--<small> mm</small></div></div>
        <div class="stat"><div class="k">Farthest</div><div class="v" id="s_max">--<small> mm</small></div></div>
        <div class="stat"><div class="k">Active zones</div><div class="v" id="s_act">--</div></div>
        <div class="stat"><div class="k">Frames</div><div class="v" id="s_fr">0</div></div>
      </div>
    </div>
  </div>
</div>

<script>
const RAW_W=__RAW_W__, RAW_H=__RAW_H__, MAX_MM=__MAX_MM__;
let autospin=true;
let flipX=1,flipY=1,stMin=0,stMed=0,stMax=0;
function toggleFlip(ax){if(ax==='x')flipX*=-1;else flipY*=-1;
  document.getElementById('fxbtn').classList.toggle('on',flipX<0);
  document.getElementById('fybtn').classList.toggle('on',flipY<0);
  const h=document.getElementById('heat');h.style.transform='scaleX('+flipX+') scaleY('+flipY+')';}
function classify(z){if(!stMax||stMax<=stMin)return 'MID';
  const t=(z-stMin)/(stMax-stMin);return t<1/3?'LOW (near)':(t<2/3?'MID':'HIGH (far)');}

// ---------- stats polling ----------
async function poll(){
  try{
    const r=await fetch('/stats.json'); const st=await r.json();
    document.getElementById('fps').textContent=(st.fps||0)+' fps';
    document.getElementById('dot').className='dot'+((st.fps>1)?' live':'');
    document.getElementById('s_fps').textContent=st.fps||0;
    document.getElementById('s_min').innerHTML=(st.min_mm||0)+'<small> mm</small>';
    document.getElementById('s_med').innerHTML=(st.med_mm||0)+'<small> mm</small>';
    document.getElementById('s_max').innerHTML=(st.max_mm||0)+'<small> mm</small>';
    document.getElementById('s_act').textContent=st.active||0;
    document.getElementById('s_fr').textContent=st.frames||0;
    stMin=st.min_mm||0;stMed=st.med_mm||0;stMax=st.max_mm||0;medTarget=st.med_mm||NaN;updateLegend();
  }catch(e){}
  setTimeout(poll,1000);
}

// ---------- depth heatmap (turbo colormap, same as the cloud) ----------
const heat=document.getElementById('heat'), hctx=heat.getContext('2d');
const himg=hctx.createImageData(RAW_W,RAW_H);
function turbo(t){t=Math.min(1,Math.max(0,t));
  const f=x=>Math.min(1,Math.max(0,x));
  return [f(1-1.8*Math.abs(t-.75)),f(1-2.2*Math.abs(t-.5)),f(1-2*Math.abs(t-.25))];}
function drawHeat(d){
  const px=himg.data; let o=0;
  for(let i=0;i<d.length;i++){
    const z=d[i];
    if(z>0&&z<MAX_MM){const c=turbo(1-z/MAX_MM);px[o]=c[0]*255;px[o+1]=c[1]*255;px[o+2]=c[2]*255;}
    else{px[o]=0;px[o+1]=0;px[o+2]=0;}
    px[o+3]=255;o+=4;
  }
  hctx.putImageData(himg,0,0);
}

// ---------- L/M/H tracker overlay on the flat heatmap ----------
// Pins floating labels (with arrows) onto the actual closest / median / farthest
// zone of every frame. Positions follow the Flip H/V mirror of #heat; colours
// are the exact turbo colour of that depth, matching the 3D views + legend.
let trackOn=true, mkT=0, medTarget=NaN;   // medTarget = live median mm from /stats.json
const tkBtn=document.getElementById('tkbtn'), marksEl=document.getElementById('marks');
const mkL=document.getElementById('mkL'), mkLv=document.getElementById('mkLv');
const mkM=document.getElementById('mkM'), mkMv=document.getElementById('mkMv');
const mkH=document.getElementById('mkH'), mkHv=document.getElementById('mkHv');
function toggleTrack(){trackOn=!trackOn;
  tkBtn.classList.toggle('on',trackOn);
  marksEl.classList.toggle('on',trackOn);}
tkBtn.classList.toggle('on',trackOn);marksEl.classList.toggle('on',trackOn);   // initial sync (button is .on in HTML)
function placeMark(el,val,i,z){
  const c=i%RAW_W,r=(i/RAW_W)|0,W=heat.clientWidth,Hh=heat.clientHeight;
  let px=(c+.5)/RAW_W*W,py=(r+.5)/RAW_H*Hh;
  if(flipX<0)px=W-px;if(flipY<0)py=Hh-py;      // follow the CSS mirror of #heat
  const t=turbo(1-z/MAX_MM),col='rgb('+(t[0]*255|0)+','+(t[1]*255|0)+','+(t[2]*255|0)+')';
  el.style.setProperty('--a',col);
  el.classList.toggle('below',py<34);          // point near top edge -> label goes under it, arrow up
  el.style.left=px+'px';el.style.top=py+'px';
  val.textContent=z+'mm';}
function trackMarks(d){
  if(!trackOn)return;
  const now=performance.now();if(now-mkT<90)return;mkT=now;   // ~11 Hz DOM updates, scan is cheap
  let mi=-1,xi=-1,md=-1,mn=1e9,mx=-1,bd=1e9;
  for(let i=0;i<d.length;i++){const z=d[i];if(z<=0||z>=MAX_MM)continue;
    if(z<mn){mn=z;mi=i;}                          // LOW = actual closest zone
    if(z>mx){mx=z;xi=i;}                          // HIGH = actual farthest zone
    if(!isNaN(medTarget)){const dz=Math.abs(z-medTarget);if(dz<bd){bd=dz;md=i;}}}  // MID = nearest to live median
  const put=(el,val,i)=>{if(i<0){el.style.display='none';return;}el.style.display='flex';placeMark(el,val,i,d[i]);};
  put(mkL,mkLv,mi);put(mkM,mkMv,md);put(mkH,mkHv,xi);}

// ---------- depth legend (exact turbo scale, ticks at live min/med/max) ----------
const lcv=document.getElementById('legend'),lctx=lcv.getContext('2d');
(function(){const im=lctx.createImageData(256,14),p=im.data;
  for(let x=0;x<256;x++){const c=turbo(1-x/255);   // left = near (t~1), right = far (t~0)
    for(let y=0;y<14;y++){const o=(y*256+x)*4;p[o]=c[0]*255;p[o+1]=c[1]*255;p[o+2]=c[2]*255;p[o+3]=255;}}
  lctx.putImageData(im,0,0);})();
function updateLegend(){const el=document.getElementById('lticks');if(!el)return;
  if(!stMax||stMax<=stMin){el.innerHTML='';return;}
  const pos=v=>Math.min(100,Math.max(0,v/MAX_MM*100)).toFixed(1)+'%';
  el.innerHTML='<span class="tick" style="left:'+pos(stMin)+'"><i></i>LOW '+stMin+'mm</span>'
             +'<span class="tick" style="left:'+pos(stMed)+'"><i></i>MID '+stMed+'mm</span>'
             +'<span class="tick" style="left:'+pos(stMax)+'"><i></i>HIGH '+stMax+'mm</span>';}

// calibration + view state (top level so the toolbar buttons work even if WebGL is absent)
let RAYS=null, calibrated=true;
const CS=0.0625, ZC=1500;   // mm->view scale, and depth centre (mm)
fetch('/rays.bin').then(r=>r.arrayBuffer()).then(ab=>{RAYS=new Float32Array(ab);}).catch(()=>{});
function toggleCalib(){calibrated=!calibrated;
  document.getElementById('calbtn').innerHTML='&#128207; Calibrated: '+(calibrated?'on':'off');}
let viewMode=0;const VIEWS=['&#9678; View: Points','&#9632; View: Mesh','&#9638; View: Heatmap'];
function cycleView(){viewMode=(viewMode+1)%3;document.getElementById('viewbtn').innerHTML=VIEWS[viewMode];}

// ---------- WebGL 3D view (points / mesh) ----------
const canvas=document.getElementById('cloud');
const gl=canvas.getContext('webgl',{antialias:true,alpha:false});
if(!gl){document.body.insertAdjacentHTML('beforeend','<p style="color:#f66">WebGL not available in this browser.</p>');}
else{
gl.enable(gl.DEPTH_TEST);
let az=0.6, el=-0.35, zoom=1.0, dragging=false, lx=0, ly=0;
function resize(){const r=canvas.getBoundingClientRect();canvas.width=r.width*devicePixelRatio;canvas.height=420*devicePixelRatio;}
resize(); addEventListener('resize',resize);
canvas.addEventListener('pointerdown',e=>{dragging=true;lx=e.clientX;ly=e.clientY;canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener('pointerup',e=>{dragging=false;});
canvas.addEventListener('pointermove',e=>{if(!dragging)return;az+=(e.clientX-lx)*0.008;el+=(e.clientY-ly)*0.008;el=Math.max(-1.5,Math.min(1.5,el));lx=e.clientX;ly=e.clientY;});
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom*=e.deltaY>0?0.92:1.08;zoom=Math.max(0.3,Math.min(4,zoom));},{passive:false});

// ---- hover probe: nearest point to cursor, with LOW/MID/HIGH class ----
let mx=0,my=0,mouseIn=false;const tip=document.getElementById('tip');
canvas.addEventListener('pointermove',e=>{const r=canvas.getBoundingClientRect();mx=e.clientX-r.left;my=e.clientY-r.top;mouseIn=true;});
canvas.addEventListener('pointerleave',()=>{mouseIn=false;hideTip();});
function projPt(i){   // CPU mirror of the vertex shader (incl. flips)
  const x=ptP[i*3]*flipX,y=ptP[i*3+1]*flipY,z=ptP[i*3+2];
  const ca=Math.cos(az),sa=Math.sin(az),ce=Math.cos(el),se=Math.sin(el);
  const rx=x*ca+z*sa,rz=-x*sa+z*ca,s=0.026*zoom;
  return [(rx*s/(canvas.width/canvas.height)+1)/2*canvas.clientWidth,
          (1-(y*ce-rz*se)*s)/2*canvas.clientHeight];}
function showTip(i){const z=ptD[i],c=turbo(1-z/MAX_MM);
  const col='rgb('+Math.round(c[0]*255)+','+Math.round(c[1]*255)+','+Math.round(c[2]*255)+')';
  tip.innerHTML='<span style="display:inline-block;width:9px;height:9px;border-radius:50%;background:'+col+';margin-right:6px"></span><b>'+z+' mm</b> &middot; '+classify(z);
  tip.style.display='block';tip.style.left=(mx+16)+'px';tip.style.top=Math.max(4,my-32)+'px';
  document.getElementById('probev').innerHTML=tip.innerHTML;}
function hideTip(){tip.style.display='none';}
function probe(){if(!mouseIn||dragging||!nPoints)return;
  let best=-1,bd=17*17;
  for(let i=0;i<nPoints;i++){const p=projPt(i),dx=p[0]-mx,dy=p[1]-my,q=dx*dx+dy*dy;if(q<bd){bd=q;best=i;}}
  if(best>=0)showTip(best);else hideTip();}

const ROT=`uniform float u_fx,u_fy;
vec3 rot(vec3 p){p=vec3(p.x*u_fx,p.y*u_fy,p.z);float ca=cos(u_az),sa=sin(u_az),ce=cos(u_el),se=sin(u_el);
  vec3 r=vec3(p.x*ca+p.z*sa,p.y,-p.x*sa+p.z*ca);
  return vec3(r.x, r.y*ce-r.z*se, r.y*se+r.z*ce);}`;
const TURBO=`vec3 turbo(float t){return clamp(vec3(1.0-1.8*abs(t-0.75),1.0-2.2*abs(t-0.5),1.0-2.0*abs(t-0.25)),0.0,1.0);}`;
function sh(t,s){const o=gl.createShader(t);gl.shaderSource(o,s);gl.compileShader(o);
  if(!gl.getShaderParameter(o,gl.COMPILE_STATUS))console.log(gl.getShaderInfoLog(o));return o;}
function mkProg(vs,fs){const p=gl.createProgram();gl.attachShader(p,sh(gl.VERTEX_SHADER,vs));gl.attachShader(p,sh(gl.FRAGMENT_SHADER,fs));gl.linkProgram(p);
  p._u={az:gl.getUniformLocation(p,'u_az'),el:gl.getUniformLocation(p,'u_el'),zoom:gl.getUniformLocation(p,'u_zoom'),aspect:gl.getUniformLocation(p,'u_aspect'),fx:gl.getUniformLocation(p,'u_fx'),fy:gl.getUniformLocation(p,'u_fy')};return p;}
function setCam(p){gl.useProgram(p);gl.uniform1f(p._u.az,az);gl.uniform1f(p._u.el,el);gl.uniform1f(p._u.zoom,zoom);gl.uniform1f(p._u.aspect,canvas.width/canvas.height);gl.uniform1f(p._u.fx,flipX);gl.uniform1f(p._u.fy,flipY);}

// -- points program --
const pProg=mkProg(
 `attribute vec3 a_pos; attribute float a_d; uniform float u_az,u_el,u_zoom,u_aspect; varying float v_d; ${ROT}
  void main(){vec3 r=rot(a_pos);float s=0.026*u_zoom;gl_Position=vec4(r.x*s/u_aspect,r.y*s,r.z*0.004,1.0);gl_PointSize=max(1.5,3.5*u_zoom);v_d=a_d;}`,
 `precision mediump float; varying float v_d; ${TURBO}
  void main(){vec2 c=gl_PointCoord-vec2(0.5);if(dot(c,c)>0.25)discard;gl_FragColor=vec4(turbo(v_d),1.0);}`);
const pPos=gl.getAttribLocation(pProg,'a_pos'), pD=gl.getAttribLocation(pProg,'a_d');
// -- mesh program (lit surface) --
const mProg=mkProg(
 `attribute vec3 a_pos; attribute vec3 a_nrm; attribute float a_d; uniform float u_az,u_el,u_zoom,u_aspect; varying float v_d; varying vec3 v_n; ${ROT}
  void main(){vec3 r=rot(a_pos);float s=0.026*u_zoom;gl_Position=vec4(r.x*s/u_aspect,r.y*s,r.z*0.004,1.0);v_n=rot(a_nrm);v_d=a_d;}`,
 `precision mediump float; varying float v_d; varying vec3 v_n; ${TURBO}
  void main(){vec3 N=normalize(v_n);float df=abs(dot(N,normalize(vec3(0.4,0.7,0.6))));gl_FragColor=vec4(turbo(v_d)*(0.32+0.68*df),1.0);}`);
const mPos=gl.getAttribLocation(mProg,'a_pos'), mNrm=gl.getAttribLocation(mProg,'a_nrm'), mD=gl.getAttribLocation(mProg,'a_d');
// -- heatmap-surface program (depth-coloured grid, flat shading) --
const hProg=mkProg(
 `attribute vec3 a_pos; attribute float a_d; uniform float u_az,u_el,u_zoom,u_aspect; varying float v_d; ${ROT}
  void main(){vec3 r=rot(a_pos);float s=0.026*u_zoom;gl_Position=vec4(r.x*s/u_aspect,r.y*s,r.z*0.004,1.0);v_d=a_d;}`,
 `precision mediump float; varying float v_d; ${TURBO}
  void main(){gl_FragColor=vec4(turbo(v_d),1.0);}`);
const hPos=gl.getAttribLocation(hProg,'a_pos'), hD=gl.getAttribLocation(hProg,'a_d');

const pBuf=gl.createBuffer(), mBuf=gl.createBuffer(), hBuf=gl.createBuffer();
let nPoints=0, nMeshV=0, nHeatV=0;
const ok=z=>z>0&&z<MAX_MM;
// per-point CPU mirror of the last frame (for hover probing)
const ptP=new Float32Array(RAW_W*RAW_H*3), ptD=new Float32Array(RAW_W*RAW_H);
// Calibration: RAYS (top level) holds per-zone (kx,ky,kz) so calibrated point
// = radial*(k), i.e. TRUE Cartesian mm (ST radial->perp + pinhole model).
function wp(c,r,z){
  if(calibrated&&RAYS){const i=(r*RAW_W+c)*3;
    return [z*RAYS[i]*CS, z*RAYS[i+1]*CS, -(z*RAYS[i+2]-ZC)*CS];}
  return [c-RAW_W/2, -(r-RAW_H/2), -(z/22.0-40.0)];
}

function buildPoints(d){
  const arr=new Float32Array(RAW_W*RAW_H*4);let n=0;
  for(let r=0;r<RAW_H;r++)for(let c=0;c<RAW_W;c++){const z=d[r*RAW_W+c];if(!ok(z))continue;
    const p=wp(c,r,z);arr[n*4]=p[0];arr[n*4+1]=p[1];arr[n*4+2]=p[2];arr[n*4+3]=1-z/MAX_MM;
    ptP[n*3]=p[0];ptP[n*3+1]=p[1];ptP[n*3+2]=p[2];ptD[n]=z;n++;}
  nPoints=n;gl.bindBuffer(gl.ARRAY_BUFFER,pBuf);gl.bufferData(gl.ARRAY_BUFFER,arr.subarray(0,n*4),gl.DYNAMIC_DRAW);
}
function nrm(A,B,C){const u=[B[0]-A[0],B[1]-A[1],B[2]-A[2]],w=[C[0]-A[0],C[1]-A[1],C[2]-A[2]];
  let x=u[1]*w[2]-u[2]*w[1],y=u[2]*w[0]-u[0]*w[2],z=u[0]*w[1]-u[1]*w[0];const L=Math.hypot(x,y,z)||1;return [x/L,y/L,z/L];}
function buildMesh(d){
  const v=[];
  const pushT=(A,B,C,za,zb,zc)=>{const n=nrm(A,B,C);
    v.push(A[0],A[1],A[2],n[0],n[1],n[2],1-za/MAX_MM, B[0],B[1],B[2],n[0],n[1],n[2],1-zb/MAX_MM, C[0],C[1],C[2],n[0],n[1],n[2],1-zc/MAX_MM);};
  for(let r=0;r<RAW_H-1;r++)for(let c=0;c<RAW_W-1;c++){
    const z00=d[r*RAW_W+c],z10=d[r*RAW_W+c+1],z01=d[(r+1)*RAW_W+c],z11=d[(r+1)*RAW_W+c+1];
    if(!ok(z00)||!ok(z10)||!ok(z01)||!ok(z11))continue;
    if(Math.max(z00,z10,z01,z11)-Math.min(z00,z10,z01,z11)>400)continue;  // no rubber sheets over edges
    const p00=wp(c,r,z00),p10=wp(c+1,r,z10),p01=wp(c,r+1,z01),p11=wp(c+1,r+1,z11);
    pushT(p00,p10,p11,z00,z10,z11);pushT(p00,p11,p01,z00,z11,z01);
  }
  nMeshV=v.length/7;gl.bindBuffer(gl.ARRAY_BUFFER,mBuf);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(v),gl.DYNAMIC_DRAW);
}
function buildHeat3D(d){
  const v=[];
  const push=(A,B,C,za,zb,zc)=>{v.push(A[0],A[1],A[2],1-za/MAX_MM, B[0],B[1],B[2],1-zb/MAX_MM, C[0],C[1],C[2],1-zc/MAX_MM);};
  for(let r=0;r<RAW_H-1;r++)for(let c=0;c<RAW_W-1;c++){
    const z00=d[r*RAW_W+c],z10=d[r*RAW_W+c+1],z01=d[(r+1)*RAW_W+c],z11=d[(r+1)*RAW_W+c+1];
    if(!ok(z00)||!ok(z10)||!ok(z01)||!ok(z11))continue;
    if(Math.max(z00,z10,z01,z11)-Math.min(z00,z10,z01,z11)>400)continue;  // no rubber sheets over edges
    const p00=wp(c,r,z00),p10=wp(c+1,r,z10),p01=wp(c,r+1,z01),p11=wp(c+1,r+1,z11);
    push(p00,p10,p11,z00,z10,z11);push(p00,p11,p01,z00,z11,z01);}
  nHeatV=v.length/4;gl.bindBuffer(gl.ARRAY_BUFFER,hBuf);gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(v),gl.DYNAMIC_DRAW);
}

async function pullCloud(){
  try{const r=await fetch('/depth.bin');const ab=await r.arrayBuffer();
    if(ab.byteLength>=RAW_W*RAW_H*2){const d=new Uint16Array(ab);
      drawHeat(d);trackMarks(d);buildPoints(d);   // points always built -> hover probe works in every view
      if(viewMode===1)buildMesh(d);else if(viewMode===2)buildHeat3D(d);}}catch(e){}
  setTimeout(pullCloud,55);
}

function render(){
  if(autospin&&!dragging)az+=0.004;
  gl.viewport(0,0,canvas.width,canvas.height);
  gl.clearColor(0.02,0.03,0.05,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
  if(viewMode===0&&nPoints>0){setCam(pProg);gl.bindBuffer(gl.ARRAY_BUFFER,pBuf);
    gl.enableVertexAttribArray(pPos);gl.vertexAttribPointer(pPos,3,gl.FLOAT,false,16,0);
    gl.enableVertexAttribArray(pD);gl.vertexAttribPointer(pD,1,gl.FLOAT,false,16,12);
    gl.drawArrays(gl.POINTS,0,nPoints);}
  else if(viewMode===1&&nMeshV>0){setCam(mProg);gl.bindBuffer(gl.ARRAY_BUFFER,mBuf);
    gl.enableVertexAttribArray(mPos);gl.vertexAttribPointer(mPos,3,gl.FLOAT,false,28,0);
    gl.enableVertexAttribArray(mNrm);gl.vertexAttribPointer(mNrm,3,gl.FLOAT,false,28,12);
    gl.enableVertexAttribArray(mD);gl.vertexAttribPointer(mD,1,gl.FLOAT,false,28,24);
    gl.drawArrays(gl.TRIANGLES,0,nMeshV);}
  else if(viewMode===2&&nHeatV>0){setCam(hProg);gl.bindBuffer(gl.ARRAY_BUFFER,hBuf);
    gl.enableVertexAttribArray(hPos);gl.vertexAttribPointer(hPos,3,gl.FLOAT,false,16,0);
    gl.enableVertexAttribArray(hD);gl.vertexAttribPointer(hD,1,gl.FLOAT,false,16,12);
    gl.drawArrays(gl.TRIANGLES,0,nHeatV);}
  probe();
  requestAnimationFrame(render);
}
document.getElementById('spin').onclick=function(){autospin=!autospin;this.textContent='\u25D4 Auto-spin: '+(autospin?'on':'off');};
document.getElementById('spin').textContent='\u25D4 Auto-spin: on';

pullCloud();
render();
} // if(gl)
poll();
</script>
</body></html>
"""


def main():
    _selftest()
    threading.Thread(target=reader_loop, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"[wv] web viewer on http://0.0.0.0:{PORT} "
          f"(binning {BINNING}, {RAW_W}x{RAW_H})", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()





