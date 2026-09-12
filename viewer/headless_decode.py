#!/usr/bin/env python3
"""Headless VL53L9CX frame verifier (pure stdlib -- no cv2/numpy needed).

Reads RAW8 frames from a FIFO (128-byte receiver stride), decodes the
distance channel, prints rolling FPS + range stats every second and saves
16-bit PGM snapshots of the distance map for inspection.

Frame payload layout (de-strided to 100 bytes/line):
    [ distance u16 LE x N | amplitude u16 LE x N | ambient u16 LE x N
      | dss 4bit x N | status line ]
Distance word: bit15 = VALID flag, low 15 bits = mm (uncalibrated radial).

Usage: VL_BINNING=2|4 VL_PIPE=/tmp/tof_pipe python3 headless_decode.py [snap_dir]
"""
import os
import sys
import time
import struct

BINNING = int(os.environ.get("VL_BINNING", "2"))
GEOM = {2: (54, 42, 148), 4: (24, 24, 38)}   # RAW_W, RAW_H, CSI capture height
RAW_W, RAW_H, CSI_H = GEOM[BINNING]
N_ZONES = RAW_W * RAW_H
CSI_W = 100
# Receiver line padding differs per board: Pi4 unicam pads to 128, RPi5 CFE
# csi2_ch0 reports bytesperline=112 for width=100.  Set VL_STRIDE accordingly.
STRIDE = int(os.environ.get("VL_STRIDE", "128"))
FRAME_BYTES = STRIDE * CSI_H
DIST_MASK = 0x7FFF
PIPE = os.environ.get("VL_PIPE", "/tmp/tof_pipe")
SNAP_DIR = sys.argv[1] if len(sys.argv) > 1 else "/tmp"


def read_exact(f, n):
    buf = b""
    while len(buf) < n:
        chunk = f.read(n - len(buf))
        if not chunk:
            raise EOFError("FIFO closed")
        buf += chunk
    return buf


def save_pgm(path, dist_raw, valid):
    """16-bit PGM; invalid zones -> 0 (black), valid -> distance mm."""
    hdr = f"P5\n{RAW_W} {RAW_H}\n65535\n".encode()
    body = bytearray(2 * N_ZONES)
    for i in range(N_ZONES):
        v = dist_raw[i] & DIST_MASK if valid[i] else 0
        struct.pack_into("<H", body, 2 * i, v)
    with open(path, "wb") as g:
        g.write(hdr + bytes(body))


def main():
    print(f"[hd] opening {PIPE} (binning {BINNING}, {RAW_W}x{RAW_H})", flush=True)
    f = open(PIPE, "rb")
    print("[hd] FIFO connected; waiting for frames...", flush=True)

    n_frames = 0
    fps_n = 0
    t_last = time.time()
    snap_count = 0
    last_snap = 0.0

    while True:
        raw = read_exact(f, FRAME_BYTES)
        # De-stride (receiver pads each line to STRIDE): keep the first CSI_W
        # bytes of every line.  join() avoids bytearray slice-assign resize
        # side effects and yields exactly CSI_W*CSI_H bytes.
        payload = b"".join(raw[y * STRIDE:(y + 1) * STRIDE][:CSI_W]
                           for y in range(CSI_H))

        dist_raw = struct.unpack_from(f"<{N_ZONES}H", bytes(payload), 0)
        vals = [v & DIST_MASK for v in dist_raw if v & 0x8000]

        n_frames += 1
        fps_n += 1
        now = time.time()
        if now - t_last >= 1.0:
            fps = fps_n / (now - t_last)
            if vals:
                srt = sorted(vals)
                print(f"[hd] {fps:5.1f} FPS  frames={n_frames}  "
                      f"valid={len(vals)}/{N_ZONES}  min={srt[0]}mm "
                      f"med={srt[len(srt)//2]}mm max={srt[-1]}mm", flush=True)
            else:
                print(f"[hd] {fps:5.1f} FPS  frames={n_frames}  valid=0/{N_ZONES}",
                      flush=True)
            t_last, fps_n = now, 0

        if now - last_snap >= 5.0 and vals:
            snap_count += 1
            p = os.path.join(SNAP_DIR, f"frame_{snap_count}_b{BINNING}.pgm")
            save_pgm(p, dist_raw, [bool(v & 0x8000) for v in dist_raw])
            print(f"[hd] snapshot -> {p}", flush=True)
            last_snap = now


if __name__ == "__main__":
    try:
        main()
    except EOFError as e:
        print(f"[hd] stopped ({e})", flush=True)
