# VL53L9CX ToF LiDAR on Raspberry Pi (MIPI CSI-2)

Live **54 × 42-zone depth streaming at 100 fps** from an ST **VL53L9CX** multi-zone
Time-of-Flight LiDAR, over **MIPI CSI-2** into a Raspberry Pi (4 or 5), decoded and
rendered in real time.

<p align="center">
  <img src="docs/images/depth-sample.png" width="480" alt="Live 54x42 depth map">
</p>

The VL53L9CX streams its result buffer over CSI-2 as RAW8. This project boots the
sensor over I²C, captures the raw stream with the Pi's Unicam receiver, un-packs the
ST result buffer, and colour-maps the depth array — all at the sensor's full 100 Hz.

---

## Features

- **Full-resolution 54 × 42 depth** (binning 2) at a true **100 fps**
- One-command bring-up + live viewer (`run_lidar.sh`) and a desktop launcher icon
- Correct decode of the ST result buffer (distance / amplitude / ambient / DSS layout)
- Live orientation control and per-frame stats overlay
- Selectable binning-4 (24 × 24) mode
- **Raspberry Pi 4 (Unicam) and Raspberry Pi 5 (RP1 CFE)** support from one codebase —
  board-specific device-tree overlays and launchers, software-controlled XSHUT on the Pi 5
- **GPU-shader particle viewer** (`run_lidar_gpu.sh`) — the whole simulation runs in
  GLSL on the Pi's V3D GPU; see [`docs/GPU_VIEWER_CONTROLS.md`](docs/GPU_VIEWER_CONTROLS.md)
- **LiDAR Web Viewer** (`run_lidar_web.sh`) — a networked browser dashboard with an
  orbitable WebGL 3D point cloud, depth heatmap and live stats (pure stdlib Python); the
  full-featured **Web Studio** (`web/lidar_web.py`: gesture recognition, theremin,
  recording/replay, PLY export) runs on the same pipeline — see
  [`docs/WEB_STUDIO.md`](docs/WEB_STUDIO.md)

---

## Hardware

| Part | Notes |
|------|-------|
| Raspberry Pi 4 (BCM2711) **or** Raspberry Pi 5 (BCM2712 + RP1) | Raspberry Pi OS, kernel 6.x. The Pi 4 uses the Unicam receiver (`bcm2835-unicam`); the Pi 5 uses the RP1 CFE CSI receiver |
| STEVAL-VL53L9 | ST evaluation board carrying the VL53L9CX module |
| FFC / flex cable | STEVAL flex → Pi CSI (camera) connector: 1 MIPI data lane + I²C |

### Wiring note — XSHUT

The VL53L9CX's **XSHUT** (enable) pin must be held **HIGH** for the sensor to boot and
run; LOW = shutdown. How it is handled differs per board:

- **Pi 4:** on this setup XSHUT is not broken out to a controllable Pi GPIO through the
  ribbon. On the STEVAL **J2 header** it's **pin 5** — tie it to **pin 8 (3V3)** with a
  jumper so the sensor is enabled. Because XSHUT is hard-tied there is no software reset:
  a fresh power-up puts the sensor in `READY_TO_BOOT` for a clean firmware init, once per
  power cycle — see [Implementation notes](#implementation-notes).

- **Pi 5:** on the current FFC wiring XSHUT lands on MIPI0 connector pin 17 (`CD0_IO0`,
  RP1 GPIO line 34), and this module has no separate power-enable net. The `dummy_cam`
  driver therefore owns that line directly and exposes a software reset:

  ```bash
  sudo sh -c 'echo 1 > /sys/devices/platform/vl53l9/xshut_cycle'   # XSHUT LOW → HIGH, wait for boot
  cat /sys/devices/platform/vl53l9/xshut_state                     # 1 = running
  ```

  `run_lidar_web.sh` does this automatically when the sensor is in shutdown. If your cable
  sits on MIPI1 instead of MIPI0, XSHUT moves to line 46 — see
  [`driver/vl53l9-mipi-pi5.dts`](driver/vl53l9-mipi-pi5.dts).

---

## Repository layout

```
.
├── app/
│   └── vl53l9_bringup.c      # I²C bring-up: init → profile → CSI config → start streaming
├── driver/
│   ├── dummy_cam.c           # stub CSI-2 subdev (+ XSHUT sysfs control on RPi5)
│   ├── vl53l9-mipi.dts       # Pi 4 device-tree overlay (csi1 + /dev/i2c-10)
│   ├── vl53l9-mipi-pi5.dts   # Pi 5 overlay (CFE; [cam0] = MIPI0 port) — see wiring note
│   └── Makefile              # builds dummy_cam.ko
├── tools/                    # userspace diagnostics: i2c_id / i2c_probe(_adv) / set_fmt
├── viewer/
│   ├── visualize.py          # OpenCV live view of the RAW8 FIFO (Pi 4 launcher)
│   ├── visualize_gpu.py      # GPU-shader particle viewer (run_lidar_gpu.sh)
│   ├── web_view.py           # pure-stdlib web dashboard (default for run_lidar_web.sh)
│   └── headless_decode.py    # no-display frame verifier / FPS + stats logger
├── web/                      # full Web Studio: gestures, theremin, recording, PLY export
├── vendor/st-uld/            # ST VL53L9CX ULD driver (SLA0111) — see LICENSE.txt
├── scripts/                  # .desktop launchers + xshut_finder.py; test/ = RPi5 bring-up tests
├── run_lidar.sh              # Pi 4: one-shot build-if-needed, stream + OpenCV viewer
├── run_lidar_gpu.sh          # Pi 4: GPU particle viewer
├── run_lidar_web.sh          # Pi 5 (CFE pipeline): web dashboard on :8080
├── Makefile                  # builds the bring-up app (+ module / tools targets)
└── docs/
```

---

## Build & install

### 1. Kernel module + device-tree overlay (one-time, per board)

The Pi needs a subdevice bound to the CSI port so the receiver will enable and DMA the
sensor's stream (the VL53L9CX is controlled out-of-band over I²C, so a small stub sensor
driver stands in on the media graph). Build it once:

```bash
make module                    # -> driver/dummy_cam.ko
```

**Raspberry Pi 4:**

```bash
sudo insmod driver/dummy_cam.ko
dtc -@ -I dts -O dtb -o vl53l9-mipi.dtbo driver/vl53l9-mipi.dts
sudo cp vl53l9-mipi.dtbo /boot/firmware/overlays/
# then add to /boot/firmware/config.txt:   dtoverlay=vl53l9-mipi
sudo reboot
```

**Raspberry Pi 5** (RP1 CFE receiver; `-Idriver` provides the dt-bindings header):

```bash
sudo insmod driver/dummy_cam.ko
dtc -@ -I dts -Idriver -O dtb -o vl53l9-mipi-pi5.dtbo driver/vl53l9-mipi-pi5.dts
sudo cp vl53l9-mipi-pi5.dtbo /boot/firmware/overlays/
# then add to /boot/firmware/config.txt:   dtoverlay=vl53l9-mipi-pi5        (MIPI1, main port)
# or                                       dtoverlay=vl53l9-mipi-pi5,cam0    (MIPI0, second port — current FFC wiring)
sudo reboot
```

Confirm the CSI capture node and dummy subdev exist:

```bash
v4l2-ctl --list-devices        # Pi 4: unicam -> /dev/video0 ; Pi 5: rp1-cfe -> /dev/video0
ls /dev/v4l-subdev*            # the dummy-csi2 sensor node(s)
```

### 2. Bring-up app

```bash
make          # builds ./vl53l9_bringup from app/ + vendor/st-uld/
```

### 3. Desktop launcher (optional)

```bash
cp scripts/lidar.desktop ~/Desktop/
gio set ~/Desktop/lidar.desktop metadata::trusted true
```

---

## Run

**Pi 4** (Unicam pipeline):

```bash
./run_lidar.sh          # 54x42 @ 100 fps OpenCV view (default)
./run_lidar.sh 4        # 24x24 (binning 4)
./run_lidar_gpu.sh      # GPU-shader particle viewer
```

**Viewer keys:** `r` cycles orientation (raw / flipV / flipH / rot180), `q` quits.
Stop everything with `sudo killall -9 python3 v4l2-ctl`.

**Pi 5** (CFE pipeline; no sudo except the one-time XSHUT wake):

```bash
./run_lidar_web.sh                  # web dashboard on http://<pi-ip>:8080
scripts/test/run_stream_test.sh     # headless streaming verification (FPS + stats)
```

Or double-click a **ToF LiDAR** desktop icon (`scripts/*.desktop`).

---

## How it works

```
VL53L9CX ──MIPI CSI-2 (RAW8, 1 lane, 1 Gbps)──▶ Pi Unicam ──DMA──▶ /dev/video0
   ▲ I²C (0x29, /dev/i2c-10)                                           │
   │                                                    v4l2-ctl --stream-to
vl53l9_bringup                                                         ▼
   init → profile → CSI cfg → start                              /tmp/tof_pipe (FIFO)
                                                                       │
                                                        viewer/visualize.py (decode + render)
```

The sensor streams its **result buffer**, not an image. For binning 2 (54 × 42 = 2268
raw zones) each CSI frame is laid out as:

```
[ distance u16 × N | amplitude u16 × N | ambient u16 × N | DSS 4-bit/zone | status line ]
```

Unicam pads each 100-byte line to a 128-byte stride; the viewer de-strides, slices the
distance array, masks the 15-bit value (`mm = raw & 0x7FFF`), reshapes to 54 × 42 and
colour-maps by range. The trailing **status line** (frame counter, temperature, metadata)
is the extra CSI row and is dropped.

On the **Pi 5** the same RAW8 stream arrives through the RP1 CFE receiver instead of
Unicam: the media pipeline is `csi2 → rp1-cfe-csi2_ch0 → /dev/video0`, the sensor's I²C
bus is the connector bus (`/dev/i2c-6` on MIPI0, `/dev/i2c-4` on MIPI1), and the video
format must be set with `tools/set_fmt` because the vendor fourcc for RAW8 there is
`'RGGB'`. The decoder side (FIFO → de-stride → 54 × 42 depth) is identical.

---

## Implementation notes

Things that were non-obvious and are baked into the code:

- **The result code from `vl53l9_init()` / `vl53l9_start()` is `-5` (TIMEOUT) even on
  success** — the ULD's command-ack poll is tuned for the STM32 reference and times out
  over Linux user-space I²C. Trust the **FSM register** (`0x008C`: 1 = `READY_TO_BOOT`,
  2 = `STANDBY`, 3 = `STREAMING`), not the return value.
- **Firmware-boot timeouts were widened** in the ULD (`_write_cmd`/`_wait_for_state`
  from 71 ms / 4 ms → 3000 ms) so the 32 KB patch load + boot handshake completes over
  I²C. A clean `vl53l9_init()` must run from `READY_TO_BOOT` (i.e. after a power-up).
- **100 fps requires DSS disabled.** With Dynamic SPAD Selection on, the extra
  acquisition pass caps the rate at exactly 50 fps; disabling it (per the datasheet's
  100 fps "Gaming" profile: precision context, regular power, 4 ms exposure) hits 100.
- **Unicam must be armed before the sensor's D-PHY (re)starts**, or no frames arrive.
  The dummy subdev pad format must match the capture height (148 for binning 2, 38 for
  binning 4).
- **Distances are raw radial, uncalibrated** 15-bit values. True perpendicular
  millimetres need ST's post-processing ISP (`transform` library) with a calibration
  buffer — not integrated here.

---

## Limitations / roadmap

- Depth is raw radial off the wire; the **Web Studio** applies ST's
  radial→perpendicular + point-cloud calibration (pinhole + distortion +
  parallax, default OTP-fallback params) for geometrically correct 3D / PLY —
  per-unit OTP calibration and TNR (ST ISP) are still not integrated
- Amplitude & ambient arrays are decoded off the wire but not yet displayed
- On the Pi 4, sensor reset is a manual power-cycle (XSHUT hard-tied); on the Pi 5 it is
  software-controlled via `xshut_cycle` sysfs (see wiring note)

---

## License

- Original work in this repository (bring-up app, viewer, kernel module, overlay,
  scripts, docs) — **MIT**, see [`LICENSE`](LICENSE).
- `vendor/st-uld/` — © STMicroelectronics, **SLA0111**, see
  [`vendor/st-uld/LICENSE.txt`](vendor/st-uld/LICENSE.txt). SLA0111 permits source
  redistribution provided the ST copyright notices and license text are retained.

Not affiliated with or endorsed by STMicroelectronics. VL53L9CX and STEVAL are ST
trademarks.
