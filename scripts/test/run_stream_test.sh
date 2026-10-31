#!/bin/bash
# VL53L9CX streaming test (RPi5 CFE): configure the media pipeline, arm the CSI
# receiver on /dev/video0, init the sensor via I2C (/dev/i2c-6), then verify
# frames headlessly through a FIFO decoder.  No sudo required (pi owns i2c-6).
set -x

REPO="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
cd "$REPO" || exit 1
[ -x ./vl53l9_bringup ] || make -s || exit 1
[ -x tools/set_fmt ] || make -sC tools || exit 1

# ---------------------------------------------------------------------------
# 1. Configure the media pipeline (idempotent; all pad formats reset on reboot)
#    SRGGB8_1X8 = 0x3014 (v4l2-ctl silently sends code=0 for named codes here).
# ---------------------------------------------------------------------------
# Sensor source pad -> csi2 sink pad: both must carry the same mbus code.
v4l2-ctl -d /dev/v4l-subdev2 --set-subdev-fmt pad=0,width=100,height=148,code=0x3014
# csi2 sink + ch0 channel pads; CFE's link_validate requires the channel pad to
# match the video node S_FMT exactly (width/height/code->fourcc).
v4l2-ctl -d /dev/v4l-subdev0 --set-subdev-fmt pad=0,width=100,height=148,code=0x3014
v4l2-ctl -d /dev/v4l-subdev0 --set-subdev-fmt pad=4,width=100,height=148,code=0x3014
# Video node format (vendor fourcc 'RGGB' == V4L2_PIX_FMT_SRGGB8 on RPi5).
tools/set_fmt 0 /dev/video0 100 148 RGGB
# Enable the internal csi2 -> ch0 media link; CFE's streamon refuses without it.
# NOTE: v4l-utils 1.30 wants the whole spec as ONE arg and numeric flags (1=enabled).
# /dev/media0 is the ISP on the Pi 5; find the rp1-cfe (CSI receiver) media device.
MEDIA=$(for m in /dev/media*; do media-ctl -d $m -p 2>/dev/null | grep -q 'driver *rp1-cfe' && echo $m && break; done)
media-ctl -d "$MEDIA" -l '"csi2":4->"rp1-cfe-csi2_ch0":0 [1]'

# ---------------------------------------------------------------------------
# 2. Headless decoder on the FIFO (reader side; blocks until writer connects)
# ---------------------------------------------------------------------------
pkill -f 'headless_dec[o]de.py' 2>/dev/null
sleep 0.3
rm -f /tmp/tof_pipe
mkfifo /tmp/tof_pipe
nohup env VL_BINNING=2 VL_STRIDE=112 python3 "$REPO/viewer/headless_decode.py" \
    > /tmp/decode.log 2>&1 &
sleep 1

# ---------------------------------------------------------------------------
# 3. Arm the CSI receiver (writer side; unblocks once decoder has FIFO open)
# ---------------------------------------------------------------------------
pkill -f 'v4l2-ctl.*stream-mmap' 2>/dev/null
sleep 0.3
nohup v4l2-ctl -d /dev/video0 --stream-mmap --stream-to=/tmp/tof_pipe \
    > /tmp/v4l2.log 2>&1 &
sleep 1.5

# ---------------------------------------------------------------------------
# 4. FW init + profile (usecase 1 = binning 2) + exposure 4ms + 100 Hz
#    + REGULAR power + DSS off.  bringup stops any in-flight stream first, so
#    the armed receiver sees a fresh D-PHY LP->HS transition.
# ---------------------------------------------------------------------------
# dummy_cam holds XSHUT LOW after every boot; wake the sensor (root-only sysfs).
if [ "$(cat /sys/devices/platform/vl53l9/xshut_state 2>/dev/null)" != "1" ]; then
    sudo sh -c 'echo 1 > /sys/devices/platform/vl53l9/xshut_cycle' || exit 1
    sleep 0.5
fi
./vl53l9_bringup --i2c /dev/i2c-6 1 4 10000 0 0

echo ---DECODE---
sleep 8
tail -25 /tmp/decode.log
echo ---V4L2LOG---
tail -5 /tmp/v4l2.log
echo ---DMSG---
dmesg | tail -12
