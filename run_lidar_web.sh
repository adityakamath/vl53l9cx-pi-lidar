#!/bin/bash
# VL53L9CX LiDAR Web Viewer launcher (RPi5 CFE pipeline). Run as a normal
# user; sudo is used once, to wake the sensor (XSHUT).
#
# Boots the sensor (bringup usecase 1 = binning-2 / 100x148 @ 100 Hz) and feeds
# the RAW8 stream to web_view.py, which serves a live WebGL depth dashboard on
# the LAN.  Open http://<pi-ip>:8080 from any device on the network.
#
# Alternative app to run_stream_test.sh -- both use the CFE capture node, so run one
# or the other.  Pass a port as $1 to override 8080.
REPO="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
cd "$REPO" || exit 1

BIN=2; STRIDE=112; WIDTH=100; HEIGHT=148
EXP=4; PERIOD=10000; POWER=0; DSS=0
PIPE=/tmp/tof_pipe_web
PORT=${1:-8080}

echo "========================================================="
echo "  VL53L9CX LiDAR Web Viewer (binning $BIN, ${EXP}ms, 100 Hz)"
echo "========================================================="

[ -x ./vl53l9_bringup ] || make -s || exit 1
[ -x tools/set_fmt ] || make -sC tools || exit 1

echo "[+] Clearing previous consumers..."
pkill -f 'web_vie[w].py' 2>/dev/null
pkill -f 'headless_dec[o]de.py' 2>/dev/null
pkill -f 'v4l2-ctl.*stream-mmap' 2>/dev/null
sleep 1

# Find V4L2 nodes by name: their numbers depend on driver probe order.
v4l_node() { for s in /sys/class/video4linux/$1*; do
    [ "$(cat "$s/name" 2>/dev/null)" = "$2" ] && { echo "/dev/$(basename "$s")"; return; }; done; }
SENSOR_SD=$(v4l_node v4l-subdev dummy-csi2); CSI2_SD=$(v4l_node v4l-subdev csi2)
VIDEO=$(v4l_node video rp1-cfe-csi2_ch0)
[ -n "$SENSOR_SD" ] && [ -n "$CSI2_SD" ] && [ -n "$VIDEO" ] \
    || { echo "[-] VL53L9 V4L2 nodes not found -- is the vl53l9-mipi-pi5 overlay loaded?"; exit 1; }

echo "[+] Setting subdev formats (dummy_cam x3, SRGGB8_1X8=0x3014)..."
v4l2-ctl -d "$SENSOR_SD" --set-subdev-fmt pad=0,width=${WIDTH},height=${HEIGHT},code=0x3014 >/dev/null
v4l2-ctl -d "$CSI2_SD" --set-subdev-fmt pad=0,width=${WIDTH},height=${HEIGHT},code=0x3014 >/dev/null
v4l2-ctl -d "$CSI2_SD" --set-subdev-fmt pad=4,width=${WIDTH},height=${HEIGHT},code=0x3014 >/dev/null

echo "[+] Setting video format (RAW8 ${WIDTH}x${HEIGHT})..."
tools/set_fmt 0 "$VIDEO" ${WIDTH} ${HEIGHT} RGGB

echo "[+] Enabling CFE media link (csi2 -> csi2_ch0)..."
MEDIA=$(for m in /dev/media*; do media-ctl -d $m -p 2>/dev/null | grep -q 'driver *rp1-cfe' && echo $m && break; done)
media-ctl -d "$MEDIA" -l '"csi2":4->"rp1-cfe-csi2_ch0":0 [1]'

echo "[+] Launching web viewer on $PIPE (port $PORT)..."
rm -f "$PIPE"
mkfifo "$PIPE"
VL_BINNING=$BIN VL_STRIDE=$STRIDE VL_PIPE=$PIPE VL_PORT=$PORT \
    nohup python3 "$REPO/viewer/web_view.py" >/tmp/lidar_web.log 2>&1 &
sleep 1

echo "[+] Arming V4L2 receiver (RAW8 ${WIDTH}x${HEIGHT} -> $PIPE)..."
nohup v4l2-ctl -d "$VIDEO" --stream-mmap --stream-to=$PIPE >/tmp/v4l2_web.log 2>&1 &
sleep 1.5

# --- Sensor XSHUT (root-only sysfs) ------------------------------------------
# dummy_cam holds XSHUT LOW after every boot; the sensor is in shutdown and will
# not answer I2C until cycled HIGH.  sudo prompts in your terminal.
XSTATE=$(cat /sys/devices/platform/vl53l9/xshut_state 2>/dev/null || echo x)
if [ "$XSTATE" != "1" ]; then
    echo "[+] XSHUT is LOW -- cycling to wake the sensor (root required)..."
    sudo sh -c 'echo 1 > /sys/devices/platform/vl53l9/xshut_cycle' \
        || { echo "[-] XSHUT cycle failed. Manual: sudo sh -c 'echo 1 > /sys/devices/platform/vl53l9/xshut_cycle'"; exit 1; }
    sleep 0.5
fi

echo "[+] Booting sensor + starting MIPI stream (usecase 1 = binning 2)..."
./vl53l9_bringup --i2c /dev/i2c-6 1 $EXP $PERIOD $POWER $DSS >/tmp/bringup_web.log 2>&1
BRINGUP_RC=$?
grep -E 'SUCCESS|No sensor response|ret=-' /tmp/bringup_web.log || true
if [ "$BRINGUP_RC" -ne 0 ] || ! grep -q 'SUCCESS: streaming' /tmp/bringup_web.log; then
    echo "[-] bringup did not reach STREAMING (rc=$BRINGUP_RC); see /tmp/bringup_web.log"
fi

IP=$(hostname -I | awk '{print $1}')
echo ""
echo "========================================================="
echo "  Open the dashboard in a browser on any LAN device:"
echo "      http://${IP}:${PORT}"
echo "  Stop: pkill -f web_view.py; pkill -f 'v4l2-ctl.*stream-mmap'"
echo "========================================================="
