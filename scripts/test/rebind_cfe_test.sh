#!/bin/bash
# Rebind CFE (restores AP_CLK path) and poll for VL53L9 @ 0x29 on i2c-10.
set -u
REPO="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
[ -x "$REPO/tools/i2c_id" ] || make -C "$REPO/tools" >/dev/null
echo "=== rebind rp1-cfe ==="
echo 1f00110000.csi > /sys/bus/platform/drivers/rp1-cfe/bind && echo REBIND_OK || echo REBIND_FAIL
dmesg | tail -3

found=0
for i in $(seq 1 40); do
    if i2cdetect -y 10 2>/dev/null | grep -q '\[29\]'; then
        found=1
        echo "ACK at 0x29 after $((i/2)) s"
        break
    fi
    sleep 0.5
done
[ "$found" = 1 ] || { echo "NO ACK within 20 s of rebind"; exit 1; }

echo "=== model ID via i2c_probe ==="
"$REPO/tools/i2c_id" /dev/i2c-10