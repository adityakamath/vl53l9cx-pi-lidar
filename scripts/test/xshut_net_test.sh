#!/bin/bash
# xshut_net_test.sh - determine whether cam0_reg (line 34) net reaches the module I2C section.
# Unbind CFE + dummy_cam -> rail OFF (line 34 LOW), probe bus, rebind, poll for sensor boot.
set -x
detect() { /usr/sbin/i2cdetect -y 10 | grep -E '^[3-7][0-9a-f]:' | tr '\n' ' '; }

echo "=== PHASE 0: rail ON (dummy_cam bound) ==="
cat /sys/devices/platform/vl53l9/xshut_cycle 2>/dev/null || true
detect

echo "=== UNBIND CFE + dummy_cam (rail OFF, line 34 LOW) ==="
echo 1f00110000.csi > /sys/bus/platform/drivers/rp1-cfe/unbind 2>&1
ls /sys/bus/platform/drivers/dummy_cam/ | grep vl53l9 && \
    echo vl53l9 > /sys/bus/platform/drivers/dummy_cam/unbind 2>&1
sleep 2
echo "--- rail OFF state ---"
cat /sys/class/regulator/regulator.8/state 2>/dev/null || echo "regulator.8 gone (expected if unbound)"
detect

echo "=== REBIND dummy_cam + CFE (rail ON, line 34 HIGH) ==="
echo vl53l9 > /sys/bus/platform/drivers/dummy_cam/bind 2>&1
sleep 1
echo 1f00110000.csi > /sys/bus/platform/drivers/rp1-cfe/bind 2>&1
cat /sys/class/regulator/regulator.8/state 2>/dev/null || true

echo "=== POLL for sensor boot at 0x29 (up to 15 s) ==="
for i in $(seq 1 15); do
    row=$(detect)
    echo "t=${i}s: ${row}"
    case "$row" in *"49:"*|*" 29"*|*[0-9a-f]9*) ;; esac
    if echo "$row" | grep -qE '(^|\s)(29)\b'; then
        echo "SENSOR BACK AT 0x29 at t=${i}s"
        break
    fi
    sleep 1
done

echo "=== FINAL: regulator state + dmesg tail ==="
cat /sys/class/regulator/regulator.8/state 2>/dev/null || true
dmesg | tail -8
