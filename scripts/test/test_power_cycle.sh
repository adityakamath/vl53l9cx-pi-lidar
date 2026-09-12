#!/bin/bash
# Live RPi5 test for dummy_cam xshut_cycle sysfs control (GPIO XSHUT based).
# Run as root with the dummy_cam module loaded, e.g.:
#   sudo bash scripts/test/test_power_cycle.sh
set -u
REPO="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
[ -x "$REPO/tools/i2c_id" ] || make -C "$REPO/tools" >/dev/null
PC=/sys/devices/platform/vl53l9/xshut_cycle
REG=/sys/class/regulator/regulator.8

gpio_level() { grep -E 'gpio-34|GPIO v2' /sys/kernel/debug/gpio | awk '{print $NF}'; }
reg_state()  { cat $REG/state; }

echo "=== [1] module + attr present ==="
lsmod | grep dummy_cam || echo "FAIL: dummy_cam not loaded"
[ -f "$PC" ] && echo "OK: xshut_cycle exists (mode $(stat -c '%a' $PC))" || { echo "FAIL: no xshut_cycle"; exit 1; }

echo "=== [2] initial state (after probe) ==="
echo "regulator.8: $(reg_state)"
echo "gpio34 level: $(gpio_level)  (expect hi = POWER-EN high)"
echo "xshut_cycle readback: $(cat $PC)  (expect 1 = XSHUT high/running)"

echo "=== [3] power cycle #1 ==="
t0=$(date +%s%N)
if echo 1 > "$PC"; then echo "OK: write accepted"; else echo "FAIL: write returned error ($?)"; fi
t1=$(date +%s%N)
echo "write took $(( (t1-t0)/1000000 )) ms (expect ~210 ms)"
echo "xshut_cycle readback after cycle: $(cat $PC)  (expect 1 = XSHUT high/running)"
echo "regulator.8: $(reg_state)"
echo "gpio34 level: $(gpio_level)  (expect hi = back to high)"

echo "=== [4] I2C recovery check (VL53L9 @ 0x29, DEVICE_MODEL_ID u32 @ 0x0000) ==="
"$REPO/tools/i2c_id" && echo "OK: sensor recovered after cycle #1" || { echo "FAIL: no ACK at 0x29"; exit 1; }

echo "=== [5] power cycle #2 + recovery again ==="
if echo 1 > "$PC"; then echo "OK: second cycle write accepted"; else echo "FAIL: second write failed"; fi
sleep 0.3
"$REPO/tools/i2c_id" && echo "OK: sensor recovered after cycle #2" || { echo "FAIL: no ACK at 0x29 after cycle #2"; exit 1; }

echo "=== [6] invalid input rejected ==="
if echo 0 > "$PC" 2>/dev/null; then echo "FAIL: write of 0 accepted"; else echo "OK: write of 0 rejected (EINVAL)"; fi
if echo abc > "$PC" 2>/dev/null; then echo "FAIL: garbage accepted"; else echo "OK: garbage rejected (EINVAL)"; fi

echo "=== [7] dmesg ==="
dmesg | grep -iE 'dummy_cam|xshut_cycle' | tail -8
echo "=== DONE ==="