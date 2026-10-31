#!/bin/bash
# power_cycle_probe.sh - XSHUT-reset VL53L9 via sysfs, poll I2C for sensor boot
uptime
echo ===UNDVOLT===
dmesg | grep -c Undervoltage
dmesg | grep Undervoltage | head -3
dmesg | grep Undervoltage | tail -2
echo ===THROTTLED===
vcgencmd get_throttled 2>/dev/null || true
echo ===CYCLE+PROBE===
echo 1 > /sys/devices/platform/vl53l9/xshut_cycle
for i in $(seq 1 12); do
    row=$(/usr/sbin/i2cdetect -y 10 | grep -E '^[4-7][0-9a-f]:' | tr '\n' ' ')
    echo "t=${i}s: ${row}"
    sleep 1
done
