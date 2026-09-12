# Userspace diagnostic / bring-up helpers for the VL53L9CX project.
#
#   i2c_id          probe 0x29 on one or more I2C buses, read DEVICE_MODEL_ID
#                   (exit 0 if the sensor answers) -- used by scripts/test/*
#   i2c_probe       full-bus scanner + VL53L9 FSM register reader (scan|fsm)
#   i2c_probe_adv   same as i2c_probe, plus `id`, using an ABI-stable local
#                   i2c_rdwr struct -- use this on kernel >= 6.18 (RPi5)
#   set_fmt         set a V4L2 pixel/meta format with a raw fourcc (bypasses
#                   v4l2-ctl's name table; needed for 'RGGB' on RPi5)
#   set_meta_fmt    set the META dataformat on a CFE video node
#
# NOTE: set_fmt / set_meta_fmt use the kernel >= 5.17 uapi meta format layout,
# so build them on the target Pi (or any system with recent headers).

CC     ?= cc
CFLAGS += -O2 -Wall

BIN := i2c_id i2c_probe i2c_probe_adv set_fmt set_meta_fmt

all: $(BIN)

$(BIN): %: %.c
	$(CC) $(CFLAGS) $< -o $@

.PHONY: all clean
clean:
	rm -f $(BIN)
