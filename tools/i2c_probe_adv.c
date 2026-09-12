// i2c_probe_adv.c - VL53L9CX bus scanner / register reader for RPi5 camera bus.
// Kernel 6.18 uapi: struct i2c_msg lives in <linux/i2c.h>; I2C_RDWR takes a
// local ABI-stable struct (nmsgs is __u32). Separate STOP+START transactions
// only (no repeated-start) per VL53L9CX bus rules.
//
// usage:
//   ./i2c_probe_adv scan <bus>              # probe all addrs 0x03..0x77 (ACK only, read-byte)
//   ./i2c_probe_adv fsm  <bus> [addr]       # read FSM reg 0x008C u16 (default addr 0x29)
//   ./i2c_probe_adv id   <bus> [addr]       # read DEVICE_MODEL_ID u32 @0x0000 (expect 31 76 6c 35)
// e.g. ./i2c_probe_adv scan /dev/i2c-10 ; ./i2c_probe_adv fsm /dev/i2c-10 0x5d

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>      /* struct i2c_msg, I2C_M_RD */
#include <linux/i2c-dev.h>  /* I2C_RDWR = 0x0707 */

struct rdwr { struct i2c_msg *msgs; unsigned int nmsgs; };

static int xfer(int fd, const struct i2c_msg *m, unsigned int n)
{
	struct rdwr r = { .msgs = (struct i2c_msg *)m, .nmsgs = n };
	if (ioctl(fd, I2C_RDWR, &r))
		return -1;
	return 0;
}

static int i2c_write(int fd, unsigned char addr, const void *buf, size_t len)
{
	struct i2c_msg msg = { .addr = addr, .flags = 0, .buf = (void *)buf, .len = len };
	return xfer(fd, &msg, 1);
}

static int i2c_read(int fd, unsigned char addr, void *buf, size_t len)
{
	struct i2c_msg msg = { .addr = addr, .flags = I2C_M_RD, .buf = buf, .len = len };
	return xfer(fd, &msg, 1);
}

/* VL53L9CX: 16-bit register access. reg is big-endian on the wire; separate
 * write-index (STOP) then read-data transactions. */
static int vl53l9_read(int fd, unsigned char addr, uint16_t reg, void *buf, size_t len)
{
	unsigned char w[2] = { (unsigned char)(reg >> 8), (unsigned char)(reg & 0xff) };
	if (i2c_write(fd, addr, w, 2))
		return -1;
	return i2c_read(fd, addr, buf, len);
}

int main(int argc, char **argv)
{
	if (argc < 3) {
		fprintf(stderr, "usage: %s scan|fsm|id <bus> [addr]\n", argv[0]);
		return 1;
	}

	int fd = open(argv[2], O_RDWR);
	if (fd < 0) {
		perror("open");
		return 1;
	}

	if (!strcmp(argv[1], "scan")) {
		for (unsigned a = 0x03; a <= 0x77; a++) {
			unsigned char dummy;
			if (!i2c_read(fd, (unsigned char)a, &dummy, 1))
				printf("0x%02X: ACK\n", a);
		}
	} else if (!strcmp(argv[1], "fsm")) {
		unsigned char addr = argc > 3 ? (unsigned char)strtol(argv[3], NULL, 0) : 0x29;
		uint8_t r[2];

		if (vl53l9_read(fd, addr, 0x008c, r, 2)) {
			fprintf(stderr, "no response from 0x%02X\n", addr);
			return 1;
		}
		uint16_t val = (r[0] << 8) | r[1];
		printf("FSM reg 0x008C @0x%02X = %u (%s)\n", addr, val,
		       val == 1 ? "READY_TO_BOOT" :
		       val == 2 ? "STANDBY" :
		       val == 3 ? "STREAMING" : "UNKNOWN");
	} else if (!strcmp(argv[1], "id")) {
		unsigned char addr = argc > 3 ? (unsigned char)strtol(argv[3], NULL, 0) : 0x29;
		uint8_t r[4];

		if (vl53l9_read(fd, addr, 0x0000, r, 4)) {
			fprintf(stderr, "no response from 0x%02X\n", addr);
			return 1;
		}
		printf("DEVICE_MODEL_ID @0x%02X = %02X %02X %02X %02X (%s)\n", addr,
		       r[0], r[1], r[2], r[3],
		       (memcmp(r, "\x31\x76\x6c\x35", 4) == 0) ? "expected VL53L9CX" : "unexpected");
	} else {
		fprintf(stderr, "unknown mode\n");
		return 1;
	}

	close(fd);
	return 0;
}