// i2c_probe.c - minimal I2C bus scanner / register reader for the RPi5 camera bus.
// No i2c-tools on the Pi, so this does raw ioctls.
//
// usage:
//   ./i2c_probe scan  <bus>              # probe all addrs 0x03..0x77 (ACK only)
//   ./i2c_probe fsm   <bus> [addr]       # read VL53L9CX FSM reg 0x008C (default addr 0x29)
// e.g. ./i2c_probe scan /dev/i2c-6 ; ./i2c_probe fsm /dev/i2c-6
//
// build on the Pi: gcc -O2 -Wall -o i2c_probe i2c_probe.c

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

static int i2c_write(int fd, unsigned char addr, const void *buf, size_t len)
{
	struct i2c_msg msg = { .addr = addr, .flags = 0, .buf = buf, .len = len };
	struct i2c_rdwr_ioctl_data rdwr = { .msgs = &msg, .nmsgs = 1 };

	return ioctl(fd, I2C_RDWR, &rdwr);
}

static int i2c_read(int fd, unsigned char addr, void *buf, size_t len)
{
	struct i2c_msg msg = { .addr = addr, .flags = I2C_M_RD, .buf = buf, .len = len };
	struct i2c_rdwr_ioctl_data rdwr = { .msgs = &msg, .nmsgs = 1 };

	return ioctl(fd, I2C_RDWR, &rdwr);
}

/* VL53L9CX: 16-bit register access. reg is big-endian on the wire. */
static int vl53l9_read_reg(int fd, unsigned char addr, uint16_t reg, uint16_t *val)
{
	unsigned char w[2] = { (unsigned char)(reg >> 8), (unsigned char)(reg & 0xff) };
	unsigned char r[2];

	if (i2c_write(fd, addr, w, 2))
		return -1;
	if (i2c_read(fd, addr, r, 2))
		return -1;
	*val = (r[0] << 8) | r[1];
	return 0;
}

int main(int argc, char **argv)
{
	if (argc < 3) {
		fprintf(stderr, "usage: %s scan <bus> | fsm <bus> [addr]\n", argv[0]);
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
			if (ioctl(fd, I2C_SLAVE, a) < 0)
				continue;
			if (!i2c_read(fd, a, &dummy, 1))
				printf("0x%02X: ACK\n", a);
		}
	} else if (!strcmp(argv[1], "fsm")) {
		unsigned char addr = argc > 3 ? (unsigned char)strtol(argv[3], NULL, 0) : 0x29;
		uint16_t val;

		if (ioctl(fd, I2C_SLAVE, addr) < 0) {
			perror("I2C_SLAVE");
			return 1;
		}
		if (vl53l9_read_reg(fd, addr, 0x008c, &val)) {
			fprintf(stderr, "no response from 0x%02X\n", addr);
			return 1;
		}
		printf("FSM reg 0x008C @0x%02X = %u (%s)\n", addr, val,
		       val == 1 ? "READY_TO_BOOT" :
		       val == 2 ? "STANDBY" :
		       val == 3 ? "STREAMING" : "UNKNOWN");
	} else {
		fprintf(stderr, "unknown mode\n");
		return 1;
	}

	close(fd);
	return 0;
}
