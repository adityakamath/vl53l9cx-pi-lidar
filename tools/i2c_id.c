/* Minimal VL53L9 I2C probe for RPi5 MIPI camera port buses.
 * Usage: i2c_id [bus ...]      (default: try /dev/i2c-6, -4, -10)
 * For each bus: address-probe 0x29; if ACK, read DEVICE_MODEL_ID
 * (u32 @ reg 0x0000, 16-bit register addressing). Expected bytes for
 * VL53L9CX: 31 76 6c 35. Exit 0 if any bus ACKed and ID was readable. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>      /* struct i2c_msg, I2C_M_RD (new uapi layout) */
#include <linux/i2c-dev.h>  /* I2C_RDWR = 0x0707 */

#define VL53L9_ADDR 0x29

/* ABI-stable local copy of the new i2c_rdwr data struct (nmsgs is __u32). */
struct rdwr { struct i2c_msg *msgs; unsigned int nmsgs; };

static int xfer(int fd, const struct i2c_msg *m, unsigned int n) {
    struct rdwr r = { .msgs = (struct i2c_msg *)m, .nmsgs = n };
    if (ioctl(fd, I2C_RDWR, &r)) return -errno;
    return 0;
}

static int try_bus(const char *path) {
    int fd = open(path, O_RDWR);
    if (fd < 0) return -1; /* not present */

    struct i2c_msg probe = { .addr = VL53L9_ADDR, .flags = 0, .len = 0 };
    if (xfer(fd, &probe, 1)) { close(fd); return -1; } /* NACK / no device */
    printf("OK: %s ACK at 0x%02X\n", path, VL53L9_ADDR);

    uint8_t buf[4];
    struct i2c_msg rd[2] = {
        { .addr = VL53L9_ADDR, .flags = 0, .len = 2 },
        { .addr = VL53L9_ADDR, .flags = I2C_M_RD, .buf = buf, .len = 4 },
    };
    uint8_t reg[2] = { 0x00, 0x00 }; /* DEVICE_MODEL_ID @ 0x0000 */
    rd[0].buf = reg;
    if (xfer(fd, rd, 2)) { printf("FAIL: model-ID read on %s\n", path); close(fd); return -1; }
    close(fd);
    printf("DEVICE_MODEL_ID bytes: %02X %02X %02X %02X (%s)\n", buf[0], buf[1], buf[2], buf[3],
           (memcmp(buf, "\x31\x76\x6C\x35", 4) == 0) ? "expected VL53L9CX" : "unexpected");
    return 0;
}

int main(int argc, char **argv) {
    const char *buses[8];
    int n = 0, found = 0;
    if (argc > 1) { for (int i = 1; i < argc && n < 8; i++) buses[n++] = argv[i]; }
    else { buses[n++] = "/dev/i2c-6"; buses[n++] = "/dev/i2c-4"; buses[n++] = "/dev/i2c-10"; }

    for (int i = 0; i < n; i++) if (!try_bus(buses[i])) found++;
    if (!found) { printf("[-] no bus ACKed 0x%02X\n", VL53L9_ADDR); return 1; }
    return 0;
}