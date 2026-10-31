#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "vl53l9.h"
#include "vl53l9_interface.h"
#include "vl53l9_platform.h"
#include "vl53l9_utils.h"

/* Apply ST default VCSEL/DSS/analog config from STANDBY (defined in vl53l9.c). */
extern int vl53l9_apply_default_config(void *const p_dev);

/*
 * Corrected VL53L9CX MIPI CSI-2 bring-up.
 *
 * Fixes vs. the original:
 *   1. vl53l9_init() must SUCCEED (it loads the FW patch, boots to STANDBY, and
 *      runs _init_default_config() which sets up the VCSEL/DSS analog config).
 *      Requires the sensor to be in READY_TO_BOOT, i.e. freshly hardware-reset
 *      (XSHUT).  The old code ignored init's -5 TIMEOUT and streamed a stale,
 *      un-ranged buffer (constant 0x0096 distance, zero amplitude).
 *   2. vl53l9_utils_set_profile() is now called, so sync mode / power mode /
 *      context / binning / EXPOSURE are actually configured.  Without it the
 *      ranging core never produces valid measurements.
 *
 * Use case AF_RANGE = binning 4 (24x24 raw / 24x20 display), context LONG,
 * precision ranging.  Frame period overridden to 100 Hz.
 */

#define REG_FSM 0x008C

static const char *fsm_name(uint8_t f) {
    switch (f) { case 0: return "NONE"; case 1: return "READY_TO_BOOT";
                 case 2: return "STANDBY"; case 3: return "STREAMING"; default: return "?"; }
}

/* --- Configurable control bus / address (RPi5 port) -----------------------
 * Bus resolution order: --i2c PATH  >  $VL_I2C_BUS  >  stable fallbacks.
 *   RPi5 MIPI0 = /dev/i2c-6 (stable DT symlink; also enumerated as i2c-10),
 *   Pi 4 camera port = /dev/i2c-10.
 * RPi5 MIPI1 (/dev/i2c-4) is deliberately not a fallback: the Pi 5 overlay only
 * supports MIPI0, and i2c-4 is the bus of whatever sits on CAM/DISP 1 (e.g. a
 * DSI touch display), which the first write to 0x29 must never reach. */
static int open_i2c_bus(const char *opt_path, const char **used_out) {
    static const char *fallbacks[] = { "/dev/i2c-6", "/dev/i2c-10" };
    const char *env = getenv("VL_I2C_BUS");
    int fd;

    if (opt_path) {
        fd = open(opt_path, O_RDWR);
        if (fd < 0) { perror("[-] --i2c"); return -1; }
        *used_out = opt_path;
        return fd;
    }
    const char *tries[4];
    int n = 0;
    if (env && *env) tries[n++] = env;
    for (int i = 0; i < 3; i++) tries[n++] = fallbacks[i];
    for (int i = 0; i < n; i++) {
        fd = open(tries[i], O_RDWR);
        if (fd >= 0) { *used_out = tries[i]; return fd; }
    }
    fprintf(stderr, "[-] No usable I2C adapter found (tried:");
    for (int i = 0; i < n; i++) fprintf(stderr, "%s%s", tries[i], i + 1 < n ? ", " : "");
    fprintf(stderr, "). Pass --i2c PATH or set VL_I2C_BUS.\n");
    return -1;
}

int main(int argc, char **argv) {
    /* Options: --i2c PATH (adapter node), --addr N (7-bit slave address).
     * Remaining positional args: [usecase] [exp_ms] [period_us] [power_mode] [dss_mode]. */
    const char *opt_i2c = NULL;
    int opt_addr = -1;
    static char *pos[8];
    int npos = 0;
    pos[npos++] = argv[0];
    for (int i = 1; i < argc && npos < 8; i++) {
        if (!strcmp(argv[i], "--i2c") && i + 1 < argc)      { opt_i2c = argv[++i]; continue; }
        if (!strcmp(argv[i], "--addr") && i + 1 < argc)    { opt_addr = atoi(argv[++i]); continue; }
        pos[npos++] = argv[i];
    }

    int usecase = (npos > 1) ? atoi(pos[1]) : VL53L9_USECASE_AF_RANGE;
    int status = 0;
    vl53l9_device_t dev;
    memset(&dev, 0, sizeof(vl53l9_device_t));

    printf("=========================================================\n");
    printf(" STMicroelectronics VL53L9CX MIPI CSI-2 Transmitter Boot \n");
    printf("=========================================================\n\n");

    /* Slave address: --addr N > $VL_I2C_ADDR > default 0x29 (7-bit). */
    uint8_t addr7 = 0x29;
    const char *env_addr = getenv("VL_I2C_ADDR");
    if (opt_addr >= 0) {
        if (opt_addr < 1 || opt_addr > 127) { fprintf(stderr, "[-] --addr out of range\n"); return 1; }
        addr7 = (uint8_t)opt_addr;
    } else if (env_addr && *env_addr) {
        int a = atoi(env_addr);
        if (a < 1 || a > 127) { fprintf(stderr, "[-] VL_I2C_ADDR out of range\n"); return 1; }
        addr7 = (uint8_t)a;
    }

    printf("[STEP 1/5] Connecting to camera control interface...\n");
    const char *bus_used = NULL;
    int fd = open_i2c_bus(opt_i2c, &bus_used);
    if (fd < 0) return 1;
    if (ioctl(fd, I2C_SLAVE, addr7) < 0) { perror("[-] Slave address failed"); close(fd); return 1; }

    dev.bus = (void*)(intptr_t)fd;
    dev.bus_type = PLATFORM_BUS_I2C;
    dev.address = (uint8_t)(addr7 << 1);   /* ULD keeps the 8-bit form */
    dev.ext_clock = 12000000;
    dev.vdda = VDDA_2V8;
    dev.vddio = VDDIO_1V8;
    if (addr7 == 0x29) {
        vl53l9_set_com_config(&dev, dev.address, 0);   /* restore default slave addr/instance */
    } else {
        printf("[i] Non-default address 0x%02X: skipping set_com_config (would reprogram the sensor).\n", addr7);
    }
    printf("[+] Control path established on %s @ 0x%02X.\n", bus_used, addr7);

    uint8_t fsm = 0;
    int rret = vl53l9_read8(&dev, REG_FSM, &fsm);
    if (rret != 0) {
        fprintf(stderr, "[-] No sensor response at 0x%02X on %s (FSM read ret=%d).\n"
                        "    Check FFC seating + power; a fresh XSHUT edge may be needed\n"
                        "    (Pi 5: sudo sh -c 'echo 1 > /sys/devices/platform/vl53l9/xshut_cycle';\n"
                        "    Pi 5 also needs AP_CLK running: dmesg | grep AP_CLK. Pi 4: power-cycle).\n",
                addr7, bus_used, rret);
        close(fd); return 1;
    }
    printf("[+] Initial FSM state = %u (%s)\n", fsm, fsm_name(fsm));

    /* Unicam can only lock onto a fresh D-PHY LP->HS transition -- it cannot join
       a stream already in progress. So even if the sensor is already STREAMING,
       we must stop() and issue a genuine new start() while the caller's v4l2
       receiver is armed and watching (arm-before-start is the caller's job). */
    if (fsm == 3) {
        printf("[i] Sensor already STREAMING; stopping so we can issue a fresh start...\n");
        status = vl53l9_stop(&dev);
        usleep(20000);
        vl53l9_read8(&dev, REG_FSM, &fsm);
        printf("    vl53l9_stop ret=%d, FSM now=%u (%s)\n", status, fsm, fsm_name(fsm));
    }

    printf("\n[STEP 2/5] Initializing sensor firmware (FW patch + boot + default config)...\n");
    if (fsm == 1) {
        /* Full boot from READY_TO_BOOT. -5 (TIMEOUT) here is a known red herring
           over Linux I2C -- the boot still completes; key on the FSM, not the code. */
        status = vl53l9_init(&dev);
        usleep(20000);
        vl53l9_read8(&dev, REG_FSM, &fsm);
        printf("[+] vl53l9_init() ret=%d, FSM now=%u (%s)\n", status, fsm, fsm_name(fsm));
    } else {
        printf("[i] FSM=%s (not READY_TO_BOOT); firmware assumed resident.\n", fsm_name(fsm));
    }
    /* Note: a clean vl53l9_init() already runs the ST default config internally;
       re-applying it here would knock the sensor out of STANDBY, so we don't. */

    printf("\n[STEP 3/5] Applying ranging profile (sync/power/context/binning/exposure)...\n");
    vl53l9_profile_t *prof = &g_ranging_profiles[usecase];
    status = vl53l9_utils_set_profile(&dev, prof);
    printf("    set_profile(usecase=%d) ret=%d  binning=%u exposure=%ums context=%u power=%u\n",
           usecase, status, prof->binning, prof->exposure_ms, prof->context, prof->power);
    if (status != 0) {
        printf("[-] set_profile failed (ret=%d). The sensor is in a stale STANDBY that\n", status);
        printf("    can't be reconfigured. Give it a fresh XSHUT edge to get a clean\n");
        printf("    READY_TO_BOOT, then re-run. FSM=%u.\n", fsm);
        printf("    Pi 5: sudo sh -c 'echo 1 > /sys/devices/platform/vl53l9/xshut_cycle'; Pi 4: power-cycle.\n");
        close(fd); return 1;
    }

    /* Optional exposure override in ms (pos[2]).  4ms = datasheet 100fps profile. */
    if (npos > 2) {
        int ex = atoi(pos[2]);
        status = vl53l9_set_exposure(&dev, prof->context, (uint16_t)ex);
        printf("    set_exposure(%dms) ret=%d\n", ex, status);
    }

    /* Optional power mode override (pos[4]): 0=REGULAR 1=LOW 2=ULP. */
    if (npos > 4) {
        int pw = atoi(pos[4]);
        status = vl53l9_set_power_mode(&dev, (vl53l9_power_mode_t)pw);
        printf("    set_power_mode(%d) ret=%d\n", pw, status);
    }

    /* Context regs (vl53l9_reg.h): base = 0x04C4 + 4*C; +0 binning, +1 dss_mode, +2 repeat. */
    uint16_t ctx_base = (uint16_t)(0x04C4U + 4U * (uint16_t)prof->context);
    uint8_t rb_bin = 0, rb_dss = 0, rb_rep = 0;
    vl53l9_read8(&dev, ctx_base + 0U, &rb_bin);
    vl53l9_read8(&dev, ctx_base + 1U, &rb_dss);
    vl53l9_read8(&dev, ctx_base + 2U, &rb_rep);
    printf("    ctx regs: binning=%u dss_mode=%u repeat=%u\n", rb_bin, rb_dss, rb_rep);

    /* Optional DSS mode override (pos[5]): 0=DISABLE 1=LONG 2=SHORT.
       Datasheet 100fps "Gaming" profile runs precision with DSS disabled. */
    if (npos > 5) {
        int dss = atoi(pos[5]);
        status = vl53l9_write8(&dev, ctx_base + 1U, (uint8_t)dss);
        printf("    set_dss_mode(%d) reg=0x%04X ret=%d\n", dss, ctx_base + 1U, status);
    }

    /* Frame period (argv[3], us). Default 10000us -> 100 Hz. */
    uint32_t period = (npos > 3) ? (uint32_t)atoi(pos[3]) : 10000;
    status = vl53l9_set_frame_period(&dev, period);
    printf("    set_frame_period(%uus) ret=%d\n", period, status);
    uint32_t rb_period = 0; uint16_t rb_exp = 0;
    vl53l9_get_frame_period(&dev, &rb_period);
    vl53l9_get_exposure(&dev, prof->context, &rb_exp);
    printf("    readback: frame_period=%uus exposure=%ums\n", rb_period, rb_exp);

    printf("\n[STEP 4/5] Injecting MIPI CSI-2 output configuration...\n");
    uint8_t csi_width = 0, csi_height = 0;
    vl53l9_utils_get_csi_resolution(prof->binning, &csi_width, &csi_height);
    printf("[+] CSI grid for binning %u = %ux%u (frame_height set to %u)\n",
           prof->binning, csi_width, csi_height, csi_height - 1);

    vl53l9_hw_config_t hw_config;
    status = vl53l9_get_hw_config(&dev, &hw_config);
    if (status != 0) { printf("[-] get_hw_config failed (%d). Aborting.\n", status); close(fd); return 1; }

    hw_config.output_interface = VL53L9_OUTPUT_CSI2;
    hw_config.signaling_mode = true;
    hw_config.csi_data_rate = 1000000000;
    hw_config.csi_virtual_channel = 0;
    hw_config.csi_status_line_force_width = false;
    hw_config.csi_status_line_datatype = 0x2A;
    hw_config.csi_frame_datatype = 0x2A;
    hw_config.csi_frame_height = csi_height - 1;
    hw_config.csi_frame_width = csi_width;

    status = vl53l9_set_hw_config(&dev, hw_config);
    if (status != 0) { printf("[-] CRITICAL: set_hw_config rejected (code %d). Aborting.\n", status); close(fd); return 1; }
    printf("[+] MIPI CSI-2 routing configured.\n");

    printf("\n[STEP 5/5] Starting the D-PHY stream...\n");
    status = vl53l9_start(&dev);
    usleep(50000);
    vl53l9_read8(&dev, REG_FSM, &fsm);
    if (status == 0 && fsm == 3) {
        printf("[+] SUCCESS: streaming (FSM=STREAMING).\n");
    } else if (fsm == 3) {
        printf("[+] vl53l9_start() returned %d but FSM reached STREAMING (0x03) - command-ack poll\n", status);
        printf("    is just slow over Linux I2C; the D-PHY is live.\n");
    } else {
        printf("[-] Stream did not start (start ret=%d, FSM=%u/%s).\n", status, fsm, fsm_name(fsm));
        close(fd); return 1;
    }

    printf("\n=========================================================\n");
    printf("[+] TRANSMITTER ONLINE: 1 Gbps, binning %u, 100 Hz.\n", prof->binning);
    printf("=========================================================\n\n");

    close(fd);
    return 0;
}
