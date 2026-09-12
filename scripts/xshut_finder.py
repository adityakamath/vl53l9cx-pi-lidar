#!/usr/bin/env python3
"""xshut_finder.py — find which RPi5 (RP1) GPIO line is wired to the VL53L9CX XSHUT net.

Hardware setup (one wire):
  * Keep the STEVAL-VL53L9 flex cable plugged into the CSI connector (MIPI0).
  * Tap the XSHUT signal on the SENSOR side of the flex connection (FPC header /
    test point on the STEVAL board) and connect it to RPi5 header pin 11 = GPIO17.
  * This tool uses GPIO17 as a passive input monitor while it drives each
    candidate RP1 line LOW then HIGH, sampling GPIO17 after every drive.

The candidate that drags the monitor along against its baseline is the XSHUT
line. Because GPIO17 has an internal pull-up, when the baseline reads HIGH the
identifying phase is LOW (only a physically connected line can drag it down);
when the sensor side pulls the net low (baseline LOW), identifying phase is HIGH.

Why this test:
  Sessions 6-7 swept several lines (35/37/50-53) holding them at power-on and
  got zero I2C ACKs, while the user separately observed XSHUT toggling during a
  sweep — but which line was driven at that moment is unknown. This tool makes
  the attribution unambiguous: one fixed monitor wire + per-line drives.

Usage (on the RPi5, as root):
  sudo python3 xshut_finder.py                # sweep default candidates 35,37,50,51,52,53
  sudo python3 xshut_finder.py --probe 5      # only watch GPIO17 for 5 s (verify the wire)
  sudo python3 xshut_finder.py --hold         # after a MATCH, keep that line HIGH;
                                              #   run `i2cdetect -y 10` in another terminal,
                                              #   Ctrl+C here to release
  sudo python3 xshut_finder.py --lines 48     # custom candidate list (comma separated)

Requires: gpiod Python bindings v2 (verified working on the target Pi).
Safety: every line is released back to free/input at exit, including on
Ctrl+C and exceptions. Lines 34/46 (cam0/cam1_reg enables = connector power)
are hard-refused; lines 38-41 (camera I2C buses) need an explicit override.
"""

import argparse
import signal
import sys
import time

try:
    import gpiod
except ImportError:
    sys.exit("ERROR: 'gpiod' Python bindings v2 not found (pip install gpiod / apt python3-gpio).")

CHIP_DEFAULT = "0"            # gpiochip0 = pinctrl-rp1 on RPi5
MONITOR_LINE_DEFAULT = 17     # header pin 11 = GPIO17 -> gpiochip0 line 17 (verified via gpioinfo)
CANDIDATES_DEFAULT = [35, 37, 50, 51, 52, 53]   # free MIPI0-side candidates (session-7 sweep set)

# Never drive these from userspace:
FORBIDDEN_LINES = {
    34: "CD0_IO0_MICCLK — cam0_reg enable (MIPI0 connector POWER rail)",
    46: "CD1_IO0_MICCLK — cam1_reg enable (MIPI1 connector POWER rail)",
}
# Driving these re-muxes the camera I2C buses away from their controller mid-test:
RISKY_LINES = {
    38: "CD0_SDA — MIPI0 camera I2C bus",
    39: "CD0_SCL — MIPI0 camera I2C bus",
    40: "CD1_SDA — MIPI1 camera I2C bus",
    41: "CD1_SCL — MIPI1 camera I2C bus",
}

ACTIVE = gpiod.line.Value.ACTIVE
INACTIVE = gpiod.line.Value.INACTIVE


def line_name(chip, off):
    try:
        return chip.get_line_info(off).name or "-"
    except Exception:
        return "-"


class Finder:
    def __init__(self, args):
        self.args = args
        self.chip = gpiod.Chip("/dev/gpiochip" + args.chip)
        self.monitor_line = args.monitor_line
        self.requests = []          # every live LineRequest (released on exit)
        self.mon_req = None

    # ---------- monitor ----------
    def open_monitor(self):
        if not self.args.no_pullup:
            try:
                req = self.chip.request_lines(
                    {self.monitor_line: gpiod.LineSettings(
                        direction=gpiod.line.Direction.INPUT, bias=gpiod.line.Bias.PULL_UP)},
                    consumer="xshut_finder",
                )
                self.mon_req = req
                self.requests.append(req)
                return
            except OSError as e:
                print(f"  note: pull-up request failed ({e}); retrying without bias", flush=True)
        req = self.chip.request_lines(
            {self.monitor_line: gpiod.LineSettings(direction=gpiod.line.Direction.INPUT)},
            consumer="xshut_finder",
        )
        self.mon_req = req
        self.requests.append(req)

    def read_monitor(self):
        return int(self.mon_req.get_value(self.monitor_line).value)

    def sample(self, n, interval_ms):
        vals = []
        for _ in range(n):
            vals.append(self.read_monitor())
            time.sleep(interval_ms / 1000.0)
        return vals

    @staticmethod
    def fmt(vals):
        return " ".join("H" if v else "L" for v in vals)

    # ---------- candidates ----------
    def drive_and_sample(self, line, level):
        """Drive `line` at `level`, settle, sample the monitor. Releases afterwards."""
        req = self.chip.request_lines(
            {line: gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT, output_value=level)},
            consumer="xshut_finder",
        )
        try:
            time.sleep(self.args.settle_ms / 1000.0)
            return self.sample(self.args.samples, self.args.interval_ms)
        finally:
            req.release()

    def run_probe(self):
        """Passively watch the monitor for N seconds (wire-verification mode)."""
        a = self.args
        print(f"probing monitor line {a.monitor_line} for {a.probe:.0f}s "
              f"(sample every 50 ms) — Ctrl+C to stop", flush=True)
        t_start = time.time()
        t_end = t_start + a.probe
        last, changes, counts = None, [], {0: 0, 1: 0}
        while True:
            now = time.time()
            if now >= t_end:
                break
            v = self.read_monitor()
            counts[v] += 1
            if v != last:
                changes.append((now - t_start, v))
                print(f"  t={changes[-1][0]:6.2f}s -> {'HIGH' if v else 'LOW'}", flush=True)
                last = v
            time.sleep(0.05)
        total = counts[0] + counts[1] or 1
        lvl = "HIGH" if counts[1] * 2 > total else "LOW"
        print(f"\nprobe done: {lvl} dominant ({counts[1]}H/{counts[0]}L), "
              f"{len(changes)} transition(s)", flush=True)
        if len(changes) <= 1:
            print("level was constant. If the XSHUT wire is NOT connected yet, this is expected", flush=True)
            print("(pull-up holds HIGH). Connect it and re-probe: a different level or transitions", flush=True)
            print("while toggling on the sensor side confirm the wire.", flush=True)

    def run_sweep(self):
        a = self.args
        names = {ln: line_name(self.chip, ln) for ln in a.lines}
        print("=" * 72, flush=True)
        print(" XSHUT FINDER — gpiochip%s (RP1 pinctrl)" % a.chip, flush=True)
        print(f" monitor   : line {a.monitor_line} \"{line_name(self.chip, a.monitor_line)}\" "
              f"(input{' + pull-up' if not a.no_pullup else ''})", flush=True)
        cand_str = " | ".join(f"{ln} {names[ln]}" for ln in a.lines)
        print(f" candidates: {cand_str}", flush=True)
        print(f" timing    : settle={a.settle_ms}ms samples={a.samples} interval={a.interval_ms}ms", flush=True)
        print("=" * 72, flush=True)

        base = self.sample(10, 25)
        bsum = sum(base)
        print(f" baseline monitor: {'HIGH' if bsum * 2 > len(base) else 'LOW'} "
              f"({bsum}/{len(base)} high)", flush=True)
        print(" (with the XSHUT wire connected this is the live sensor-side XSHUT level)\n", flush=True)

        results = []
        for i, ln in enumerate(a.lines, 1):
            print(f"[{i}/{len(a.lines)}] line {ln} \"{names[ln]}\"", flush=True)
            pre = self.sample(3, 20)
            b_high = sum(pre) * 2 >= len(pre)   # fresh per-candidate baseline
            try:
                low_vals = self.drive_and_sample(ln, INACTIVE)
                high_vals = self.drive_and_sample(ln, ACTIVE)
            except OSError as e:
                print(f"    SKIPPED — cannot request line ({e})", flush=True)
                results.append((ln, names[ln], None))
                continue
            lo_follows = all(v == 0 for v in low_vals)
            hi_follows = all(v == 1 for v in high_vals)
            print(f"    baseline (pre-drive): {'HIGH' if b_high else 'LOW'}", flush=True)
            print(f"    drive LOW : monitor {self.fmt(low_vals)}", flush=True)
            print(f"    drive HIGH: monitor {self.fmt(high_vals)}", flush=True)
            # The identifying phase depends on the baseline, because our own
            # internal pull-up makes a HIGH reading trivial when the net is not
            # actively pulled low by something else:
            #   B=HIGH -> only an actually-connected line can drag it LOW.
            #   B=LOW  -> (sensor side pulls the net down) only a connected
            #             line can drive it back HIGH; LOW readings are trivial.
            if b_high:
                verdict = "MATCH" if lo_follows else "NO-MATCH"
            else:
                verdict = "MATCH" if hi_follows else "NO-MATCH"
            if verdict == "MATCH":
                how = ("LOW phase dragged the net down (baseline was HIGH)" if b_high
                       else "HIGH phase drove the net up against a low baseline")
                print(f"    >>> MATCH — line {ln} is connected to the XSHUT net ({how}) <<<", flush=True)
            else:
                both = low_vals + high_vals
                flapped = 0 < sum(both) < len(both) and (len(set(low_vals)) > 1 or len(set(high_vals)) > 1)
                if flapped:
                    print("    no match — monitor flapped (net may have a competing driver)", flush=True)
                else:
                    stayed = "HIGH" if b_high else "LOW"
                    print(f"    no match — monitor never left {stayed}", flush=True)
            results.append((ln, names[ln], verdict))

        print("\n" + "=" * 72, flush=True)
        print(" SUMMARY", flush=True)
        for ln, name, verdict in results:
            mark = {"MATCH": "<<< XSHUT >>>", None: "(skipped)"}.get(verdict, "")
            print(f"  line {ln:<3} {name:<18} {verdict or 'SKIPPED':<9} {mark}", flush=True)

        matches = [ln for ln, _, v in results if v == "MATCH"]
        if not matches:
            print("\nNo candidate drove the monitor. Possibilities:", flush=True)
            print("  - XSHUT wire not connected / broken (re-run --probe to verify)", flush=True)
            print("  - XSHUT net is on a line outside this set (try --lines with other free RP1 lines)", flush=True)
            print("  - the flex cable does not carry XSHUT through at all", flush=True)
            return

        if a.hold:
            lvl = ACTIVE if a.hold == "high" else INACTIVE
            req = self.chip.request_lines(
                {matches[0]: gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT, output_value=lvl)},
                consumer="xshut_finder_hold",
            )
            self.requests.append(req)
            state = "released — sensor enabled" if lvl == ACTIVE else "asserted — sensor shut down"
            print(f"\nHOLDING line {matches[0]} {'HIGH' if lvl == ACTIVE else 'LOW'} (XSHUT {state}).", flush=True)
            print("  Now, in ANOTHER terminal:  i2cdetect -y 10   (expect ACK at 0x29)", flush=True)
            print("  Ctrl+C here to release the line.", flush=True)
            try:
                while True:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass

def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--chip", default=CHIP_DEFAULT, help="gpiochip number (default %(default)s = RP1)")
    p.add_argument("--monitor-line", type=int, default=MONITOR_LINE_DEFAULT,
                   help="monitor line on the chip (default %(default)s = GPIO17 = header pin 11)")
    p.add_argument("--lines", default=",".join(map(str, CANDIDATES_DEFAULT)),
                   help=f"comma-separated candidate lines to sweep "
                        f"(default: {','.join(map(str, CANDIDATES_DEFAULT))})")
    p.add_argument("--probe", type=float, metavar="SECONDS",
                   help="only watch the monitor for N seconds and exit (wire verification)")
    p.add_argument("--hold", choices=["high", "low"], default=None,
                   help="after a MATCH, keep that line at the given level until Ctrl+C")
    p.add_argument("--settle-ms", type=int, default=80, help="ms to wait after driving before sampling")
    p.add_argument("--samples", type=int, default=5, help="monitor samples per drive phase")
    p.add_argument("--interval-ms", type=int, default=25, help="ms between monitor samples")
    p.add_argument("--no-pullup", action="store_true", help="do not enable internal pull-up on the monitor")
    p.add_argument("--i-know-what-im-doing", action="store_true",
                   help="allow sweeping camera I2C bus lines 38-41 (breaks those buses while held)")
    args = p.parse_args()

    try:
        args.lines = [int(x) for x in args.lines.split(",") if x.strip()]
    except ValueError:
        sys.exit("ERROR: --lines must be comma-separated integers, e.g. --lines 35,37,48")
    if not args.lines:
        sys.exit("ERROR: no candidate lines given")

    for ln in args.lines:
        if ln in FORBIDDEN_LINES:
            sys.exit(f"REFUSING to drive line {ln}: {FORBIDDEN_LINES[ln]}")
        if ln in RISKY_LINES and not args.i_know_what_im_doing:
            sys.exit(f"line {ln} is {RISKY_LINES[ln]}; remove it or pass --i-know-what-im-doing "
                     f"(driving it re-muxes the camera I2C bus away from its controller)")

    if not (0 <= args.monitor_line < 64):
        sys.exit("ERROR: monitor line out of range for this chip")

    finder = Finder(args)

    def cleanup():
        released = False
        for req in reversed(finder.requests):
            try:
                req.release()
                released = True
            except Exception:
                pass
        if released:
            print("\nall GPIO requests released (lines back to free/input).", flush=True)

    def on_sigint(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_sigint)
    try:
        finder.open_monitor()
        if args.probe is not None:
            finder.run_probe()
        else:
            finder.run_sweep()
    except KeyboardInterrupt:
        print("\ninterrupted.", flush=True)
    finally:
        cleanup()


if __name__ == "__main__":
    main()

