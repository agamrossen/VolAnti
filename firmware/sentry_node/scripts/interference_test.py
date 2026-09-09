#!/usr/bin/env python
"""
interference_test.py - do the alert outputs pollute the microphone?

    conda activate acoustic-detector
    python scripts/interference_test.py

The requirement is not "the outputs are silent" - a buzzer is supposed to be
loud. It is that the noise floor RETURNS to baseline once they stop, and that
the two SILENT outputs (LED and e-paper) never move it at all. An e-paper
refresh draws tens of milliamps in bursts and the WS2812 is a switching load on
the same 3V3 rail; either could couple into an analogue-adjacent MEMS
microphone, and that would quietly raise the detector's floor for as long as the
indicator was lit.

Method: the mono meter (`M`) with the outputs commanded between segments. This
works only because `U` states PERSIST across the passive modes - entering the
meter does not reset them.

PASS: LED, e-paper and post segments within +/- 15 % of baseline rms.
Buzzer and motor segments are EXPECTED to be elevated; masking during an alert
is acceptable, going deaf afterwards is not.

The comparison is WITHIN-RUN. the port's quiet-room median rms of 605 is
printed only as a sanity footnote.
"""
import argparse
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from capture_trace import open_serial, resolve_port                # noqa: E402
from trace_proto import parse_stream                               # noqa: E402

STAGE1B_QUIET_RMS = 605.0
TOL = 0.15

RED = "\033[31m"
GREEN = "\033[32m"
BOLD = "\033[1m"
OFF = "\033[0m"


def one_command(port, cmd):
    """Fire a U command and wait for its ACK."""
    with open_serial(port, timeout=0.2) as s:
        s.write((cmd + "\n").encode())
        s.flush()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < 4.0:
            c = s.read(4096)
            if not c:
                continue
            buf += c
            recs, _, used = parse_stream(bytes(buf))
            for r in recs:
                if r["_kind"] == "ack":
                    return r["status_name"]
            buf = buf[used:]
    return "NO_ACK"


def meter_segment(port, seconds, label):
    """Run the mono meter for `seconds` and return the median rms."""
    vals = []
    with open_serial(port, timeout=0.2) as s:
        s.write(b"M\n")
        s.flush()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < seconds:
            c = s.read(65536)
            if not c:
                continue
            buf += c
            recs, _, used = parse_stream(bytes(buf))
            for r in recs:
                if r["_kind"] == "met":
                    vals.append(r["rms"])
            buf = buf[used:]
        s.write(b"\n")
        s.flush()
        t1 = time.time()
        while time.time() - t1 < 2.0:
            if not s.read(4096):
                break
    med = statistics.median(vals) if vals else float("nan")
    print(f"    {label:<22} n={len(vals):3d}  median rms {med:9.1f}")
    return med


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--short", action="store_true",
                    help="halve every segment (a quick smoke pass)")
    a = ap.parse_args()
    port = resolve_port(a.port)
    k = 0.5 if a.short else 1.0

    print(f"port {port}")
    print(f"{BOLD}  acoustic non-interference test{OFF}")
    print("  Keep the room as quiet as you can for the whole run.\n")

    print("  segments:")
    base = meter_segment(port, 10 * k, "baseline (all off)")

    one_command(port, "U l 255 255 255")
    led = meter_segment(port, 10 * k, "LED white")
    one_command(port, "U l 0 0 0")

    one_command(port, "U e a")            # refresh happens inside the window
    epd = meter_segment(port, 15 * k, "e-paper refresh")

    one_command(port, "U b1")
    buz = meter_segment(port, 3 * k, "buzzer ON (expected up)")
    one_command(port, "U b0")

    one_command(port, "U v1")
    mot = meter_segment(port, 3 * k, "motor ON (expected up)")
    one_command(port, "U v0")

    post = meter_segment(port, 10 * k, "post (all off)")

    print(f"\n{BOLD}  results{OFF}   baseline = {base:.1f} rms")
    ok = True
    for label, v in (("LED white", led), ("e-paper refresh", epd),
                     ("post", post)):
        if base and base == base:                    # not NaN
            dev = (v - base) / base
            good = abs(dev) <= TOL
            ok &= good
            tag = f"{GREEN}PASS{OFF}" if good else f"{RED}FAIL{OFF}"
            print(f"    [{tag}] {label:<18} {v:9.1f}  "
                  f"{dev * 100:+6.1f}% vs baseline (tol +/-{TOL * 100:.0f}%)")
    for label, v in (("buzzer", buz), ("motor", mot)):
        print(f"    [info] {label:<18} {v:9.1f}  "
              f"{(v - base) / base * 100:+6.1f}%  (elevation here is expected)")

    print(f"\n    footnote: the quiet-room reference was "
          f"{STAGE1B_QUIET_RMS:.0f} rms; this test compares WITHIN this run, "
          f"not against that number.")
    print(f"\n{BOLD}  INTERFERENCE TEST: {'PASS' if ok else 'FAIL'}{OFF}")
    if not ok:
        print("    A silent output moved the floor. That matters: it raises "
              "the detector's noise floor for as long as the indicator is "
              "lit. Record it and bring it to planning - do not compensate "
              "for it in the detector.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
