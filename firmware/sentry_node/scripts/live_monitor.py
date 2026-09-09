#!/usr/bin/env python
"""
live_monitor.py - watch the microphone in plain text.

Two modes, matching the firmware:

  meter   level only, no detector. RUN THIS FIRST after wiring. A dead or
          mis-clocked INMP441 reads as a constant, as all-zero or as
          full-scale, and all three are obvious here and invisible once the
          adaptive floor has swallowed them.
  live    the full detector at the DEPLOYMENT default (HIGH_ALERT, 1.70),
          per-frame score / top bin / chain, and a loud line on every alert.

Ctrl-C exits cleanly: it tells the device to stop, drains the sentinel and
closes the port, so the next command does not land in a half-finished stream.

Needs pyserial -> run under the ESP-IDF python:
    eim run "python scripts/live_monitor.py meter" v6.0.2
"""
import argparse
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent))
from capture_trace import resolve_port          # noqa: E402
from trace_proto import parse_stream            # noqa: E402

BIN_HZ = 16000.0 / 2048.0        # 7.8125 Hz per FFT bin


def fmt_meter(m):
    """int16 units, not dBFS. The device does not convert; a unit conversion
    on the device is a place for a bug to hide."""
    full = 32768.0
    rms, pk = m["rms"], m["peak_abs"]
    dbfs = 20 * (0.0 if rms <= 0 else __import__("math").log10(rms / full))
    bar = "#" * min(40, int(40 * rms / 3000.0))
    warn = ""
    if m["timeouts"]:
        warn = "  !! I2S TIMEOUTS - mic not clocking (check BCLK/WS/SD)"
    elif rms == 0:
        warn = "  !! dead silence - mic not driving data"
    elif pk >= 32700:
        warn = "  !! clipping"
    return (f"  rms {rms:9.1f} ({dbfs:6.1f} dBFS)  peak {pk:6d}  "
            f"dc {m['dc_offset']:+9.1f}  min/max {m['vmin']:6d}/{m['vmax']:6d}"
            f"  |{bar:<40}|{warn}")


def fmt_tone(t):
    """PLUMBING ONLY. A clean peak here means the samples arrive right way up
    and at the right rate. It is not detection and implies nothing about
    range."""
    L = [f"  averaged {t['n_avg']} frames   bin width {t['bin_hz']:.4f} Hz",
         f"  PEAK bin {t['peak_bin']}  = {t['peak_hz']:.1f} Hz",
         f"  DC (bin 0) magnitude {t['dc_mag']:.4f}  "
         f"(measured, never removed)",
         "  top bins:"]
    for b, m in zip(t["top_bin"], t["top_mag"]):
        L.append(f"    bin {b:4d}  {b * t['bin_hz']:8.1f} Hz   mag {m:10.4f}")
    L.append("")
    L.append("  A 440 Hz tone should peak at bin 56 (440 / 7.8125 = 56.3).")
    L.append("  This validates PLUMBING ONLY - never detection, never range.")
    return "\n".join(L)


def fmt_frame(r, thr):
    f0 = r["f0_hz"]
    hit = "*" if r["above_thr"] else " "
    chain = r["chain"]
    bar = "#" * min(24, max(0, int(24 * r["score"] / 3.0)))
    return (f"  f{r['frame']:<6d} t{r['t_s']:8.2f}s  score {r['score']:7.4f}{hit}"
            f"  f0 {f0:7.1f}Hz (bin {r['f0_bin']:4d})  teeth {r['teeth']:2d}"
            f"  chain {chain:2d}  |{bar:<24}|")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["meter", "tone", "live"])
    ap.add_argument("--avg", type=int, default=16,
                    help="tone mode: frames to average (16 ~ 0.5 s)")
    ap.add_argument("--port", default=None)
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after N seconds (0 = until Ctrl-C)")
    a = ap.parse_args()

    port = resolve_port(a.port)
    cmd = {"meter": "M", "tone": f"T {a.avg}", "live": "L"}[a.mode]
    print(f"port {port}   mode {a.mode}   Ctrl-C to stop")
    if a.mode == "tone":
        print("PLUMBING CHECK ONLY: does a known tone land in the right FFT "
              "bin? This is not detection and implies nothing about range.")
    if a.mode == "live":
        print("running the detector at the DEPLOYMENT default (HIGH_ALERT). "
              "An alert here means the COMB DETECTOR fired on room audio - it "
              "says nothing about drones being present.")
    print()

    buf = bytearray()
    n_alert = 0
    thr = None
    t0 = time.time()
    stopped = False
    with serial.Serial(port, 115200, timeout=0.2) as s:
        s.reset_input_buffer()
        s.reset_output_buffer()
        time.sleep(0.3)
        s.read(s.in_waiting or 0)
        s.write((cmd + "\n").encode())
        s.flush()
        try:
            while True:
                chunk = s.read(65536)
                if chunk:
                    buf += chunk
                    recs, _, used = parse_stream(bytes(buf))
                    for r in recs:
                        k = r["_kind"]
                        if k == "hdr":
                            thr = r["threshold"]
                            print(f"  [{r['name']}] fs={r['fs']} "
                                  f"thr={thr:.4f}")
                        elif k == "met":
                            print(fmt_meter(r))
                        elif k == "ton":
                            print(fmt_tone(r))
                            stopped = True
                        elif k == "rec":
                            print(fmt_frame(r, thr))
                        elif k == "alt":
                            n_alert += 1
                            print(f"  *** ALERT #{n_alert}  frame "
                                  f"{r['frame']} t={r['t_s']:.2f}s  "
                                  f"f0={r['f0_hz']:.1f}Hz  "
                                  f"score={r['score']:.4f}  "
                                  f"chain={r['chain']} ***")
                        elif k == "end":
                            print(f"  [sentinel] {r['n_frames']} records, "
                                  f"{r['n_events']} events")
                            stopped = True
                    # keep the unconsumed tail: a record split across a read
                    # boundary must survive to the next chunk
                    buf = buf[used:]
                    if stopped:
                        break
                if a.seconds and time.time() - t0 > a.seconds:
                    break
        except KeyboardInterrupt:
            print("\n  stopping...")
        finally:
            # tell the device to leave the mode, then drain to its sentinel so
            # the port is left clean for the next command
            if not stopped:
                try:
                    s.write(b"\n")
                    s.flush()
                    t1 = time.time()
                    while time.time() - t1 < 2.0:
                        c = s.read(4096)
                        if not c:
                            break
                except Exception:
                    pass
    print(f"\n  done. alerts: {n_alert}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
