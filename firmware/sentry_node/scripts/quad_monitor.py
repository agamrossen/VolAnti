#!/usr/bin/env python
"""
quad_monitor.py - four live level bars, four DC offsets, two bus health columns.

    conda activate acoustic-detector
    python scripts/quad_monitor.py           Ctrl-C to stop

This is the quad equivalent of the mono meter, and it exists to answer one
question before any analysis is attempted: are all four microphones alive?

PER-BUS TIMEOUT COUNTERS ARE SHOWN IN RED WHEN NONZERO. A dead bus is the
failure this tool is for. The firmware deliberately keeps trying and keeps
emitting rather than exiting cleanly on a timeout, because a silent clean exit
is exactly how the mono meter hides the same fault today.

Reference points from the port's single microphone: quiet-room rms median 605,
DC offset +422 / -410 (it changes sign between runs, which is expected).
"""
import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from capture_trace import open_serial, resolve_port                # noqa: E402
from quad_analysis import CH_NAMES                                 # noqa: E402
from trace_proto import parse_stream                               # noqa: E402

RED = "\033[31m"
GREEN = "\033[32m"
DIM = "\033[2m"
OFF = "\033[0m"


def bar(v, vmax=3000.0, width=28):
    n = 0 if vmax <= 0 else int(width * min(1.0, v / vmax))
    return "#" * n + "-" * (width - n)


def render(r):
    lines = []
    for c in range(4):
        rms = r["rms"][c]
        lines.append(
            f"  {CH_NAMES[c]:<9} |{bar(rms)}|  rms {rms:8.1f}  "
            f"dc {r['dc'][c]:+8.1f}  peak {r['peak'][c]:6d}")
    bus = []
    for b, label in ((0, "A(GPIO6) M1+M2"), (1, "B(GPIO7) M3+M4")):
        t, s, f = r["timeouts"][b], r["short_reads"][b], r["frames_total"][b]
        colour = RED if t else GREEN
        bus.append(f"  bus {label}: {colour}timeouts {t}{OFF}  "
                   f"short {s}  frames {f}")
    return "\n".join(lines + bus)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after N seconds (0 = until Ctrl-C)")
    a = ap.parse_args()

    port = resolve_port(a.port)
    print(f"port {port}   quad meter   Ctrl-C to stop")
    print(f"{DIM}  single-mic reference: quiet rms ~605, "
          f"dc +422/-410 (sign varies between runs){OFF}\n")

    n_rec = 0
    warned = False
    with open_serial(port, timeout=0.2) as s:
        s.write(b"Q\n")
        s.flush()
        buf = bytearray()
        t0 = time.time()
        try:
            while True:
                chunk = s.read(65536)
                if chunk:
                    buf += chunk
                    recs, _, used = parse_stream(bytes(buf))
                    for r in recs:
                        k = r["_kind"]
                        if k == "qmt":
                            n_rec += 1
                            print(render(r))
                            print()
                            if (any(r["timeouts"]) and not warned):
                                warned = True
                                print(f"{RED}  !! a bus is timing out. Check "
                                      f"BCLK/WS at the M3/M4 pads and SD-B on "
                                      f"GPIO7 before going further "
                                      f"(QUAD_I2S_NOTES.md has the "
                                      f"failure-mode table).{OFF}\n")
                        elif k == "sta":
                            print(f"{RED}  !! device reported an I2S read "
                                  f"timeout (mode {r['mode_chr']}){OFF}")
                        elif k == "end":
                            print("  [sentinel]")
                            return 0
                    buf = buf[used:]
                if a.seconds and time.time() - t0 > a.seconds:
                    break
        except KeyboardInterrupt:
            print("\n  stopping...")
        finally:
            try:
                s.write(b"\n")
                s.flush()
                t1 = time.time()
                while time.time() - t1 < 2.0:
                    if not s.read(4096):
                        break
            except Exception:
                pass
    print(f"\n  done. {n_rec} meter records.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
