#!/usr/bin/env python
"""Which tier fires on a given sound, attributed and live.

The event ring answers this too, but the ring is RAM and its RTC mirror does
not survive a brownout. This depends on neither: it decodes the binary alert
records off the wire as they happen and prints tier, score, threshold and f0
for every one.

v1 -> `alt`, Tier-2 -> `al2`, Tier-3 -> `al3`, Tier-4 -> the T4R text line
with fired=1. All four are on the same stream."""
import sys, time, re, argparse
from collections import Counter
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import serial                                                   # noqa: E402
from capture_trace import resolve_port                          # noqa: E402
from trace_proto import parse_stream                            # noqa: E402
T4RE = re.compile(rb"T4R f=(\d+) t=([\d.]+) W4=([\d.]+) f0=([\d.]+)[^\n]*"
                  rb"fired=1")

ap = argparse.ArgumentParser()
ap.add_argument("--seconds", type=float, default=900.0)
ap.add_argument("--tag", default="attrib")
ap.add_argument("--out", type=Path,
                default=Path(__file__).resolve().parent.parent / "captures",
                help="where to write the capture (default: ../captures)")
a = ap.parse_args()
SCRATCH = a.out
SCRATCH.mkdir(parents=True, exist_ok=True)

port = resolve_port(None)
s = serial.Serial()
s.port, s.baudrate, s.timeout = port, 115200, 0.2
s.dtr = s.rts = False
s.open()
print(f"### attributing alerts on {port} for {a.seconds/60:.0f} min", flush=True)
print("### TALK and PLAY THE PIANO - every alert is printed with its tier",
      flush=True)

t0 = time.time()
buf = bytearray()
text = bytearray()
events = []
counts = Counter()
while time.time() - t0 < a.seconds:
    c = s.read(65536)
    if not c:
        continue
    buf += c
    text += c
    recs, _, used = parse_stream(bytes(buf))
    for r in recs:
        k = r["_kind"]
        el = time.time() - t0
        if k == "alt":
            counts["v1"] += 1
            events.append((el, "v1", r["f0_hz"], r["score"], r["chain"]))
            print(f"  [{el:7.1f}s] v1  f0={r['f0_hz']:7.1f} Hz  "
                  f"score={r['score']:6.2f}  chain={r['chain']}", flush=True)
        elif k == "al2":
            counts["T2"] += 1
            events.append((el, "T2", r["f02_hz"], r["score2"], r["hits"]))
            print(f"  [{el:7.1f}s] T2  f0={r['f02_hz']:7.1f} Hz  "
                  f"score={r['score2']:6.2f}  hits={r['hits']}", flush=True)
        elif k == "al3":
            counts["T3"] += 1
            events.append((el, "T3", r["r_hz"], r["W"], r["hits"]))
            print(f"  [{el:7.1f}s] T3  rate={r['r_hz']:6.1f}/s  "
                  f"W={r['W']:6.2f}  hits={r['hits']}", flush=True)
    buf = buf[used:]
    for m in T4RE.finditer(bytes(c)):
        el = time.time() - t0
        counts["T4"] += 1
        events.append((el, "T4", float(m.group(4)), float(m.group(3)), 0))
        print(f"  [{el:7.1f}s] T4  f0={float(m.group(4)):7.1f} Hz  "
              f"W4={float(m.group(3)):6.2f}", flush=True)
s.close()

hrs = (time.time() - t0) / 3600.0
(SCRATCH / f"{a.tag}.log").write_bytes(bytes(text))
print(f"\n### {hrs*60:.1f} minutes of speech and piano")
print(f"{'tier':>5}  {'events':>7}  {'per hour':>9}   {'f0 / rate seen':>28}")
for tier in ("v1", "T2", "T3", "T4"):
    ev = [e for e in events if e[1] == tier]
    if ev:
        f0s = sorted(e[2] for e in ev)
        rng = f"{f0s[0]:.0f} .. {f0s[-1]:.0f}"
        sc = sorted(e[3] for e in ev)
        rng += f"   score {sc[0]:.2f}..{sc[-1]:.2f}"
    else:
        rng = "-"
    print(f"{tier:>5}  {counts[tier]:7d}  {counts[tier]/hrs if hrs else 0:9.1f}"
          f"   {rng:>28}")
