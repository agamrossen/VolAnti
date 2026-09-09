#!/usr/bin/env python
"""Does v1 fire on real speech? Measured on the device, not on the corpus.

The corpus cannot answer this: its synthetic `speech` confuser fires zero at
the shipped operating point, and a synthetic corpus cannot calibrate a feature
nobody put into it. So this measures the real thing on the shipped image.

Every alert is decoded from the wire with its tier. For v1 alerts the
per-frame record around the decision is kept too, so the f0 and the teeth the
veto actually saw are on the record rather than inferred - the shipped veto is
`f0 >= 250 AND teeth >= 11` and those are its two inputs."""
import sys, time, re, argparse
from collections import Counter
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import serial                                                   # noqa: E402
from capture_trace import resolve_port                          # noqa: E402
from trace_proto import parse_stream                            # noqa: E402
T4RE = re.compile(rb"T4R f=(\d+) t=([\d.]+) W4=([\d.]+) f0=([\d.]+) "
                  rb"ratio=[\d.]+ hit=(\d) above=(\d) excl=\d hits=\d/8 "
                  rb"age=\d+ fired=1")

ap = argparse.ArgumentParser()
ap.add_argument("--seconds", type=float, default=300.0)
ap.add_argument("--tag", default="speech")
ap.add_argument("--out", type=Path,
                default=Path(__file__).resolve().parent.parent / "captures",
                help="where to write the capture (default: ../captures)")
a = ap.parse_args()
SCRATCH = a.out
SCRATCH.mkdir(parents=True, exist_ok=True)

s = serial.Serial()
s.port, s.baudrate, s.timeout = resolve_port(None), 115200, 0.2
s.dtr = s.rts = False
s.open()
print(f"### SPEECH TEST: {a.seconds/60:.0f} minutes on {s.port}", flush=True)
print("### every alert is printed with its tier as it happens", flush=True)

t0 = time.time()
buf = bytearray(); raw = bytearray()
events = []; counts = Counter()
recent = []            # rolling per-frame v1 records, for teeth at the decision
t4_prev = False
while time.time() - t0 < a.seconds:
    c = s.read(65536)
    if not c:
        continue
    buf += c; raw += c
    recs, _, used = parse_stream(bytes(buf))
    for r in recs:
        k = r["_kind"]; el = time.time() - t0
        if k == "rec":
            recent.append((r.get("f0_hz"), r.get("teeth"), r.get("score")))
            if len(recent) > 400:
                del recent[:200]
        elif k == "alt":
            counts["v1"] += 1
            teeth = recent[-1][1] if recent else None
            events.append((el, "v1", r["f0_hz"], r["score"], teeth))
            print(f"  [{el:6.1f}s] v1  f0={r['f0_hz']:7.1f} Hz  "
                  f"score={r['score']:5.2f}  teeth={teeth}  "
                  f"chain={r['chain']}", flush=True)
        elif k == "al2":
            counts["T2"] += 1
            events.append((el, "T2", r["f02_hz"], r["score2"], None))
            print(f"  [{el:6.1f}s] T2  f0={r['f02_hz']:7.1f} Hz  "
                  f"score={r['score2']:5.2f}", flush=True)
        elif k == "al3":
            counts["T3"] += 1
            events.append((el, "T3", r["r_hz"], r["W"], None))
            print(f"  [{el:6.1f}s] T3  rate={r['r_hz']:5.1f}/s  "
                  f"W={r['W']:5.2f}", flush=True)
    buf = buf[used:]
    # Tier-4 LATCHES, not latch-held frames
    for m in T4RE.finditer(bytes(c)):
        if not t4_prev:
            counts["T4"] += 1
            el = time.time() - t0
            events.append((el, "T4", float(m.group(4)), float(m.group(3)), None))
            print(f"  [{el:6.1f}s] T4  f0={float(m.group(4)):7.1f} Hz  "
                  f"W4={float(m.group(3)):5.2f}", flush=True)
        t4_prev = True
    if b"fired=0" in c:
        t4_prev = False
s.close()
(SCRATCH / f"{a.tag}.log").write_bytes(bytes(raw))

mins = (time.time() - t0) / 60.0
print(f"\n### {mins:.1f} minutes of speech at 1 m")
print(f"{'tier':>5}  {'alerts':>6}  {'per hour':>9}   detail")
for tier in ("v1", "T2", "T3", "T4"):
    ev = [e for e in events if e[1] == tier]
    d = "-"
    if ev:
        f0 = sorted(e[2] for e in ev)
        d = f"f0 {f0[0]:.0f}..{f0[-1]:.0f} Hz"
        th = [e[4] for e in ev if e[4] is not None]
        if th:
            d += f"   teeth {min(th)}..{max(th)}"
    print(f"{tier:>5}  {counts[tier]:6d}  {counts[tier]/(mins/60) if mins else 0:9.1f}"
          f"   {d}")
print(f"\nsection 4.3 requires ZERO v1 alerts. v1 = {counts['v1']}.")
