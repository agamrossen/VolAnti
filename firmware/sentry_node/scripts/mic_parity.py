#!/usr/bin/env python
"""
mic_parity.py - THE Stage-1b acceptance test.

Real air through the C detector and through src/detector.py must agree
DECISION FOR DECISION. The golden vectors prove the port reproduces a sealed
recording; this proves it reproduces whatever the microphone actually heard,
including whatever the microphone gets wrong.

The device streams, interleaved, both the int16 samples it fed the pipeline AND
its own per-frame trace. The host re-runs the reference detector on exactly
those samples at exactly that threshold, and diffs with the SAME comparator the
golden gate uses.

TWO PHASES, because no single interpreter here has both dependencies:
pyserial lives in the ESP-IDF python, numpy and the reference detector live in
the conda env. Splitting is honest; auto-importing across environments is not.

    eim run "python scripts/mic_parity.py capture --seconds 60 --out captures/mic1" v6.0.2
    conda run -n acoustic-detector python scripts/mic_parity.py analyse --in captures/mic1

`capture` writes <out>.bin (the raw stream) and <out>.wav (listen to it - that
is the point of having a microphone). `analyse` needs no board.
"""
import argparse
import struct
import sys
import time
import wave
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent.parent
sys.path.insert(0, str(HERE))

from trace_proto import parse                                    # noqa: E402


# ---------------------------------------------------------------------------
# phase 1: capture   (needs pyserial; run under the ESP-IDF python)
# ---------------------------------------------------------------------------
def phase_capture(a):
    import serial                                                # noqa: E402
    from capture_trace import open_serial, resolve_port          # noqa: E402

    port = resolve_port(a.port)
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    budget = a.seconds + 30.0

    print(f"port {port}   parity capture, {a.seconds} s "
          f"(hard timeout {budget:.0f} s)")
    buf = bytearray()
    with open_serial(port, timeout=0.5) as s:
        s.write(f"Y {int(a.seconds)}\n".encode())
        s.flush()
        t0 = time.time()
        last = t0
        from trace_proto import MAGIC_END
        while time.time() - t0 < budget:
            c = s.read(65536)
            if c:
                buf += c
                last = time.time()
                if MAGIC_END.to_bytes(4, "little") in buf[-8192:]:
                    time.sleep(0.2)
                    buf += s.read(s.in_waiting or 0)
                    break
            elif time.time() - last > 10.0 and buf:
                print("  stream went quiet", file=sys.stderr)
                break

    recs, bad = parse(bytes(buf))
    hdr = next((r for r in recs if r["_kind"] == "hdr"), None)
    end = next((r for r in recs if r["_kind"] == "end"), None)
    pcm = [r for r in recs if r["_kind"] == "pcm"]
    trc = [r for r in recs if r["_kind"] == "rec"]
    alt = [r for r in recs if r["_kind"] == "alt"]

    Path(str(out) + ".bin").write_bytes(bytes(buf))

    if hdr is None or end is None:
        print("FAIL no header or no sentinel - capture incomplete",
              file=sys.stderr)
        return 2

    samples = reassemble(pcm)
    if samples is None:
        print("FAIL sample stream has a gap", file=sys.stderr)
        return 3

    wp = Path(str(out) + ".wav")
    with wave.open(str(wp), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(hdr["fs"])
        w.writeframes(struct.pack(f"<{len(samples)}h", *samples))

    print(f"  {len(buf)} bytes  frames {len(trc)}/{hdr['n_frames']}  "
          f"samples {len(samples)}  alerts {len(alt)}  bad_chk {bad}")
    print(f"  wrote {out}.bin and {wp}  ({len(samples) / hdr['fs']:.1f} s)")
    print(f"\n  now: conda run -n acoustic-detector python "
          f"{Path(__file__).name} analyse --in {out}")
    return 0


def reassemble(pcm):
    """Concatenate the PCM records in stream order and verify there is no gap.
    A gap would silently shift every later frame, so it is a hard error."""
    out = []
    nxt = 0
    for r in sorted(pcm, key=lambda r: r["seq"]):
        if r["first_index"] != nxt:
            return None
        out.extend(r["samples"])
        nxt += r["n"]
    return out


# ---------------------------------------------------------------------------
# phase 2: analyse   (needs numpy + src/detector.py; run under conda)
# ---------------------------------------------------------------------------
def phase_analyse(a):
    import numpy as np                                           # noqa: E402
    sys.path.insert(0, str(ROOT / "src"))
    import detector as D                                         # noqa: E402
    import operating_point as op                                 # noqa: E402
    from compare_trace import compare_records, report            # noqa: E402

    raw = Path(str(a.inp) + ".bin").read_bytes()
    recs, bad = parse(raw)
    hdr = next((r for r in recs if r["_kind"] == "hdr"), None)
    end = next((r for r in recs if r["_kind"] == "end"), None)
    dev = [r for r in recs if r["_kind"] == "rec"]
    pcm = [r for r in recs if r["_kind"] == "pcm"]
    alt = [r for r in recs if r["_kind"] == "alt"]
    if hdr is None or end is None:
        raise SystemExit("capture has no header or no sentinel")

    samples = reassemble(pcm)
    if samples is None:
        raise SystemExit("sample stream has a gap - cannot re-derive")
    q = np.array(samples, np.int16)

    # the preset the DEVICE says it ran, matched to a named preset
    thr = hdr["threshold"]
    preset = next((k for k, v in op.PRESETS.items()
                   if abs(v["threshold"] - thr) < 1e-12), None)
    if preset is None:
        raise SystemExit(f"device ran at {thr}, which is not a named preset")
    cfg, _ = op.preset_config(preset)

    # exactly the golden conversion: q / 32767 in float32
    x = q.astype(np.float32) / 32767.0
    tr = D.CombDetector(cfg).analyze(x)
    ev, fr = D.track_frames(tr, thr, cfg)

    n = len(tr["t"])
    f0_bin = np.round((tr["f0"] - cfg.f_search_lo) / cfg.f_step).astype(int)
    ref = [{
        "frame": str(i), "t_s": f"{tr['t'][i]:.6f}",
        "score": f"{tr['score'][i]:.6f}", "f0_bin": str(int(f0_bin[i])),
        "f0_hz": f"{tr['f0'][i]:.1f}", "f0_raw_hz": f"{tr['f0_raw'][i]:.1f}",
        "teeth": str(int(tr["teeth"][i])),
        "floor_fast": str(int(tr["fast"][i])),
        "reanch": str(int(tr["reanch"][i])),
        "n_held_bins": str(int(tr["n_held"][i])),
        "above_thr": str(int(fr["above_thr"][i])),
        "cont_accepted": str(int(fr["accepted"][i])),
        "chain": str(int(fr["chain"][i])),
        "fired": str(int(fr["active"][i])),
    } for i in range(n)]

    # The expectation is the PYTHON run itself: this test asks whether C and
    # Python agree, not whether either matches something decided in advance.
    exp = {"preset": preset, "verdict": "FIRES" if ev else "NO FIRE",
           "n_events": len(ev), "longest_chain": int(fr["chain"].max()),
           "n_floor_fast": int(tr["fast"].sum()), "role": "mic parity"}

    R = compare_records(f"mic:{Path(a.inp).name}", dev, ref, thr, exp, end,
                        bad, hdr)
    report(R)

    print(f"\n  samples {len(q)}  ({len(q) / cfg.fs:.1f} s)  preset {preset} "
          f"thr {thr:.4f}")
    print(f"  device alerts {len(alt)}   python events {len(ev)}")
    if len(alt) != len(ev):
        print(f"  FAIL  alert count disagrees: device {len(alt)} vs "
              f"python {len(ev)}")
        R["pass"] = False
    print("\n" + "=" * 70)
    print("STAGE 1b MIC PARITY: " + ("PASS" if R["pass"] else "FAIL"))
    if R["pass"] and len(ev) == 0:
        print("  (no alerts, as expected for a quiet room / talking / claps -"
              " see the RUNBOOK)")
    return 0 if R["pass"] else 1


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("--seconds", type=float, default=60.0)
    c.add_argument("--out", required=True)
    c.add_argument("--port", default=None)
    an = sub.add_parser("analyse")
    an.add_argument("--in", dest="inp", required=True)
    a = ap.parse_args()
    return phase_capture(a) if a.cmd == "capture" else phase_analyse(a)


if __name__ == "__main__":
    sys.exit(main())
