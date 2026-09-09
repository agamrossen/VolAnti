#!/usr/bin/env python
"""
quad_parity.py - THE four-channel acceptance gate.

    python scripts/quad_parity.py capture  --seconds 60 --out captures/.../qp
    python scripts/quad_parity.py analyse  --in  captures/.../qp

Same methodology as the Stage-1b mic parity that proved the single-microphone
path: the device streams BOTH the interleaved 4-channel samples it actually fed
the pipeline AND its own per-frame trace; the host re-runs the Python reference
on exactly those samples at exactly that threshold, and diffs decision for
decision.

PASS = zero mismatches on above_thr, cont_accepted, fired and chain.

Until this passes, no four-microphone number means anything - the summed path
would be producing numbers nobody has checked against the sealed reference.

It also measures the USB link: PCM4 records carry mandatory sequence numbers,
so loss is counted exactly rather than silently spliced. A gap breaks the
sample-exact premise, so the analysis refuses to splice across one.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import quad_analysis as qa                                      # noqa: E402
from capture_trace import open_serial, resolve_port             # noqa: E402
from trace_proto import MAGIC_END, parse                        # noqa: E402

RED, GREEN, BOLD, OFF = "\033[31m", "\033[32m", "\033[1m", "\033[0m"


def phase_capture(a):
    port = resolve_port(getattr(a, "port", None))
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    budget = a.seconds + 40.0
    print(f"port {port}   quad parity capture, {a.seconds:.0f} s")

    buf = bytearray()
    with open_serial(port, timeout=0.5) as s:
        s.write(f"Z {int(a.seconds)}\n".encode())
        s.flush()
        t0 = last = time.time()
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

    Path(str(out) + ".bin").write_bytes(bytes(buf))
    recs, bad = parse(bytes(buf))
    hdr = next((r for r in recs if r["_kind"] == "hdr"), None)
    end = next((r for r in recs if r["_kind"] == "end"), None)
    dev = [r for r in recs if r["_kind"] == "rec"]
    ch, rep = qa.assemble_pcm4(recs)

    print(f"  {len(buf)} bytes   device frames {len(dev)}   "
          f"pcm4 records {rep['n_records']}   bad_chk {bad}")
    print(f"  LINK: dropped {rep['dropped']} / {rep.get('expected', 0)} "
          f"({rep['drop_rate'] * 100:.3f}%)   contiguous runs {rep['runs']}   "
          f"usable {ch.shape[1]} frames ({ch.shape[1] / 16000:.1f} s)")
    if rep["dropped"]:
        print(f"{RED}  !! the link dropped records. Parity is sample-exact by "
              f"premise, so the analysis uses ONLY the longest contiguous run "
              f"and never splices across a gap.{OFF}")

    np.savez_compressed(str(out) + ".npz", channels=ch, fs=16000)
    meta = {"threshold": hdr["threshold"] if hdr else None,
            "device_frames": len(dev), "sentinel": bool(end),
            "bad_chk": bad, **rep}
    Path(str(out) + "_capture.json").write_text(json.dumps(meta, indent=2))
    print(f"  wrote {out}.bin / .npz / _capture.json")
    if hdr is None or end is None:
        print(f"{RED}  FAIL no header or no sentinel - capture incomplete{OFF}")
        return 2
    return 0


def phase_analyse(a):
    import quad_reference as qref                               # noqa: E402
    sys.path.insert(0, str(HERE))
    from compare_trace import compare_records, report           # noqa: E402
    import operating_point as op                                # noqa: E402

    inp = Path(a.inp)
    raw = Path(str(inp) + ".bin").read_bytes()
    recs, bad = parse(raw)
    hdr = next((r for r in recs if r["_kind"] == "hdr"), None)
    end = next((r for r in recs if r["_kind"] == "end"), None)
    dev = [r for r in recs if r["_kind"] == "rec"]
    if hdr is None or end is None:
        raise SystemExit("capture has no header or no sentinel")

    ch, rep = qa.assemble_pcm4(recs)
    if ch.shape[1] < 16000:
        raise SystemExit("not enough contiguous audio to analyse")

    thr = hdr["threshold"]
    preset = next((k for k, v in op.PRESETS.items()
                   if abs(v["threshold"] - thr) < 1e-12), None)
    cfg, _ = op.preset_config(preset or op.DEFAULT_PRESET)

    # THE reference, four channels, same frame geometry as the firmware.
    tr = qref.analyze_quad(ch, cfg=cfg)
    ev, fr = qref.track(tr, thr, cfg)

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

    exp = {"preset": preset or "runtime", "n_events": len(ev),
           "verdict": "FIRES" if ev else "NO FIRE",
           "longest_chain": int(fr["chain"].max()) if n else 0,
           "n_floor_fast": int(tr["fast"].sum()), "role": "quad parity"}

    # The device emits frames only once its window is primed, exactly like the
    # reference; if the device produced fewer (a truncated capture), compare
    # the overlap and say so rather than pretending.
    m = min(len(dev), len(ref))
    if len(dev) != len(ref):
        print(f"  note: device {len(dev)} frames vs reference {len(ref)}; "
              f"comparing the first {m} (a truncated capture, not a mismatch)")
    R = compare_records(f"quad:{inp.name}", dev[:m], ref[:m], thr, exp, end,
                        bad, hdr)
    report(R)

    print(f"\n  channels {ch.shape[0]}  samples {ch.shape[1]} "
          f"({ch.shape[1] / cfg.fs:.1f} s)  threshold {thr:.4f}")
    print(f"  link drop rate {rep['drop_rate'] * 100:.3f}%  "
          f"({rep['dropped']} records)")
    print(f"  python events {len(ev)}")
    ok = bool(R["pass"]) and rep["dropped"] == 0
    if rep["dropped"]:
        print(f"{RED}  link loss makes this run non-authoritative: rerun a "
              f"shorter segment{OFF}")
    print("\n" + "=" * 70)
    print(f"{BOLD}QUAD PARITY: {'PASS' if ok else 'FAIL'}{OFF}")
    Path(str(inp) + "_parity.json").write_text(json.dumps(
        {"pass": ok, "threshold": thr, "frames": m,
         "python_events": len(ev), "link": rep}, indent=2))
    return 0 if ok else 1


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
