#!/usr/bin/env python
"""
fft_path_proof.py - the summed-window FFT path, proved on real device audio.

    python scripts/fft_path_proof.py                    # every archived quad capture
    python scripts/fft_path_proof.py --npz a.npz b.npz  # named captures

WHAT IS BEING CLAIMED, EXACTLY
------------------------------
The firmware's guard used to transform each of the four channels and sum the
spectra. It now sums the four WINDOWS and transforms once, which is the same
thing by linearity of the DFT:

    sum_c FFT(w. x_c)  ==  FFT(w. sum_c x_c)

In exact arithmetic that is an identity, not an approximation. In float32 it is
not: four roundings of a sum of transforms is a different rounding from one
transform of a sum. So the claim shipped is NOT bit identity - it is DECISION
identity, and a claim of that shape has to be measured on real audio rather
than asserted from the algebra.

This is that measurement. It replays archived FOUR-CHANNEL DEVICE captures -
the same int16 samples the pipeline consumed - through `src/detector.py`'s own
front_end / combiner / back_end both ways, and diffs every field the quad
parity gate diffs: above_thr, cont_accepted, chain, fired. It also reports how
far the two scores actually drifted, because "no decisions moved" on a clip
whose scores are nowhere near the threshold would be a weak result, and the
margin is what says whether it is weak.

WHY THIS IS A HOST TEST AND NOT A DEVICE TEST
---------------------------------------------
It answers the numerical question - does re-associating the sum move a
decision - and that question is about float32 arithmetic, not about the ESP32.
The device still has to prove it for itself: `Z` with the summed path against
`Z` with the per-channel path on the same archived capture. Until that has run,
this file is evidence and not a gate.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "src"))

import detector as D                                            # noqa: E402
import operating_point as op                                    # noqa: E402
from quad_reference import q_to_float                           # noqa: E402

RED, GREEN, BOLD, OFF = "\033[31m", "\033[32m", "\033[1m", "\033[0m"


def _trace(x, cfg, thr, summed):
    """One trace, both paths through the SAME reference functions.

    `summed=False` is the shipped per-channel path: transform each channel,
    then combiner(). `summed=True` is the new one: add the four windows, then
    ONE transform. combiner() is not called on that path because there is
    nothing left to combine - which is the point, and also why this cannot
    drift into being a second implementation of it.
    """
    det = D.CombDetector(cfg)
    c = det.cfg
    n_ch, n = x.shape
    st = D.DetectorState(det.n_bins, c, thr)
    n_frames = max(0, 1 + (n - c.n_fft) // c.hop)
    out = {k: np.empty(n_frames) for k in ("t", "f0", "f0_raw", "score")}
    out["teeth"] = np.zeros(n_frames, np.int16)
    for i in range(n_frames):
        s0 = i * c.hop
        t = (s0 + c.n_fft) / c.fs
        blk = x[:, s0:s0 + c.n_fft]
        if summed:
            spec = det.front_end(blk.sum(axis=0, dtype=np.float32))
        else:
            spec = det.combiner([det.front_end(blk[ch]) for ch in range(n_ch)])
        rec = det.back_end(spec, st, t, thr)
        out["t"][i] = t
        for k in ("f0", "f0_raw", "score", "teeth"):
            out[k][i] = rec[k]
    return out


def compare(npz, preset="HIGH_ALERT", seconds=None):
    d = np.load(npz)
    x = d["audio"]
    if seconds:
        x = x[:, :int(seconds) * int(d["fs"])]
    x = np.stack([q_to_float(ch) for ch in x])
    cfg, thr = op.preset_config(preset)

    a = _trace(x, cfg, thr, summed=False)
    b = _trace(x, cfg, thr, summed=True)

    eva, fra = D.track_frames(a, thr, cfg)
    evb, frb = D.track_frames(b, thr, cfg)

    mism = {k: int(np.sum(fra[k] != frb[k]))
            for k in ("above_thr", "accepted", "chain", "active")}
    mism["f0"] = int(np.sum(a["f0"] != b["f0"]))
    dscore = np.abs(a["score"] - b["score"])
    scored = np.abs(a["score"]) >= 0.1
    # The tightest decision margin on this clip: how close any frame came to
    # the threshold. A zero-mismatch result on a clip that never approached
    # the threshold proves much less than one on a clip that grazed it.
    margin = np.min(np.abs(a["score"] - thr)) if len(a["score"]) else np.nan
    return {
        "clip": Path(npz).parent.name + "/" + Path(npz).stem,
        "frames": len(a["t"]), "thr": thr,
        "events": (len(eva), len(evb)),
        "mismatches": mism,
        "score_dev_med": float(np.median(dscore[scored])) if scored.any() else 0.0,
        "score_dev_max": float(dscore.max()) if len(dscore) else 0.0,
        "closest_margin": float(margin),
        "ok": sum(mism.values()) == 0 and len(eva) == len(evb),
    }


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--npz", nargs="*", default=None)
    p.add_argument("--seconds", type=float, default=None,
                   help="truncate each capture, for a quick pass")
    p.add_argument("--preset", default="HIGH_ALERT")
    a = p.parse_args(argv)

    files = ([Path(f) for f in a.npz] if a.npz else
             sorted((ROOT / "firmware/sentry_node/captures").rglob("*_raw.npz")))
    files = [f for f in files if np.load(f)["audio"].ndim == 2
             and np.load(f)["audio"].shape[0] == 4]
    if not files:
        print("no four-channel captures found")
        return 1

    rows = []
    for f in files:
        rows.append(compare(f, a.preset, a.seconds))
        r = rows[-1]
        print(f"  {'ok  ' if r['ok'] else 'FAIL'} {r['clip']:<44} "
              f"{r['frames']:5d} fr  ev {r['events'][0]}/{r['events'][1]}  "
              f"dscore med {r['score_dev_med']:.2e} max {r['score_dev_max']:.2e}  "
              f"nearest margin {r['closest_margin']:.4f}", flush=True)
        if not r["ok"]:
            print(f"       mismatches: {r['mismatches']}")

    ok = all(r["ok"] for r in rows)
    tot = sum(r["frames"] for r in rows)
    worst = max(r["score_dev_max"] for r in rows)
    tight = min(r["closest_margin"] for r in rows)
    print()
    print(f"{BOLD}FFT PATH PROOF: {(GREEN + 'PASS') if ok else (RED + 'FAIL')}"
          f"{OFF}{BOLD}  {len(rows)} captures, {tot} frames{OFF}")
    print(f"  worst score deviation {worst:.3e}; tightest approach to the "
          f"threshold on any frame {tight:.4f}")
    if ok and worst >= tight:
        print(f"  {RED}NOTE the deviation is not smaller than the tightest "
              f"margin, so zero mismatches here is luck, not headroom.{OFF}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
