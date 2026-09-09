"""
quad_reference.py - run THE Python reference over four channels.

This is the offline authority for every quad decision: the threshold sweep, the
parity gate, all of it. It calls src/detector.py's OWN front_end / combiner /
back_end - it does not reimplement any of the maths. That is the difference
between an acceptance test and a plausible-looking second opinion.

The frame geometry mirrors the firmware exactly:

    frame i consumes x[i*hop: i*hop + n_fft] on EVERY channel
    spec_c = front_end(block_c)          per channel
    spec   = combiner([spec_0.. spec_3])   unweighted complex sum
    rec    = back_end(spec, state, t)    t = frame END time, causal

and the int16 -> float conversion is the same one the golden vectors were cut
with: q / 32767.0 computed in float32.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent.parent.parent
sys.path.insert(0, str(ROOT / "src"))

import detector as D                                            # noqa: E402
import operating_point as op                                    # noqa: E402


def q_to_float(q):
    """int16 -> float32, exactly as make_golden.py quantised it."""
    return np.asarray(q, np.int16).astype(np.float32) / 32767.0


def analyze_quad(channels, cfg=None, thr_ref=None):
    """channels: (n_ch, n_samples) int16 or float32. Returns the same per-frame
    dict shape as CombDetector.analyze(), so every downstream tool that already
    understands a mono trace understands this one."""
    cfg = cfg or op.preset_config(op.DEFAULT_PRESET)[0]
    det = D.CombDetector(cfg)
    c = det.cfg

    x = np.asarray(channels)
    if x.dtype != np.float32:
        x = np.stack([q_to_float(ch) for ch in x])
    n_ch, n = x.shape

    st = D.DetectorState(det.n_bins, c, thr_ref)
    n_frames = max(0, 1 + (n - c.n_fft) // c.hop)
    out = {k: np.empty(n_frames) for k in ("t", "f0", "f0_raw", "score")}
    out["teeth"] = np.zeros(n_frames, np.int16)
    out["reanch"] = np.zeros(n_frames, bool)
    out["flat"] = np.zeros(n_frames, np.float32)
    out["fast"] = np.zeros(n_frames, bool)
    out["n_held"] = np.zeros(n_frames, np.int16)

    for i in range(n_frames):
        s0 = i * c.hop
        t = (s0 + c.n_fft) / c.fs                  # frame END time: causal
        spectra = [det.front_end(x[ch, s0:s0 + c.n_fft]) for ch in range(n_ch)]
        spec = det.combiner(spectra)
        rec = det.back_end(spec, st, t, thr_ref)
        out["t"][i] = t
        for k in ("f0", "f0_raw", "score", "teeth", "reanch", "flat",
                  "fast", "n_held"):
            out[k][i] = rec[k]
    out["config"] = c.to_dict()
    return out


def track(trace, thr, cfg=None):
    """Chain logic, from the reference. Never simulated separately."""
    cfg = cfg or op.preset_config(op.DEFAULT_PRESET)[0]
    return D.track_frames(trace, thr, cfg)


def sweep_thresholds(trace, grid, cfg=None, duration_s=None):
    """Run the FULL reference tracker at every threshold in `grid`.

    Alerts-per-hour falls straight out of the chain logic - which is exactly
    why the sweep must use the reference rather than counting frames over a
    threshold. A frame count is not an alert.
    """
    cfg = cfg or op.preset_config(op.DEFAULT_PRESET)[0]
    rows = []
    for thr in grid:
        ev, fr = D.track_frames(trace, float(thr), cfg)
        n_ev = len(ev)
        row = {
            "threshold": float(thr),
            "events": n_ev,
            "longest_chain": int(fr["chain"].max()) if len(fr["chain"]) else 0,
            "frames_above": int(fr["above_thr"].sum()),
        }
        if duration_s and duration_s > 0:
            row["events_per_hour"] = n_ev * 3600.0 / duration_s
        rows.append(row)
    return rows
