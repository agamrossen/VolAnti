"""
make_golden.py - cut the golden vectors from the committed detector config.

Three vectors, at the NORMAL preset:

  golden_strong_pos   the exact threat platform (7" 3-blade, loaded hover rpm)
                      detected with the widest margin. Sanity only.
  golden_marginal_neg the negative that comes closest to firing.
  golden_marginal_pos the positive that barely fires.

The marginal pair is the point. A strong positive proves almost nothing about
a port: the ESP32 runs float32 where this runs float64, and that difference can
only change a decision that was already balanced on a knife edge. Both
marginals are selected by computing the threshold at which each clip's verdict
flips and taking the clips whose flip point is nearest the operating point.

Each trace CSV carries the whole per-frame state, including the floor and
jitter state, so the device trace is comparable field for field:

  frame, t_s, score, f0_bin, f0_hz, f0_raw_hz, teeth, floor_fast, reanch,
  above_thr, cont_accepted, chain, fired

  floor_fast  the tonality-gated floor adapted fast on this frame. If the
              firmware's gate disagrees with this column the floors have
              diverged and every later field is void.
  f0_raw_hz   argmax before re-anchoring. The jitter gate is computed on this,
              not on the accepted chain, so the firmware needs it.
  reanch      re-anchoring moved this frame. Re-anchoring was rejected, so
              this column must be all zeros; it is emitted anyway as a
              tripwire against accidentally shipping it enabled.

`--high-alert` cuts one more vector, at the HIGH_ALERT preset.

Run: conda run -n acoustic-detector python src/make_golden.py
"""

import json
import sys
from pathlib import Path

import numpy as np

import evaluate
import operating_point as op
import synth
from detector import CombDetector, track_frames

_HERE = Path(__file__).resolve().parent
DATA_DIR = _HERE.parent / "data"

TAIL_S = 2.0
GRID = np.round(np.arange(0.20, 4.01, 0.005), 4)


def fires(trace, thr, cfg):
    return bool(track_frames(trace, thr, cfg, collect=False)[0])


def flip_threshold(trace, cfg):
    """Highest threshold at which this clip still fires. Firing is monotone in
    the threshold, so bisection is exact on this grid."""
    if not fires(trace, float(GRID[0]), cfg):
        return None
    lo, hi = 0, len(GRID) - 1
    if fires(trace, float(GRID[hi]), cfg):
        return float(GRID[hi])
    while lo < hi - 1:
        mid = (lo + hi) // 2
        if fires(trace, float(GRID[mid]), cfg):
            lo = mid
        else:
            hi = mid
    return float(GRID[lo])


def write_vector(name, x, cfg, det, thr, comment, meta):
    q = np.clip(np.round(x * 32767), -32768, 32767).astype(np.int16)
    xq = q.astype(np.float32) / 32767.0
    tr = det.analyze(xq)
    ev, fr = track_frames(tr, thr, cfg)
    synth.to_c_header(xq, name, DATA_DIR / f"{name}.h", comment=comment)

    f0_bin = np.round((tr["f0"] - cfg.f_search_lo) / cfg.f_step).astype(int)
    lines = [
        f"# {name}_trace - per-frame reference output for Stage 1a",
        f"# {comment}",
        f"# config: {json.dumps(cfg.to_dict())}",
        f"# threshold: {thr:.6f}   events: {len(ev)}   "
        f"VERDICT: {'FIRES' if ev else 'NO FIRE'}",
        f"# peak score {tr['score'].max():.6f}   longest chain "
        f"{int(fr['chain'].max())}/{cfg.track_need}",
        "# chain is the counter AFTER the frame; fired is 1 while an event is",
        "# latched. floor_fast/f0_raw_hz/reanch are v2 state - match the whole",
        "# TRAJECTORY, not just the verdict.",
        "frame,t_s,score,f0_bin,f0_hz,f0_raw_hz,teeth,floor_fast,reanch,"
        "n_held_bins,above_thr,cont_accepted,chain,fired"]
    for i in range(len(tr["t"])):
        lines.append(
            f"{i},{tr['t'][i]:.6f},{tr['score'][i]:.6f},{f0_bin[i]},"
            f"{tr['f0'][i]:.1f},{tr['f0_raw'][i]:.1f},{int(tr['teeth'][i])},"
            f"{int(tr['fast'][i])},{int(tr['reanch'][i])},"
            f"{int(tr['n_held'][i])},"
            f"{int(fr['above_thr'][i])},{int(fr['accepted'][i])},"
            f"{int(fr['chain'][i])},{int(fr['active'][i])}")
    (DATA_DIR / f"{name}_trace.csv").write_text("\n".join(lines))

    info = {"name": name, "n_samples": len(q), "dur_s": len(q) / cfg.fs,
            "peak_score": float(tr["score"].max()),
            "longest_chain": int(fr["chain"].max()), "need": cfg.track_need,
            "verdict": "FIRES" if ev else "NO FIRE", "n_events": len(ev),
            "flip_threshold": flip_threshold(tr, cfg), "threshold": thr,
            "n_floor_fast": int(tr["fast"].sum()),
            "n_reanch": int(tr["reanch"].sum()),
            "n_held_frames": int((tr["n_held"] > 0).sum()), **meta}
    print(f"  {name:<20} {info['dur_s']:5.1f} s  peak {info['peak_score']:.3f}"
          f"  chain {info['longest_chain']}/{cfg.track_need}"
          f"  {info['verdict']:<8} flip@{info['flip_threshold']}"
          f"  fast_frames={info['n_floor_fast']}")
    return info


# ---------------------------------------------------------------------------
# HIGH_ALERT vector. The three parity vectors are cut at NORMAL (2.14); nothing
# else in the golden set exercises HIGH_ALERT (1.70), which is what the device
# ships with. This cuts exactly one more vector and does not touch the three.
#
# The detector Config is identical for both presets (both band offsets are
# 0.0), so the per-frame score/f0/teeth columns are preset-independent. Only
# above_thr / cont_accepted / chain / fired differ. That is why one extra
# vector at 1.70 is enough: it tests the tracker at the deployment threshold,
# not a different detector.
# ---------------------------------------------------------------------------

HIGH_ALERT_PREFERRED = ("p7_static", "approach")   # hover, slow approach


def _truncate(det, cfg, x, trace, probe_thr):
    """Same rule main() uses: cut just past the decision frame, then re-measure
    on the truncated waveform, because the noise floor is a function of
    everything heard."""
    _, fr = track_frames(trace, probe_thr, cfg)
    i = int(np.argmax(fr["chain"]))
    n = min(len(x), int(round((float(trace["t"][i]) + TAIL_S) * cfg.fs)))
    n = max(n, cfg.n_fft + 30 * cfg.hop)
    xt = x[:n]
    return xt, flip_threshold(det.analyze(xt), cfg)


def cut_high_alert():
    """Cut one vector that fires at HIGH_ALERT and does not fire at NORMAL."""
    cfg, thr_norm = op.preset_config("NORMAL")
    thr_ha = op.HIGH_ALERT["threshold"]
    det = CombDetector(cfg)
    print(f"cutting the HIGH_ALERT vector: must fire at {thr_ha:.4f} and NOT "
          f"at {thr_norm:.4f}")
    print(f"  preferring {' / '.join(HIGH_ALERT_PREFERRED)} - hover and slow "
          f"approach are what HIGH_ALERT exists to catch")

    # positives only: the negatives are the bulk of the corpus and cannot
    # supply a vector that must fire.
    clips = [c for c in evaluate.build_corpus() if c["kind"] == "positive"]
    print(f"  scanning {len(clips)} positives")

    best = None
    for c in clips:
        x, meta = evaluate.compose(c["kind"], c["snr_db"], c["noise"],
                                   c["seed"], c["dur"], c["onset"],
                                   cls=c["cls"], **c["kw"])
        flip = flip_threshold(det.analyze(x), cfg)
        # firing is monotone in the threshold, so "fires at thr" is flip >= thr
        if flip is None or not (thr_ha <= flip < thr_norm):
            continue
        xt, ft = _truncate(det, cfg, x, det.analyze(x), thr_ha)
        if ft is None or not (thr_ha <= ft < thr_norm):
            continue          # truncation moved it out of the window
        # prefer hover/approach, then nearest to thr_ha from above
        rank = (0 if c["cls"] in HIGH_ALERT_PREFERRED else 1, ft - thr_ha)
        if best is None or rank < best[0]:
            best = (rank, c, xt, ft, meta)

    if best is None:
        raise SystemExit("no positive fires at HIGH_ALERT but not at NORMAL - "
                         "cannot cut the vector")
    _, c, xt, ft, meta = best
    print(f"  selected {c['cls']} seed={c['seed']} snr={c['snr_db']:+.0f}dB "
          f"bed={c['noise']}  flips at {ft:.3f}  "
          f"(fires at {thr_ha:.2f}, silent at {thr_norm:.2f})")

    info = write_vector(
        "golden_high_alert_pos", xt, cfg, det, thr_ha,
        f"HIGH_ALERT POSITIVE (deployment default): {c['cls']} "
        f"seed={c['seed']} snr={c['snr_db']:+.0f}dB bed={c['noise']} "
        f"f0={meta['f0_true']:.1f}Hz; flips at {ft:.3f} - FIRES at "
        f"{thr_ha:.4f}, does NOT fire at {thr_norm:.4f}",
        {"role": "high-alert positive", "preset": "HIGH_ALERT",
         "cls": c["cls"], "seed": c["seed"], "snr_db": c["snr_db"],
         "bed": c["noise"], "f0_true": meta["f0_true"],
         "flip_vs_thr": ft - thr_ha})

    # merge, do not rewrite: the three parity vectors and their traces are
    # untouched on disk and their entries are preserved verbatim.
    gvp = DATA_DIR / "golden_vectors.json"
    gv = json.loads(gvp.read_text())
    for v in gv["vectors"]:
        v.setdefault("preset", "NORMAL")
    gv["vectors"] = [v for v in gv["vectors"]
                     if v["name"] != "golden_high_alert_pos"] + [info]
    gvp.write_text(json.dumps(gv, indent=2))
    print(f"\nmerged into {gvp} (the three parity vectors were not rewritten)")
    return info


def main():
    cfg, _ = op.preset_config("NORMAL")
    thr = op.NORMAL["threshold"]
    det = CombDetector(cfg)
    DATA_DIR.mkdir(exist_ok=True)
    print(f"cutting golden vectors at NORMAL preset: thr={thr:.4f}, "
          f"floor={cfg.floor_mode}, max_jitter={cfg.max_jitter}")

    clips = evaluate.build_corpus()
    scored = []
    for c in clips:
        x, meta = evaluate.compose(c["kind"], c["snr_db"], c["noise"],
                                   c["seed"], c["dur"], c["onset"],
                                   cls=c["cls"], **c["kw"])
        tr = det.analyze(x)
        scored.append({"c": c, "x": x, "meta": meta, "trace": tr,
                       "flip": flip_threshold(tr, cfg)})
    print(f"  scanned {len(scored)} clips")

    pos = [s for s in scored if s["meta"]["drone_present"]]
    neg = [s for s in scored if not s["meta"]["drone_present"]
           and s["c"]["kind"] not in evaluate.FRIENDLY]

    def truncated(s):
        """Truncate just past the decision frame, then re-measure on the
        truncated waveform - the noise floor is a function of everything heard,
        so the exported vector must be the thing that was measured."""
        probe = min(s["flip"] if s["flip"] is not None else thr, thr)
        _, fr = track_frames(s["trace"], probe, cfg)
        i = int(np.argmax(fr["chain"]))
        n = min(len(s["x"]),
                int(round((float(s["trace"]["t"][i]) + TAIL_S) * cfg.fs)))
        n = max(n, cfg.n_fft + 30 * cfg.hop)
        xt = s["x"][:n]
        return xt, flip_threshold(det.analyze(xt), cfg)

    def pick(cands, want_fire):
        best = None
        for s in cands:
            if s["flip"] is None:
                continue
            xt, ft = truncated(s)
            if ft is None:
                continue
            fires_at = ft >= thr
            if fires_at != want_fire:
                continue
            d = abs(ft - thr)
            if best is None or d < best[0]:
                best = (d, s, xt, ft)
        return best

    neg_c = sorted([s for s in neg if s["flip"] is not None],
                   key=lambda s: -s["flip"])[:15]
    b_neg = pick(neg_c, want_fire=False) or pick(neg_c, want_fire=True)
    pos_c = sorted([s for s in pos if s["flip"] is not None],
                   key=lambda s: abs(s["flip"] - thr))[:15]
    b_pos = pick(pos_c, want_fire=True) or pick(pos_c, want_fire=False)

    infos = []
    print("\nvectors:")
    # strong positive: the exact platform, widest margin
    strong = max([s for s in pos if s["c"]["cls"] == "p7_flyby"
                  and s["flip"] is not None], key=lambda s: s["flip"])
    c = strong["c"]
    infos.append(write_vector(
        "golden_strong_pos", strong["x"], cfg, det, thr,
        f"STRONG POSITIVE (exact platform): p7_flyby seed={c['seed']} "
        f"snr={c['snr_db']:+.0f}dB bed={c['noise']} "
        f"f0={strong['meta']['f0_true']:.1f}Hz; flips at {strong['flip']:.3f}",
        {"role": "strong positive", "cls": "p7_flyby", "seed": c["seed"],
         "snr_db": c["snr_db"], "bed": c["noise"],
         "f0_true": strong["meta"]["f0_true"]}))

    _, s_n, x_n, ft_n = b_neg
    c = s_n["c"]
    infos.append(write_vector(
        "golden_marginal_neg", x_n, cfg, det, thr,
        f"MARGINAL NEGATIVE: {c['kind']} seed={c['seed']} "
        f"level={c['snr_db']:+.0f}dB bed={c['noise']}; flips at {ft_n:.3f} "
        f"vs operating {thr:.4f}",
        {"role": "marginal negative", "kind": c["kind"], "seed": c["seed"],
         "bed": c["noise"], "flip_vs_thr": ft_n - thr}))

    _, s_p, x_p, ft_p = b_pos
    c = s_p["c"]
    infos.append(write_vector(
        "golden_marginal_pos", x_p, cfg, det, thr,
        f"MARGINAL POSITIVE: {c['cls']} seed={c['seed']} "
        f"snr={c['snr_db']:+.0f}dB bed={c['noise']} "
        f"f0={s_p['meta']['f0_true']:.1f}Hz; flips at {ft_p:.3f} vs "
        f"operating {thr:.4f}",
        {"role": "marginal positive", "cls": c["cls"], "seed": c["seed"],
         "snr_db": c["snr_db"], "bed": c["noise"],
         "f0_true": s_p["meta"]["f0_true"], "flip_vs_thr": ft_p - thr}))

    (DATA_DIR / "golden_vectors.json").write_text(json.dumps(
        {"preset": "NORMAL", "threshold": thr, "config": cfg.to_dict(),
         "vectors": infos}, indent=2))
    print(f"\nwrote {DATA_DIR / 'golden_vectors.json'}")
    return infos


if __name__ == "__main__":
    if "--high-alert" in sys.argv:
        cut_high_alert()
    else:
        main()
