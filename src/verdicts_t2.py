"""
verdicts_t2.py - the summary tables: what Tier 2 actually bought, and at what
price.

Five tables, in order of trust:

  1. The real captures. Four files of real audio, replayed through v1 alone
     and through v1+T2 at the calibrated constants. This is the only evidence
     in the file that involves a microphone.
  2. The corpus delta, paired, with McNemar and Wilson - what T2 adds on the
     sealed positives, and on the new long-loiter class the old corpus could
     not express.
  3. The range-proxy ladder. Range factors, never metres.
  4. Friendly drones. An operator-disclosure item, not a gate, and it is
     expected to get worse.
  5. Wind. v1's published numbers must reproduce untouched, and T2's gust
     behaviour is reported rather than assumed.

Two rules this file follows without exception
  - No absolute range claim. Factors only, with the modelling error stated
    next to the number.
  - Nothing is compared at a fixed threshold. tau2 has already been
    recalibrated against the combined false-alarm budget before any P(d) here
    is quoted, which is the whole reason the number means anything.
"""

import json
import math
import sys
import time
from pathlib import Path

import numpy as np

import detector as D
import detector_t2 as T2
import evaluate as E
import evaluate_t2 as ET
import operating_point as op
import synth

ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = ROOT / "data"
FIG_DIR = ROOT / "figures"

# The ladder. Each step is 6 dB, which is one doubling of range under
# spherical spreading, so the five cells ARE range factors 1, 2, 4, 8, 16
# relative to the loudest.
LADDER_SNRS_INBAND = (13.0, 7.0, 1.0, -5.0, -11.0)
LADDER_N = 24
LADDER_DUR = 45.0
LADDER_WINDOW_S = 30.0        # "detected within 30 s of the source starting"


def chosen_config(path=None):
    doc = json.loads((path or (DATA_DIR / "t2_config.json")).read_text())
    return T2.T2Config.from_dict(doc["t2"]), doc


# ===========================================================================
# 1. the real captures
# ===========================================================================

def real_capture_table(t2cfg, cfg=None, caps=None):
    cfg = cfg or op.preset_config("HIGH_ALERT")[0]
    det = D.CombDetector(cfg)
    blob = caps or ET.run_captures(cfg)
    rises = blob["tau2_rises"]
    if t2cfg.tau2_rise_s not in rises:
        raise SystemExit(f"the cached captures hold tau2_rise {rises}, not "
                         f"{t2cfg.tau2_rise_s}; re-run --pass")
    ri = rises.index(t2cfg.tau2_rise_s)
    rows = []
    for name, c in blob["caps"].items():
        rec = {"trace": c["trace"], "score2": c["score2"], "f02": c["f02"],
               "dur": c["dur"], "onset": 0.0}
        ev1 = D.track(c["trace"], ET.THR1, cfg)
        ev2 = ET.t2_events(rec, ri, t2cfg, det=det, cfg=cfg)
        merged = ET.merge_events(ev1, ev2)
        kap = c["kappa"][ri]
        rows.append({
            "capture": name, "seconds": c["dur"],
            "v1_events": len(ev1),
            "v1_first_s": (float(ev1[0]["t_on"]) if ev1 else None),
            "t2_events": len(ev2),
            "t2_first_s": (float(ev2[0]["t_on"]) if ev2 else None),
            "combined_alerts": len(merged),
            "v1_score_med": float(np.median(c["trace"]["score"])),
            "v1_score_p99": float(np.percentile(c["trace"]["score"], 99)),
            "t2_score_med": float(np.median(c["score2"][ri])),
            "t2_score_p99": float(np.percentile(c["score2"][ri], 99)),
            "kappa_med": float(np.nanmedian(kap)),
            "kappa_p10": float(np.nanpercentile(kap, 10)),
        })
    return rows


def print_real_table(rows, t2cfg):
    print("\n" + "=" * 78)
    print("1. THE REAL CAPTURES  (four channels, real audio)")
    print("=" * 78)
    print(f"   v1 at the sealed 1.700; T2 at tau2={t2cfg.tau2:.2f}, "
          f"tau2_rise={t2cfg.tau2_rise_s:.0f}s, N2={t2cfg.n2}, M2={t2cfg.m2}")
    print(f"\n   {'capture':<14}{'s':>5}  {'v1 ev':>6}{'first':>7}  "
          f"{'T2 ev':>6}{'first':>7}  {'alerts':>7}  "
          f"{'v1 med':>7}{'v1 p99':>7}  {'T2 med':>7}{'T2 p99':>7}  "
          f"{'kappa':>6}")
    for r in rows:
        f1 = "-" if r["v1_first_s"] is None else f"{r['v1_first_s']:.1f}"
        f2 = "-" if r["t2_first_s"] is None else f"{r['t2_first_s']:.1f}"
        print(f"   {r['capture']:<14}{r['seconds']:>5.0f}  "
              f"{r['v1_events']:>6}{f1:>7}  {r['t2_events']:>6}{f2:>7}  "
              f"{r['combined_alerts']:>7}  "
              f"{r['v1_score_med']:>7.3f}{r['v1_score_p99']:>7.3f}  "
              f"{r['t2_score_med']:>7.3f}{r['t2_score_p99']:>7.3f}  "
              f"{r['kappa_med']:>6.3f}")


# ===========================================================================
# 2. the corpus delta
# ===========================================================================

def paired_delta(records, ri, t2cfg, cfg, det, subset):
    """v1-only vs v1-OR-T2 on the same clips. Returns (k_v1, k_both, b, c, n).

    Under an OR, c is always 0 by construction - T2 cannot lose a positive.
    That is stated rather than hidden: the interesting question for an OR is
    not "is the gain real" but "was it bought inside the budget", which is the
    calibration constraint, not this test.
    """
    kv = kb = b = c = 0
    for r in subset:
        a = bool(D.track(r["trace"], ET.THR1, cfg))
        z = bool(ET.combined_events(r, ri, ET.THR1, cfg, t2cfg, det))
        kv += a
        kb += z
        b += (z and not a)
        c += (a and not z)
    return kv, kb, b, c, len(subset)


def corpus_delta_table(t2cfg, cfg=None, blob=None):
    cfg = cfg or op.preset_config("HIGH_ALERT")[0]
    det = D.CombDetector(cfg)
    blob = blob or ET.run_pass(cfg)
    ri = blob["tau2_rises"].index(t2cfg.tau2_rise_s)
    recs = blob["records"]
    pos, neg, fri = ET.split_records(recs)

    groups = []
    for cls, _ in E.POS_MIX:
        groups.append((cls, [r for r in pos if r.get("cls") == cls]))
    groups.append(("p7_loiter 45s",
                   [r for r in pos if r.get("cls") == "p7_loiter"
                    and r["dur"] == 45.0]))
    groups.append(("p7_loiter 20s",
                   [r for r in pos if r.get("cls") == "p7_loiter"
                    and r["dur"] == 20.0]))
    groups.append(("ALL sealed positives",
                   [r for r in pos if r.get("cls") != "p7_loiter"]))

    rows = []
    for name, sub in groups:
        if not sub:
            continue
        kv, kb, b, c, n = paired_delta(recs, ri, t2cfg, cfg, det, sub)
        rows.append({"group": name, "n": n, "k_v1": kv, "k_both": kb,
                     "b": b, "c": c, "p": ET.mcnemar(b, c),
                     "ci_v1": E.wilson(kv, n), "ci_both": E.wilson(kb, n)})
    frik = sum(bool(ET.combined_events(r, ri, ET.THR1, cfg, t2cfg, det))
               for r in fri)
    friv = sum(bool(D.track(r["trace"], ET.THR1, cfg)) for r in fri)
    return rows, (friv, frik, len(fri))


def print_corpus_table(rows, friendly):
    print("\n" + "=" * 78)
    print("2. THE CORPUS DELTA  (paired, same clips, v1 pinned at 1.700)")
    print("=" * 78)
    print(f"\n   {'class':<24}{'n':>5}  {'v1 P(d)':>18}  {'v1+T2 P(d)':>18}  "
          f"{'+/-':>7}  {'McNemar p':>10}")
    for r in rows:
        p1, lo1, hi1 = r["ci_v1"]
        p2, lo2, hi2 = r["ci_both"]
        print(f"   {r['group']:<24}{r['n']:>5}  "
              f"{p1:.2f} [{lo1:.2f}-{hi1:.2f}]  "
              f"{p2:.2f} [{lo2:.2f}-{hi2:.2f}]  "
              f"{r['b']:>3}/{r['c']:<3}  {r['p']:>10.4g}")
    fv, fb, fn = friendly
    print(f"\n   friendly_multirotor fires: v1 {fv}/{fn} -> v1+T2 {fb}/{fn}"
          f"   ({100.0 * fb / max(fn, 1):.0f}%)")
    print("   OPERATOR-DISCLOSURE ITEM, not a gate. Tier-2 integrates a steady")
    print("   source, and a friendly multirotor loitering nearby IS one. The")
    print("   device has never had, and does not now have, any way to tell a")
    print("   friendly rotor from a hostile one - it is the same physics.")


# ===========================================================================
# 3. the range-proxy ladder
# ===========================================================================

def range_ladder_synthetic(t2cfg, cfg=None, n=LADDER_N, force=False):
    """P(detect within 30 s) vs in-band SNR, on the loiter class.

    Each 6 dB step is one doubling of range under spherical spreading, so the
    cells are range factors. The output is a factor. It is never metres, and
    it never will be from synthetic audio: the constant that would turn it
    into metres is the source level and the site's propagation, neither of
    which this corpus knows.
    """
    import hashlib
    import pickle
    cfg = cfg or op.preset_config("HIGH_ALERT")[0]
    det = D.CombDetector(cfg)
    key = hashlib.sha1(json.dumps(
        {"snrs": list(LADDER_SNRS_INBAND), "n": n, "dur": LADDER_DUR,
         "tau2_rise": t2cfg.tau2_rise_s, "v": 2},
        sort_keys=True).encode()).hexdigest()[:16]
    path = ET.CACHE / f"t2_ladder_{key}.pkl"
    if path.exists() and not force:
        with open(path, "rb") as f:
            cached = pickle.load(f)
    else:
        tier = T2.Tier2(det, T2.T2Config(tau2_rise_s=t2cfg.tau2_rise_s))
        rng = np.random.default_rng(20260819)
        cached = []
        t0 = time.time()
        for snr in LADDER_SNRS_INBAND:
            for i in range(n):
                seed = int(rng.integers(1, 2 ** 31))
                onset = float(rng.uniform(3.0, 6.0))
                x, meta = ET.compose_loiter(snr, ("wind", "pink")[i % 2], seed,
                                            LADDER_DUR, onset)
                tr, s2, f2 = ET.analyze_one(x, det, [tier])
                cached.append({"snr": snr, "onset": onset, "trace": tr,
                               "score2": s2, "f02": f2, "dur": LADDER_DUR,
                               "f0_true": meta["f0_true"]})
            print(f"    ladder {snr:+.0f} dB done ({time.time() - t0:.0f} s)",
                  flush=True)
        ET.CACHE.mkdir(exist_ok=True)
        with open(path, "wb") as f:
            pickle.dump(cached, f, protocol=4)

    rows = []
    ref = max(LADDER_SNRS_INBAND)
    for snr in LADDER_SNRS_INBAND:
        sub = [r for r in cached if r["snr"] == snr]
        kv = kb = 0
        tv, tb = [], []
        for r in sub:
            lim = r["onset"] + LADDER_WINDOW_S
            e1 = [e for e in D.track(r["trace"], ET.THR1, cfg)
                  if e["t_on"] <= lim]
            e2 = [e for e in ET.t2_events(r, 0, t2cfg, det=det, cfg=cfg)
                  if e["t_on"] <= lim]
            if e1:
                kv += 1
                tv.append(e1[0]["t_on"] - r["onset"])
            m = ET.merge_events(e1, e2)
            if m:
                kb += 1
                tb.append(m[0][0] - r["onset"])
        rows.append({
            "snr_inband": snr, "n": len(sub),
            "range_factor": 10.0 ** ((ref - snr) / 20.0),
            "k_v1": kv, "k_both": kb,
            "ci_v1": E.wilson(kv, len(sub)), "ci_both": E.wilson(kb, len(sub)),
            "t_v1": float(np.median(tv)) if tv else None,
            "t_both": float(np.median(tb)) if tb else None})
    return rows


def range_ladder_real(t2cfg, alphas=(1.0, 0.5, 0.25, 0.125, 0.0625),
                      cfg=None):
    """x = alpha * rotor_raw + quiet_raw, per channel.

    Modelling error: the scaled capture carries its own ambient down
    with it, so at alpha = 1 the bed is doubled. At alpha <= 0.5 the scaled
    capture contributes under 1 dB to the total bed and the error is
    negligible; at alpha = 1 it is 3 dB and that row should be read as
    indicative only.

    Under spherical spreading alpha = r0/r, so the reported factor is 1/alpha
    relative to the capture's own distance. Never metres.
    """
    cfg = cfg or op.preset_config("HIGH_ALERT")[0]
    det = D.CombDetector(cfg)
    q = ET.CAPS / "quiet_raw.npz"
    if not q.exists():
        return []
    quiet = np.load(q)["audio"].astype(np.float64)
    rows = []
    for ref in ("rotor_4m_0", "rotor_10m_1"):
        p = ET.CAPS / f"{ref}_raw.npz"
        if not p.exists():
            continue
        rot = np.load(p)["audio"].astype(np.float64)
        n = min(rot.shape[1], quiet.shape[1])
        for a in alphas:
            x = (a * rot[:, :n] + quiet[:, :n]) / 32767.0
            r = T2.analyze_quad_t2(x.astype(np.float32), cfg,
                                   T2.T2Config(**{**t2cfg.to_dict(),
                                                  "excluded_f0": ()}),
                                   thr_ref=ET.THR1)
            ev1, _ = D.track_frames(r, ET.THR1, cfg)
            merged = ET.merge_events(ev1, r["t2_events"])
            rows.append({
                "ref": ref, "alpha": a, "range_factor": 1.0 / a,
                "v1_events": len(ev1), "t2_events": len(r["t2_events"]),
                "alerts": len(merged),
                "first_s": (float(merged[0][0]) if merged else None),
                "v1_med": float(np.median(r["score"])),
                "t2_med": float(np.median(r["t2"]["score2"]))})
    return rows


def print_ladders(syn, real):
    print("\n" + "=" * 78)
    print("3. THE RANGE-PROXY LADDER   *** RANGE FACTORS ONLY, NEVER METRES ***")
    print("=" * 78)
    print("\n   SYNTHETIC (p7_loiter, 45 s, wind + pink beds, in-band SNR axis)")
    print(f"   P(detected within {LADDER_WINDOW_S:.0f} s of the source "
          f"starting), 95% Wilson")
    print(f"\n   {'in-band SNR':>12}{'range x':>9}{'n':>5}  {'v1 only':>18}  "
          f"{'v1 + T2':>18}  {'t v1':>7}{'t both':>8}")
    for r in syn:
        p1, lo1, hi1 = r["ci_v1"]
        p2, lo2, hi2 = r["ci_both"]
        t1 = "-" if r["t_v1"] is None else f"{r['t_v1']:.1f}"
        t2 = "-" if r["t_both"] is None else f"{r['t_both']:.1f}"
        print(f"   {r['snr_inband']:>+11.0f} dB{r['range_factor']:>8.1f}x"
              f"{r['n']:>5}  {p1:.2f} [{lo1:.2f}-{hi1:.2f}]  "
              f"{p2:.2f} [{lo2:.2f}-{hi2:.2f}]  {t1:>7}{t2:>8}")
    if real:
        print("\n   REAL CAPTURE  x = alpha*rotor + quiet, per channel")
        print("   (the scaled capture brings its own ambient: <1 dB error at "
              "alpha<=0.5, 3 dB at alpha=1)")
        print(f"\n   {'reference':<14}{'alpha':>7}{'range x':>9}  "
              f"{'v1 ev':>6}{'T2 ev':>6}{'alerts':>7}{'first s':>9}  "
              f"{'v1 med':>7}{'T2 med':>7}")
        for r in real:
            f = "-" if r["first_s"] is None else f"{r['first_s']:.1f}"
            print(f"   {r['ref']:<14}{r['alpha']:>7.4f}{r['range_factor']:>8.1f}x"
                  f"  {r['v1_events']:>6}{r['t2_events']:>6}{r['alerts']:>7}"
                  f"{f:>9}  {r['v1_med']:>7.3f}{r['t2_med']:>7.3f}")


def plot_ladder(syn, real, path=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    path = path or (FIG_DIR / "t2_range_proxy.png")
    FIG_DIR.mkdir(exist_ok=True)
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))

    x = [r["range_factor"] for r in syn]
    for key, lab, style in (("ci_v1", "v1 only", "o--"),
                            ("ci_both", "v1 + Tier-2", "s-")):
        p = [r[key][0] for r in syn]
        lo = [r[key][0] - r[key][1] for r in syn]
        hi = [r[key][2] - r[key][0] for r in syn]
        ax[0].errorbar(x, p, yerr=[lo, hi], fmt=style, capsize=3, label=lab)
    ax[0].set_xscale("log", base=2)
    ax[0].set_xticks(x)
    ax[0].set_xticklabels([f"{v:.0f}x" if v >= 1 else f"{v:.2f}x" for v in x])
    ax[0].set_xlabel("range factor r/r0  (6 dB per doubling)")
    ax[0].set_ylabel(f"P(detected within {LADDER_WINDOW_S:.0f} s)")
    ax[0].set_ylim(-0.02, 1.02)
    ax[0].grid(alpha=0.3)
    ax[0].legend(loc="lower left", fontsize=8)
    ax[0].set_title("synthetic loiter, 95% Wilson", fontsize=10)

    if real:
        for ref, mark in (("rotor_4m_0", "o-"), ("rotor_10m_1", "s-")):
            sub = [r for r in real if r["ref"] == ref]
            if sub:
                ax[1].plot([r["range_factor"] for r in sub],
                           [r["alerts"] for r in sub], mark, label=ref)
        ax[1].set_xscale("log", base=2)
        ax[1].set_xlabel("range factor 1/alpha, relative to the capture")
        ax[1].set_ylabel("alerts in 120 s (either tier)")
        ax[1].grid(alpha=0.3)
        ax[1].legend(fontsize=8)
    ax[1].set_title("real capture, alpha-scaled", fontsize=10)
    fig.suptitle("Tier-2 range proxy - FACTORS ONLY, never metres "
                 "(steady source, indoor bed)", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"\n   wrote {path}")
    return path


# ===========================================================================
# 4/5. false alarms and wind
# ===========================================================================

def fa_and_wind_table(t2cfg, cfg=None, blob=None):
    cfg = cfg or op.preset_config("HIGH_ALERT")[0]
    det = D.CombDetector(cfg)
    blob = blob or ET.run_pass(cfg)
    ri = blob["tau2_rises"].index(t2cfg.tau2_rise_s)
    recs = blob["records"]
    _, neg, _ = ET.split_records(recs)
    W = ET.t2_weights()

    out = {}
    for tag, on in (("v1 only", False), ("v1 + T2", True)):
        w, u, by, h = ET.fa_combined(neg, ri, ET.THR1, cfg, t2cfg, W, det,
                                     non_gust_only=True, t2_on=on)
        wg, ug, byg, hg = ET.fa_combined(neg, ri, ET.THR1, cfg, t2cfg, W, det,
                                         non_gust_only=False, t2_on=on)
        out[tag] = {"w": w, "u": u, "by": by, "h": h,
                    "w_all": wg, "u_all": ug, "h_all": hg}
    # T2 alone on the gusty bed - the one place a broadband masker is expected
    # to shut it up rather than set it off.
    gust = [r for r in neg if r["noise"] == E.GUST_BED]
    g_ev = sum(len(ET.t2_events(r, ri, t2cfg, det=det, cfg=cfg)) for r in gust)
    g_h = sum(r["dur"] for r in gust) / 3600.0
    out["gust"] = {"t2_events": g_ev, "hours": g_h,
                   "rate": g_ev / max(g_h, 1e-9)}
    return out


def print_fa_table(fa):
    print("\n" + "=" * 78)
    print("4/5. FALSE ALARMS AND WIND")
    print("=" * 78)
    print(f"\n   {'system':<12}{'wFA/h ng':>10}{'uFA/h ng':>10}"
          f"{'wFA/h all':>11}{'hours ng':>10}")
    for tag in ("v1 only", "v1 + T2"):
        r = fa[tag]
        print(f"   {tag:<12}{r['w']:>10.3f}{r['u']:>10.3f}"
              f"{r['w_all']:>11.3f}{r['h']:>10.1f}")
    print(f"\n   budget: {ET.COMBINED_BUDGET:.1f} weighted FA/h on non-gust "
          f"beds. Gust behaviour is reported")
    print("   separately and NOT charged to the threshold - a gust-driven "
          "false alarm and a")
    print("   gust-driven miss are the same defect, and taxing the threshold "
          "for it buys quiet")
    print("   by going deaf.")
    g = fa["gust"]
    print(f"\n   Tier-2 on the GUSTY bed: {g['t2_events']} events in "
          f"{g['hours']:.1f} h = {g['rate']:.2f}/h")
    print("   (prediction being tested: a broadband rise saturates S2 at "
          "sat_log in teeth AND")
    print("    gaps alike, so teeth-minus-gaps collapses and Tier-2 goes "
          "QUIET rather than off.)")
    print("\n   per-family contribution, v1 + T2, non-gust:")
    by = fa["v1 + T2"]["by"]
    base = fa["v1 only"]["by"]
    for k, v in sorted(by.items(), key=lambda x: -x[1]["contrib"]):
        b0 = base.get(k, {}).get("events", 0)
        print(f"     {k:<22}{v['events']:>4} ev "
              f"(v1 alone {b0:>3})  {v['hours']:>6.2f} h  "
              f"w{v['weight']:.2f}  contrib {v['contrib']:.3f}")


# ===========================================================================
# driver
# ===========================================================================

def main(argv=None):
    t2cfg, doc = chosen_config()
    if not doc.get("calibrated", True):
        print("\n  *** WARNING: data/t2_config.json is PROVISIONAL "
              "(calibrated=false).")
        print("      Run `python src/evaluate_t2.py --calibrate` first; the "
              "tables below")
        print("      are being produced at T2Config's defaults, which no "
              "measurement chose.\n")

    cfg = op.preset_config("HIGH_ALERT")[0]
    blob = ET.run_pass(cfg)
    caps = ET.run_captures(cfg)

    rows = real_capture_table(t2cfg, cfg, caps)
    print_real_table(rows, t2cfg)

    crows, friendly = corpus_delta_table(t2cfg, cfg, blob)
    print_corpus_table(crows, friendly)

    syn = range_ladder_synthetic(t2cfg, cfg)
    real = range_ladder_real(t2cfg, cfg=cfg)
    print_ladders(syn, real)
    try:
        plot_ladder(syn, real)
    except Exception as e:                              # noqa: BLE001
        print(f"   (figure skipped: {type(e).__name__}: {e})")

    fa = fa_and_wind_table(t2cfg, cfg, blob)
    print_fa_table(fa)

    out = {"t2_config": t2cfg.to_dict(), "real_captures": rows,
           "corpus_delta": crows,
           "friendly": {"v1": friendly[0], "both": friendly[1],
                        "n": friendly[2]},
           "range_ladder_synthetic": syn, "range_ladder_real": real,
           "false_alarms": {k: {kk: vv for kk, vv in v.items() if kk != "by"}
                            for k, v in fa.items()}}
    p = DATA_DIR / "t2_verdicts.json"
    p.write_text(json.dumps(out, indent=2, default=str))
    print(f"\n   wrote {p}")
    return out


if __name__ == "__main__":
    sys.exit(0 if main() else 0)
