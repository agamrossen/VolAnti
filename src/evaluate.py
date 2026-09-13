"""
evaluate.py - the scored harness for the reference detector.

What it measures
----------------
1. Positives are the actual threat. The primary class is the known airframe:
   7" 3-blade props (7040) on 2807-class ~1300KV motors, loaded hover
   7.5-9.5k rpm -> blade pass 375-475 Hz. Plus throttle transients (punch to
   ~12k, ~600 Hz; descent), 2-blade variants (240-360 Hz), the 5" secondary
   threat (>= ~330 Hz), and slow approaches. n >= 50 per SNR cell, because
   n=24 gives +-0.1 cell noise and that noise was being read as signal.
2. Every P(d) used for a decision is reported with a 95% Wilson interval.
3. Per-bed reporting throughout (pink / steady wind / gusting wind). Pooled
   headline numbers hid a 0.97-vs-0.33 split between steady and gusting wind.
4. Two-tier alert band and two named presets (NORMAL / HIGH_ALERT).
5. The FA budget for preset selection is measured on non-gust beds, because a
   gust-driven false alarm and a gust-driven miss are the same defect and
   should not both be charged to the threshold.

Everything the detector needs per frame is cached in one analysis pass per
floor mode, so parameter sweeps (thresholds, band offsets, discriminators)
never trigger another FFT.

Run:
  python evaluate.py --stage floors    compare the noise-floor candidates
  python evaluate.py --stage tune      re-anchoring, discriminator and band sweeps
  python evaluate.py --full            final measurement at the committed config
"""

import argparse
import hashlib
import json
import math
import pickle
import time
from pathlib import Path

import numpy as np

import site_profile
import synth
from detector import CombDetector, Config, track

_HERE = Path(__file__).resolve().parent
DATA_DIR = _HERE.parent / "data"
FIG_DIR = _HERE.parent / "figures"
CACHE = _HERE.parent / ".trace_cache"

SNRS = [-12, -9, -6, -3, 0, 3, 6, 9, 12]
NEG_SEEDS = 20
NEG_LEVELS = [0.0, 6.0, 12.0]
NOISE_ONLY = 40
NEG_DUR = 30.0

FADE_S = 0.5
ONSET_LO, ONSET_HI = 0.5, 3.0
REF_ONSET = 1.0

FRIENDLY = set(site_profile.FRIENDLY)
BEDS = ["wind", "wind_gusty", "pink", "wind"]
GUST_BED = "wind_gusty"

# ---------------------------------------------------------------------------
# positive class mix - 54 clips per SNR cell.
# The counts encode a threat model, so they are stated as counts, not weights.
# ---------------------------------------------------------------------------
POS_MIX = [
    ("p7_flyby",   15),   # 7" 3-blade, loaded hover rpm, transiting
    ("p7_static",  15),   # 7" 3-blade, loaded hover rpm, hovering
    ("punch",       4),   # throttle punch-out to ~12k rpm (~600 Hz)
    ("descent",     3),   # revs bleeding off
    ("blade2",      7),   # 2-blade variants, 240-360 Hz
    ("five_inch",   5),   # secondary 5" threat, >= ~330 Hz
    ("approach",    5),   # slow closing approach - the Task-1 floor guard
]
POS_PER_SNR = sum(n for _, n in POS_MIX)      # 54


# ---------------------------------------------------------------------------
# statistics
# ---------------------------------------------------------------------------

def wilson(k, n, z=1.96):
    """95% Wilson score interval. Correct at the 0/n and n/n ends, which is
    where a normal approximation is most badly wrong and where P(d) lives."""
    if n == 0:
        return (float("nan"), float("nan"), float("nan"))
    p = k / n
    d = 1.0 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return p, max(0.0, c - h), min(1.0, c + h)


def ci_str(k, n):
    p, lo, hi = wilson(k, n)
    return f"{p:.2f} [{lo:.2f}-{hi:.2f}]"


# ---------------------------------------------------------------------------
# clip composition
# ---------------------------------------------------------------------------

def confuser_onset(seed):
    return float(np.random.default_rng([int(seed), 9173]).uniform(ONSET_LO,
                                                                 ONSET_HI))


def raised_cosine(nseg, fs):
    L = min(int(round(FADE_S * fs)), nseg // 2)
    w = np.ones(nseg)
    ramp = 0.5 * (1.0 - np.cos(np.pi * np.arange(L) / L))
    w[:L] = ramp
    w[nseg - L:] = ramp[::-1]
    return w


def _make_positive(cls, seed, dur, onset):
    """Returns (src, f0_true). Source duration is dur - onset."""
    rng = np.random.default_rng(seed + 4242)
    d = dur - onset
    if cls in ("p7_flyby", "p7_static"):
        rpm = float(rng.uniform(7500, 9500))          # loaded hover, 7" 3-blade
        if cls == "p7_flyby":
            s, f0, _ = synth.drone_flyby(
                dur=d, rpm=rpm, blades=3, seed=seed,
                speed_ms=float(rng.uniform(15, 35)),
                closest_m=float(rng.uniform(25, 60)))
        else:
            s, f0 = synth.drone_static(dur=d, rpm=rpm, blades=3, seed=seed)
        return s, f0
    if cls in ("punch", "descent"):
        s, f0 = synth.drone_throttle(
            dur=d, blades=3, rpm_hover=float(rng.uniform(7500, 9500)),
            rpm_peak=float(rng.uniform(11000, 12800)),
            mode="punch" if cls == "punch" else "descent", seed=seed)
        return s, f0
    if cls == "blade2":
        # 2-blade variants land at 240-360 Hz
        rpm = float(rng.uniform(7200, 10800))
        if seed % 2:
            s, f0, _ = synth.drone_flyby(
                dur=d, rpm=rpm, blades=2, seed=seed,
                speed_ms=float(rng.uniform(15, 35)),
                closest_m=float(rng.uniform(25, 60)))
        else:
            s, f0 = synth.drone_static(dur=d, rpm=rpm, blades=2, seed=seed)
        return s, f0
    if cls == "five_inch":
        rpm = float(rng.uniform(6600, 14000))         # 5" 3-blade, >= ~330 Hz
        s, f0, _ = synth.drone_flyby(
            dur=d, rpm=rpm, blades=3, seed=seed,
            speed_ms=float(rng.uniform(18, 40)),
            closest_m=float(rng.uniform(25, 60)))
        return s, f0
    if cls == "approach":
        s, f0 = synth.drone_approach(
            dur=d, blades=3, rpm=float(rng.uniform(7500, 9500)),
            r_start=float(rng.uniform(220, 380)),
            r_end=float(rng.uniform(35, 60)), seed=seed)
        return s, f0
    raise ValueError(cls)


def compose(kind, snr_db, noise, seed, dur, onset, cls=None, fs=synth.FS, **kw):
    rng = np.random.default_rng(seed + 5077)
    n = int(round(dur * fs))
    bed = synth.NOISE_KINDS[noise](n, rng)
    meta = {"kind": kind, "snr_db": snr_db, "noise": noise, "seed": seed,
            "dur": dur, "onset": onset, "cls": cls,
            "drone_present": False, "f0_true": None}

    if kind == "noise_only":
        out = bed
    elif kind == "positive":
        src, f0 = _make_positive(cls, seed, dur, onset)
        meta.update(drone_present=True, f0_true=f0)
        i0 = int(round(onset * fs))
        full = np.zeros(n)
        full[i0:i0 + len(src)] = src
        p_sig = np.mean(src ** 2)
        p_bed = np.mean(bed[i0:i0 + len(src)] ** 2)
        g = np.sqrt(p_bed * 10.0 ** (snr_db / 10.0) / (p_sig + 1e-30))
        out = bed + g * full
    else:
        # generator() and not CONFUSERS[kind]: identical for every sealed
        # family (it checks CONFUSERS first) and it makes the veto
        # families reachable without touching the sealed seed stream.
        src_full = synth.generator(kind)(dur=dur - REF_ONSET, seed=seed)
        i0 = int(round(onset * fs))
        src = src_full[:n - i0]
        src_e = src * raised_cosine(len(src), fs)
        p_sig = np.mean(src_e ** 2)
        p_bed = np.mean(bed[i0:i0 + len(src)] ** 2)
        g = np.sqrt(p_bed * 10.0 ** (snr_db / 10.0) / (p_sig + 1e-30))
        full = np.zeros(n)
        full[i0:i0 + len(src)] = g * src_e
        out = bed + full
        if kind == "friendly_multirotor":
            meta["f0_true"] = synth.friendly_multirotor_f0(seed)[0]

    peak = np.max(np.abs(out))
    return out / peak * 0.95 if peak > 0 else out, meta


def build_corpus():
    rng = np.random.default_rng(20260807)
    clips = []
    for snr in SNRS:
        i = 0
        for cls, count in POS_MIX:
            for _ in range(count):
                seed = int(rng.integers(1, 2 ** 31))
                dur = 10.0 if cls == "approach" else 8.0
                clips.append(dict(kind="positive", cls=cls, snr_db=snr,
                                  noise=BEDS[i % len(BEDS)], seed=seed,
                                  dur=dur,
                                  onset=float(rng.uniform(1.0, 2.5)), kw={}))
                i += 1
    for conf in synth.CONFUSERS:
        for lvl in NEG_LEVELS:
            for _ in range(NEG_SEEDS):
                seed = int(rng.integers(1, 2 ** 31))
                clips.append(dict(kind=conf, cls=None, snr_db=lvl,
                                  noise=BEDS[seed % len(BEDS)], seed=seed,
                                  dur=NEG_DUR, onset=confuser_onset(seed),
                                  kw={}))
    for _ in range(NOISE_ONLY):
        seed = int(rng.integers(1, 2 ** 31))
        clips.append(dict(kind="noise_only", cls=None, snr_db=0.0,
                          noise=BEDS[seed % len(BEDS)], seed=seed,
                          dur=NEG_DUR, onset=0.0, kw={}))
    return clips


# ---------------------------------------------------------------------------
# one analysis pass, cached on disk
# ---------------------------------------------------------------------------

# The veto's three fields are deliberately absent. This key names what the
# analysis pass computes, and the veto acts in the tracker, on f0 and teeth
# that are already in the cached trace. Adding them would invalidate every
# cached pass (17 min each) to express a dependency that does not exist.
ANALYSIS_KEYS = ("floor_mode", "tau_rise_s", "tau_fall_s", "tau_rise_fast_s",
                 "tau_energy_s", "gate_ratio", "flat_hi", "minstat_frames",
                 "minstat_sub", "minstat_bias", "sat_log", "reanchor",
                 "reanchor_margin", "reanchor_min_tooth", "reanchor_depth",
                 "teeth_level", "f_search_lo", "f_search_hi", "n_harm_max",
                 "hold_mode", "hold_chain_min", "hold_frac", "hold_persist",
                 "hold_release_s", "hold_w_bins", "hold_max_bins",
                 "f_alert_lo", "f_alert_hi", "track_need", "track_miss",
                 "cont_frac", "cont_min_hz", "min_teeth", "max_jitter")


def analysis_key(cfg, neg_stride, thr_ref):
    d = {k: getattr(cfg, k) for k in ANALYSIS_KEYS}
    d["neg_stride"] = neg_stride
    d["thr_ref"] = None if cfg.hold_mode == "off" else thr_ref
    d["corpus"] = "v2"
    return hashlib.sha1(json.dumps(d, sort_keys=True).encode()).hexdigest()[:16]


def analyze_corpus(cfg, neg_stride=1, tag="", thr_ref=None):
    """One FFT pass over the corpus for THIS floor/re-anchor configuration.
    Everything the tracker and the discriminators need is cached; sweeps over
    thresholds, band offsets and discriminator settings then cost nothing."""
    CACHE.mkdir(exist_ok=True)
    key = analysis_key(cfg, neg_stride, thr_ref)
    path = CACHE / f"traces_{key}.pkl"
    if path.exists():
        with open(path, "rb") as f:
            return pickle.load(f)

    clips = build_corpus()
    pos = [c for c in clips if c["kind"] == "positive"]
    neg = [c for c in clips if c["kind"] != "positive"][::neg_stride]
    sel = pos + neg
    det = CombDetector(cfg)
    print(f"  [{tag or cfg.floor_mode}] analysing {len(sel)} clips "
          f"({len(pos)} pos, {len(neg)} neg)", flush=True)

    out, t0 = [], time.time()
    for j, c in enumerate(sel):
        x, meta = compose(c["kind"], c["snr_db"], c["noise"], c["seed"],
                          c["dur"], c["onset"], cls=c["cls"], **c["kw"])
        tr = det.analyze(x, thr_ref=thr_ref)
        meta["trace"] = {"t": tr["t"].astype(np.float32),
                         "f0": tr["f0"].astype(np.float32),
                         "f0_raw": tr["f0_raw"].astype(np.float32),
                         "score": tr["score"].astype(np.float32),
                         "teeth": tr["teeth"],
                         "reanch": tr["reanch"],
                         "n_held": tr["n_held"]}
        out.append(meta)
        if (j + 1) % 200 == 0:
            print(f"    {j + 1}/{len(sel)}  ({time.time() - t0:.0f} s)",
                  flush=True)
    with open(path, "wb") as f:
        pickle.dump(out, f, protocol=4)
    print(f"  [{tag or cfg.floor_mode}] done in {time.time() - t0:.0f} s",
          flush=True)
    return out


# ---------------------------------------------------------------------------
# measurement helpers (all operate on cached traces - no FFTs)
# ---------------------------------------------------------------------------

def split(records):
    pos = [r for r in records if r["drone_present"]]
    neg = [r for r in records if not r["drone_present"]
           and r["kind"] not in FRIENDLY]
    fri = [r for r in records if r["kind"] in FRIENDLY]
    return pos, neg, fri


def fa_rates(neg, cfg, thr, weights, non_gust_only=True):
    """Weighted and unweighted false-alarm rates, plus the per-family split.
    weighted = sum_f prevalence[f] * (events per hour of family-f audio)."""
    rows = [r for r in neg if not (non_gust_only and r["noise"] == GUST_BED)]
    hours, evs = {}, {}
    for r in rows:
        hours[r["kind"]] = hours.get(r["kind"], 0.0) + r["dur"] / 3600.0
    for r in rows:
        n = len(track(r["trace"], thr, cfg))
        if n:
            evs[r["kind"]] = evs.get(r["kind"], 0) + n
    by = {k: {"events": v, "hours": hours[k],
              "rate": v / hours[k],
              "weight": weights.get(k, 0.0),
              "contrib": weights.get(k, 0.0) * v / hours[k]}
          for k, v in evs.items()}
    tot_h = sum(hours.values())
    return (sum(v["contrib"] for v in by.values()),
            sum(v["events"] for v in by.values()) / max(tot_h, 1e-9),
            by, tot_h)


def pd_by_bed(pos, cfg, thr, beds=("pink", "wind", GUST_BED), prio_only=False):
    """P(d) per bed with Wilson CIs. prio_only restricts to positives whose
    true fundamental lies in the priority band - the Task-4 objective."""
    out = {}
    for bed in beds:
        sub = [r for r in pos if r["noise"] == bed]
        if prio_only:
            sub = [r for r in sub
                   if cfg.f_prio_lo <= r["f0_true"] <= cfg.f_prio_hi]
        k = sum(bool(track(r["trace"], thr, cfg)) for r in sub)
        out[bed] = (k, len(sub)) + wilson(k, len(sub))[1:]
    return out


def pd_by_snr(pos, cfg, thr, prio_only=False, bed=None):
    out = {}
    for s in SNRS:
        sub = [r for r in pos if r["snr_db"] == s]
        if bed:
            sub = [r for r in sub if r["noise"] == bed]
        if prio_only:
            sub = [r for r in sub
                   if cfg.f_prio_lo <= r["f0_true"] <= cfg.f_prio_hi]
        k = sum(bool(track(r["trace"], thr, cfg)) for r in sub)
        out[s] = (k, len(sub))
    return out


def snr_at(pd, level=0.90):
    """Lowest SNR step whose lower 95% bound clears `level`. Using the bound,
    not the point estimate, is what n>=50 was raised for: a 0.92 point estimate
    on n=24 has a lower bound of 0.75 and is not evidence of anything."""
    for s in SNRS:
        k, n = pd[s]
        if n and wilson(k, n)[1] >= level:
            return s
    return None


def calibrate(neg, cfg, weights, budget, grid, non_gust_only=True):
    """Lowest threshold on `grid` whose weighted FA/h is inside `budget`."""
    lo, hi = 0, len(grid) - 1
    if fa_rates(neg, cfg, float(grid[hi]), weights, non_gust_only)[0] > budget:
        return None
    while lo < hi:
        mid = (lo + hi) // 2
        if fa_rates(neg, cfg, float(grid[mid]), weights,
                    non_gust_only)[0] <= budget:
            hi = mid
        else:
            lo = mid + 1
    return float(grid[lo])


# ---------------------------------------------------------------------------
# stage drivers
# ---------------------------------------------------------------------------

GRID = np.round(np.arange(0.20, 4.01, 0.02), 4)
NORMAL_BUDGET = 1.0
HIGH_ALERT_BUDGET = 4.0

FLOOR_VARIANTS = [
    ("legacy", dict(floor_mode="legacy")),
    ("gated", dict(floor_mode="gated")),
    ("tonality", dict(floor_mode="tonality")),
    ("minstat", dict(floor_mode="minstat")),
]


def _base_cfg(**kw):
    return Config(**kw)


def stage_floors(neg_stride=2):
    """Compare floor candidates at a matched false-alarm budget."""
    W = site_profile.weights()
    print("=" * 78)
    print(f"GUST-ROBUST NOISE FLOOR  (matched at weighted FA/h <= "
          f"{NORMAL_BUDGET} on non-gust beds)")
    print("=" * 78)
    res = {}
    for name, kw in FLOOR_VARIANTS:
        cfg = _base_cfg(**kw)
        recs = analyze_corpus(cfg, neg_stride, tag=name)
        pos, neg, _ = split(recs)
        thr = calibrate(neg, cfg, W, NORMAL_BUDGET, GRID)
        if thr is None:
            print(f"  {name}: cannot reach the budget at any threshold")
            continue
        beds = pd_by_bed(pos, cfg, thr)
        app = [r for r in pos if r["cls"] == "approach"]
        ka = sum(bool(track(r["trace"], thr, cfg)) for r in app)
        allk = sum(bool(track(r["trace"], thr, cfg)) for r in pos)
        wfa, ufa, _, hrs = fa_rates(neg, cfg, thr, W)
        res[name] = {"thr": thr, "beds": beds, "approach": (ka, len(app)),
                     "all": (allk, len(pos)), "wfa": wfa, "ufa": ufa,
                     "hours": hrs}
        print(f"\n  {name:<9} thr {thr:.2f}   wFA/h {wfa:.2f}  uFA/h {ufa:.2f} "
              f" ({hrs:.1f} h non-gust)")
        for bed, (k, n, lo, hi) in beds.items():
            print(f"      {bed:<11} P(d) {k}/{n} = {ci_str(k, n)}")
        print(f"      {'APPROACH':<11} P(d) {ka}/{len(app)} = "
              f"{ci_str(ka, len(app))}   <-- floor-absorption guard")
        print(f"      {'ALL':<11} P(d) {allk}/{len(pos)} = "
              f"{ci_str(allk, len(pos))}")

    # selection: max gusty P(d) subject to pink/steady loss <= 0.02 and no
    # approach-class loss (the guard).
    if "legacy" in res:
        base = res["legacy"]
        bp = base["beds"]["pink"][0] / max(base["beds"]["pink"][1], 1)
        bw = base["beds"]["wind"][0] / max(base["beds"]["wind"][1], 1)
        ba = base["approach"][0] / max(base["approach"][1], 1)
        print("\n  selection (vs legacy: pink/steady loss <= 0.02, approach "
              "must not drop):")
        best, best_g = None, -1
        for name, r in res.items():
            p = r["beds"]["pink"][0] / max(r["beds"]["pink"][1], 1)
            w = r["beds"]["wind"][0] / max(r["beds"]["wind"][1], 1)
            g = r["beds"][GUST_BED][0] / max(r["beds"][GUST_BED][1], 1)
            a = r["approach"][0] / max(r["approach"][1], 1)
            ok = (bp - p) <= 0.02 and (bw - w) <= 0.02 and a >= ba - 1e-9
            why = []
            if (bp - p) > 0.02:
                why.append(f"pink -{bp - p:.3f}")
            if (bw - w) > 0.02:
                why.append(f"steady -{bw - w:.3f}")
            if a < ba - 1e-9:
                why.append(f"approach -{ba - a:.3f}")
            print(f"    {name:<9} gusty {g:.3f}  pink {p:.3f}  steady {w:.3f} "
                  f" approach {a:.3f}   {'ELIGIBLE' if ok else 'REJECTED: ' + ', '.join(why)}")
            if ok and g > best_g:
                best, best_g = name, g
        print(f"\n  WINNER: {best}  (gusty P(d) {best_g:.3f} vs legacy "
              f"{res['legacy']['beds'][GUST_BED][0] / max(res['legacy']['beds'][GUST_BED][1], 1):.3f})")
        res["_winner"] = best
    (DATA_DIR / "task1_floors.json").write_text(json.dumps(
        {k: v for k, v in res.items()}, indent=2, default=str))
    return res


def stage_tune(floor_mode, neg_stride=2):
    """Re-anchoring, discriminators and band offsets."""
    W = site_profile.weights()
    out = {}

    print("\n" + "=" * 78)
    print("SUBHARMONIC RE-ANCHORING")
    print("=" * 78)
    for on in (False, True):
        cfg = _base_cfg(floor_mode=floor_mode, reanchor=on)
        recs = analyze_corpus(cfg, neg_stride,
                              tag=f"{floor_mode}+reanchor={on}")
        pos, neg, _ = split(recs)
        thr = calibrate(neg, cfg, W, NORMAL_BUDGET, GRID)
        wfa, ufa, by, _ = fa_rates(neg, cfg, thr, W)
        beds = pd_by_bed(pos, cfg, thr)
        allk = sum(bool(track(r["trace"], thr, cfg)) for r in pos)
        # frame-level octave-slip proxy: fraction of in-band post-onset frames
        # whose argmax is a multiple/submultiple of the truth
        slip = tot = 0
        for r in pos:
            tr = r["trace"]
            m = ((tr["t"] >= max(cfg.t_warmup_s, r["onset"]))
                 & (tr["f0"] >= cfg.f_alert_lo) & (tr["f0"] <= cfg.f_alert_hi))
            if not m.any():
                continue
            f, ft = tr["f0"][m], r["f0_true"]
            tot += int(m.sum())
            for mult in (1 / 3, 0.5, 2.0, 3.0):
                slip += int((np.abs(f - ft * mult) / (ft * mult) <= 0.15).sum())
        masq = {k: by.get(k, {}).get("events", 0)
                for k in ("livestock", "helicopter", "motorbike_accel",
                          "diesel_generator", "apc_wheeled", "tractor")}
        out[f"reanchor_{on}"] = {"thr": thr, "wfa": wfa, "ufa": ufa,
                                 "pd_all": allk / len(pos),
                                 "slip": slip / max(tot, 1),
                                 "masq": masq,
                                 "beds": beds}
        print(f"  reanchor={str(on):<5} thr {thr:.2f}  wFA/h {wfa:.2f}  "
              f"P(d) all {ci_str(allk, len(pos))}  slip {100*slip/max(tot,1):.2f}%"
              f"  masquerade events {masq}")
    return out


def _sweep_offline(recs, cfg_kw, W, budget, thr_grid, teeth_list, jit_list,
                   off_list):
    """All of Tasks 3+4 in one offline sweep over cached traces."""
    pos, neg, _ = split(recs)
    best = []
    for mt in teeth_list:
        for mj in jit_list:
            for op, og in off_list:
                cfg = _base_cfg(min_teeth=mt, max_jitter=mj,
                                thr_off_prio=op, thr_off_gen=og, **cfg_kw)
                thr = calibrate(neg, cfg, W, budget, thr_grid)
                if thr is None:
                    continue
                pr = pd_by_snr(pos, cfg, thr, prio_only=True)
                s90 = snr_at(pr, 0.90)
                s50 = snr_at(pr, 0.50)
                allk = sum(bool(track(r["trace"], thr, cfg)) for r in pos)
                gk = [r for r in pos if r["noise"] == GUST_BED]
                gkk = sum(bool(track(r["trace"], thr, cfg)) for r in gk)
                best.append({"min_teeth": mt, "max_jitter": mj,
                             "off_prio": op, "off_gen": og, "thr": thr,
                             "snr90": s90, "snr50": s50,
                             "pd_all": allk / len(pos),
                             "pd_gust": gkk / max(len(gk), 1)})
    return best



# ---------------------------------------------------------------------------
# final measurement
# ---------------------------------------------------------------------------

V1_EQUIVALENT = dict(floor_mode="legacy", reanchor=False, min_teeth=0,
                     max_jitter=1.0, f_alert_lo=220.0, f_alert_hi=1700.0,
                     thr_off_prio=0.0, thr_off_gen=0.0)


def measure(recs, cfg, thr, W, label=""):
    """Everything the report needs, at one (config, threshold)."""
    pos, neg, fri = split(recs)
    wfa_ng, ufa_ng, by_ng, h_ng = fa_rates(neg, cfg, thr, W, True)
    wfa_all, ufa_all, by_all, h_all = fa_rates(neg, cfg, thr, W, False)
    beds = pd_by_bed(pos, cfg, thr)
    beds_prio = pd_by_bed(pos, cfg, thr, prio_only=True)
    per_snr = {b: pd_by_snr(pos, cfg, thr, prio_only=True, bed=b)
               for b in ("pink", "wind", GUST_BED)}
    prio_all = pd_by_snr(pos, cfg, thr, prio_only=True)
    cls = {}
    for c in [m[0] for m in POS_MIX]:
        sub = [r for r in pos if r["cls"] == c]
        k = sum(bool(track(r["trace"], thr, cfg)) for r in sub)
        cls[c] = (k, len(sub))
    allk = sum(bool(track(r["trace"], thr, cfg)) for r in pos)
    frik = sum(bool(track(r["trace"], thr, cfg)) for r in fri)
    lat = []
    for r in pos:
        ev = track(r["trace"], thr, cfg)
        if ev and r["cls"] == "p7_static":
            lat.append(ev[0]["t_on"] - r["onset"])
    return {"label": label, "thr": thr,
            "wfa_nongust": wfa_ng, "ufa_nongust": ufa_ng, "by_nongust": by_ng,
            "hours_nongust": h_ng,
            "wfa_all": wfa_all, "ufa_all": ufa_all, "by_all": by_all,
            "hours_all": h_all,
            "beds": beds, "beds_prio": beds_prio, "per_snr_bed": per_snr,
            "prio_all": prio_all,
            "snr90": snr_at(prio_all, 0.90), "snr50": snr_at(prio_all, 0.50),
            "by_class": cls, "pd_all": (allk, len(pos)),
            "friendly": (frik, len(fri)),
            "latency_med": float(np.median(lat)) if lat else None}


def print_measure(m):
    print(f"\n--- {m['label']}   thr {m['thr']:.3f} ---")
    print(f"  weighted FA/h  non-gust {m['wfa_nongust']:.3f}   "
          f"all beds {m['wfa_all']:.3f}")
    print(f"  unweighted FA/h non-gust {m['ufa_nongust']:.3f}   "
          f"all beds {m['ufa_all']:.3f}   ({m['hours_nongust']:.1f} h / "
          f"{m['hours_all']:.1f} h)")
    print(f"  P(d) by bed (all positives / priority-band only):")
    for b in ("pink", "wind", GUST_BED):
        k, n, lo, hi = m["beds"][b]
        k2, n2, lo2, hi2 = m["beds_prio"][b]
        print(f"    {b:<11} {ci_str(k, n):<20} {ci_str(k2, n2)}")
    print(f"  priority-band P(d)>=0.90 at {m['snr90']} dB, >=0.50 at "
          f"{m['snr50']} dB  (lower 95% bound)")
    print(f"  P(d) all positives {ci_str(*m['pd_all'])}   latency "
          f"{m['latency_med']}")
    print(f"  by class: " + "  ".join(
        f"{c}={k}/{n}" for c, (k, n) in m["by_class"].items()))
    print(f"  friendly multirotor fires {m['friendly'][0]}/{m['friendly'][1]}")
    if m["by_nongust"]:
        print(f"  FA families (non-gust): " + ", ".join(
            f"{k}:{v['events']}ev w{v['contrib']:.2f}"
            for k, v in sorted(m["by_nongust"].items(),
                               key=lambda x: -x[1]["contrib"])))


def stage_full():
    import operating_point as op
    W = site_profile.weights()
    print("=" * 78)
    print("FINAL MEASUREMENT (full corpus)")
    print("=" * 78)
    out = {}

    cfg_v1 = _base_cfg(**V1_EQUIVALENT)
    recs_v1 = analyze_corpus(cfg_v1, 1, tag="v1-equivalent")
    _, neg_v1, _ = split(recs_v1)
    thr_v1 = calibrate(neg_v1, cfg_v1, W, NORMAL_BUDGET, GRID)
    m_v1 = measure(recs_v1, cfg_v1, thr_v1, W,
                   "BEFORE (v1 detector, matched to NORMAL budget)")
    print_measure(m_v1)
    out["before"] = m_v1

    cfg_c, _ = op.preset_config("NORMAL")
    recs_c = analyze_corpus(cfg_c, 1, tag="committed")
    _, neg_c, _ = split(recs_c)
    for name, budget in (("NORMAL", NORMAL_BUDGET),
                         ("HIGH_ALERT", HIGH_ALERT_BUDGET)):
        cfg, _ = op.preset_config(name)
        thr = calibrate(neg_c, cfg, W, budget, GRID)
        print(f"\n  calibrated {name}: threshold={thr:.4f} "
              f"(budget {budget} weighted FA/h, non-gust)")
        m = measure(recs_c, cfg, thr, W, f"AFTER preset {name}")
        print_measure(m)
        out[name] = m
    print("\n  >>> paste into src/operating_point.py:")
    for name in ("NORMAL", "HIGH_ALERT"):
        print(f"  {name} = dict(threshold={out[name]['thr']:.4f}, "
              f"off_prio=0.0, off_gen=0.0)")

    # sweep table at the committed detector, NORMAL offsets
    cfg, _ = op.preset_config("NORMAL")
    recs = recs_c
    pos, neg, _ = split(recs)
    lines = ["# operating_point_sweep.csv",
             "# weighted FA/h is measured on NON-GUST beds; gust behaviour is",
             "# reported separately and not charged to the threshold.",
             "# P(d) columns are PRIORITY-BAND positives, per bed.",
             "thr,fa_weighted_nongust,fa_unweighted_nongust,fa_weighted_all,"
             "pd_prio_pink,pd_prio_wind,pd_prio_gusty,pd_all,snr90_prio,"
             "families"]
    for thr in np.round(np.arange(0.6, 3.21, 0.02), 4):
        wfa, ufa, by, _ = fa_rates(neg, cfg, float(thr), W, True)
        wfa_a, _, _, _ = fa_rates(neg, cfg, float(thr), W, False)
        bp = pd_by_bed(pos, cfg, float(thr), prio_only=True)
        pa = sum(bool(track(r["trace"], float(thr), cfg)) for r in pos)
        s90 = snr_at(pd_by_snr(pos, cfg, float(thr), prio_only=True), 0.90)
        lines.append(",".join([f"{thr:.4f}", f"{wfa:.4f}", f"{ufa:.4f}",
                               f"{wfa_a:.4f}",
                               f"{bp['pink'][0]/max(bp['pink'][1],1):.4f}",
                               f"{bp['wind'][0]/max(bp['wind'][1],1):.4f}",
                               f"{bp[GUST_BED][0]/max(bp[GUST_BED][1],1):.4f}",
                               f"{pa/len(pos):.4f}",
                               "" if s90 is None else str(s90),
                               " ".join(sorted(by, key=lambda k: -by[k]["contrib"]))]))
    (DATA_DIR / "operating_point_sweep.csv").write_text("\n".join(lines))
    print(f"\nwrote {DATA_DIR / 'operating_point_sweep.csv'}")
    (DATA_DIR / "eval_results.json").write_text(json.dumps(out, indent=2,
                                                           default=str))
    print(f"wrote {DATA_DIR / 'eval_results.json'}")
    return out

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", choices=["floors", "tune"], default=None)
    ap.add_argument("--floor", default="legacy")
    ap.add_argument("--stride", type=int, default=2)
    ap.add_argument("--full", action="store_true")
    a = ap.parse_args()
    DATA_DIR.mkdir(exist_ok=True)
    if a.stage == "floors":
        stage_floors(a.stride)
    elif a.stage == "tune":
        stage_tune(a.floor, a.stride)
    elif a.full:
        stage_full()


# ---------------------------------------------------------------------------
# in-band SNR (200-2000 Hz)
# ---------------------------------------------------------------------------

def inband_snr_db(kind, snr_db, noise, seed, dur, onset, cls=None,
                  lo=200.0, hi=2000.0, fs=None):
    """
    SNR measured only over the alert band, alongside the corpus's nominal
    broadband SNR.

    Why both are reported: the nominal figure is set on total power, and
    `wind_noise` rolls off steeply above 200 Hz while pink noise does not. At
    matched total power pink puts far more energy where the drone actually
    lives, so a "+6 dB" pink clip and a "+6 dB" wind clip are not the same
    problem at all. The in-band figure is the one that predicts detection; the
    broadband figure is kept because every earlier result is indexed by it.
    """
    fs = fs or synth.FS
    n = int(round(dur * fs))
    bed = synth.NOISE_KINDS[noise](n, np.random.default_rng(seed + 5077))
    if kind == "noise_only":
        return float("nan")
    if kind == "positive":
        src, _ = _make_positive(cls, seed, dur, onset)
        env = 1.0
    else:
        src_full = synth.CONFUSERS[kind](dur=dur - REF_ONSET, seed=seed)
        i0p = int(round(onset * fs))
        src = src_full[:n - i0p]
        env = raised_cosine(len(src), fs)
    src = src * env
    i0 = int(round(onset * fs))
    seg = slice(i0, i0 + len(src))
    p_sig = np.mean(src ** 2)
    p_bed = np.mean(bed[seg] ** 2)
    g = np.sqrt(p_bed * 10.0 ** (snr_db / 10.0) / (p_sig + 1e-30))

    m = min(len(src), n - i0)
    f = np.fft.rfftfreq(m, 1.0 / fs)
    band = (f >= lo) & (f <= hi)
    S = np.abs(np.fft.rfft(g * src[:m])) ** 2
    B = np.abs(np.fft.rfft(bed[i0:i0 + m])) ** 2
    ps, pb = float(S[band].sum()), float(B[band].sum())
    if pb <= 0 or ps <= 0:
        return float("nan")
    return 10.0 * np.log10(ps / pb)


def inband_snr_table(clips, path=None):
    """Cached {(kind, seed): in-band SNR dB} over the whole corpus."""
    path = path or (CACHE / "inband_snr.json")
    CACHE.mkdir(exist_ok=True)
    if path.exists():
        return {tuple(k.split("|")): v
                for k, v in json.loads(path.read_text()).items()}
    out = {}
    for c in clips:
        v = inband_snr_db(c["kind"], c["snr_db"], c["noise"], c["seed"],
                          c["dur"], c["onset"], cls=c["cls"])
        out[f"{c['kind']}|{c['seed']}"] = v
    path.write_text(json.dumps(out))
    return {tuple(k.split("|")): v for k, v in out.items()}
