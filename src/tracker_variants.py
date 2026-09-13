"""
tracker_variants.py - v1's tracker, varied and priced.

Why the variants are ordered this way
-------------------------------------
Three changes were proposed, in this order: (a) family-ratio continuity,
(b) a continuity-tolerance sweep, (c) velocity-predictive continuity. A
reproduction on the real 4 m rig recording measured which mechanism actually
refuses it, and it is none of those three:

    the chain reaches 9, well past track_need = 6, and never fires;
    at every frame where it stood at 6 or more, the median relative step of
    the raw argmax was 0.0054 to 0.0091 against max_jitter = 0.004.

The jitter gate refuses it, on a chain the continuity rule had already
accepted. So `max_jitter` is added to the set and is scanned first.
Continuity - the thing (b) proposes to relax - was not binding on that audio,
which does not mean it is never binding, so it is still scanned.

The rule this file exists to follow
-----------------------------------
Every variant is judged at a recalibrated threshold. At a fixed threshold
every score-inflating change looks like an improvement; that is the mistake
comb-hold was rejected for. Each variant here is recalibrated to the same
weighted false-alarm budget before any P(d) is compared, and the comparison
is paired McNemar on the same 486 positives.

Why this is cheap
-----------------
The cached corpus traces hold t / f0 / f0_raw / score / teeth - all produced by
back_end, which does not read a single tracker constant. So the whole variant
plane re-runs the decision over saved numbers and costs no FFT at all. The
cache key includes the tracker fields, so `analyze_corpus` would recompute; the
traces are therefore taken once at the committed config and re-tracked here.
"""

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import detector as D                                            # noqa: E402
import evaluate as E                                            # noqa: E402
import operating_point as op                                    # noqa: E402
import site_profile                                             # noqa: E402


# ---------------------------------------------------------------------------
# the variant tracker
#
# The sealed tracker in src/detector.py is not changed for experiments, so the
# family-ratio and velocity rules cannot go in TrackerState. This is the only
# duplicated decision logic in the project and it is allowed to exist for
# exactly one reason: with every flag off it is proved identical to the sealed
# tracker over the whole corpus, frame for frame. If that check ever fails,
# this file is wrong and the sealed one is right.
# ---------------------------------------------------------------------------

class VariantTracker(D.TrackerState):
    """v1's tracker plus three switches, all default-off."""

    def __init__(self, cfg, thr, ratios=(2.0, 0.5), velocity=False,
                 normalise_jitter=False):
        super().__init__(cfg, thr)
        self.ratios = tuple(ratios)
        self.velocity = velocity
        self.normalise_jitter = normalise_jitter
        self.prev_f0 = None

    def step(self, t, f0, score, teeth=None, f0_raw=None):
        c = self.c
        thr_eff = D.band_threshold(f0, self.thr, c)
        ok = (score > thr_eff and t >= c.t_warmup_s
              and c.f_alert_lo <= f0 <= c.f_alert_hi)
        if ok and c.min_teeth > 0 and teeth is not None:
            ok = teeth >= c.min_teeth
        is_oct = False
        ratio = 1.0
        if ok and self.last_f0 is not None:
            lf = self.last_f0
            if abs(f0 - lf) <= max(c.cont_frac * lf, c.cont_min_hz):
                pass
            else:
                hit = None
                for r in self.ratios:
                    tgt = lf * r
                    if abs(f0 - tgt) <= max(c.cont_frac * tgt, c.cont_min_hz):
                        hit = r
                        break
                if hit is not None:
                    is_oct, ratio = True, hit
                elif self.velocity and self.prev_f0 is not None:
                    pred = lf + (lf - self.prev_f0)
                    if abs(f0 - pred) <= max(c.cont_frac * abs(pred),
                                             c.cont_min_hz):
                        pass                       # a chirp, tracked
                    else:
                        ok = False
                else:
                    ok = False
        if ok:
            self.count += 1
            if not is_oct:
                self.prev_f0 = self.last_f0
                self.last_f0 = f0
            self.chain_f0s.append(self.last_f0)
            # The jitter gate sees the family-normalised argmax. Feeding it the
            # raw value on a chain that legitimately alternates between a shaft
            # line and its third harmonic makes the median step ~100%, which is
            # not jitter - it is the tracker doing its job. Normalising is what
            # makes ratio continuity worth having at all.
            raw = f0 if f0_raw is None else f0_raw
            self.chain_raw.append(raw / ratio if self.normalise_jitter and ratio
                                  else raw)
            if (self.count >= c.track_need and not self.fired
                    and self._jitter_ok()):
                self.fired, self.t_on = True, t
        else:
            self.count = max(0, self.count - c.track_miss)
            if self.count == 0:
                if self.fired:
                    self.events.append(
                        {"t_on": self.t_on, "t_off": t,
                         "f0": float(np.median(self.chain_f0s))})
                self.fired, self.last_f0, self.prev_f0 = False, None, None
                self.chain_f0s, self.chain_raw = [], []
        return ok, is_oct, self.count, self.fired


def track_variant(trace, thr, cfg, **kw):
    tk = VariantTracker(cfg, thr, **kw)
    raw = trace.get("f0_raw")
    teeth = trace.get("teeth")
    n = len(trace["t"])
    for i in range(n):
        tk.step(float(trace["t"][i]), float(trace["f0"][i]),
                float(trace["score"][i]),
                None if teeth is None else teeth[i],
                None if raw is None else float(raw[i]))
    return tk.finish(trace["t"][-1] if n else 0.0)


# ---------------------------------------------------------------------------
# The fast path, and why it is allowed to exist
#
# The variant plane is ten variants x a threshold grid x 1153 non-gust negative
# clips x 934 frames. Run frame by frame in Python, that takes hours.
#
# The saving is exact rather than approximate. A frame can only extend a chain
# if `score > thr` and `t >= warmup` and f0 is inside the alert band; the last
# two do not depend on the threshold, so `eligible` is computed once per clip.
# Every other frame is a miss, and a run of g consecutive misses does exactly
# one thing to the state: it takes `count` down by track_miss each time,
# floored at zero, closing any open event at the frame where it first reaches
# zero. That is arithmetic, not iteration, so the loop visits only the frames
# that can matter.
#
# This is checked against the sealed tracker over the whole corpus. If it ever
# disagrees, the sealed one is right.
# ---------------------------------------------------------------------------

def fast_events(trace, thr, cfg, ratios=(2.0, 0.5), velocity=False,
                normalise_jitter=False):
    """Event count only - which is all the false-alarm side ever needs."""
    t = np.asarray(trace["t"], float)
    f0 = np.asarray(trace["f0"], float)
    sc = np.asarray(trace["score"], float)
    raw = np.asarray(trace.get("f0_raw", f0), float)
    n = len(t)
    if n == 0:
        return 0
    # both band offsets are 0.0 in every shipped preset, so thr_eff is flat;
    # assert it rather than assume it, because a future preset could change it
    assert cfg.thr_off_prio == 0.0 and cfg.thr_off_gen == 0.0, (
        "a two-tier threshold offset makes thr_eff frequency-dependent and "
        "this fast path invalid")
    elig = ((t >= cfg.t_warmup_s) & (f0 >= cfg.f_alert_lo)
            & (f0 <= cfg.f_alert_hi))
    if cfg.min_teeth > 0 and trace.get("teeth") is not None:
        elig &= np.asarray(trace["teeth"]) >= cfg.min_teeth
    idx = np.flatnonzero(elig & (sc > thr))
    if not len(idx):
        return 0

    count = 0
    last = None
    prev = None
    chain_raw = []
    fired = False
    n_ev = 0
    pos = 0
    for i in idx:
        gap = i - pos
        pos = i + 1
        if gap and count:
            # the misses between the previous candidate and this one
            steps = min(gap, -(-count // cfg.track_miss))
            count = max(0, count - cfg.track_miss * steps)
            if count == 0:
                if fired:
                    n_ev += 1
                fired, last, prev, chain_raw = False, None, None, []
        ratio = 1.0
        ok = True
        if last is not None:
            if abs(f0[i] - last) <= max(cfg.cont_frac * last, cfg.cont_min_hz):
                pass
            else:
                hit = None
                for r in ratios:
                    tgt = last * r
                    if abs(f0[i] - tgt) <= max(cfg.cont_frac * tgt,
                                               cfg.cont_min_hz):
                        hit = r
                        break
                if hit is not None:
                    ratio = hit
                elif velocity and prev is not None:
                    pred = last + (last - prev)
                    ok = abs(f0[i] - pred) <= max(cfg.cont_frac * abs(pred),
                                                  cfg.cont_min_hz)
                else:
                    ok = False
        if ok:
            count += 1
            if ratio == 1.0:
                prev, last = last, f0[i]
            # The sealed tracker appends the raw argmax, unnormalised, and that
            # is not an oversight to fix quietly - it is the baseline. On an
            # octave hop the raw value doubles, the median relative step goes
            # to ~100%, and the jitter gate refuses the chain. Normalising is a
            # variant, switched on with the family-ratio rule it belongs to,
            # because without it ratio continuity buys a chain the jitter gate
            # then throws away.
            chain_raw.append(raw[i] / ratio if normalise_jitter else raw[i])
            if count >= cfg.track_need and not fired and cfg.max_jitter < 1.0:
                if len(chain_raw) >= 4:
                    a = np.asarray(chain_raw)
                    d = np.abs(np.diff(a)) / np.maximum(a[:-1], 1e-9)
                    if float(np.median(d)) <= cfg.max_jitter:
                        fired = True
                else:
                    fired = True
            elif count >= cfg.track_need and not fired:
                fired = True
        else:
            count = max(0, count - cfg.track_miss)
            if count == 0:
                if fired:
                    n_ev += 1
                fired, last, prev, chain_raw = False, None, None, []
    # a chain still open at the end of the clip is still an event
    if fired:
        n_ev += 1
    return n_ev


# ---------------------------------------------------------------------------
# the variants
# ---------------------------------------------------------------------------

VARIANTS = [
    ("baseline (committed)",              {},                          {}),
    ("jitter 0.008",                      {"max_jitter": 0.008},       {}),
    ("jitter 0.012",                      {"max_jitter": 0.012},       {}),
    ("jitter off",                        {"max_jitter": 1.0},         {}),
    ("cont 3%",                           {"cont_frac": 0.03},         {}),
    ("cont 4%",                           {"cont_frac": 0.04},         {}),
    ("family {2,3}",                      {},   {"ratios": (2.0, 0.5, 3.0, 1 / 3), "normalise_jitter": True}),
    ("velocity-predictive",               {},   {"velocity": True}),
    ("family {2,3} + jitter 0.008",       {"max_jitter": 0.008},
     {"ratios": (2.0, 0.5, 3.0, 1 / 3), "normalise_jitter": True}),
    ("family {2,3} + cont 3% + jit 0.008",
     {"max_jitter": 0.008, "cont_frac": 0.03},
     {"ratios": (2.0, 0.5, 3.0, 1 / 3), "normalise_jitter": True}),
]

GRID = np.round(np.arange(1.20, 3.01, 0.01), 4)


def mcnemar(a, b):
    """Exact two-sided binomial on the discordant pairs."""
    from math import comb
    n01 = int(np.sum(~a & b))          # variant gains
    n10 = int(np.sum(a & ~b))          # variant loses
    n = n01 + n10
    if n == 0:
        return n01, n10, 1.0
    k = min(n01, n10)
    p = sum(comb(n, i) for i in range(k + 1)) / (2.0 ** n) * 2.0
    return n01, n10, min(1.0, p)


def run(budget=None, grid=GRID):
    cfg0, thr0 = op.preset_config("HIGH_ALERT")
    recs = E.analyze_corpus(cfg0, 1, "tracker-variants", thr0)
    pos = [r for r in recs if r["kind"] == "positive"]
    neg = [r for r in recs if r["kind"] != "positive"
           and r["noise"] != E.GUST_BED]
    W = site_profile.weights()
    budget = budget or op.HIGH_ALERT_BUDGET_WEIGHTED_PER_HOUR
    hours = {}
    for r in neg:
        hours[r["kind"]] = hours.get(r["kind"], 0.0) + r["dur"] / 3600.0

    base_hit = None
    out = []
    for name, cfg_kw, tk_kw in VARIANTS:
        cfg = replace(cfg0, **cfg_kw) if cfg_kw else cfg0

        # Recalibrate, by a feasible-set scan over the whole grid rather than a
        # bisection. The aggregate weighted rate is not monotone in the
        # threshold - measured: 10.04 at 1.44 against 10.21 at 1.46 - so a
        # bisection would land wherever its first probe happened to fall.
        chosen = None
        for t in grid:
            by = {}
            for r in neg:
                n = fast_events(r["trace"], float(t), cfg, **tk_kw)
                if n:
                    by[r["kind"]] = by.get(r["kind"], 0) + n
            wfa = sum(W.get(k, 0.0) * v / hours[k] for k, v in by.items())
            if wfa <= budget:
                chosen = (float(t), wfa)
                break
        if chosen is None:
            out.append({"name": name, "thr": None})
            continue
        thr, wfa_at = chosen

        hit = np.array([fast_events(r["trace"], thr, cfg, **tk_kw) > 0
                        for r in pos])
        if base_hit is None:
            base_hit = hit
        n01, n10, p = mcnemar(base_hit, hit)
        cls = {}
        for c in ("p7_static", "approach", "p7_flyby", "punch", "blade2"):
            sub = [i for i, r in enumerate(pos) if r["cls"] == c]
            cls[c] = float(hit[sub].mean()) if sub else float("nan")
        out.append({"name": name, "thr": thr, "wfa": wfa_at,
                    "pd_all": float(hit.mean()), "cls": cls,
                    "gain": n01, "loss": n10, "p": p})
        print(f"  ... {name}: thr {thr:.2f}", flush=True)
    return out


def main(argv=None):
    rows = run()
    print(f"{'variant':<38} {'thr':>5} {'wFA/h':>6} {'P(d)':>6} {'hover':>6} "
          f"{'appr':>6} {'transit':>7} {'punch':>6} {'gain':>5} {'loss':>5} "
          f"{'p':>8}")
    print("-" * 106)
    for r in rows:
        if r["thr"] is None:
            print(f"{r['name']:<38}  no threshold meets the budget")
            continue
        c = r["cls"]
        print(f"{r['name']:<38} {r['thr']:5.2f} {r['wfa']:6.2f} "
              f"{r['pd_all']:6.3f} {c['p7_static']:6.3f} {c['approach']:6.3f} "
              f"{c['p7_flyby']:7.3f} {c['punch']:6.3f} "
              f"{r['gain']:5d} {r['loss']:5d} {r['p']:8.4f}")
    print("\ngain/loss are DISCORDANT pairs against the baseline on the same "
          "486 positives;\np is an exact two-sided McNemar. Every row is at "
          "its OWN recalibrated threshold.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
