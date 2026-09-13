"""
operating_point.py - the detection threshold and the two presets, in one place.

Every consumer (evaluate.py, make_golden.py, the firmware header generator,
the tests) reads its threshold from here, so moving the operating point is a
one-line edit below. Rerun `python src/export_config.py` afterwards so the
firmware picks it up.

How the value below was chosen
------------------------------
Decision rule, applied mechanically:
    minimise the SNR at which P(d) >= 0.90, subject to weighted FA/h <= 2.0;
    ties broken by lower weighted FA/h.
The 2.0 ceiling is a judgement call, not a derivation: a miss can cost a life
and a false alarm costs a user two seconds and a snooze press, so the cost
asymmetry says "be sensitive". The ceiling exists only because alarm fatigue
is itself a failure mode - a device people stop trusting protects nobody - and
roughly one dismissable alert per half hour is judged to stay usable.

The threshold is a point estimate of a noisy statistic. Measured sampling sd
is ~0.17 (~11%) from source onset placement alone, within the binding family.
Do not quote it as exact, do not tune it in the third decimal place, and do
not treat a 0.05 difference between two candidate values as meaningful.
"""

from pathlib import Path

# ============================================================================
# The original single operating point, kept for reference. The two presets
# below are what the firmware and the tests use.
THRESHOLD = 2.06
# ============================================================================

# Provenance of the value above, filled in by evaluate.py --full.
# These are reporting fields; nothing reads them to make a decision.
SELECTED_BY = "min SNR at P(d)>=0.90, subject to weighted FA/h <= 2.0"
FA_BUDGET_WEIGHTED_PER_HOUR = 2.0
CORPUS_TAG = "rural-v1"

_HERE = Path(__file__).resolve().parent
SWEEP_CSV = _HERE.parent / "data" / "operating_point_sweep.csv"

# The value chosen before the site profile existed, kept only so reports can
# quote the comparison. It was the lowest threshold with zero false events over
# a 2.73 h generic corpus, and it was set by a single family (motorbike_accel)
# that is close to absent at the modelled site. Never use it as a default.
LEGACY_THRESHOLD_GENERIC_CORPUS = 1.5982892163594564


def relative_range_db(thr, ref=LEGACY_THRESHOLD_GENERIC_CORPUS):
    """
    Relative detection-range change implied by moving the threshold, in dB of
    required SNR, via the sweep's P(d)>=0.90 point. Callers pass the SNR
    difference; this only converts to a range ratio at 6 dB per doubling.

    No absolute range can be claimed from synthetic audio. This is a ratio
    versus the old operating point on the same synthetic corpus and nothing
    more. It is not metres.
    """
    raise NotImplementedError("use range_ratio(delta_snr_db)")


def range_ratio(delta_snr_db):
    """
    Relative range multiplier for a change in the SNR needed to detect.

    Spherical spreading gives 6 dB per doubling of distance, so a detector
    that needs `delta_snr_db` less SNR reaches 2**(delta/6) times further.
    Relative only - see the warning above. Absorption makes the true figure
    worse than this at long range, so this is an optimistic bound.
    """
    return 2.0 ** (delta_snr_db / 6.0)


# ============================================================================
# The two presets, one line each. Each is (threshold, priority-band offset,
# general-band offset). The effective threshold for a frame is threshold + the
# offset for the band its f0 falls in, so the priority band (where the known
# threat physically lives) can be made more sensitive and the general band
# stricter.
#
# Measured on the full synthetic corpus (12.3 h negatives, 486 positives):
#   NORMAL      thr 2.14  0.86 wFA/h  P(d) all 0.37  hover 0.13  transit 0.53
#   HIGH_ALERT  thr 1.70  3.57 wFA/h  P(d) all 0.70  hover 0.59  transit 0.83
# The offsets are 0.0 because the two-tier band was measured to have no
# effect: the threat and the binding confuser (livestock) both live in the
# priority band, so any priority offset is exactly cancelled by the threshold
# recalibration. The machinery is kept in case recordings move the band.
#
# Budgets are weighted FA/h measured on non-gust beds. Gust behaviour is
# reported separately and deliberately not charged to the threshold: a
# gust-driven false alarm and a gust-driven miss are the same defect, and
# taxing the threshold for it would buy quiet by going deaf.
# ============================================================================
NORMAL = dict(threshold=2.1400, off_prio=0.0, off_gen=0.0)     # quiet posture
HIGH_ALERT = dict(threshold=1.7000, off_prio=0.0, off_gen=0.0)  # default

PRESETS = {"NORMAL": NORMAL, "HIGH_ALERT": HIGH_ALERT}
# HIGH_ALERT is the default because a miss costs far more than a false alarm.
# NORMAL detects a hovering drone 0.13 of the time, and at the detection
# horizon an incoming drone is acoustically quasi-static - head-on Doppler is a
# near-constant offset and level grows slowly until the final seconds - so
# hover and slow approach are the primary use case, not an edge case.
# HIGH_ALERT buys hover 0.13 -> 0.59 and transit 0.53 -> 0.83 for about one
# dismissable alert per 17 minutes. NORMAL stays selectable for a quieter
# posture.
DEFAULT_PRESET = "HIGH_ALERT"

NORMAL_BUDGET_WEIGHTED_PER_HOUR = 1.0
HIGH_ALERT_BUDGET_WEIGHTED_PER_HOUR = 4.0


def preset_config(name=DEFAULT_PRESET, **overrides):
    """Build a detector Config for a named preset. Single source of truth for
    everything downstream (evaluate, make_golden, the firmware header)."""
    from detector import Config
    p = PRESETS[name]
    kw = dict(COMMITTED_DETECTOR)
    kw.update(thr_off_prio=p["off_prio"], thr_off_gen=p["off_gen"])
    kw.update(overrides)
    return Config(**kw), p["threshold"]


# Detector settings adopted by measurement. Anything not listed here keeps its
# Config default, and each rejected candidate stays in the code, switched off.
COMMITTED_DETECTOR = dict(
    # The tonality-gated floor, and not the aggressive version: tau_rise_fast
    # 0.35 s ate the drone (a closing drone is a rise). At 1.5 s, gated only
    # on a genuinely flat whitened rise, it is strictly non-inferior - 0
    # regressions in 486 paired positives - and gains 3. The gusty gain is not
    # significant (+0.016, McNemar p=0.50) and must not be quoted as a gust fix.
    floor_mode="tonality",
    tau_rise_fast_s=1.5,
    flat_hi=0.70,
    # Re-anchoring rejected: no measured change in FA or P(d).
    reanchor=False,
    # Harmonic-extent gate rejected: drones and livestock both saturate at
    # 12/12 teeth.
    min_teeth=0,
    # Jitter gate adopted, measured on the raw argmax (the accepted chain is
    # capped at 2% by the continuity rule and carries no information).
    max_jitter=0.004,
)
