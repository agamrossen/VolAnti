#!/usr/bin/env python
"""
field_verdict.py - the day's answers, in minutes, in the order they matter.

    conda activate acoustic-detector
    cd ~/acoustic-detector/firmware/sentry_node
    python scripts/field_verdict.py                 # today's session
    python scripts/field_verdict.py captures/<session>
    python scripts/field_verdict.py --no-cache

WHAT IT IS FOR. A field day produces evidence faster than anyone can read it,
and the reading is where the value is. Every number below comes from replaying
archived samples through the PYTHON REFERENCE - src/detector.py,
src/detector_t2.py, src/detector_t3.py, src/band_verdict.py,
src/envelope_wash.py - never from the device's live verdict. That is what makes
a question asked at midnight answerable about a capture taken at noon, and it
is the entire reason every stage archives raw PCM.

IT DECIDES NOTHING BY ITSELF. It applies rules that were written down BEFORE
the data existed - field/DECISION_RULES_2026-08-21.md - and names which rule
fired. Changing a constant, a band or a threshold is a planning decision made
from this evidence, never by this script and never in the field (R5).

THE ORDER IS THE POINT:

  0  GATE     the day's quiet baseline, and the nulls computed FROM it
  1  R1       the band verdict (D-A) - FIRST, per the runbook's own priority
  2  R2/R4    the four-motor table - the project's most important prediction
  3           the distance ladder, in relative dB and range factors only
  4  R3       ambient/guard segments, and the Tier-3 rate histogram
  5  the protocol     the windscreen A/B - the foam decision, measured
  6  the protocol     the tilt ladder - mounting angle is this device's only beam

Sections 5 and 6 are the design in-enclosure characterization, and they are
printed after the day's verdict because that is what they are: instrument
characterization, not a verdict. They are in the brief's own order.

WHICH STAGE IS WHICH is `stage_vocab.py`, and it is the ONLY copy - the same
table decides what `family_field_analysis.py`'s adoption gate may score and
what `field_capture.py` warns about at the rig. A capture whose name no section
recognises is NAMED AT THE END and the tool exits non-zero if that was all of
them. It used to be dropped in silence, which is how a session nobody could
read gets quoted as a session with nothing in it.

THE GATE IS MECHANICAL, NOT ADVISORY. With no same-day quiet baseline this
tool REFUSES to print a band verdict. In the port the band rule was wrong three
different ways - it confirmed the band using the room's own 309 Hz hum, and the
comb statistic's null turned out to be 32.8 dB against a threshold of 15 -
and both failures were invisible without a same-day baseline to measure the
null against. A verdict printed without one is worse than no verdict, because
it looks exactly like a verdict.

Sections 5 and 6 are the exception that proves that rule, and deliberately:
both are INTERNALLY controlled - the A/B is its own control, the ladder is
referenced to its own 0 deg row - so neither needs the day's baseline and
neither is refused without one. Where a baseline does exist, the A/B also
prints the shift against it.
"""
import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent
ROOT = PROJ.parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "src"))

import numpy as np                                             # noqa: E402
import band_verdict as BV                                      # noqa: E402
import envelope_wash as EW                                     # noqa: E402
import detector as D                                           # noqa: E402
import detector_t2 as T2                                       # noqa: E402
import detector_t3 as T3                                       # noqa: E402
import detector_t4 as T4                                       # noqa: E402
import operating_point as op                                   # noqa: E402
import profile_store as PS                                     # noqa: E402
import stage_vocab as SV                                       # noqa: E402

FS = 16000
GREEN = "\033[32m"
RED = "\033[31m"
YELLOW = "\033[33m"
BOLD = "\033[1m"
DIM = "\033[2m"
OFF = "\033[0m"

MIN_QUIET_S = 600.0          # ten minutes, per the card and the rules
TAU3_SHIPPED = 20.0
TAU3_SWEEP = [14.0, 17.0, 20.0, 24.0, 27.0, 30.0]
FLOORS = [60.0, 100.0]       # 60 is replay-only; the field floor is 100


def wilson(k, n, z=1.96):
    """95% interval on a proportion. Every latched fraction gets one, because
    'latched 0.46' on 13 updates and on 1300 are different claims."""
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - h) / d, (c + h) / d)


def to_float(audio):
    a = np.asarray(audio)
    if a.dtype in (np.float32, np.float64):
        return a
    return a.astype(np.float64) / 32767.0


# ---------------------------------------------------------------------------
# session loading + a cache, so re-running is cheap
# ---------------------------------------------------------------------------

def load_session(sess):
    caps = {}
    for p in sorted(sess.glob("*_raw.npz")):
        stage = p.name[:-len("_raw.npz")]
        with np.load(p) as z:
            audio = z["audio"]
            rep = json.loads(str(z["report"])) if "report" in z else {}
        meta_p = sess / f"{stage}_meta.json"
        meta = json.loads(meta_p.read_text()) if meta_p.exists() else {}
        caps[stage] = {"audio": audio, "report": rep, "meta": meta,
                       "path": p, "seconds": audio.shape[1] / FS}
    return caps


def cache_key(path, tag):
    st = path.stat()
    return hashlib.sha256(
        f"{path.name}:{st.st_size}:{int(st.st_mtime)}:{tag}".encode()
    ).hexdigest()[:20]


class Cache:
    def __init__(self, sess, enabled=True):
        self.dir = sess / ".verdict_cache"
        self.enabled = enabled
        if enabled:
            self.dir.mkdir(exist_ok=True)

    def get(self, key):
        if not self.enabled:
            return None
        p = self.dir / f"{key}.json"
        if p.exists():
            try:
                return json.loads(p.read_text())
            except Exception:                                  # noqa: BLE001
                return None
        return None

    def put(self, key, value):
        if self.enabled:
            (self.dir / f"{key}.json").write_text(json.dumps(value))
        return value


# ---------------------------------------------------------------------------
# the tier replays
# ---------------------------------------------------------------------------

def replay_v1_t2(x, thr=None):
    """v1 and Tier-2 at their FROZEN operating points, over the same samples.

    This calls the SAME entry point rig_report.py uses - T2.analyze_quad_t2 for
    the frame stream and D.track_frames for v1's events - rather than a second
    implementation of the loop. A verdict tool whose replay disagrees with the
    session report is worse than no verdict tool."""
    cfg, thr_default = op.preset_config("HIGH_ALERT")
    thr = thr_default if thr is None else thr
    t2cfg = T2.T2Config()
    r = T2.analyze_quad_t2(x, cfg, t2cfg, thr_ref=thr)
    ev1, _ = D.track_frames(r, thr, cfg)
    return {"v1_events": len(ev1),
            "v1_peak": float(np.max(r["score"])),
            "v1_med": float(np.median(r["score"])),
            "t2_events": len(r["t2_events"]),
            "t2_peak": float(np.max(r["t2"]["score2"])),
            "threshold": float(thr)}


def replay_t3(x, tau3=TAU3_SHIPPED, fire_lo=100.0):
    cfg = T3.T3Config(fire_lo=fire_lo, tau3=tau3)
    out = T3.analyse(x, cfg)
    recs = out["records"]
    if not recs:
        return {"updates": 0, "events": 0, "latched": 0.0,
                "W_med": float("nan"), "W_p90": float("nan"),
                "r_mode": float("nan"), "tau3": tau3, "fire_lo": fire_lo}
    W = np.array([r["W"] for r in recs], float)
    Wa = np.array([r["W_any"] for r in recs], float)
    ra = np.array([r["r_any"] for r in recs], float)
    lat = float(np.mean([1.0 if r["fired3"] else 0.0 for r in recs]))
    fired = [r for r in recs if r["fired3"]]
    rr = np.array([r["r"] for r in fired], float) if fired else np.array([])
    lo, hi = wilson(sum(1 for r in recs if r["fired3"]), len(recs))
    return {"updates": len(recs), "events": int(out["n_events"]),
            "latched": lat, "latched_ci": [lo, hi],
            "W_med": float(np.median(W)), "W_p90": float(np.percentile(W, 90)),
            "W_any_med": float(np.median(Wa)),
            "W_any_p99": float(np.percentile(Wa, 99)),
            "r_any_med": float(np.median(ra)),
            "r_mode": float(np.median(rr)) if rr.size else float("nan"),
            "tau3": tau3, "fire_lo": fire_lo}


def replay_t4(x):
    """Tier-4 at its calibrated point, as DATA and never as a detection.

    It ships disarmed and the reason is unchanged: zero events in 487 s of real
    drone-free audio bounds its rate at 16/h against a 0.40/h allowance, and
    certifying it needs ~7.5 h. It is replayed here because the design asks
    for the Tier-4 null shift across the windscreen A/B and the protocol for its score
    against tilt, and a null that is not re-measured on the instrument is a
    null inherited across hardware - which this brief forbids in its first
    paragraph."""
    cfg = T4.T4Config()
    out = T4.analyse(x, cfg)
    recs = out["records"]
    if not recs:
        return {"updates": 0, "events": 0, "W4_med": float("nan"),
                "W4_p99": float("nan"), "f0_at_max": float("nan"),
                "tau4": cfg.tau4}
    W = np.array([r["W4"] for r in recs], float)
    i = int(np.argmax(W))
    return {"updates": len(recs), "events": int(out["n_events"]),
            "W4_med": float(np.median(W)),
            "W4_p99": float(np.percentile(W, 99)),
            "f0_at_max": float(recs[i]["f0"]), "tau4": float(cfg.tau4)}


def cached(cache, cap, tag, fn):
    """One replay per (capture file, tag). The A/B and the ladder ask the same
    questions of stages the R2 and distance tables already replayed, and a
    capture replayed twice in one run is a minute of somebody's evening."""
    key = cache_key(cap["path"], tag)
    got = cache.get(key)
    return got if got is not None else cache.put(key, fn())


def per_channel_rates(x):
    """The winning envelope rate per microphone. Four channels agreeing to
    within a percent is a source; four channels disagreeing is the room."""
    X = np.atleast_2d(np.asarray(x))
    out = []
    for c in range(X.shape[0]):
        try:
            a = EW.analyse(X[c:c + 1])
            r = a["r_median"]
        except Exception:                                      # noqa: BLE001
            r = float("nan")
        out.append(float(r))
    return out


def of_section(caps, sec):
    """The stages this section prints, in name order. The membership test is
    `stage_vocab.section`, never a `startswith` written here: a section that
    picks its own stages is a second stage vocabulary, and the two this file
    used to carry disagreed with the adoption gate's about `guard_*` from the
    day that gate was written."""
    return sorted(k for k in caps if SV.section(k) == sec)


ENV_BIN_HZ = 2000.0 / 1024.0     # FS_ENV / ENV_NFFT = 1.953 Hz


def spread_resolution_pct(r_centre, bins=2.0):
    """The SMALLEST spread the envelope spectrum could possibly resolve.

    Measured CONSEQUENCE, found by running this tool on a synthetic pair
    tonight: the envelope bin is 1.95 Hz, so four motors within +-1.5% of
    141 Hz span 4.2 Hz - about two bins - and merge into ONE line. The
    in-stage estimator below therefore returns "not resolvable" for exactly
    the spread the project most wants to measure.

    That is a resolution limit, not an absence of spread, and the two must
    never be confused. The resolvable measurement is the CROSS-SOLO one: each
    motor recorded alone, each rate measured separately, the spread taken
    across those four numbers. See solo_spread()."""
    if not np.isfinite(r_centre) or r_centre <= 0:
        return float("nan")
    return 100.0 * bins * ENV_BIN_HZ / r_centre


def solo_spread(caps):
    """The spread ACTUALLY ACHIEVED, from the per-motor solo stages.

    Servo testers cannot set a spread precisely, so the number on the stage
    label is an intention. Each motor recorded ALONE gives its own rate at
    full resolution, and the spread across those is a measurement rather than
    an estimate. This is why the solo ladder is worth its sixty seconds a
    motor even after the band verdict is settled."""
    rates = {}
    for k in sorted(caps):
        if SV.section(k) != "solo":
            continue
        try:
            a = EW.analyse(to_float(caps[k]["audio"]))
            r = float(a["r_median"])
        except Exception:                                      # noqa: BLE001
            r = float("nan")
        if np.isfinite(r):
            rates[k] = r
    if len(rates) < 2:
        return None
    v = np.array(list(rates.values()), float)
    return {"rates": rates, "mean": float(v.mean()),
            "spread_pct": float(100.0 * (v.max() - v.min()) / v.mean())}


def measured_spread(x, r_centre):
    """The in-stage estimator: four closely-spaced lines in ONE four-motor
    recording. Kept because when the spread IS large enough to resolve it is
    the most direct measurement there is - but see spread_resolution_pct()
    for when it cannot be."""
    if not np.isfinite(r_centre) or r_centre <= 0:
        return float("nan"), 0
    X = np.atleast_2d(np.asarray(x))
    a = EW.channel_mean_envelope(X)
    freqs, psd_frames, _times = EW.envelope_spectrum(a)
    if not len(psd_frames):
        return float("nan"), 0
    prom = EW.prominence_db(psd_frames.mean(axis=0))
    lo, hi = r_centre * 0.93, r_centre * 1.07
    sel = (freqs >= lo) & (freqs <= hi)
    if sel.sum() < 5:
        return float("nan"), 0
    f, p = freqs[sel], prom[sel]
    # local maxima at least 3 dB over the local floor
    peaks = [i for i in range(1, len(p) - 1)
             if p[i] > p[i - 1] and p[i] >= p[i + 1] and p[i] >= 3.0]
    if len(peaks) < 2:
        return float("nan"), len(peaks)
    fs_pk = f[peaks]
    return float((fs_pk.max() - fs_pk.min()) / r_centre), len(peaks)


# ---------------------------------------------------------------------------
# 0. THE GATE
# ---------------------------------------------------------------------------

def stage_gate(caps, min_quiet_s):
    print(f"\n{BOLD}{'=' * 74}{OFF}")
    print(f"{BOLD}  0. GATE - the day's quiet baseline{OFF}")
    print(f"{BOLD}{'=' * 74}{OFF}")

    quiet_keys = of_section(caps, "quiet")
    if not quiet_keys:
        print(f"\n  {RED}NO QUIET BASELINE IN THIS SESSION.{OFF}")
        print("""
  No band verdict will be printed, and that is a rule, not a preference.

  The band rule has been wrong three separate ways, and every one of
  them was invisible without a same-day baseline:
    * it CONFIRMED the band using the room's own 309 Hz hum - quiet scored
      T_p 21.3 dB at 309 Hz and the rotor 20.7 dB at the same 309 Hz;
    * the comb statistic's null was 32.8 dB against a threshold of 15,
      because it maximises over ~11,000 (f0, harmonic) pairs;
    * both were fixed by measuring the null on the day's own quiet stage.

  GO AND RECORD ONE:

      python scripts/field_capture.py --stage quiet_baseline --seconds 600

  Same place, same rig present but OFF, same wind. Ten minutes.
""")
        return None

    key = max(quiet_keys, key=lambda k: caps[k]["seconds"])
    q = caps[key]
    x = to_float(q["audio"])
    secs = q["seconds"]
    short = secs < min_quiet_s
    print(f"\n  using '{key}': {secs:.0f} s "
          f"({'OK' if not short else RED + 'SHORT' + OFF}, "
          f"want >= {min_quiet_s:.0f} s)")
    if short:
        print(f"  {YELLOW}the baseline is shorter than the rule asks for. The "
              f"nulls below are computed anyway and every verdict that uses "
              f"them is marked PROVISIONAL.{OFF}")

    freqs, quiet_psd = BV.welch_psd(x)
    comb_null = BV.split_half_null(x)
    print(f"\n  {BOLD}the day's nulls - what this site scores by chance{OFF}")
    if comb_null.get("usable"):
        print(f"    in-band comb null    T_p {comb_null['T_p_db']:5.2f} dB "
              f"@ {comb_null.get('T_p_hz', float('nan')):6.1f} Hz")
        print(f"                         C_p {comb_null['C_p_db']:5.2f} dB "
              f"@ f0 {comb_null.get('C_p_f0_hz', float('nan')):6.1f} Hz")
        print(f"    {DIM}(quiet first half vs second half - 'no rig at all' "
              f"through the same statistic){OFF}")
    else:
        print(f"    {YELLOW}too short to split in half - no comb null{OFF}")

    t3q = replay_t3(x, tau3=TAU3_SHIPPED, fire_lo=100.0)
    print(f"    envelope null        W_any median {t3q['W_any_med']:5.2f}, "
          f"p99 {t3q['W_any_p99']:5.2f}   "
          f"{DIM}(indoor precedent ~11.0){OFF}")
    print(f"    Tier-3 on quiet      {t3q['events']} events, "
          f"latched {t3q['latched']:.3f}, W median {t3q['W_med']:.2f}")
    if t3q["events"]:
        print(f"    {YELLOW}Tier-3 fires on the QUIET baseline. Every Tier-3 "
              f"number today must be read against that.{OFF}")

    # Tier-4's null is the number its threshold was BUILT from - tau4 = 30.5
    # is null_p99 + 3.0 on 487 s of real drone-free audio - so it is the one
    # statistic on this card that a new enclosure or a new site can invalidate
    # outright. the design standing rule: no number is inherited across
    # hardware.
    t4q = replay_t4(x)
    print(f"    comb-4 null          W4 median {t4q['W4_med']:5.2f}, "
          f"p99 {t4q['W4_p99']:5.2f}   "
          f"{DIM}(487 s indoor precedent: median 21.4, p99 27.3, "
          f"tau4 {t4q['tau4']:.1f}){OFF}")
    if t4q["events"]:
        print(f"    {RED}Tier-4 fires on the QUIET baseline. Its threshold "
              f"stands on a null that this site does not reproduce - say so "
              f"before quoting any Tier-4 number from today.{OFF}")

    tones = stationary_tones(freqs, quiet_psd)
    if tones:
        print(f"\n  {BOLD}stationary tones in the baseline{OFF} "
              f"{DIM}(candidate exclusion entries - type them on the device, "
              f"log them on paper; they are NOT persisted){OFF}")
        for f, p in tones:
            print(f"    {f:7.1f} Hz   +{p:4.1f} dB     E 1 {int(round(f))} 10")
    return {"key": key, "seconds": secs, "short": short,
            "quiet_psd": quiet_psd, "freqs": freqs, "comb_null": comb_null,
            "t3": t3q, "t4": t4q, "x": x, "tones": tones}


def stationary_tones(freqs, psd, n=6):
    """The loudest narrow lines in the baseline. These are the site's own
    machinery, and a Tier-2 hit on one of them is the room, not a threat."""
    band = (freqs >= 80) & (freqs <= 2000)
    f, p = freqs[band], psd[band]
    lp = 10 * np.log10(np.maximum(p, 1e-30))
    med = np.median(lp)
    idx = [i for i in range(2, len(lp) - 2)
           if lp[i] > lp[i - 1] and lp[i] >= lp[i + 1] and lp[i] - med > 12]
    idx.sort(key=lambda i: -(lp[i] - med))
    out = []
    for i in idx:
        if all(abs(f[i] - g) > 15 for g, _ in out):
            out.append((float(f[i]), float(lp[i] - med)))
        if len(out) >= n:
            break
    return out


# ---------------------------------------------------------------------------
# 1. R1 - the band verdict
# ---------------------------------------------------------------------------

def stage_band(caps, gate):
    print(f"\n{BOLD}{'=' * 74}{OFF}")
    print(f"{BOLD}  1. R1 - THE BAND VERDICT (D-A){OFF}")
    print(f"{BOLD}{'=' * 74}{OFF}")
    if gate is None:
        print(f"\n  {RED}REFUSED: no quiet baseline. See the gate above.{OFF}")
        return None

    solo_keys = of_section(caps, "solo")
    if len(solo_keys) < 3:
        print(f"\n  {YELLOW}R1 needs >= 3 per-motor solo stages; this session "
              f"has {len(solo_keys)}: {solo_keys or 'none'}{OFF}")
        print(f"  {DIM}capture them with "
              f"`field_capture.py --stage solo_m1_hover --seconds 60`{OFF}")
        return None

    measurements = []
    for k in solo_keys:
        m = BV.measure(to_float(caps[k]["audio"]), quiet_psd=gate["quiet_psd"])
        m["motor"] = k
        measurements.append(m)
        print(f"\n  {k}")
        print(f"    best in-band tooth   T_p {m['T_p_db']:6.2f} dB "
              f"@ {m['T_p_hz']:7.1f} Hz")
        print(f"    best comb            C_p {m['C_p_db']:6.2f} dB "
              f"@ f0 {m['C_p_f0_hz']:7.1f} Hz")
        print(f"    strong teeth in 200-800 Hz: "
              f"{m['strong_teeth_in_prio']}   outside: "
              f"{m['strong_teeth_out_of_prio']}")
        print(f"    high band vs quiet   {m['hb_vs_quiet_db']:+6.2f} dB")

    V = BV.verdict(measurements, null=gate["comb_null"])
    print()
    print(BV.format_verdict(V))
    if gate["short"]:
        print(f"  {YELLOW}PROVISIONAL: the quiet baseline was shorter than "
              f"{MIN_QUIET_S:.0f} s{OFF}")
    print(f"\n  {BOLD}Rule R1 says:{OFF}")
    v = str(V.get("verdict", "")).upper()
    if "CONFIRMED" in v:
        print("    200-800 Hz stands. D-A closes.")
    elif "SHIFTED" in v:
        print("    Set the runtime band to the measured blade-pass +-30% for "
              "the rest of the field work.")
        print("    Log a Stage-0 partial reopen. NO recalibration in the "
              "field (R5).")
    elif "NO USABLE COMB" in v:
        print("    D-A stays open for the loaded-flight case. The envelope "
              "path's priority rises - see R4.")
    else:
        print("    MIXED - the rule prints the deltas to the nearest verdict "
              "above. No action without planning.")
    return V


# ---------------------------------------------------------------------------
# 2. R2 / R4 - the four-motor table
# ---------------------------------------------------------------------------

def stage_quad(caps, gate, cache):
    print(f"\n{BOLD}{'=' * 74}{OFF}")
    print(f"{BOLD}  2. R2 - THE FOUR-MOTOR TABLE{OFF}")
    print(f"{BOLD}{'=' * 74}{OFF}")
    print(f"  {DIM}The project's most important prediction: single motor "
          f"W ~ 30.7, four motors at\n  +-1.5% spread predicted W ~ 20.4 - "
          f"which sits exactly on tau3 = 20.{OFF}")

    keys = of_section(caps, "quad")
    if not keys:
        print(f"\n  {YELLOW}no quad_* stages in this session{OFF}")
        return None

    ss = solo_spread(caps)
    if ss:
        print(f"\n  {BOLD}cross-solo spread (the RESOLVABLE measurement){OFF}")
        for k, r in ss["rates"].items():
            print(f"    {k:16s} {r:7.2f} Hz")
        print(f"    {BOLD}spread {ss['spread_pct']:.2f}%{OFF} across "
              f"{len(ss['rates'])} motors, mean {ss['mean']:.1f} Hz")
        print(f"    {DIM}Each motor alone, each rate at full resolution. This "
              f"is what R2 should be read against.{OFF}")
    else:
        print(f"\n  {YELLOW}no solo stages, so the achieved spread cannot be "
              f"measured at full resolution - only estimated in-stage "
              f"below{OFF}")

    rows = []
    for k in sorted(keys):
        x = to_float(caps[k]["audio"])
        secs = caps[k]["seconds"]
        t3 = replay_t3(x, TAU3_SHIPPED, 100.0)
        v12 = replay_v1_t2(x)
        rates = per_channel_rates(x)
        centre = float(np.nanmedian([r for r in rates if np.isfinite(r)])
                       if any(np.isfinite(r) for r in rates) else np.nan)
        spread, n_pk = measured_spread(x, centre)
        rows.append({"stage": k, "seconds": secs, "t3": t3, "v12": v12,
                     "rates": rates, "centre": centre,
                     "spread": spread, "n_peaks": n_pk})

        print(f"\n  {BOLD}{k}{OFF}  ({secs:.0f} s)")
        if secs < 60:
            print(f"    {YELLOW}under 60 s - R2 gives no verdict on this "
                  f"stage{OFF}")
        lo, hi = t3["latched_ci"]
        print(f"    Tier-3   W median {t3['W_med']:6.2f}  p90 "
              f"{t3['W_p90']:6.2f}   winning rate "
              f"{t3['r_mode']:6.1f} Hz")
        print(f"             latched {t3['latched']:.3f} "
              f"[{lo:.3f}-{hi:.3f}]   events {t3['events']}   "
              f"updates {t3['updates']}")
        print(f"    per-channel rates  "
              f"{['%.1f' % r if np.isfinite(r) else 'nan' for r in rates]}")
        if np.isfinite(spread):
            print(f"    MEASURED spread    {spread * 100:.2f}%  "
                  f"({n_pk} lines resolved)   {DIM}estimate from audio, not "
                  f"from the servo-tester setting{OFF}")
        else:
            lim = spread_resolution_pct(centre)
            print(f"    in-stage spread    {DIM}not resolvable ({n_pk} line"
                  f"{'s' if n_pk != 1 else ''}){OFF}")
            if np.isfinite(lim):
                print(f"                       {DIM}the envelope bin is "
                      f"{ENV_BIN_HZ:.2f} Hz, so nothing below ~{lim:.1f}% is "
                      f"separable at {centre:.0f} Hz. NOT the same as zero "
                      f"spread.{OFF}")
        print(f"    v1       {v12['v1_events']} events, peak "
              f"{v12['v1_peak']:.3f} (thr {v12['threshold']:.3f})")
        t2p = v12["t2_peak"]
        print(f"    Tier-2   {v12['t2_events']} events, peak "
              f"{'%.3f' % t2p if t2p is not None else 'n/a'} (tau2 1.150)")

    # the free sweep - replay is free, so the whole feasible set costs nothing
    print(f"\n  {BOLD}tau3 sweep (replay is free){OFF}  "
          f"{DIM}latched fraction{OFF}")
    hdr = "    stage           floor " + " ".join(
        f"t{t:5.1f}" for t in TAU3_SWEEP)
    print(hdr)
    for k in sorted(keys):
        x = to_float(caps[k]["audio"])
        for fl in FLOORS:
            cells = []
            for t in TAU3_SWEEP:
                key = cache_key(caps[k]["path"], f"t3:{t}:{fl}")
                got = cache.get(key)
                if got is None:
                    got = cache.put(key, replay_t3(x, t, fl))
                cells.append(f"{got['latched']:6.3f}")
            print(f"    {k:15s} {fl:5.0f} " + " ".join(cells))

    print(f"\n  {BOLD}Rule R2 says:{OFF}")
    print("    (a) measured spread in +-1-2% with W median >= 20 and latched "
          ">= 0.5  -> the four-motor case HOLDS")
    print("    (b) matched strong but spread collapses (W < 15)             "
          "     -> FC-matched only; neither promote nor abandon")
    print("    (c) even MATCHED at <= 10 m gives W median < 15              "
          "     -> ABANDON track; Tier-3 demotes to an instrument")
    print(f"    {DIM}No verdict on < 60 s of audio. Wilson intervals are "
          f"printed above.{OFF}")

    # R4: does the calibrated pair see the steady threat proxy at all?
    print(f"\n  {BOLD}Rule R4 - does the CALIBRATED pair see it?{OFF}")
    for r in rows:
        if r["seconds"] < 60:
            continue
        v12, t3 = r["v12"], r["t3"]
        calibrated = v12["v1_events"] + v12["t2_events"]
        if calibrated == 0 and t3["latched"] >= 0.5:
            print(f"    {RED}{r['stage']}: v1+Tier-2 detect NOTHING while "
                  f"Tier-3 latches {t3['latched']:.2f}.{OFF}")
            print(f"      That is the reference result repeating on four "
                  f"loaded motors. By R4 the envelope path graduates from "
                  f"'live configuration' to NECESSARY.")
        elif calibrated == 0:
            print(f"    {r['stage']}: no calibrated events, and Tier-3 "
                  f"latched {t3['latched']:.2f} (< 0.5) - inconclusive.")
        else:
            print(f"    {GREEN}{r['stage']}: the calibrated pair fired "
                  f"({v12['v1_events']} v1, {v12['t2_events']} T2).{OFF}")
    return rows


# ---------------------------------------------------------------------------
# 3. the distance ladder
# ---------------------------------------------------------------------------

def stage_distance(caps, gate):
    keys = of_section(caps, "dist")
    if not keys:
        return None
    # WINDSCREEN ARMS DO NOT BELONG IN A DISTANCE LADDER, and leaving them in
    # was a real defect: this table prints "high band +-X dB vs the first
    # stage" under the caption "a 6 dB drop is a range factor of 2", so a
    # dist_mid_ws_on / dist_mid_ws_off pair would read a FOAM difference as a
    # RANGE difference - the one table on the card whose whole purpose is that
    # dB means distance here.
    #
    # the protocol tilt ladder got this care from the start (tilt_angle() returns None
    # for the face-away control, "the control is not an angle and must never
    # be plotted as one"); the protocol did not. The arms are measured in the protocol, against
    # each other, which is the comparison they were captured for.
    ws = [k for k in keys if SV.windscreen_state(k)]
    keys = [k for k in keys if not SV.windscreen_state(k)]
    if not keys:
        print(f"\n{BOLD}  3. DISTANCE LADDER{OFF}")
        print(f"  {YELLOW}every distance stage carries a windscreen suffix - "
              f"there is no ladder here,\n  only an A/B.{OFF}")
        return None
    print(f"\n{BOLD}{'=' * 74}{OFF}")
    print(f"{BOLD}  3. DISTANCE LADDER{OFF}")
    print(f"{BOLD}{'=' * 74}{OFF}")
    print(f"  {DIM}Relative dB and range FACTORS only. The label is metadata, "
          f"never a claim:\n  this project makes no absolute range "
          f"statements.{OFF}\n")
    ref_hb = None
    for k in keys:
        x = to_float(caps[k]["audio"])
        f_, p_ = BV.welch_psd(x)
        hb = BV.band_power_db(f_, p_, BV.HB_LO, BV.HB_HI)
        t3 = replay_t3(x, TAU3_SHIPPED, 100.0)
        if ref_hb is None:
            ref_hb = hb
        d = hb - ref_hb
        print(f"    {k:15s} high band {d:+6.2f} dB vs the first stage   "
              f"W median {t3['W_med']:6.2f}   latched {t3['latched']:.3f}")
    if ws:
        print(f"\n    {DIM}not in this ladder, because they differ by FOAM "
              f"and not by distance: {', '.join(ws)}\n    "
              f"They are measured against each other in the windscreen A/B.{OFF}")
    print(f"\n    {DIM}A 6 dB drop is a range factor of 2 in free field. "
          f"Outdoors over ground it is not free field, so treat the factor as "
          f"an upper bound.{OFF}")
    return True


# ---------------------------------------------------------------------------
# 4. R3 - ambient conduct
# ---------------------------------------------------------------------------

def stage_ambient(caps, gate):
    keys = of_section(caps, "ambient")
    if not keys:
        return None
    print(f"\n{BOLD}{'=' * 74}{OFF}")
    print(f"{BOLD}  4. R3 - AMBIENT CONDUCT{OFF}")
    print(f"{BOLD}{'=' * 74}{OFF}")
    for k in keys:
        x = to_float(caps[k]["audio"])
        t3 = replay_t3(x, TAU3_SHIPPED, 100.0)
        v12 = replay_v1_t2(x)
        print(f"\n  {k}  ({caps[k]['seconds']:.0f} s)")
        print(f"    v1 {v12['v1_events']} events, Tier-2 {v12['t2_events']}, "
              f"Tier-3 {t3['events']} (latched {t3['latched']:.3f})")
        # the insect question: WHERE does Tier-3 look when it is not firing?
        cfg = T3.T3Config(fire_lo=100.0, tau3=TAU3_SHIPPED)
        recs = T3.analyse(x, cfg)["records"]
        if recs:
            ra = np.array([r["r_any"] for r in recs], float)
            ra = ra[np.isfinite(ra)]
            if ra.size:
                hist, edges = np.histogram(ra, bins=8,
                                           range=(40, 650))
                print(f"    Tier-3 whole-grid winning rate histogram "
                      f"{DIM}(the insect question){OFF}")
                for c, e0, e1 in zip(hist, edges[:-1], edges[1:]):
                    bar = "#" * int(40 * c / max(hist.max(), 1))
                    print(f"      {e0:5.0f}-{e1:5.0f} Hz |{bar}")
    print(f"\n  {BOLD}Rule R3 says:{OFF}")
    print("    <= 2 Tier-3 latches, none > 30 s, no confuser class 3x  -> "
          "Tier-3 stays the field-default pair")
    print("    worse                                                   -> "
          "demote to scripted stages; v1+Tier-2 becomes the guard default")
    print(f"    {DIM}Either way the latches are the deliverable, not the "
          f"embarrassment. Promotion to a DEPLOYMENT default needs R2(a) and "
          f"a real FA calibration - an afternoon cannot grant that.{OFF}")
    return True


# ---------------------------------------------------------------------------
# 5. the design - THE WINDSCREEN A/B
# ---------------------------------------------------------------------------

# Wind's own continuum, below the alert band. Tier-4's grid floor sits at
# 110 Hz because wind manufactures combs below it - the tier's null max drops
# 23.2 -> 15.7 when the floor moves 60 -> 110 - so this is the band the foam
# is bought to remove, and the band where a windscreen that works shows first.
WIND_LO, WIND_HI = 20.0, 200.0


def _num(v):
    """None -> NaN, so a statistic that is unavailable prints as unavailable
    instead of raising or, worse, printing a plausible default. Tier-2's
    `kappa` is unavailable on the one-FFT path and reads NaN there, never
    1.0, and this column follows the same rule."""
    return float("nan") if v is None else float(v)


def arm_measure(cap, cache):
    """One arm of the A/B, in every statistic the foam could move."""
    x = to_float(cap["audio"])
    freqs, psd = BV.welch_psd(x)
    return {
        "seconds": cap["seconds"],
        "wind_db": BV.band_power_db(freqs, psd, WIND_LO, WIND_HI),
        "prio_db": BV.band_power_db(freqs, psd, BV.PRIO_LO, BV.PRIO_HI),
        "hb_db": BV.band_power_db(freqs, psd, BV.HB_LO, BV.HB_HI),
        "t3": cached(cache, cap, f"ws-t3/{TAU3_SHIPPED}/100",
                     lambda: replay_t3(x, TAU3_SHIPPED, 100.0)),
        "t4": cached(cache, cap, "ws-t4/1", lambda: replay_t4(x)),
        "v12": cached(cache, cap, "ws-v1t2/1", lambda: replay_v1_t2(x)),
        "tones": stationary_tones(freqs, psd),
        # KEPT so the other arm can be Measured at these frequencies rather
        # than merely ranked against it - see tone_diff.
        "freqs": freqs, "psd": psd,
    }


def tone_prominence_at(freqs, psd, f0, tol_hz=15.0):
    """The prominence, in dB over the local median, of whatever sits within
    tol_hz of f0 - using stationary_tones' own definition so the two numbers
    are comparable. Returns None if f0 is outside the band it works in."""
    band = (freqs >= 80) & (freqs <= 2000)
    f, pw = freqs[band], psd[band]
    if not len(f):
        return None
    lp = 10 * np.log10(np.maximum(pw, 1e-30))
    med = np.median(lp)
    near = np.abs(f - f0) <= tol_hz
    if not near.any():
        return None
    return float(np.max(lp[near]) - med)


def tone_diff(a_tones, b_freqs, b_psd, tol_hz=15.0, floor_db=12.0):
    """Lines in `a` that are NOT STILL THERE in b's spectrum.

    IT MEASURES b, IT DOES NOT RANK IT, and that distinction is the whole of
    this function. It used to compare a's top-six against b's top-six and call
    a line "gone" when it was missing from the other LIST - so a line that was
    still fully present but had fallen out of the ranking, because that arm
    happened to carry six louder ones, was reported as removed.

    That bias pointed exactly the wrong way. The screen-ON arm is the arm most
    likely to have extra loud lines (the fan running, foam rustle), so the
    tool manufactured a positive result for the windscreen out of a ranking
    artifact - on the one line-level measurement the protocol asks the A/B to produce,
    and the one a foam decision would be taken from. Demonstrated on a
    synthetic pair where all three shared lines were reported "gone behind the
    foam" while unchanged in the ON audio.

    A line is gone when its prominence in b has fallen below the criterion
    stationary_tones uses to call something a line at all. Anything else is a
    ranking, not a measurement."""
    out = []
    for f, p in a_tones:
        q = tone_prominence_at(b_freqs, b_psd, f, tol_hz)
        if q is None or q < floor_db:
            out.append((f, p, q))
    return out


def stage_windscreen(caps, gate, cache):
    pairs, lone = SV.windscreen_pairs(caps)
    if not pairs and not lone:
        return None
    print(f"\n{BOLD}{'=' * 74}{OFF}")
    print(f"{BOLD}  5. WINDSCREEN A/B{OFF}")
    print(f"{BOLD}{'=' * 74}{OFF}")
    for ln in (
        "The foam decision becomes a MEASURED one, and the winning",
        "configuration is the one every null and the field card assume",
        "afterwards. Wind at the capsule is the in-band masker at 14 m",
        "(+3.3 dB excess is what masking leaves) and a windscreen is worth",
        "10-25 dB of it, which is why it is the cheapest range purchase",
        "available and why it is measured rather than assumed.",
    ):
        print(f"  {DIM}{ln}{OFF}")
    print(f"\n  {YELLOW}THE A/B ASSUMES BOTH ARMS SAW THE SAME WIND, AND "
          f"NOTHING HERE CAN CHECK THAT.{OFF}")
    for ln in (
        "Back to back, same position, same throttle, minutes apart. A gust",
        "between the two arms is indistinguishable from a screen that works.",
    ):
        print(f"  {DIM}{ln}{OFF}")

    for name in lone:
        st = SV.windscreen_state(name)
        print(f"\n  {RED}{name}: the '{st}' arm only - NO A/B.{OFF}")
        print(f"    {DIM}A level with no control is not a weak comparison, it "
              f"is not a comparison. Capture\n    "
              f"{SV.windscreen_base(name)}_ws_"
              f"{'off' if st == 'on' else 'on'} in the same conditions, or "
              f"this capture answers nothing.{OFF}")

    rows = []
    for base in sorted(pairs):
        on_k, off_k = pairs[base]["on"], pairs[base]["off"]
        on, off = arm_measure(caps[on_k], cache), arm_measure(caps[off_k],
                                                              cache)
        rows.append({"base": base, "on": on, "off": off,
                     "on_stage": on_k, "off_stage": off_k})
        print(f"\n  {BOLD}{base}{OFF}   screen ON {on['seconds']:.0f} s  vs  "
              f"OFF {off['seconds']:.0f} s")
        if min(on["seconds"], off["seconds"]) < 60:
            print(f"    {YELLOW}under 60 s on one arm - the nulls below are "
                  f"short-sample estimates{OFF}")
        print(f"    {BOLD}band level, ON minus OFF{OFF}   "
              f"{DIM}negative = the screen removed energy{OFF}")
        for lab, k in ((f"{WIND_LO:.0f}-{WIND_HI:.0f} Hz  wind", "wind_db"),
                       (f"{BV.PRIO_LO:.0f}-{BV.PRIO_HI:.0f} Hz alert",
                        "prio_db"),
                       (f"{BV.HB_LO / 1000:.1f}-{BV.HB_HI / 1000:.1f} kHz "
                        "high", "hb_db")):
            print(f"      {lab:22s} {on[k] - off[k]:+7.2f} dB   "
                  f"{DIM}(on {on[k]:7.2f}, off {off[k]:7.2f}){OFF}")

        print(f"    {BOLD}the nulls the thresholds stand on{OFF}")
        print(f"      Tier-3 W_any p99      on {on['t3']['W_any_p99']:6.2f}   "
              f"off {off['t3']['W_any_p99']:6.2f}   "
              f"shift {on['t3']['W_any_p99'] - off['t3']['W_any_p99']:+6.2f}")
        print(f"      Tier-3 latched        on {on['t3']['latched']:6.3f}   "
              f"off {off['t3']['latched']:6.3f}   "
              f"events {on['t3']['events']} / {off['t3']['events']}")
        print(f"      Tier-4 W4 p99         on {on['t4']['W4_p99']:6.2f}   "
              f"off {off['t4']['W4_p99']:6.2f}   "
              f"shift {on['t4']['W4_p99'] - off['t4']['W4_p99']:+6.2f}")
        print(f"      Tier-4 events         on {on['t4']['events']:6d}   "
              f"off {off['t4']['events']:6d}   "
              f"{DIM}(tau4 {on['t4']['tau4']:.1f}, and it ships disarmed - "
              f"these are data){OFF}")
        print(f"      v1 peak / Tier-2 peak on {on['v12']['v1_peak']:6.3f} / "
              f"{_num(on['v12']['t2_peak']):6.3f}   "
              f"off {off['v12']['v1_peak']:6.3f} / "
              f"{_num(off['v12']['t2_peak']):6.3f}")
        print(f"      v1 / Tier-2 events    on "
              f"{on['v12']['v1_events']} / {on['v12']['t2_events']}   "
              f"off {off['v12']['v1_events']} / {off['v12']['t2_events']}")
        if gate is not None:
            print(f"      {DIM}against the day's own baseline "
                  f"('{gate['key']}'): Tier-3 W_any p99 "
                  f"{on['t3']['W_any_p99'] - gate['t3']['W_any_p99']:+.2f} "
                  f"on, "
                  f"{off['t3']['W_any_p99'] - gate['t3']['W_any_p99']:+.2f} "
                  f"off{OFF}")

        killed = tone_diff(off["tones"], on["freqs"], on["psd"])
        added = tone_diff(on["tones"], off["freqs"], off["psd"])
        print(f"    {BOLD}stationary lines{OFF}   "
              f"{DIM}the vortex-shedding question{OFF}")
        # The third element is what the OTHER arm actually measured at that
        # frequency - printed, because "gone" and "down to 6 dB" are different
        # findings and only one of them is a screen that works.
        if killed:
            for f, p, q in killed:
                q_s = "not in band" if q is None else f"{q:4.1f} dB"
                print(f"      {f:7.1f} Hz  +{p:4.1f} dB bare, {q_s} with the "
                      f"screen")
        else:
            print(f"      {DIM}no line present bare and gone with the "
                  f"screen{OFF}")
        if added:
            for f, p, q in added:
                q_s = "not in band" if q is None else f"{q:4.1f} dB"
                print(f"      {YELLOW}{f:7.1f} Hz  +{p:4.1f} dB WITH the "
                      f"screen, {q_s} without it{OFF}")

    print(f"\n  {BOLD}What this decides:{OFF} nothing, here. The winning "
          f"configuration is a planning\n  decision (R5) taken from the "
          f"deltas above, and once taken, every null\n  and every "
          f"number on the field card belongs to THAT configuration and no "
          f"other.")
    return rows


# ---------------------------------------------------------------------------
# 6. the design - THE TILT LADDER
# ---------------------------------------------------------------------------

def stage_tilt(caps, gate, cache):
    keys = of_section(caps, "tilt")
    if not keys:
        return None
    keys.sort(key=SV.tilt_sort_key)
    print(f"\n{BOLD}{'=' * 74}{OFF}")
    print(f"{BOLD}  6. TILT LADDER{OFF}")
    print(f"{BOLD}{'=' * 74}{OFF}")
    for ln in (
        "Beamforming across the four capsules is closed with a number:",
        "-0.02 to +0.03 dB in the comb band. MOUNTING ANGLE is the only beam",
        "this device has, because the cone mouths become directive -",
        "Predicted - above roughly 4-5 kHz, which is Tier-3",
        "and Tier-4 territory. The prediction is therefore that the",
        f"{BV.HB_LO / 1000:.1f}-{BV.HB_HI / 1000:.1f} kHz column moves with "
        f"angle and the {BV.PRIO_LO:.0f}-{BV.PRIO_HI:.0f} Hz column does not.",
        "Both are printed, so the prediction can fail where somebody sees it.",
    ):
        print(f"  {DIM}{ln}{OFF}")

    ref = SV.TILT_REFERENCE if SV.TILT_REFERENCE in caps else None
    if ref is None:
        print(f"\n  {YELLOW}no '{SV.TILT_REFERENCE}' in this session, so "
              f"there is no 0 deg row to refer the\n  ladder to. Absolute "
              f"levels are printed and no delta is: a ladder without\n  its "
              f"own reference measures the site, not the mounting angle."
              f"{OFF}")
    if SV.TILT_CONTROL not in caps:
        print(f"\n  {YELLOW}no '{SV.TILT_CONTROL}' face-away control. The "
              f"ladder can then show a trend with\n  angle without ever "
              f"showing that the cones are directive at all.{OFF}")
    if len([k for k in keys if SV.tilt_angle(k) is not None]) < 2:
        print(f"\n  {YELLOW}fewer than two angles - this is a stage, not a "
              f"ladder.{OFF}")

    rows = []
    for k in keys:
        cap = caps[k]
        x = to_float(cap["audio"])
        freqs, psd = BV.welch_psd(x)
        r = {"stage": k, "deg": SV.tilt_angle(k),
             "seconds": cap["seconds"],
             "prio_db": BV.band_power_db(freqs, psd, BV.PRIO_LO, BV.PRIO_HI),
             "hb_db": BV.band_power_db(freqs, psd, BV.HB_LO, BV.HB_HI),
             "t3": cached(cache, cap, f"tilt-t3/{TAU3_SHIPPED}/100",
                          lambda x=x: replay_t3(x, TAU3_SHIPPED, 100.0)),
             "t4": cached(cache, cap, "tilt-t4/1", lambda x=x: replay_t4(x)),
             "v12": cached(cache, cap, "tilt-v1t2/1",
                           lambda x=x: replay_v1_t2(x))}
        rows.append(r)

    base = next((r for r in rows if r["stage"] == ref), None)

    print(f"\n  {BOLD}per-tier score against mounting angle{OFF}   "
          f"{DIM}scores are scores; dB columns are band levels{OFF}")
    print(f"    {'stage':<12}{'deg':>5}{'s':>6}"
          f"{'v1 pk':>8}{'T2 pk':>8}{'T3 W':>8}{'T4 W4':>8}"
          f"{'alert dB':>10}{'high dB':>9}")
    for r in rows:
        deg = "away" if r["deg"] is None else f"{r['deg']:.0f}"
        print(f"    {r['stage']:<12}{deg:>5}{r['seconds']:6.0f}"
              f"{r['v12']['v1_peak']:8.3f}"
              f"{_num(r['v12']['t2_peak']):8.3f}"
              f"{r['t3']['W_med']:8.2f}{r['t4']['W4_med']:8.2f}"
              f"{r['prio_db']:10.2f}{r['hb_db']:9.2f}")

    if base is not None:
        print(f"\n  {BOLD}the same table as a ladder{OFF}, against "
              f"'{ref}'   {DIM}every column is a DIFFERENCE{OFF}")
        print(f"    {'stage':<12}{'deg':>5}"
              f"{'v1 pk':>8}{'T2 pk':>8}{'T3 W':>8}{'T4 W4':>8}"
              f"{'alert dB':>10}{'high dB':>9}")
        for r in rows:
            deg = "away" if r["deg"] is None else f"{r['deg']:.0f}"
            d_t2 = _num(r["v12"]["t2_peak"]) - _num(base["v12"]["t2_peak"])
            print(f"    {r['stage']:<12}{deg:>5}"
                  f"{r['v12']['v1_peak'] - base['v12']['v1_peak']:+8.3f}"
                  f"{d_t2:+8.3f}"
                  f"{r['t3']['W_med'] - base['t3']['W_med']:+8.2f}"
                  f"{r['t4']['W4_med'] - base['t4']['W4_med']:+8.2f}"
                  f"{r['prio_db'] - base['prio_db']:+10.2f}"
                  f"{r['hb_db'] - base['hb_db']:+9.2f}")

    print(f"\n  {BOLD}Tier-3 latched fraction by angle{OFF}   "
          f"{DIM}Wilson 95%, because 'latched 0.46' on 13 updates and on "
          f"1300 are different claims{OFF}")
    for r in rows:
        lo, hi = r["t3"]["latched_ci"]
        deg = "away" if r["deg"] is None else f"{r['deg']:.0f} deg"
        print(f"    {r['stage']:<12}{deg:>8}  latched "
              f"{r['t3']['latched']:.3f} [{lo:.3f}-{hi:.3f}]   "
              f"events {r['t3']['events']}   updates {r['t3']['updates']}")

    print(f"\n  {BOLD}What this decides:{OFF} nothing, here. The mounting "
          f"recommendation - most likely a\n  compromise tilt splitting the "
          f"horizon and the sky, since the threat can arrive at\n  up to "
          f"~300 m altitude - is a planning decision (R5) made from this "
          f"table, and it\n  goes on the field card and in the mount "
          f"documentation as a MEASURED one.")
    return rows


# ---------------------------------------------------------------------------
# every capture that no section above printed
# ---------------------------------------------------------------------------

def report_dispositions(caps):
    """Names every capture no section printed, and says which kind of
    not-printed it is. Returns the analysed pile, so the caller can refuse a
    session that produced no analysis at all.

    THIS IS THE LOUD PART ON PURPOSE. Before it existed, a capture whose name
    matched none of five scattered `startswith` tests was dropped without a
    word: the card printed its headings, every section under them was empty or
    absent, and the session read as 'nothing happened' rather than 'nothing
    was looked at'. `rotor_4m_0` - the name the only real rotor audio this
    project owns is archived under - is exactly such a name.
    """
    analysed, aside, unknown = SV.split(sorted(caps))
    if not aside and not unknown:
        return analysed
    print(f"\n{BOLD}{'=' * 74}{OFF}")
    print(f"{BOLD}  CAPTURES NO SECTION ABOVE PRINTED{OFF}")
    print(f"{BOLD}{'=' * 74}{OFF}")
    for k in aside:
        cls = SV.stage_class(k) or "unclassified"
        sec = SV.section(k)
        why = ("recognised, and analysed by nothing on purpose"
               if sec else
               f"a {cls} stage to the adoption gate, and no section here "
               f"analyses it")
        print(f"  {YELLOW}{k:<22}{OFF} {caps[k]['seconds']:6.0f} s   {why}")
    for k in unknown:
        near = SV.nearest(k)
        hint = f"   did you mean '{near[0]}'?" if near else ""
        print(f"  {RED}{k:<22}{OFF} {caps[k]['seconds']:6.0f} s   NOT A STAGE "
              f"NAME THIS TOOL KNOWS{hint}")
    if unknown:
        print(f"\n  {DIM}Rename the capture, or add the name to "
              f"scripts/stage_vocab.py - which is the ONE\n  place that "
              f"decides, so the adoption gate and the capture tool learn it "
              f"too.{OFF}")
    return analysed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session", nargs="?", default=None)
    ap.add_argument("--min-quiet", type=float, default=MIN_QUIET_S)
    ap.add_argument("--no-cache", action="store_true")
    a = ap.parse_args()

    sess = Path(a.session) if a.session else PS.session_dir("field1")
    if not sess.exists():
        print(f"{RED}  no such session: {sess}{OFF}")
        return 2
    t0 = time.time()
    caps = load_session(sess)
    if not caps:
        print(f"{RED}  no *_raw.npz captures in {sess}{OFF}")
        return 2

    print(f"\n{BOLD}  FIELD VERDICT CARD{OFF}")
    print(f"  session {sess}")
    print(f"  {len(caps)} captures, "
          f"{sum(c['seconds'] for c in caps.values()) / 60:.1f} minutes of audio")
    print(f"  rules: field/DECISION_RULES_2026-08-21.md "
          f"{DIM}(pre-committed before any of this data existed){OFF}")

    cache = Cache(sess, enabled=not a.no_cache)
    gate = stage_gate(caps, a.min_quiet)
    stage_band(caps, gate)
    stage_quad(caps, gate, cache)
    stage_distance(caps, gate)
    stage_ambient(caps, gate)
    stage_windscreen(caps, gate, cache)
    stage_tilt(caps, gate, cache)
    analysed = report_dispositions(caps)

    # THE NEGATIVE-HOURS LINE. Printed here because
    # this is the one moment somebody is looking at a field day's numbers, and
    # the question "how much closer did today get Tier-4 to being certifiable"
    # has never had an answer in front of them. It is one line and it goes up.
    try:
        import negative_hours as NH
        tot = NH.totals()
        h4 = tot["T4"] / 3600.0
        print(f"\n{BOLD}  NEGATIVE HOURS{OFF}  "
              f"T4 {h4:.3f} h of {NH.TARGET_H['T4']:.1f} h  "
              f"{DIM}(zero events there bounds the rate at "
              f"{3.0 / h4:.1f}/h; the allowance is 0.40/h){OFF}"
              if h4 > 0 else
              f"\n{BOLD}  NEGATIVE HOURS{OFF}  T4 0.000 h - unbounded")
        print(f"  {DIM}full ledger: scripts/negative_hours.py . "
              f"Guarding overnight on the power bank is the cheapest way to "
              f"accrue them.{OFF}")
    except Exception as e:                                   # noqa: BLE001
        print(f"  {DIM}negative-hours ledger unavailable: {e}{OFF}")

    print(f"\n{BOLD}{'=' * 74}{OFF}")
    print(f"  done in {time.time() - t0:.1f} s")
    print(f"  {DIM}Nothing here changes a constant. R5: no field retuning - "
          f"data, then offline analysis, then the next planning session.{OFF}")
    print(f"{BOLD}{'=' * 74}{OFF}\n")

    # EXIT 3, NOT 0, WHEN NOTHING WAS ANALYSED. The session had captures - it
    # is not empty, so it is not a 2 - and no section could print one of them.
    # Every heading above then stands over nothing, which is the failure this
    # project has been bitten by once already: a green-looking report of a run
    # that measured nothing. `family_field_analysis.py` refuses the same way
    # with the same code.
    if not analysed:
        print(f"{RED}{BOLD}  REFUSED{OFF}{RED} - {len(caps)} capture(s), and "
              f"no section of this card could print a single one of them. "
              f"There is\n  no verdict here, and an empty card is not a "
              f"pass.{OFF}\n")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
