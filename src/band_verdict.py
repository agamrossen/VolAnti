"""
band_verdict.py - the band decision rule, fixed before the rig recordings were
made and applied here as written.

Why this is code and not a judgement call. The rig recordings answer one
question above all others: does a four-motor rig actually put a blade-passage
comb into the 200-800 Hz band the detector searches? The first real rotor
recording put +0.2 dB there and +12.6 dB above 3.2 kHz.

A question that important should not be settled by whoever looks at the
spectrum first, on a laptop, with the rig still spinning. So the rule was fixed
in advance and this module applies it. The report prints the verdict first and
the evidence second.

The instrument is floor-free, deliberately. Every statistic below is computed
from raw Welch PSDs and local-median prominences. It never touches the adaptive
floor, because an instrument that inherits the detector's floor reports the
floor rather than the signal - a planted 425 Hz comb was estimated at 1000 Hz
that way.

A correction to the rule as written, found by testing it
--------------------------------------------------------
The rule was fixed with absolute thresholds: a tooth prominence T_p >= 10 dB
inside 200-800 Hz confirms the band. Applied literally to the first real rotor
audio, it prints BAND CONFIRMED - the exact opposite of what that audio shows.

The reason is that a prominence measured on one capture cannot tell a rotor
tone from a room tone. Measured on those captures:

    quiet room   T_p = 21.3 dB at 309 Hz,  C_p = 45.2 dB
    rotor at 4 m T_p = 20.7 dB at 309 Hz,  C_p = 26.6 dB

The room's own mains/fan lines near 305-312 Hz are more prominent than anything
the rotor contributes, and the rule would have confirmed the band using them.

There is a second problem: the statistic maximises over ~1901 candidate
fundamentals and six harmonics, so its null value is not zero. Measured on pure
noise, C_p reaches 18-21 dB on an 8-second capture - above the 15 dB "BAND
SHIFTED" threshold, on noise.

The fix keeps the rule's intent. The high band is already measured against the
day's quiet baseline; the same baseline is applied to the other two statistics.
T_p and C_p are computed on the excess spectrum - the capture's PSD divided by
the day's quiet PSD - so they answer "what did the rig add", which is the
question the rule is asking. Room lines divide out. The absolute values are
still computed and reported as `T_p_abs` / `C_p_abs`, so nothing is hidden and
the literal rule can still be read off the report.

If no quiet baseline is supplied the module falls back to the absolute
statistics and sets `baseline_missing`, which the verdict prints as a warning,
because in that case the numbers mean something different.

A second correction: C_p needs an empirical null
------------------------------------------------
C_p maximises over ~1901 candidate fundamentals x 6 harmonics. That is 11,000
chances to find a coincidence, so its value under "nothing is there" is large
and it depends on the recording length and the room. Measured by splitting a
quiet capture in half and running one half against the other - literally "no
rig at all" through the same instrument:

    null  T_p =  4.1 dB       null  C_p = 32.8 dB at f0 1227 Hz
    rotor T_p =  9.0 dB       rotor C_p = 18.9 dB at f0 1229 Hz

The null C_p is nearly twice the fixed 15 dB threshold, and higher than the
rotor's own C_p at essentially the same frequency. A literal reading would
have printed BAND SHIFTED, at 1229 Hz, on noise.

T_p survives the same test: null 4.1 dB against a rotor's 9.0-10.6 dB, so the
10 dB confirm threshold is meaningful (if tight).

So `verdict()` accepts `null_T_p_db` / `null_C_p_db`, computed by
`split_half_null()` from the day's own quiet capture, and raises each threshold
to `max(fixed, null + margin)`. A recording session always has a quiet stage,
so this costs nothing and makes the rule mean what it was meant to. Both the
fixed and the effective thresholds are printed.
"""

import numpy as np

FS = 16000
WELCH_N_FFT = 32768          # 2 s at 16 kHz
LOCAL_HALF = 8
LOCAL_SKIP = 1
PRIO_LO, PRIO_HI = 200.0, 800.0
COMB_F0_LO, COMB_F0_HI = 100.0, 2000.0
COMB_N_HARM = 6
COMB_CAP_DB = 12.0
F_MAX_HARM = 7800.0
HB_LO, HB_HI = 3200.0, 7800.0

# Thresholds fixed before any recording was made; not tuned here.
T_P_CONFIRM_DB = 10.0        # a tooth this prominent in 200-800 confirms
C_P_SHIFTED_DB = 15.0        # a comb this prominent anywhere <= 2 kHz
HB_ELEVATION_DB = 10.0       # high band this far over quiet

# How far above the measured null a statistic must sit before it counts as
# evidence. Not a tuning knob: it is the margin by which a real effect has to
# beat "this room, this length of recording, by chance".
NULL_MARGIN_DB = 3.0


def welch_psd(x, fs=FS, n_fft=WELCH_N_FFT, skip_s=2.0):
    """Channel-averaged Welch PSD, 50% overlap, Hann. Skips the INMP441
    start-up transient."""
    X = np.atleast_2d(np.asarray(x, np.float64))
    if X.dtype != np.float64:
        X = X.astype(np.float64)
    X = X[:, int(skip_s * fs):]
    hop = n_fft // 2
    w = np.hanning(n_fft)
    acc, k = None, 0
    for i in range(0, X.shape[1] - n_fft + 1, hop):
        P = (np.abs(np.fft.rfft(X[:, i:i + n_fft] * w, axis=1)) ** 2
             ).mean(axis=0)
        acc = P if acc is None else acc + P
        k += 1
    freqs = np.fft.rfftfreq(n_fft, 1.0 / fs)
    if k == 0:
        return freqs, np.zeros(len(freqs))
    return freqs, acc / k


def prominence_at(freqs, psd, f, half=LOCAL_HALF, skip=LOCAL_SKIP):
    """dB over the local median, excluding the immediate neighbours."""
    df = freqs[1] - freqs[0]
    i = int(round(f / df))
    if i <= half or i >= len(psd) - half - 1:
        return float("nan")
    idx = np.arange(i - half, i + half + 1)
    idx = idx[np.abs(idx - i) > skip]
    ref = float(np.median(psd[idx]))
    return 10.0 * np.log10((psd[i] + 1e-30) / (ref + 1e-30))


def best_inband_tooth(freqs, psd, lo=PRIO_LO, hi=PRIO_HI):
    """T_p: the most prominent single tooth inside the priority band."""
    df = freqs[1] - freqs[0]
    i0, i1 = int(lo / df), min(len(psd) - LOCAL_HALF - 2, int(hi / df))
    best, bf = -1e9, float("nan")
    for i in range(max(i0, LOCAL_HALF + 1), i1 + 1):
        p = prominence_at(freqs, psd, i * df)
        if np.isfinite(p) and p > best:
            best, bf = p, i * df
    return float(best), float(bf)


def best_comb(freqs, psd, lo=COMB_F0_LO, hi=COMB_F0_HI, step=1.0,
              n_harm=COMB_N_HARM, cap=COMB_CAP_DB, f_max=F_MAX_HARM):
    """C_p: the best 6-harmonic comb prominence over f0 in 100-2000 Hz.

    Sum of min(prominence, cap) over harmonics at or below f_max. The cap is
    what stops one strong line reading as a comb - the same discipline the
    envelope instrument uses.
    """
    best = {"C_p": -1e9, "f0": float("nan"), "per_harmonic_db": [],
            "teeth_hz": []}
    for f0 in np.arange(lo, hi + 1e-9, step):
        s, per, teeth = 0.0, [], []
        for k in range(1, n_harm + 1):
            f = k * f0
            if f > f_max or f > freqs[-1]:
                break
            p = prominence_at(freqs, psd, f)
            if not np.isfinite(p):
                break
            s += min(p, cap)
            per.append(p)
            teeth.append(f)
        if len(per) >= 3 and s > best["C_p"]:
            best = {"C_p": float(s), "f0": float(f0),
                    "per_harmonic_db": [float(v) for v in per],
                    "teeth_hz": [float(v) for v in teeth]}
    return best


def band_power_db(freqs, psd, lo, hi):
    m = (freqs >= lo) & (freqs < hi)
    return 10.0 * np.log10(float(psd[m].mean()) + 1e-30)


def split_half_null(quiet_x, fs=FS):
    """The instrument's own null, from the day's quiet capture.

    Runs the first half of the quiet recording against the second half - "no
    rig at all" through exactly the same statistic - and returns the T_p and
    C_p it produces by chance. Anything the rig scores has to beat this.
    """
    X = np.atleast_2d(np.asarray(quiet_x, np.float64))
    n = X.shape[1] // 2
    if n < WELCH_N_FFT * 2:
        return {"T_p_db": float("nan"), "C_p_db": float("nan"),
                "usable": False}
    freqs, a = welch_psd(X[:, :n], fs)
    _, b = welch_psd(X[:, n:], fs)
    ex = a / np.maximum(b, 1e-30)
    t, tf = best_inband_tooth(freqs, ex)
    c = best_comb(freqs, ex)
    return {"T_p_db": float(t), "T_p_hz": float(tf),
            "C_p_db": float(c["C_p"]), "C_p_f0_hz": float(c["f0"]),
            "usable": True}


def measure(x, fs=FS, quiet_psd=None):
    """Everything the band rule needs, from one capture.

    With `quiet_psd` supplied, T_p and C_p are computed on the excess spectrum
    (this capture / the quiet baseline), which is what makes them describe the
    rig rather than the room. See the correction note at the top of this file.
    """
    freqs, psd = welch_psd(x, fs)
    # absolute, i.e. the rule exactly as written - kept and reported
    t_abs, tf_abs = best_inband_tooth(freqs, psd)
    c_abs = best_comb(freqs, psd)
    hb = band_power_db(freqs, psd, HB_LO, HB_HI)

    if quiet_psd is not None and len(quiet_psd) == len(psd):
        excess = psd / np.maximum(quiet_psd, 1e-30)
        t_p, t_f = best_inband_tooth(freqs, excess)
        comb = best_comb(freqs, excess)
        baseline_missing = False
    else:
        excess = psd
        t_p, t_f, comb = t_abs, tf_abs, c_abs
        baseline_missing = True

    out = {"T_p_db": t_p, "T_p_hz": t_f, "C_p_db": comb["C_p"],
           "C_p_f0_hz": comb["f0"],
           "C_p_per_harmonic_db": comb["per_harmonic_db"],
           "C_p_teeth_hz": comb["teeth_hz"],
           "T_p_abs_db": t_abs, "T_p_abs_hz": tf_abs,
           "C_p_abs_db": c_abs["C_p"], "C_p_abs_f0_hz": c_abs["f0"],
           "baseline_missing": baseline_missing,
           "hb_db": hb, "hb_vs_quiet_db": float("nan"),
           "strong_teeth_in_prio": 0, "strong_teeth_out_of_prio": 0}
    if quiet_psd is not None:
        out["hb_vs_quiet_db"] = hb - band_power_db(freqs, quiet_psd,
                                                   HB_LO, HB_HI)
    for f, p in zip(comb["teeth_hz"], comb["per_harmonic_db"]):
        if p >= T_P_CONFIRM_DB:
            if PRIO_LO <= f <= PRIO_HI:
                out["strong_teeth_in_prio"] += 1
            else:
                out["strong_teeth_out_of_prio"] += 1
    out["_freqs"], out["_psd"], out["_excess"] = freqs, psd, excess
    return out


# ---------------------------------------------------------------------------
# the rule, as written
# ---------------------------------------------------------------------------

def verdict(solo_measurements, null=None):
    """Apply the band rule.

    `solo_measurements` is a list of dicts from measure(), one per per-motor
    solo capture at the hover-band throttle step, each carrying a 'motor' key.

    Returns a dict whose 'verdict' is one of BAND CONFIRMED / BAND SHIFTED /
    NO USABLE COMB / MIXED, with the numbers that produced it. There is no
    silent fallthrough: MIXED prints the deltas to the nearest verdict.
    """
    n = len(solo_measurements)
    # Effective thresholds: never below what this room produces by chance.
    t_thr, c_thr = T_P_CONFIRM_DB, C_P_SHIFTED_DB
    if null and null.get("usable"):
        if np.isfinite(null.get("T_p_db", float("nan"))):
            t_thr = max(t_thr, null["T_p_db"] + NULL_MARGIN_DB)
        if np.isfinite(null.get("C_p_db", float("nan"))):
            c_thr = max(c_thr, null["C_p_db"] + NULL_MARGIN_DB)
    motors_confirming = sorted({m.get("motor") for m in solo_measurements
                                if m["T_p_db"] >= t_thr})
    n_conf = len(motors_confirming)
    best_C = max((m["C_p_db"] for m in solo_measurements), default=float("nan"))
    best_C_m = max(solo_measurements, key=lambda m: m["C_p_db"],
                   default=None) if solo_measurements else None
    hb = [m["hb_vs_quiet_db"] for m in solo_measurements
          if np.isfinite(m.get("hb_vs_quiet_db", float("nan")))]
    hb_med = float(np.median(hb)) if hb else float("nan")

    R = {"n_captures": n, "motors_confirming": motors_confirming,
         "baseline_missing": any(m.get("baseline_missing")
                                 for m in solo_measurements),
         "T_p_abs_max_db": max((m.get("T_p_abs_db", float("nan"))
                                for m in solo_measurements), default=float("nan")),
         "C_p_abs_max_db": max((m.get("C_p_abs_db", float("nan"))
                                for m in solo_measurements), default=float("nan")),
         "n_motors_confirming": n_conf,
         "best_T_p_db": max((m["T_p_db"] for m in solo_measurements),
                            default=float("nan")),
         "best_C_p_db": best_C,
         "best_C_p_f0_hz": best_C_m["C_p_f0_hz"] if best_C_m else float("nan"),
         "hb_vs_quiet_db_median": hb_med,
         "thresholds": {"T_p_confirm_db": T_P_CONFIRM_DB,
                        "C_p_shifted_db": C_P_SHIFTED_DB,
                        "hb_elevation_db": HB_ELEVATION_DB},
         "effective_thresholds": {"T_p_db": float(t_thr),
                                  "C_p_db": float(c_thr)},
         "null": null or {"usable": False}}

    if n_conf >= 3:
        R["verdict"] = "BAND CONFIRMED"
        R["action"] = ("v1/Tier-2 as designed. Tier-2's 2.0x range factor is "
                       "the story. Proceed to the distance ladder.")
        return R

    teeth_out = (best_C_m["strong_teeth_out_of_prio"] if best_C_m else 0)
    if np.isfinite(best_C) and best_C >= c_thr and teeth_out > 0:
        lo = min(best_C_m["C_p_teeth_hz"]) if best_C_m["C_p_teeth_hz"] else 200.0
        hi = max(best_C_m["C_p_teeth_hz"]) if best_C_m["C_p_teeth_hz"] else 800.0
        R["verdict"] = "BAND SHIFTED"
        R["suggested_t2_lo_hz"] = float(max(70.0, min(lo, 1900.0)))
        R["suggested_t2_hi_hz"] = float(min(2000.0, max(hi, R["suggested_t2_lo_hz"] + 100)))
        R["action"] = (
            f"SAME DAY: re-run the key stages with the Tier-2 band set to the "
            f"measured comb - `H <thr1> <thr2> a 0 0 "
            f"{R['suggested_t2_lo_hz']:.0f} {R['suggested_t2_hi_hz']:.0f}` "
            f"(runtime, no rebuild). Re-deriving the band itself is a "
            f"separate decision.")
        return R

    if (not np.isfinite(best_C) or best_C < c_thr) and \
            np.isfinite(hb_med) and hb_med >= HB_ELEVATION_DB:
        R["verdict"] = "NO USABLE COMB"
        R["action"] = (
            "The envelope/wash path becomes the priority. The comb tiers "
            "remain for the rest of the fleet, but a blade-passage comb "
            "detector is the wrong primary instrument for this airframe.")
        return R

    R["verdict"] = "MIXED"
    R["action"] = "No rule matched cleanly. Deltas to each verdict below."
    R["deltas"] = {
        "to BAND CONFIRMED": f"{3 - n_conf} more motors needed at "
                             f"T_p >= {t_thr:.1f} dB "
                             f"(best T_p seen {R['best_T_p_db']:.1f} dB)",
        "to BAND SHIFTED": f"C_p is {best_C:.1f} dB, needs "
                           f"{c_thr:.1f} dB with teeth outside "
                           f"200-800 Hz (teeth outside: {teeth_out})",
        "to NO USABLE COMB": f"high band is {hb_med:+.1f} dB over quiet, "
                             f"needs >= {HB_ELEVATION_DB:.0f} dB",
    }
    return R


def format_verdict(R):
    """The verdict block, printed first in the report."""
    L = []
    L.append("=" * 78)
    L.append(f"BAND VERDICT:  {R['verdict']}")
    L.append("=" * 78)
    L.append("")
    L.append(R["action"])
    L.append("")
    L.append(f"  captures judged            {R['n_captures']}")
    et = R.get("effective_thresholds", {})
    L.append(f"  motors with an in-band tooth >= "
             f"{et.get('T_p_db', R['thresholds']['T_p_confirm_db']):.1f} dB   "
             f"{R['n_motors_confirming']}  {R['motors_confirming']}")
    L.append(f"  best in-band tooth T_p     {R['best_T_p_db']:.1f} dB")
    L.append(f"  best comb C_p              {R['best_C_p_db']:.1f} dB "
             f"at f0 = {R['best_C_p_f0_hz']:.0f} Hz")
    L.append(f"  high band vs quiet         "
             f"{R['hb_vs_quiet_db_median']:+.1f} dB")
    L.append(f"  (absolute, the rule as literally written: "
             f"T_p {R.get('T_p_abs_max_db', float('nan')):.1f} dB, "
             f"C_p {R.get('C_p_abs_max_db', float('nan')):.1f} dB)")
    nl = R.get("null", {})
    if nl.get("usable"):
        L.append(f"  measured null (quiet split-half): T_p "
                 f"{nl['T_p_db']:.1f} dB, C_p {nl['C_p_db']:.1f} dB")
        L.append(f"  effective thresholds (fixed vs null+3 dB): "
                 f"T_p {et.get('T_p_db', float('nan')):.1f}, "
                 f"C_p {et.get('C_p_db', float('nan')):.1f} dB")
    else:
        L.append("  !! NO NULL MEASURED. C_p's null in a measured quiet")
        L.append("     room was 32.8 dB against a fixed threshold of")
        L.append("     15 - a BAND SHIFTED verdict without a null is noise.")
    if R.get("baseline_missing"):
        L.append("")
        L.append("  !! NO QUIET BASELINE SUPPLIED. T_p and C_p are ABSOLUTE,")
        L.append("     so a room tone counts as a rotor tone. On the first")
        L.append("     rotor recordings that difference flipped the verdict.")
        L.append("     Capture a quiet stage and re-run before deciding.")
    if "deltas" in R:
        L.append("")
        L.append("  distance to each verdict:")
        for k, v in R["deltas"].items():
            L.append(f"    {k:22s} {v}")
    if "suggested_t2_lo_hz" in R:
        L.append("")
        L.append(f"  suggested Tier-2 band      "
                 f"{R['suggested_t2_lo_hz']:.0f}-"
                 f"{R['suggested_t2_hi_hz']:.0f} Hz")
    return "\n".join(L)
