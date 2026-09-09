#!/usr/bin/env python
"""
rig_report.py - the morning after. One markdown file per rig session.

    python scripts/rig_report.py captures/<session>

Point it at a session directory and it replays EVERY raw capture in it offline
through the Python reference: v1 alone, v1 + Tier-2, and each combiner variant.
The output is one markdown report written into the session directory, next to
the audio it describes.

WHY THIS EXISTS, AND WHY IT IS OFFLINE. A rig day produces evidence faster than
anyone can read it, and the reading is where the value is. Every number here
comes from replaying archived samples through `src/detector.py`'s own
front_end / combiner / back_end - never from the device's live verdict - so a
question that occurs at midnight can be asked of a capture taken at noon.
That is the entire reason every stage archives raw PCM.

THE FIRST TABLE IS THE ONE THAT MATTERS. "Where is the energy?" comes before
"did it detect?", because the analysis found a real 7" rotor putting
+0.2 dB into the 200-800 Hz priority band the detector searches and +12.6 dB
into 3.2-7.8 kHz. If that repeats on the four-motor rig, no amount of detector
tuning is the answer and the band is.

NOTHING HERE DECIDES ANYTHING. It reports. Changing a constant, a band or the
shipped combiner is a planning decision made from this evidence, not by this
script.
"""
import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "src"))

import band_verdict as BV                                       # noqa: E402
import combiners as CX                                          # noqa: E402
import envelope_wash as EW                                      # noqa: E402
import homespeech as HS                                         # noqa: E402
import detector as D                                            # noqa: E402
import detector_t2 as T2                                        # noqa: E402
import operating_point as op                                    # noqa: E402
import profile_store as PS                                      # noqa: E402

FS = 16000
N_FFT = 2048
HOP = 512

# The bands the first table splits energy into. The boundaries are the ones
# that matter to this detector: the priority band it searches, the rest of the
# alert band, and the region above it where the comb's upper teeth live.
BANDS = ((100, 200), (200, 400), (400, 800), (800, 1600),
         (1600, 3200), (3200, 7800))

CX_VARIANTS = (("a", None), ("b", None), ("c", 1500.0), ("c", 2800.0))


# ---------------------------------------------------------------------------
# loading
# ---------------------------------------------------------------------------

def load_session(sess: Path):
    """Every *_raw.npz in the directory, newest manifest entry first."""
    caps = {}
    for p in sorted(sess.glob("*_raw.npz")):
        try:
            z = np.load(p)
            caps[p.name[:-len("_raw.npz")]] = {
                "path": p, "audio": z["audio"],
                "fs": int(z["fs"]) if "fs" in z.files else FS,
                "report": (json.loads(str(z["report"]))
                           if "report" in z.files else {})}
        except Exception as e:                              # noqa: BLE001
            print(f"  ! {p.name}: {type(e).__name__}: {e}")
    return caps


def to_float(audio):
    a = np.asarray(audio)
    if a.dtype == np.float32:
        return a
    return np.stack([np.asarray(ch, np.int16).astype(np.float32) / 32767.0
                     for ch in a])


# ---------------------------------------------------------------------------
# 1. where is the energy?
# ---------------------------------------------------------------------------

def band_levels(x, skip_s=2.0, stride=4):
    """Median-over-time band levels, dB. Median rather than mean because the
    first ~2 s of every INMP441 capture is a startup transient 25 dB above the
    room, and one mean would be the transient."""
    w = np.hanning(N_FFT).astype(np.float64)
    f = np.arange(N_FFT // 2 + 1) * (FS / N_FFT)
    n = x.shape[1]
    rows = []
    for i in range(int(skip_s * FS), n - N_FFT, HOP * stride):
        S = np.abs(np.fft.rfft(x[:, i:i + N_FFT].astype(np.float64) * w,
                               axis=1)).mean(axis=0)
        rows.append(S)
    if not rows:
        return {}
    M = np.median(np.array(rows), axis=0)
    out = {}
    for lo, hi in BANDS:
        m = (f >= lo) & (f < hi)
        out[f"{lo}-{hi}"] = 20.0 * np.log10(
            np.sqrt((M[m] ** 2).mean()) + 1e-12)
    return out


def energy_table(caps, ref="quiet"):
    base = None
    if ref in caps:
        base = band_levels(to_float(caps[ref]["audio"]))
    rows = []
    for name, c in caps.items():
        lv = band_levels(to_float(c["audio"]))
        rows.append({"capture": name, "levels": lv,
                     "vs_ref": ({k: lv[k] - base[k] for k in lv}
                                if base else None)})
    return rows, ref if base else None


# ---------------------------------------------------------------------------
# 2/3. detection, per capture, per tier, per combiner
# ---------------------------------------------------------------------------

def detect_row(x, cfg, thr, t2cfg, combiner=None):
    r = T2.analyze_quad_t2(x, cfg, t2cfg, thr_ref=thr, combiner=combiner)
    ev1, _ = D.track_frames(r, thr, cfg)
    ev2 = r["t2_events"]
    return {
        "v1_events": len(ev1),
        "v1_first_s": float(ev1[0]["t_on"]) if ev1 else None,
        "v1_med": float(np.median(r["score"])),
        "v1_p99": float(np.percentile(r["score"], 99)),
        "v1_max": float(r["score"].max()),
        "v1_f0_med": float(np.median(r["f0"])),
        "t2_events": len(ev2),
        "t2_first_s": float(ev2[0]["t_on"]) if ev2 else None,
        "t2_med": float(np.median(r["t2"]["score2"])),
        "t2_p99": float(np.percentile(r["t2"]["score2"], 99)),
        "t2_max": float(r["t2"]["score2"].max()),
        "t2_f0_med": float(np.median(r["t2"]["f02"])),
        "kappa_med": float(np.nanmedian(r["t2"]["kappa"])),
        "kappa_p10": float(np.nanpercentile(r["t2"]["kappa"], 10)),
        "seconds": x.shape[1] / FS,
    }


def tooth_snr_row(x, combiner, f0):
    import eval_combiners as EC
    return EC.tooth_snr_table(x, combiner, f0)


def solo_motor_of(name):
    """`solo_M3_hover` -> 3. Anything else -> None."""
    import re
    m = re.match(r"solo_M(\d)_", name)
    return int(m.group(1)) if m else None


def analyse(caps, t2cfg, do_combiners=True, seconds=None, do_wash=True):
    cfg, thr = op.preset_config("HIGH_ALERT")
    quiet_psd = None
    if "quiet" in caps:
        _, quiet_psd = BV.welch_psd(to_float(caps["quiet"]["audio"]))
    out = {}
    for name, c in caps.items():
        x = to_float(c["audio"])
        if seconds:
            x = x[:, :int(seconds * FS)]
        t0 = time.time()
        row = {"base": detect_row(x, cfg, thr, t2cfg)}
        if do_wash:
            w = EW.analyse(x)
            row["wash"] = {k: w[k] for k in
                           ("r_best", "W_best", "W_p90", "r_median",
                            "stable_run_s", "L_hb_max", "L_hb_p90",
                            "seconds")}
            row["wash"]["per_harmonic_db"] = [float(v)
                                              for v in w["per_harmonic_db"]]
            row["wash"]["bands_db"] = {k: float(v)
                                       for k, v in w["bands_db"].items()}
        bm = BV.measure(x, quiet_psd=quiet_psd)
        row["band"] = {k: v for k, v in bm.items() if not k.startswith("_")}
        row["band"]["motor"] = solo_motor_of(name)
        if do_combiners:
            import eval_combiners as EC
            f0, med, _ = EC.estimate_f0_track(x, cfg)
            row["f0_track_median"] = med
            row["combiners"] = {}
            for cx, split in CX_VARIANTS:
                cf = CX.make_combiner(cx, f_split=split or 1500.0)
                lab = CX.label(cx, split)
                row["combiners"][lab] = {
                    **detect_row(x, cfg, thr, t2cfg, combiner=cf),
                    "tooth_snr_db": tooth_snr_row(x, cf, f0)}
        row["secs_to_analyse"] = time.time() - t0
        out[name] = row
        print(f"  {name}: {row['base']['seconds']:.0f} s analysed in "
              f"{row['secs_to_analyse']:.0f} s", flush=True)
    return out


# ---------------------------------------------------------------------------
# markdown
# ---------------------------------------------------------------------------

def md_table(head, rows):
    out = ["| " + " | ".join(head) + " |",
           "|" + "|".join("---" for _ in head) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(v) for v in r) + " |")
    return "\n".join(out)


def fmt(v, n=2, dash="—"):
    return dash if v is None else f"{v:.{n}f}"


def compute_verdict(caps, res, hover_key="hover"):
    """The pre-committed D-A rule, on the per-motor solo captures at the
    hover-band throttle step, with the day's own quiet capture supplying both
    the baseline and the empirical null."""
    null = None
    if "quiet" in caps:
        null = BV.split_half_null(to_float(caps["quiet"]["audio"]))
    solo = []
    for name, r in res.items():
        b = r.get("band", {})
        if b.get("motor") is None:
            continue
        if hover_key and hover_key not in name:
            continue
        solo.append(dict(b))
    if not solo:
        return None, null
    return BV.verdict(solo, null=null), null


def build_report(sess, caps, energy, ref, res, t2cfg, manifest,
                 verdict=None, null=None, homespeech=None):
    L = []
    A = L.append
    A(f"# Rig session report — `{sess.name}`")
    A("")
    if verdict is not None:
        A("```")
        A(BV.format_verdict(verdict))
        A("```")
        A("")
        A("*The verdict is printed before the evidence on purpose: planning "
          "pre-committed this rule so the decision would not be made by "
          "whoever looked at the spectrum first.*")
        A("")
    elif any(r.get("band", {}).get("motor") for r in res.values()):
        A("> **No per-motor solo capture at the hover throttle step was found, "
          "so the D-A band verdict could not be computed.** That stage is the "
          "day's most important measurement.")
        A("")
    else:
        A("> **No per-motor solo captures in this session** — the D-A band "
          "verdict needs them (`solo_M<n>_hover`).")
        A("")
    A(f"Generated {datetime.now():%Y-%m-%d %H:%M} by `scripts/rig_report.py`.")
    A("")
    A("Every number below comes from **replaying the archived samples offline** "
      "through `src/detector.py`'s own front_end / combiner / back_end — never "
      "from the device's live verdict. That is what the raw archive is for.")
    A("")
    A(f"**Tier-2 constants used:** tau2 {t2cfg.tau2:.2f}, "
      f"tau2_rise {t2cfg.tau2_rise_s:.0f} s, N2 {t2cfg.n2}, M2 {t2cfg.m2}, "
      f"band {t2cfg.f2_lo:.0f}–{t2cfg.f2_hi:.0f} Hz. "
      f"**v1** at the sealed 1.700.")
    A("")
    A(f"**Captures:** {len(caps)} raw files, "
      f"{sum(r['base']['seconds'] for r in res.values()) / 60.0:.1f} minutes "
      f"of four-channel audio.")
    A("")

    # ---- 1. energy ------------------------------------------------------
    A("## 1. WHERE IS THE ENERGY?")
    A("")
    A("This table comes first on purpose. The detector searches **200–800 Hz** "
      "for the blade-pass comb. In the only real rotor recording in "
      "existence put **+0.2 dB** into that band and **+12.6 dB** above 3.2 kHz "
      "— i.e. the band may be aimed at the wrong place. If the four-motor rig "
      "repeats that, it is the finding of the day and no detector tuning is "
      "the answer.")
    A("")
    A("Median-over-time band level, dB"
      + (f", **relative to `{ref}`**." if ref else " (absolute — no `quiet` "
                                                    "capture to compare to)."))
    A("")
    head = ["capture"] + [f"{lo}–{hi} Hz" for lo, hi in BANDS]
    rows = []
    for e in energy:
        vals = e["vs_ref"] if e["vs_ref"] else e["levels"]
        rows.append([f"`{e['capture']}`"]
                    + [f"{vals[f'{lo}-{hi}']:+.1f}" for lo, hi in BANDS])
    A(md_table(head, rows))
    A("")

    # ---- 2. detection ---------------------------------------------------
    A("## 2. DETECTION — v1 alone vs v1 + Tier-2")
    A("")
    A("`first` is seconds from the start of the capture to the first alert of "
      "that tier. An **alert** is what the operator dismisses; the device fires "
      "on the OR of the two tiers.")
    A("")
    head = ["capture", "s", "v1 ev", "v1 first", "v1 med", "v1 p99",
            "T2 ev", "T2 first", "T2 med", "T2 p99", "T2 f0", "kappa"]
    rows = []
    for name, r in res.items():
        b = r["base"]
        rows.append([f"`{name}`", f"{b['seconds']:.0f}",
                     b["v1_events"], fmt(b["v1_first_s"], 1),
                     fmt(b["v1_med"]), fmt(b["v1_p99"]),
                     b["t2_events"], fmt(b["t2_first_s"], 1),
                     fmt(b["t2_med"]), fmt(b["t2_p99"]),
                     f"{b['t2_f0_med']:.0f}", fmt(b["kappa_med"], 3)])
    A(md_table(head, rows))
    A("")
    A("### What to read here")
    A("")
    A("- **`quad_hover_steady`** is the case Tier-2 exists for. v1 is predicted "
      "to go quiet within ~15 s as its 6 s floor absorbs the comb; Tier-2's "
      "floor is an order of magnitude slower.")
    A("- **`talk_near_rig`** is the hard negative gate. Speech IS a harmonic "
      "comb, and in the reference captures it out-scored the real rotor. Any T2 event here "
      "at the calibrated tau2 is a headline **negative** result.")
    A("- **`quiet`** should show zero of everything. If it does not, nothing "
      "else in this report can be read.")
    A("")

    # ---- 3. combiners ---------------------------------------------------
    if any("combiners" in r for r in res.values()):
        A("## 3. COMBINER VARIANTS")
        A("")
        A("The shipped default is **CX-A** (unweighted complex sum) and stays "
          "CX-A. Changing it is a planning decision that needs this evidence "
          "first. Half a wavelength on the 60.96 mm baseline is 2813 Hz and "
          "the comb score reaches 7800 Hz, so the top third of the scored "
          "teeth are already past the point where a complex sum can cancel "
          "what it meant to add.")
        A("")
        head = ["capture", "combiner", "v1 ev", "v1 med", "T2 ev", "T2 med",
                "tooth SNR k=1", "k=4", "k=8", "k=12"]
        rows = []
        for name, r in res.items():
            for lab, v in r.get("combiners", {}).items():
                ts = v["tooth_snr_db"]

                def tk(k):
                    return ("—" if k > len(ts) or not np.isfinite(ts[k - 1])
                            else f"{ts[k - 1]:.1f}")
                rows.append([f"`{name}`", lab, v["v1_events"],
                             fmt(v["v1_med"]), v["t2_events"],
                             fmt(v["t2_med"]),
                             tk(1), tk(4), tk(8), tk(12)])
        A(md_table(head, rows))
        A("")
        A("Tooth SNR is **floor-independent** — a harmonic measured against its "
          "own neighbourhood in the same frame — so it compares combiners "
          "without the adaptive floor's seconds-long memory entering the "
          "comparison. It is also blind to a flat gain, which is what lets a "
          "4× complex sum be compared with a 2× power sum at all.")
        A("")

    # ---- 3b. the envelope / wash instrument -----------------------------
    if any("wash" in r for r in res.values()):
        A("## 3b. THE ENVELOPE (WASH) INSTRUMENT")
        A("")
        A("A rotor's high-frequency broadband noise is generated once per "
          "blade per revolution, so its ENVELOPE should be periodic at the "
          "shaft or blade-pass rate even when the spectrum carries no comb. "
          "On the reference captures this separated the rotor from quiet and "
          "from speech by a factor of thirty, at both distances, with the two "
          "captures agreeing on the rate to 3.7%.")
        A("")
        A("**MEASUREMENT ONLY.** Nothing here gates or fires anything.")
        A("")
        head = ["capture", "r_best Hz", "W", "W p90", "stable s",
                "L_hb max", "L_hb p90", "harmonics dB (r,2r,3r,4r)"]
        rows = []
        for name, r in res.items():
            w = r.get("wash")
            if not w:
                continue
            hh = "  ".join("—" if not np.isfinite(v) else f"{v:.1f}"
                           for v in w["per_harmonic_db"])
            rows.append([f"`{name}`", f"{w['r_best']:.0f}",
                         f"{w['W_best']:.1f}", f"{w['W_p90']:.1f}",
                         f"{w['stable_run_s']:.1f}",
                         f"{w['L_hb_max']:.1f}", f"{w['L_hb_p90']:.1f}", hh])
        A(md_table(head, rows))
        A("")
        A("A wash score above ~20 with a stable run of seconds is a rotor. "
          "Quiet and speech scored 1.1 and 0.6 on the reference captures. "
          "Note that speech is LOUDER than the rotor in this band — the level "
          "statistic alone would false-alarm on it, and only the periodicity "
          "separates them.")
        A("")
        A("For a per-motor solo series, `r_best` against throttle step IS the "
          "rotation-rate ladder, and it is an independent check on any "
          "blade-pass frequency read off the spectrum.")
        A("")

    # ---- 4. coherence ---------------------------------------------------
    A("## 4. COHERENCE (κ) — telemetry only, never gated on")
    A("")
    A("κ = |Σ X_c|² / (4 Σ |X_c|²): 1.0 coherent and aligned, ~0.25 "
      "independent, measured over the Tier-2 winner's teeth below 1.5 kHz.")
    A("")
    A("The discriminator κ was hoped to buy is **wind**: pressure fluctuation "
      "at a capsule is locally generated and largely uncorrelated between "
      "ports even 40 mm apart, while an acoustic wave is correlated. On the "
      "reference indoor captures κ was 0.92–0.98 below 800 Hz for *everything* "
      "— quiet, speech and rotor alike — because indoors they are all acoustic. "
      "**The fan-wind stage is the only capture that tests the premise.**")
    A("")
    head = ["capture", "κ median", "κ p10"]
    rows = [[f"`{n}`", fmt(r['base']['kappa_med'], 3),
             fmt(r['base']['kappa_p10'], 3)] for n, r in res.items()]
    A(md_table(head, rows))
    A("")

    # ---- 4b. recorded speech --------------------------------------------
    if homespeech:
        rows_hs, pooled = homespeech
        A("## 4b. RECORDED SPEECH (D-B evidence)")
        A("")
        A("Real speech through the real detector. The synthetic speech proxy "
          "in the Tier-2 corpus fires v1 at 33 events/hour and on its own "
          "pushes v1 past its budget; the question is whether real speech "
          "does anything like that.")
        A("")
        head = ["file", "s", "v1 ev", "T2 ev", "score med", "p99", "max",
                "% above thr", "longest chain"]
        rows = []
        for r in rows_hs:
            if "error" in r:
                rows.append([f"`{r['file']}`", "—", "—", "—", "—", "—", "—",
                             "—", f"ERROR: {r['error']}"])
                continue
            rows.append([f"`{r['file']}`", f"{r['seconds']:.0f}",
                         r["v1_events"], r["t2_events"],
                         f"{r['score_med']:.2f}", f"{r['score_p99']:.2f}",
                         f"{r['score_max']:.2f}",
                         f"{100 * r['frac_above_thr']:.1f}",
                         r["longest_chain"]])
        A(md_table(head, rows))
        A("")
        A(f"**Pooled: {pooled['v1_events']} v1 events and "
          f"{pooled['t2_events']} Tier-2 events in "
          f"{pooled['seconds']:.0f} s.** With zero events the 95% one-sided "
          f"upper bound on the rate is "
          f"**{pooled['v1_upper_95_per_hour']:.1f} events/hour** — that is "
          f"what this much audio can bound, and nothing tighter.")
        A("")
        for r in rows_hs:
            if "margin" not in r:
                continue
            A(f"Threshold margin for `{r['file']}` — how far the operating "
              f"point would have to fall before this recording fires:")
            A("")
            A(md_table(["threshold", "events", "longest chain"],
                       [[f"{m['thr']:.2f}", m["events"], m["longest_chain"]]
                        for m in r["margin"]]))
            A("")
            break
        A("**Caveat, stated:** a MacBook microphone is not an INMP441 behind "
          "a cone. Absolute levels and the high-frequency response differ. "
          "What transfers is the mechanism — voiced speech is a harmonic comb "
          "with a 110–230 Hz fundamental and ten to twenty live harmonics, "
          "which is a property of human phonation rather than of the "
          "microphone.")
        A("")

    # ---- 5. provenance --------------------------------------------------
    A("## 5. PROVENANCE")
    A("")
    if manifest:
        A(f"`manifest.jsonl` holds {len(manifest)} stage records. Raw-audio "
          f"loss accounting, per capture:")
        A("")
        head = ["capture", "seconds kept", "records", "dropped", "runs"]
        rows = []
        for name, c in caps.items():
            rep = c.get("report", {})
            rows.append([f"`{name}`",
                         f"{c['audio'].shape[1] / FS:.1f}",
                         rep.get("n_records", "—"),
                         rep.get("dropped", "—"), rep.get("runs", "—")])
        A(md_table(head, rows))
    else:
        A("No `manifest.jsonl` in this session directory.")
    A("")
    A("---")
    A("")
    A("**No absolute range claim appears in this report and none should be "
      "derived from it.** Distances recorded during the session are relative, "
      "for this rig on this day.")
    A("")
    A("**Scope:** acoustic detection and local alerting only.")
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("session")
    ap.add_argument("--t2-config", default=None)
    ap.add_argument("--no-combiners", action="store_true",
                    help="skip the combiner sweep (much faster)")
    ap.add_argument("--seconds", type=float, default=None,
                    help="analyse only the first N seconds of each capture")
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-wash", action="store_true",
                    help="skip the envelope instrument")
    ap.add_argument("--homespeech", default=None, metavar="DIR",
                    help="a directory of recorded speech to replay through "
                         "the reference (D-B evidence)")
    ap.add_argument("--hover-key", default="hover",
                    help="substring identifying the hover-throttle solo "
                         "captures for the D-A verdict (default 'hover')")
    a = ap.parse_args(argv)

    sess = Path(a.session)
    if not sess.is_dir():
        print(f"not a directory: {sess}")
        return 1
    doc = json.loads(Path(a.t2_config
                          or (ROOT / "data" / "t2_config.json")).read_text())
    t2cfg = T2.T2Config.from_dict(doc["t2"])

    print(f"  session {sess}")
    caps = load_session(sess)
    if not caps:
        print("  no *_raw.npz captures found. A session with only traces "
              "cannot be re-analysed — that is exactly the failure the raw "
              "archive exists to prevent.")
        return 1
    print(f"  {len(caps)} raw captures")

    energy, ref = energy_table(caps)
    res = analyse(caps, t2cfg, do_combiners=not a.no_combiners,
                  seconds=a.seconds, do_wash=not a.no_wash)
    manifest = PS.manifest_read(sess)

    verdict, null = compute_verdict(caps, res, hover_key=a.hover_key)
    if verdict is not None:
        print()
        print(BV.format_verdict(verdict))
        print()

    hs = None
    if a.homespeech:
        print(f"  replaying recorded speech from {a.homespeech}")
        hs = HS.analyse_dir(a.homespeech, t2cfg=t2cfg)
        print(f"    {hs[1]['v1_events']} v1 events in "
              f"{hs[1]['seconds']:.0f} s "
              f"(95% upper bound {hs[1]['v1_upper_95_per_hour']:.1f}/h)")

    md = build_report(sess, caps, energy, ref, res, t2cfg, manifest,
                      verdict=verdict, null=null, homespeech=hs)
    # --out redirects BOTH files. The capture directory is a
    # sacrosanct input - four files of the only real rotor audio in existence -
    # and a report tool must be able to read it without writing a byte into it.
    out = Path(a.out) if a.out else (sess / "RIG_REPORT.md")
    out.write_text(md)
    js = out.with_suffix(".json")
    js.write_text(json.dumps(
        {"t2_config": t2cfg.to_dict(), "energy": energy, "results": res,
         "band_verdict": verdict, "null": null,
         "homespeech": (hs[1] if hs else None)},
        indent=2, default=str))
    print(f"\n  wrote {out}")
    print(f"  wrote {js}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
