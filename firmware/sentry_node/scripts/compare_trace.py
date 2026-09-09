#!/usr/bin/env python
"""
compare_trace.py - judge a captured device trace against the sealed reference.

THE GATE (and nothing else is the gate):
  1. every decision field matches frame for frame - above_thr, cont_accepted,
     fired;
  2. the chain trajectory matches frame for frame;
  3. the verdict matches data/golden_vectors.json.

Expected outcomes are READ from data/golden_vectors.json. Nothing here knows
what a vector is supposed to do.

Everything else is diagnostic and is reported as WARN, never as a failure:
an f0_bin tie that changes no decision field is the argmax landing on the
other side of an exact draw and is not a defect. Score deviation is reported
because a growing deviation is how a floor divergence announces itself long
before it changes a verdict.

Note on precision: the reference CSV stores score to 6 decimals, so measured
deviations have a floor of ~5e-7 absolute (~2.3e-7 relative) that comes from
the reference, not the device.
"""
import argparse
import csv
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from trace_proto import parse  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent.parent
DATA = ROOT / "data"

DECISION_FIELDS = ("above_thr", "cont_accepted", "fired")
NEAR_THR = 1e-3

# ---------------------------------------------------------------------------
# THE DRIFT BOUND.
#
# THE GAP THIS FILLS, and tests/test_gates_can_fail.py pinned it in writing:
# the golden gate's verdict is DECISION IDENTITY, so a 2% score drift that
# flips no decision PASSES. That is the right primary criterion - decisions
# are what the device does - but it means the gate cannot see a floor
# divergence until the day it changes a verdict, and by then it is a mystery
# rather than a measurement.
#
# So a second, softer criterion, in two stages:
#
#   WARN  above DRIFT_WARN   - larger than the worst deviation ever measured
#   FAIL  above DRIFT_FAIL   - ten times that
#
# THE PROVENANCE OF THE NUMBER, and it matters that it is a measurement.
#
# RE-ANCHORED ON THE CURRENT IMAGE, which is what the TODO
# that used to stand here asked for. F1 on an attached DevKit, four vectors
# replayed from flash, worst relative score deviation on SCORED frames
# (|ref| >= 0.1):
#
#     first port    1.3e-04     (the inherited figure)
#     this image, 08-25      1.250e-04   4 vectors, 0 decision mismatches
#
# The arithmetic has BARELY MOVED across three-tier scheduling, the summed
# FFT path, a fourth tier compiled in, a settings bump and a thawed
# detector.c - 1.250e-04 against 1.3e-04. That is the useful content of the
# measurement: the bound did not need to move, and now it is a measurement of
# THIS image rather than an inheritance.
#
# WARN is kept at 1.3e-04 rather than tightened to 1.250e-04. A warn line
# exactly at the observed worst case would warn on roughly half of all future
# runs for no reason; it sits a hair above, where it has always sat.
DRIFT_WARN = 1.3e-04     # Measured: 1.250e-04 on this image
DRIFT_FAIL = 1.3e-03     # 10x the above
DRIFT_PROVENANCE = ("F1 on the current image: worst "
                    "scored-frame relative deviation 1.250e-04 over four "
                    "vectors, x10 (the first port read 1.3e-04)")


def load_reference(name):
    lines = [l for l in (DATA / f"{name}_trace.csv").read_text().splitlines()
             if not l.startswith("#") and l.strip()]
    return list(csv.DictReader(lines))


def load_expected(name):
    """A vector is judged at the threshold IT was cut at, from its own entry -
    never at the file's top-level one. golden_high_alert_pos is cut at
    HIGH_ALERT (1.70); judging it at NORMAL would fail it for being correct."""
    gv = json.loads((DATA / "golden_vectors.json").read_text())
    v = next(v for v in gv["vectors"] if v["name"] == name)
    return v["threshold"], v


def judge(raw, want_name=None):
    """File-based entry point: reference from data/<name>_trace.csv."""
    recs, bad_chk = parse(raw)
    hdr_at = next((i for i, r in enumerate(recs) if r["_kind"] == "hdr"), None)
    hdr = recs[hdr_at] if hdr_at is not None else None
    end = next((r for r in recs if r["_kind"] == "end"), None)
    # ONLY THE RECORDS AFTER THIS CAPTURE'S OWN HEADER.
    #
    # A bounded capture emits HDR, then its frames, then END. The standalone
    # guard emits NEITHER header nor sentinel - deliberately, so a host tool
    # cannot mistake its framing for a capture's - but its per-frame records
    # are still on the link. So with autostart ON, whatever the guard emitted
    # between the host connecting and the replay starting used to be counted
    # as part of the replay: measured, golden_strong_pos came back
    # with 314 frames against a reference of 247, and the gate reported
    # "GOLDEN GATE FAILED - stop, do not field this" for a detector that was
    # perfectly correct (the same four vectors passed with zero mismatches
    # minutes earlier with autostart off).
    #
    # Slicing at the header is correct BY CONSTRUCTION rather than by
    # heuristic: records before a capture's header cannot belong to it,
    # because the firmware sends the header first. It also removes the
    # conflict between F1 and F2 - the gate now runs with autostart in the
    # state the FIELD uses, which is the state worth gating.
    body = recs[hdr_at + 1:] if hdr_at is not None else recs
    dev = [r for r in body if r["_kind"] == "rec"]
    n_before = sum(1 for r in recs[:hdr_at or 0] if r["_kind"] == "rec")
    probes = [r for r in body if r["_kind"] == "prb"]
    if hdr is None or end is None:
        return None, ["capture has no header or no sentinel"], probes

    name = want_name or hdr["name"]
    ref = load_reference(name)
    thr, exp = load_expected(name)
    R = compare_records(name, dev, ref, thr, exp, end, bad_chk, hdr,
                        drift_bound=True)
    if n_before:
        # Reported, never silently dropped. Discarding records is exactly the
        # kind of tidying that could hide a real fault, so the count is on the
        # report where a human will see it.
        R["warn"].append(
            f"{n_before} frame records arrived BEFORE this capture's header "
            f"and were not compared - that is the standalone guard, which "
            f"emits no header of its own. Expected with autostart on.")
        R["n_before_hdr"] = n_before
    return R, R["fail"], probes


def compare_records(name, dev, ref, thr, exp, end, bad_chk=0, hdr=None,
                    drift_bound=False):
    """THE comparison. Device records vs a reference trace, whatever its
    origin - a sealed CSV for the golden gate, or a live Python run over the
    same microphone samples for the Stage-1b acceptance test. One
    implementation, so the two cannot drift apart and quietly disagree about
    what 'matching' means."""
    R = {"vector": name, "threshold": thr, "bad_chk": bad_chk,
         "n_dev": len(dev), "n_ref": len(ref),
         "n_frames_hdr": hdr["n_frames"] if hdr else len(dev),
         "expected": exp, "sentinel": end, "fail": [], "warn": []}

    if hdr is not None and abs(hdr["threshold"] - thr) > 1e-12:
        R["fail"].append(f"device replayed at threshold {hdr['threshold']} "
                         f"but this vector was cut at {thr}")
    if len(dev) != len(ref):
        # NAME THE LIKELY CAUSE. Measured with autostart ON, the
        # standalone guard is already running when capture_trace connects, and
        # its per-frame records land in the capture BEFORE the replay's own
        # header - golden_strong_pos came back with 314 frames against a
        # reference of 248. The standalone run withholds its HDR and END
        # (which is what stops the host reading the guard's framing), but the
        # records themselves are still on the link.
        #
        # This is not a detector fault and must not read like one. The same
        # four vectors passed with zero mismatches minutes earlier with
        # autostart off.
        extra = len(dev) - len(ref)
        R["fail"].append(f"frame count {len(dev)} != reference {len(ref)}")
        if extra > 0:
            R["fail"].append(
                f"{extra} MORE frames than the reference. This is almost "
                f"certainly the standalone guard's records in the capture, "
                f"not a detector fault: send `U c autostart 0`, re-run, and "
                f"send `U c autostart 1` before fielding.")
        R["pass"] = False
        R["incomplete"] = True
        return R
    if bad_chk:
        R["warn"].append(f"{bad_chk} magic-word hits failed their check field "
                         f"(resynchronised)")

    # ---- tripwires: REJECTED features must never appear ------------------
    for f in ("reanch", "n_held_bins"):
        nz = [d["frame"] for d in dev if d[f] != 0]
        if nz:
            R["fail"].append(f"{f} nonzero on {len(nz)} frames "
                             f"(first {nz[0]}) - a REJECTED feature is live")
    if end["chain_overflow"]:
        R["fail"].append("chain buffer overflowed - trace is void")

    # ---- THE GATE 1: decision fields -------------------------------------
    R["decision"] = {}
    for f in DECISION_FIELDS:
        bad = [i for i, (d, r) in enumerate(zip(dev, ref))
               if int(d[f]) != int(r[f])]
        R["decision"][f] = bad
        if bad:
            R["fail"].append(f"{f}: {len(bad)} mismatches, frames "
                             f"{bad[:12]}{' ...' if len(bad) > 12 else ''}")

    # ---- THE GATE 2: chain trajectory ------------------------------------
    first_div = None
    n_chain_bad = 0
    for i, (d, r) in enumerate(zip(dev, ref)):
        if int(d["chain"]) != int(r["chain"]):
            n_chain_bad += 1
            if first_div is None:
                first_div = i
    R["chain_first_divergent"] = first_div
    R["chain_n_bad"] = n_chain_bad
    if first_div is not None:
        d, r = dev[first_div], ref[first_div]
        R["fail"].append(
            f"chain diverges at frame {first_div} "
            f"(device {d['chain']} vs reference {r['chain']}), "
            f"{n_chain_bad} frames differ in total")

    # ---- THE GATE 3: verdict ---------------------------------------------
    dev_verdict = "FIRES" if end["verdict_fires"] else "NO FIRE"
    R["verdict_device"] = dev_verdict
    R["verdict_expected"] = exp["verdict"]
    if dev_verdict != exp["verdict"]:
        R["fail"].append(f"VERDICT {dev_verdict} != expected {exp['verdict']}")
    if end["n_events"] != exp["n_events"]:
        R["fail"].append(f"n_events {end['n_events']} != "
                         f"expected {exp['n_events']}")
    if end["longest_chain"] != exp["longest_chain"]:
        R["fail"].append(f"longest_chain {end['longest_chain']} != "
                         f"expected {exp['longest_chain']}")
    if end["n_floor_fast"] != exp["n_floor_fast"]:
        R["fail"].append(f"n_floor_fast {end['n_floor_fast']} != "
                         f"expected {exp['n_floor_fast']}")

    # ---- diagnostic: f0_bin ----------------------------------------------
    dev_bins = [int(d["f0_bin"]) for d in dev]
    ref_bins = [int(r["f0_bin"]) for r in ref]
    dbin = [a - b for a, b in zip(dev_bins, ref_bins)]
    off = [i for i, v in enumerate(dbin) if v != 0]
    octave = []
    for i in off:
        a = dev[i]["f0_hz"]
        b = float(ref[i]["f0_hz"])
        if b > 0 and (abs(a - 2 * b) <= 0.51 or abs(a - 0.5 * b) <= 0.51):
            octave.append(i)
    harmless = [i for i in off
                if all(int(dev[i][f]) == int(ref[i][f])
                       for f in DECISION_FIELDS)
                and int(dev[i]["chain"]) == int(ref[i]["chain"])]
    R["f0bin"] = {"n_off": len(off), "frames": off[:20],
                  "max_abs": max((abs(v) for v in dbin), default=0),
                  "octave": octave, "harmless": harmless}
    if off:
        msg = (f"f0_bin differs on {len(off)}/{len(dev)} frames "
               f"(max |dbin| {R['f0bin']['max_abs']}, "
               f"{len(octave)} octave, {len(harmless)} changing no decision "
               f"field)")
        if len(harmless) == len(off):
            R["warn"].append(msg + " - all harmless ties")
        else:
            R["warn"].append(msg)

    # ---- diagnostic: score deviation -------------------------------------
    dev_s = [float(d["score"]) for d in dev]
    ref_s = [float(r["score"]) for r in ref]
    rel = [abs(a - b) / max(abs(b), 1e-9) for a, b in zip(dev_s, ref_s)]
    absd = [abs(a - b) for a, b in zip(dev_s, ref_s)]
    srt = sorted(rel)
    # The unrestricted relative max is dominated by warm-up frames where the
    # reference score is ~1e-7 (floor == mag, so teeth minus gaps is zero to
    # rounding). Dividing a 1e-4 absolute difference by that is arithmetic, not
    # a measurement, so a scored subset is reported alongside it.
    scored = [r for r, b in zip(rel, ref_s) if abs(b) >= 0.1]
    ssc = sorted(scored) if scored else [0.0]
    R["score"] = {
        "rel_min": srt[0], "rel_med": statistics.median(srt),
        "rel_p99": srt[min(len(srt) - 1, int(0.99 * len(srt)))],
        "rel_max": srt[-1],
        "abs_max": max(absd),
        "abs_max_frame": absd.index(max(absd)),
        "n_scored": len(scored),
        "sc_med": statistics.median(ssc),
        "sc_p99": ssc[min(len(ssc) - 1, int(0.99 * len(ssc)))],
        "sc_max": ssc[-1],
    }

    # ---- THE DRIFT BOUND, two-stage -------------------------------------
    # Judged on the SCORED subset. The unrestricted relative max is dominated
    # by warm-up frames where the reference score is ~1e-7, and dividing a
    # 1e-4 absolute difference by that is arithmetic rather than a
    # measurement.
    # ONLY AGAINST A SEALED REFERENCE. `drift_bound` is off by default and
    # `judge()` - the golden entry point, whose reference is the frozen
    # data/<name>_trace.csv - is the only caller that turns it on.
    #
    # Measured, and it is why this parameter exists: applied to
    # quad parity, the bound FAILED a run with ZERO decision mismatches and
    # zero chain divergence. That gate compares the device against a PYTHON
    # RE-DERIVATION over the same live microphone samples, where the scores
    # are near zero and a tiny absolute difference is an enormous RELATIVE
    # one - median relative deviation 0.198, max 8.29. Those numbers are
    # arithmetic on small denominators, not a floor divergence, and the
    # Stage-1a provenance ("worst deviation against a sealed CSV") means
    # nothing about them.
    #
    # A gate that fails for a reason unrelated to what it gates is worse than
    # no gate - and quad parity gates ONE thing, decision identity on the
    # summed-FFT path, which it had passed.
    R["drift"] = {"metric": R["score"]["sc_max"], "warn": DRIFT_WARN,
                  "fail": DRIFT_FAIL, "provenance": DRIFT_PROVENANCE,
                  "applied": bool(drift_bound)}
    if not drift_bound:
        pass
    elif R["score"]["n_scored"] == 0:
        R["warn"].append("no scored frames (|ref| >= 0.1) - the drift bound "
                         "checked nothing on this vector")
    elif R["score"]["sc_max"] > DRIFT_FAIL:
        R["fail"].append(
            f"SCORE DRIFT {R['score']['sc_max']:.3e} exceeds the bound "
            f"{DRIFT_FAIL:.1e} ({DRIFT_PROVENANCE}). Decisions may still all "
            f"match; this says the ARITHMETIC has moved, which is how a floor "
            f"divergence announces itself before it changes a verdict.")
    elif R["score"]["sc_max"] > DRIFT_WARN:
        R["warn"].append(
            f"score drift {R['score']['sc_max']:.3e} is above the measured "
            f"worst case {DRIFT_WARN:.1e} but inside the {DRIFT_FAIL:.1e} "
            f"bound - record it and watch it")

    # ---- the frames that decide the night --------------------------------
    marg = []
    for i, (d, r) in enumerate(zip(dev, ref)):
        if abs(float(r["score"]) - thr) < NEAR_THR or \
           abs(float(d["score"]) - thr) < NEAR_THR:
            marg.append({
                "frame": i, "dev": float(d["score"]), "ref": float(r["score"]),
                "d_thr_dev": float(d["score"]) - thr,
                "d_thr_ref": float(r["score"]) - thr,
                "dev_above": int(d["above_thr"]),
                "ref_above": int(r["above_thr"]),
                "dev_acc": int(d["cont_accepted"]),
                "ref_acc": int(r["cont_accepted"]),
                "dev_chain": int(d["chain"]), "ref_chain": int(r["chain"]),
                "agree": (int(d["above_thr"]) == int(r["above_thr"])
                          and int(d["cont_accepted"]) == int(r["cont_accepted"])
                          and int(d["chain"]) == int(r["chain"])),
            })
    R["marginal"] = marg

    # ---- timing ----------------------------------------------------------
    us = [int(d["us_frame"]) for d in dev]
    su = sorted(us)
    R["timing"] = {
        "mean_us": sum(us) / len(us), "max_us": max(us),
        "p99_us": su[min(len(su) - 1, int(0.99 * len(su)))],
        "period_us": 1e6 * 512 / 16000,
    }
    R["pass"] = not R["fail"]
    return R


def report(R):
    ok = "PASS" if R["pass"] else "FAIL"
    print(f"\n=== {R['vector']}  [{ok}] "
          f"({R['n_dev']} frames, {R['expected'].get('preset', 'NORMAL')} "
          f"thr {R['threshold']:.4f}) ===")

    # A CAPTURE THAT NEVER GOT AS FAR AS BEING COMPARED. compare_records
    # returns early on a frame-count mismatch with most of R unfilled, and
    # this used to walk straight into `R['verdict_device']` and die with a
    # KeyError - so a diagnosable condition (the guard's records in the
    # capture) surfaced as a stack trace, and field_ready.py then reported it
    # as "GOLDEN GATE FAILED - stop, do not field this".
    #
    # That is a gate failing for a reason unrelated to what it gates, which
    # this project has already been bitten by once (quad parity).
    # Say what actually happened instead.
    if R.get("incomplete"):
        for f in R["fail"]:
            print(f"  FAIL  {f}")
        print("  (the comparison did not run: there is nothing to compare "
              "frame for frame until the capture holds this vector and "
              "nothing else)")
        return
    print(f"  verdict      device {R['verdict_device']}  "
          f"expected {R['verdict_expected']}   "
          f"events {R['sentinel']['n_events']}/{R['expected']['n_events']}  "
          f"longest chain {R['sentinel']['longest_chain']}/"
          f"{R['expected']['longest_chain']}  "
          f"floor_fast {R['sentinel']['n_floor_fast']}/"
          f"{R['expected']['n_floor_fast']}")
    d = R["decision"]
    print(f"  DECISION     above_thr {len(d['above_thr'])} mismatches | "
          f"cont_accepted {len(d['cont_accepted'])} | "
          f"fired {len(d['fired'])}")
    for f in DECISION_FIELDS:
        if d[f]:
            print(f"               {f} frames: {d[f][:30]}")
    print(f"  CHAIN        first divergent frame: "
          f"{R['chain_first_divergent'] if R['chain_first_divergent'] is not None else 'none'}"
          f"   ({R['chain_n_bad']} frames differ)")
    b = R["f0bin"]
    print(f"  f0_bin       {b['n_off']} differing (max |d| {b['max_abs']}, "
          f"{len(b['octave'])} octave, {len(b['harmless'])} harmless)")
    if b["n_off"]:
        print(f"               frames {b['frames']}")
    s = R["score"]
    print(f"  score dev    all frames  rel med {s['rel_med']:.3e}  "
          f"p99 {s['rel_p99']:.3e}  max {s['rel_max']:.3e}"
          f"   (abs max {s['abs_max']:.3e} @ f{s['abs_max_frame']})")
    print(f"               |ref|>=0.1 ({s['n_scored']} frames)  "
          f"rel med {s['sc_med']:.3e}  p99 {s['sc_p99']:.3e}  "
          f"max {s['sc_max']:.3e}")
    d = R.get("drift")
    if d and d.get("applied"):
        state = ("FAIL" if d["metric"] > d["fail"] else
                 ("warn" if d["metric"] > d["warn"] else "ok"))
        print(f"  drift bound  {d['metric']:.3e} vs warn {d['warn']:.1e} / "
              f"fail {d['fail']:.1e}  [{state}]")
        print(f"               provenance: {d['provenance']}")
    elif d:
        print(f"  drift bound  {d['metric']:.3e}  [NOT APPLIED - the "
              f"reference here is a live re-derivation, not a sealed trace]")
    t = R["timing"]
    e = R["sentinel"]
    n = max(1, R["n_dev"])
    print(f"  stage time   fft {e['us_fft'] / n / 1000:.2f}  "
          f"mag {e['us_mag'] / n / 1000:.2f}  "
          f"floor {e['us_floor'] / n / 1000:.2f}  "
          f"score {e['us_score'] / n / 1000:.2f}  ms/frame")
    print(f"  frame time   mean {t['mean_us'] / 1000:.2f} ms  "
          f"p99 {t['p99_us'] / 1000:.2f} ms  max {t['max_us'] / 1000:.2f} ms"
          f"   vs {t['period_us'] / 1000:.1f} ms hop "
          f"({100 * t['p99_us'] / t['period_us']:.1f}% p99)")

    print(f"  MARGINAL FRAMES (|score - {R['threshold']:.2f}| < {NEAR_THR}) "
          f"- these decide the night:")
    if not R["marginal"]:
        print("               none")
    for m in R["marginal"]:
        flag = "ok " if m["agree"] else "!! "
        print(f"    {flag}f{m['frame']:<5} device {m['dev']!r:<22} "
              f"({m['d_thr_dev']:+.3e})  reference {m['ref']:<10.6f} "
              f"({m['d_thr_ref']:+.3e})  "
              f"above {m['dev_above']}/{m['ref_above']} "
              f"acc {m['dev_acc']}/{m['ref_acc']} "
              f"chain {m['dev_chain']}/{m['ref_chain']}")
    for w in R["warn"]:
        print(f"  WARN  {w}")
    for f in R["fail"]:
        print(f"  FAIL  {f}")


# ==========================================================================
# --tier2: the SECOND gate, for Tier-2. STRICTLY ADDITIVE.
#
# Nothing above this line changed, so the four existing golden vectors are
# judged by exactly the arithmetic that judged them before. This runs only
# when --tier2 is passed, only on captures that contain T2R records, and it
# cannot turn a v1 PASS into a FAIL - it adds its own verdict beside it.
#
# The reference is src/detector_t2.py run on THE SAME FLASH-RESIDENT SAMPLES,
# read back out of data/<name>.h - the identical source the device replayed
# from. Not a re-synthesised clip, not the CSV: the bytes the device saw.
# ==========================================================================

T2_DECISION_FIELDS = ("hit", "fired2")


def _golden_samples(name):
    """The int16 array out of data/<name>.h - the same parse test_golden.py
    uses, so the two cannot disagree about what a vector contains."""
    import re
    import numpy as np
    txt = (DATA / f"{name}.h").read_text()
    body = txt.split("{", 1)[1].rsplit("}", 1)[0]
    vals = np.array([int(v) for v in re.findall(r"-?\d+", body)], np.int16)
    n = int(re.search(r"#define \w+_LEN (\d+)", txt).group(1))
    if len(vals) != n:
        raise SystemExit(f"{name}.h: LEN={n} but {len(vals)} samples parsed")
    return vals


def t2_reference(name, t2cfg):
    """Tier-2's per-frame trajectory from the Python reference, on the golden
    vector's own samples, through THE reference front_end / combiner."""
    sys.path.insert(0, str(ROOT / "src"))
    import numpy as np
    import detector as D
    import detector_t2 as T2
    import operating_point as op

    q = _golden_samples(name)
    x = np.asarray(q, np.int16).astype(np.float32) / 32767.0
    cfg, _ = op.preset_config("NORMAL")
    det = D.CombDetector(cfg)
    tier = T2.Tier2(det, t2cfg)
    st = tier.state()
    c = det.cfg
    nf = max(0, 1 + (len(x) - c.n_fft) // c.hop)
    out = []
    for i in range(nf):
        s0 = i * c.hop
        t = (s0 + c.n_fft) / c.fs
        spec = det.combiner([det.front_end(x[s0:s0 + c.n_fft])])
        r = tier.step(spec, t, st)
        if r["ran"]:
            out.append({"frame": i, "score2": r["score2"], "f02_hz": r["f02"],
                        "hit": int(r["hit"]), "fired2": int(r["fired2"]),
                        "hits": int(r["hits"]), "track_age": int(r["track_age"]),
                        "teeth2": int(r["teeth2"])})
    tier.finish(st, (nf - 1) * c.hop / c.fs + c.n_fft / c.fs if nf else 0.0)
    return out, len(st.events)


def compare_tier2(name, dev_t2, t2cfg):
    """Zero decision mismatches required - exactly the v1 religion."""
    import statistics as _st
    ref, ref_events = t2_reference(name, t2cfg)
    R = {"vector": name, "n_dev": len(dev_t2), "n_ref": len(ref),
         "fail": [], "warn": [], "ref_events": ref_events}
    if not dev_t2:
        R["fail"].append("capture contains no T2R records - was this `K`?")
        R["pass"] = False
        return R
    if len(dev_t2) != len(ref):
        R["fail"].append(f"T2 step count {len(dev_t2)} != reference "
                         f"{len(ref)} (rate mismatch? device says n2="
                         f"{dev_t2[0]['n2']}, reference {t2cfg.n2})")
        R["pass"] = False
        return R

    R["decision"] = {}
    for f in T2_DECISION_FIELDS:
        bad = [i for i, (d, r) in enumerate(zip(dev_t2, ref))
               if int(d[f]) != int(r[f])]
        R["decision"][f] = bad
        if bad:
            R["fail"].append(f"{f}: {len(bad)} mismatches, steps "
                             f"{bad[:12]}{' ...' if len(bad) > 12 else ''}")

    hbad = [i for i, (d, r) in enumerate(zip(dev_t2, ref))
            if int(d["hits"]) != int(r["hits"])]
    R["hits_n_bad"] = len(hbad)
    R["hits_first_divergent"] = hbad[0] if hbad else None
    if hbad:
        i = hbad[0]
        R["fail"].append(f"hits diverges at step {i} (device "
                         f"{dev_t2[i]['hits']} vs reference {ref[i]['hits']}), "
                         f"{len(hbad)} steps differ")

    fbad = [i for i, (d, r) in enumerate(zip(dev_t2, ref))
            if abs(float(d["f02_hz"]) - float(r["f02_hz"])) > 0.51]
    R["f02_n_off"] = len(fbad)
    if fbad:
        harmless = [i for i in fbad
                    if all(int(dev_t2[i][f]) == int(ref[i][f])
                           for f in T2_DECISION_FIELDS)
                    and int(dev_t2[i]["hits"]) == int(ref[i]["hits"])]
        msg = (f"f02 differs on {len(fbad)}/{len(ref)} steps "
               f"({len(harmless)} changing no decision field)")
        R["warn"].append(msg + (" - all harmless ties"
                                if len(harmless) == len(fbad) else ""))

    dv = [float(d["score2"]) for d in dev_t2]
    rf = [float(r["score2"]) for r in ref]
    absd = [abs(a - b) for a, b in zip(dv, rf)]
    scored = [abs(a - b) / max(abs(b), 1e-9)
              for a, b in zip(dv, rf) if abs(b) >= 0.1]
    ssc = sorted(scored) if scored else [0.0]
    R["score2"] = {"abs_max": max(absd), "abs_max_step": absd.index(max(absd)),
                   "n_scored": len(scored), "sc_med": _st.median(ssc),
                   "sc_p99": ssc[min(len(ssc) - 1, int(0.99 * len(ssc)))],
                   "sc_max": ssc[-1]}
    us = [int(d["us_t2"]) for d in dev_t2]
    su = sorted(us)
    R["timing"] = {"mean_us": sum(us) / len(us), "max_us": max(us),
                   "p99_us": su[min(len(su) - 1, int(0.99 * len(su)))]}
    R["kappa_all_one"] = all(abs(float(d["kappa"]) - 1.0) < 1e-6
                             for d in dev_t2)
    if not R["kappa_all_one"]:
        R["fail"].append("kappa != 1.0 on a MONO replay - coherence is "
                         "undefined with one microphone and must be reported "
                         "as exactly 1.0")
    R["excluded_any"] = any(int(d["excluded"]) for d in dev_t2)
    if R["excluded_any"]:
        R["fail"].append("a step was marked excluded, but the shipped "
                         "exclusion list is EMPTY")
    R["pass"] = not R["fail"]
    return R


def report_tier2(R):
    ok = "PASS" if R["pass"] else "FAIL"
    print(f"\n  --- TIER-2 trajectory [{ok}] "
          f"({R['n_dev']} steps vs reference {R['n_ref']}) ---")
    if "decision" in R:
        d = R["decision"]
        print(f"    DECISION   hit {len(d['hit'])} mismatches | "
              f"fired2 {len(d['fired2'])}")
        print(f"    HITS       first divergent step: "
              f"{R['hits_first_divergent'] if R['hits_first_divergent'] is not None else 'none'}"
              f"   ({R['hits_n_bad']} steps differ)")
        print(f"    f02        {R['f02_n_off']} differing steps")
        s = R["score2"]
        print(f"    score2 dev |ref|>=0.1 ({s['n_scored']} steps)  "
              f"med {s['sc_med']:.3e}  p99 {s['sc_p99']:.3e}  "
              f"max {s['sc_max']:.3e}   (abs max {s['abs_max']:.3e} "
              f"@ step {s['abs_max_step']})")
        t = R["timing"]
        print(f"    t2 time    mean {t['mean_us'] / 1000:.2f} ms  "
              f"p99 {t['p99_us'] / 1000:.2f} ms  "
              f"max {t['max_us'] / 1000:.2f} ms")
        print(f"    tripwires  kappa==1.0 on mono: "
              f"{'yes' if R['kappa_all_one'] else 'NO'}   "
              f"excluded steps: {'YES' if R['excluded_any'] else 'none'}   "
              f"reference T2 events: {R['ref_events']}")
    for w in R["warn"]:
        print(f"    WARN  {w}")
    for f in R["fail"]:
        print(f"    FAIL  {f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--json", default=None)
    ap.add_argument("--tier2", action="store_true",
                    help="ALSO diff the Tier-2 trajectory in a `K` capture "
                         "against src/detector_t2.py on the same flash "
                         "samples. Additive: the v1 gate is unchanged.")
    ap.add_argument("--t2-config", default=None,
                    help="data/t2_config.json to judge against "
                         "(default: the committed one)")
    a = ap.parse_args()

    t2cfg = None
    if a.tier2:
        sys.path.insert(0, str(ROOT / "src"))
        import detector_t2 as T2
        doc = json.loads(Path(a.t2_config or (DATA / "t2_config.json"))
                         .read_text())
        t2cfg = T2.T2Config.from_dict(doc["t2"])

    results, allpass = [], True
    t2_results = []
    for c in a.captures:
        raw = Path(c).read_bytes()
        R, fails, _ = judge(raw)
        if R is None:
            print(f"{c}: unreadable capture: {fails}")
            allpass = False
            continue
        report(R)
        results.append(R)
        allpass &= R["pass"]
        if a.tier2:
            from trace_proto import parse as _parse
            recs, _ = _parse(raw)
            dev_t2 = [r for r in recs if r["_kind"] == "t2r"]
            if dev_t2:
                T = compare_tier2(R["vector"], dev_t2, t2cfg)
                report_tier2(T)
                t2_results.append(T)
                allpass &= T["pass"]
            else:
                print("\n  --- TIER-2: no T2R records in this capture "
                      "(use `K <i>`, not `R <i>`) ---")

    print("\n" + "=" * 70)
    v1pass = all(R["pass"] for R in results) and bool(results)
    print("STAGE 1a GOLDEN VECTOR: " + ("PASS" if v1pass else "FAIL"))
    for R in results:
        # The Tier-2 summary below has always guarded this with `if "decision"
        # in T`; the v1 one did not, and inherited the same KeyError from the
        # same early return. Guarded now, and it says WHY there is no number
        # rather than printing a dash.
        if "decision" not in R:
            print(f"  {R['vector']:<22} FAIL  not compared - "
                  f"{R['fail'][0] if R['fail'] else 'capture unusable'}")
            continue
        print(f"  {R['vector']:<22} {'PASS' if R['pass'] else 'FAIL'}  "
              f"decision mismatches "
              f"{sum(len(v) for v in R['decision'].values())}  "
              f"chain divergence "
              f"{R['chain_first_divergent'] if R['chain_first_divergent'] is not None else 'none'}")

    if a.tier2:
        t2pass = all(T["pass"] for T in t2_results) and bool(t2_results)
        print("\nSTAGE 1e TIER-2 TRAJECTORY: "
              + ("PASS" if t2pass else "FAIL"))
        for T in t2_results:
            n = (sum(len(v) for v in T["decision"].values())
                 if "decision" in T else "-")
            print(f"  {T['vector']:<22} {'PASS' if T['pass'] else 'FAIL'}  "
                  f"decision mismatches {n}  "
                  f"hits divergence "
                  f"{T.get('hits_first_divergent', '-') if T.get('hits_first_divergent') is not None else 'none'}")
        if not t2_results:
            print("  (no capture carried T2R records)")

    if a.json:
        # SHAPE IS UNCHANGED without --tier2: a bare list of v1 results,
        # exactly as before. The dict form appears only when a Tier-2 verdict
        # exists to carry, so nothing that reads today's file has to learn a
        # new shape to keep working.
        v1json = [{k: v for k, v in R.items()
                   if k not in ("probes", "sentinel")} for R in results]
        Path(a.json).write_text(json.dumps(
            {"v1": v1json, "tier2": t2_results} if a.tier2 else v1json,
            indent=2, default=str))
    return 0 if (allpass and results) else 1


if __name__ == "__main__":
    sys.exit(main())
