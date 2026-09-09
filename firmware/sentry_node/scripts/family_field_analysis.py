#!/usr/bin/env python
"""
family_field_analysis.py - the family {2,3} rule's field verdict,
pre-committed.

    conda activate acoustic-detector
    cd ~/acoustic-detector/firmware/sentry_node

    python scripts/family_field_analysis.py                  # today's session
    python scripts/family_field_analysis.py captures/<session>
    python scripts/family_field_analysis.py --json

WRITTEN BEFORE THE DATA EXISTS, AND THAT IS THE WHOLE OF ITS VALUE. Field-3
has not happened. It exists for the same reason the decision rules do: the
band verdict came back CONFIRMED, then SHIFTED, then NO USABLE COMB, and only
the third was right. Rules written after seeing data are indistinguishable from
rules fitted to it.

THE QUESTION. `trk_family` is in the firmware, persisted, default 0. Flag off
is bit-identical to the sealed tracker; flag on is the family {2,3} continuity
rule plus family-normalised jitter, adopted on the corpus by NON-INFERIORITY
(0 regressions, 2 gains, McNemar p = 0.50 on 486 paired positives). It has
never run on real air. This file is the paired real-air test that
`DECISION_RULES` step 3 demands, and nothing else may promote the flag.

WHAT IT DOES. Every recognised stage of a Field session is replayed through the
Python reference ONCE, and the resulting per-frame series is then TRACKED
TWICE - sealed against family - at the shipped threshold.

WHY ONE PASS SERVES BOTH, and it is exact rather than approximate: the tracker
consumes (t, f0, score, teeth, f0_raw), and `back_end` does not read a single
tracker constant, so the whole variant question re-runs the DECISION over saved
numbers and costs no FFT at all. That is the same saving
`src/tracker_variants.py` documents and relies on, and the one assumption it
rests on - comb-hold off, so `back_end` never touches a tracker - is ASSERTED
below rather than believed.

WHY THE PYTHON VARIANT AND NOT THE DEVICE'S OWN VERDICT. The device runs one
flag setting at a time, so it can never produce a paired comparison; and its
live verdict is a single bit per stage. `tests/test_tracker_family.py` compiles
the SHIPPED `main/detector.c` for the host and proves the C flag and
`tracker_variants.VariantTracker` agree decision-for-decision - flag off and
flag on - over the golden vectors and the corpus. So replaying the archived
PCM through the Python variant answers the question about the shipped flag.
It follows that whatever `U c family` was set to on the day does not bias this
analysis: both arms replay the same samples. It still has to be in the log,
because it decides what the OPERATOR heard.

A NULL STAGE IS NOT A CHANCE TO SCORE. Pre-committed here, before any data: on
a stage with no source running, an alert the family rule raises and the sealed
tracker does not is a NEW FALSE ALARM, and it counts as a regression. It is not
a gain. Any rule that lets a change bank its extra firings on the quiet stages
and its extra detections on the loud ones can only ever look good.

EXIT STATUS - read the VERDICT WORD, not the code:

    0   the analysis produced a verdict (ADOPT, DO NOT ADOPT, or NOT ADOPTED)
    2   no such session, or no *_raw.npz captures in it
    3   captures exist but none is a stage this analysis recognises, or none
        of them has a source in it - either way there is no verdict, and an
        empty table that reads like a pass is exactly what this refuses to
        print.
"""
import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent                          # firmware/sentry_node
ROOT = PROJ.parent.parent                   # repo root
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "src"))

import numpy as np                                              # noqa: E402

import detector as D                                            # noqa: E402
import operating_point as op                                    # noqa: E402
import profile_store as PS                                      # noqa: E402
import tracker_variants as TV                                   # noqa: E402
import quad_reference as QR                                     # noqa: E402
from field_verdict import Cache, cache_key, load_session        # noqa: E402

GREEN = "\033[32m"
RED = "\033[31m"
YELLOW = "\033[33m"
BOLD = "\033[1m"
DIM = "\033[2m"
OFF = "\033[0m"

RULES = "field/DECISION_RULES_2026-08-21.md"
RULE_NAME = "THE FAMILY {2,3} RULE"

# The shipped threshold. Taken from the preset rather than
# written as a literal so it cannot drift away from what the device runs; the
# literal is kept beside it only so a silent move is visible in the header.
SHIPPED_THR = 1.70

# Bumped whenever anything that changes a cached per-stage number changes.
CACHE_TAG = "family-field/1"

# ---------------------------------------------------------------------------
# WHAT COUNTS AS WHICH KIND OF STAGE
#
# These three names USED to be defined here, and `field_verdict.py` decided the
# same question a second way, with its own scattered prefix tests. The two
# disagreed - `guard_*` was an ambient stage there and UNCLASSIFIED here - and
# neither file could see it. They now come from `stage_vocab.py`, which is the
# only copy; `tests/test_stage_vocab.py` asserts these are the SAME objects and
# not equal ones, because a copy passes an equality test on the day it is made
# and fails silently the day after.
#
# Everything the old comment said still holds and is said there: matched by
# prefix because the Field-3 card is still being written, null wins a tie, and
# anything matching neither list is reported as UNCLASSIFIED and left OUT of
# the verdict - loudly, and named, so nobody discovers afterwards that the
# stage they cared about was never in the arithmetic.
# ---------------------------------------------------------------------------
from stage_vocab import (NULL_PREFIXES, SOURCE_PREFIXES,     # noqa: E402,F401
                         stage_class)


# ---------------------------------------------------------------------------
# the replay, and the two trackers over it
# ---------------------------------------------------------------------------

def family_kw():
    """The measured family row, fetched from `tracker_variants.VARIANTS` at
    call time. Copying it would leave this file analysing a variant nobody
    measured; `tests/test_family_field.py` proves the lookup still resolves."""
    rows = {name: kw for name, _, kw in TV.VARIANTS}
    kw = rows.get("family {2,3}")
    if kw is None:
        raise KeyError("tracker_variants.VARIANTS has no 'family {2,3}' row - "
                       "this analysis has nothing to compare against")
    return dict(kw)


def summarise(events):
    """Events, first-alert time, total latched duration, and the f0 the first
    event settled on. DURATION IS THE SUM over the stage's events, not the
    first event's length: a rule that turns one 12 s event into three 4 s ones
    has not lost anything, and a column that said otherwise would report it as
    a loss."""
    if not events:
        return {"events": 0, "first_s": None, "dur_s": 0.0, "f0": None,
                "detected": False}
    first = events[0]
    return {"events": len(events),
            "first_s": float(first["t_on"]),
            "dur_s": float(sum(e["t_off"] - e["t_on"] for e in events)),
            "f0": float(first["f0"]),
            "detected": True}


def track_pair(trace, thr, cfg, kw=None):
    """The cheap half: ONE per-frame series, tracked twice.

    Sealed is `detector.track_frames` and family is
    `tracker_variants.track_variant` - the shipped code and the measured
    variant, never a third implementation of the chain rule.
    """
    kw = family_kw() if kw is None else kw
    if not len(trace["t"]):
        empty = summarise([])
        return {"sealed": empty, "family": empty, "frames": 0}
    sealed = D.track_frames(trace, thr, cfg)[0]
    family = TV.track_variant(trace, thr, cfg, **kw)
    return {"sealed": summarise(sealed), "family": summarise(family),
            "frames": int(len(trace["t"]))}


def replay(audio, cfg, thr):
    """The expensive half, run ONCE per stage.

    `quad_reference.analyze_quad` is `src/detector.py`'s own front_end /
    combiner / back_end over four channels - the offline authority every other
    quad decision in this project is made with.
    """
    # The one-pass saving is only exact while back_end has no tracker in it.
    # comb-hold was REJECTED and hold_mode is "off"; if a future config turns
    # it back on, back_end steps a TrackerState of its own at a fixed
    # threshold and the two arms stop sharing a front end.
    assert cfg.hold_mode == "off", (
        "comb-hold puts a tracker inside back_end, so one pass no longer "
        "serves both arms of this comparison")
    x = np.atleast_2d(np.asarray(audio))
    return QR.analyze_quad(x, cfg, thr_ref=thr)


def stage_row(name, cap, cfg, thr, kw, cache):
    key = cache_key(cap["path"], f"{CACHE_TAG}:{thr:.4f}:{sorted(kw.items())}")
    hit = cache.get(key)
    if hit is not None:
        return hit
    tr = replay(cap["audio"], cfg, thr)
    row = track_pair(tr, thr, cfg, kw)
    return cache.put(key, row)


def collect(caps, cfg, thr, kw, cache, source=(), null=(), verbose=True):
    """Every recognised stage, replayed and tracked -> (rows, unknown)."""
    rows, unknown = [], []
    for name in sorted(caps):
        cls = stage_class(name, source, null)
        if cls is None:
            unknown.append(name)
            continue
        cap = caps[name]
        if verbose:
            print(f"  {DIM}... replaying {name} ({cap['seconds']:.0f} s){OFF}",
                  flush=True)
        r = stage_row(name, cap, cfg, thr, kw, cache)
        rows.append({"stage": name, "class": cls,
                     "seconds": float(cap["seconds"]),
                     "device_alerts": cap.get("meta", {}).get("device_alerts"),
                     **r})
    return rows, unknown


# ---------------------------------------------------------------------------
# THE PRE-COMMITTED GATE
# ---------------------------------------------------------------------------

def verdict(rows):
    """the design C-3, written down before Field-3 existed:

        zero stage-level regressions AND at least one gain -> ADOPT, the flag
        ships default-on; any regression -> DO NOT ADOPT, it stays off and the
        loss is published.

    A regression is either of these, and both were named in advance:
      * a SOURCE stage the sealed tracker detected and the family rule did not;
      * a NULL stage the family rule alerted on and the sealed tracker did not
        - a new false alarm.

    A gain is a SOURCE stage the family rule detected and the sealed tracker
    did not. Nothing on a null stage is ever a gain: a false alarm the family
    rule happens to REMOVE is reported, and does not count towards the gate,
    because this gate is about detection and one removed FA on one afternoon
    is not a false-alarm measurement.
    """
    src = [r for r in rows if r["class"] == "source"]
    nul = [r for r in rows if r["class"] == "null"]

    a = np.array([r["sealed"]["detected"] for r in src], bool)
    b_ = np.array([r["family"]["detected"] for r in src], bool)
    gains, losses, p = TV.mcnemar(a, b_) if len(src) else (0, 0, 1.0)

    lost_stages = [r["stage"] for r in src
                   if r["sealed"]["detected"] and not r["family"]["detected"]]
    gain_stages = [r["stage"] for r in src
                   if r["family"]["detected"] and not r["sealed"]["detected"]]
    fa_added = [r["stage"] for r in nul
                if r["family"]["detected"] and not r["sealed"]["detected"]]
    fa_removed = [r["stage"] for r in nul
                  if r["sealed"]["detected"] and not r["family"]["detected"]]

    regressions = list(lost_stages) + list(fa_added)
    if not src:
        word = "NO VERDICT"
        why = ("no stage with a source in it - the gate is about detection "
               "and there was nothing to detect")
    elif regressions:
        word, why = "DO NOT ADOPT", ("the flag stays off, and the loss is "
                                     "published")
    elif gains >= 1:
        word, why = "ADOPT", "trk_family ships default-on"
    else:
        word, why = "NOT ADOPTED", ("no regression and no gain - this day is "
                                    "silent on the question, and the gate "
                                    "requires a gain")
    return {"verdict": word, "why": why,
            "n_source": len(src), "n_null": len(nul),
            "b_family_only": int(gains), "c_sealed_only": int(losses),
            "mcnemar_p": float(p),
            "gain_stages": gain_stages, "lost_stages": lost_stages,
            "fa_added": fa_added, "fa_removed": fa_removed,
            "regressions": regressions}


# ---------------------------------------------------------------------------
# printing
# ---------------------------------------------------------------------------

def _cell(s):
    ev = s["events"]
    first = "  -  " if s["first_s"] is None else f"{s['first_s']:5.2f}"
    dur = f"{s['dur_s']:5.1f}"
    f0 = "  -" if s["f0"] is None else f"{s['f0']:3.0f}"
    return f"{ev:3d} {first} {dur} {f0}"


def print_table(rows):
    print(f"\n{BOLD}  PER-STAGE, SEALED vs FAMILY {{2,3}}{OFF}")
    print(f"  {'stage':<20}{'class':<8}{'s':>6}{'dev':>5}  |"
          f"{'ev first   dur  f0':>21}  |{'ev first   dur  f0':>21}  |")
    print(f"  {'':<20}{'':<8}{'':>6}{'':>5}  |{'SEALED':>21}  |"
          f"{'FAMILY {2,3}':>21}  |")
    print("  " + "-" * 88)
    for r in rows:
        s, f = r["sealed"], r["family"]
        if r["class"] == "source":
            if s["detected"] and not f["detected"]:
                mark = f"{RED}REGRESSION{OFF}"
            elif f["detected"] and not s["detected"]:
                mark = f"{GREEN}gain{OFF}"
            else:
                mark = f"{DIM}={OFF}"
        else:
            if f["detected"] and not s["detected"]:
                mark = f"{RED}NEW FALSE ALARM{OFF}"
            elif s["detected"] and not f["detected"]:
                mark = f"{YELLOW}FA removed{OFF}"
            else:
                mark = f"{DIM}={OFF}"
        dev = "  -" if r.get("device_alerts") is None else \
            f"{r['device_alerts']:3d}"
        print(f"  {r['stage']:<20}{r['class']:<8}{r['seconds']:6.0f}"
              f"{dev:>5}  |{_cell(s):>21}  |{_cell(f):>21}  |  {mark}")
    print(f"\n  {DIM}dev = alerts the DEVICE raised live, at whatever flag it "
          f"was running.\n  Provenance, not evidence: both columns above are "
          f"offline replays of the same\n  archived samples, so the day's "
          f"flag setting cannot bias them.\n  dur is the SUM of the stage's "
          f"event durations; f0 is the first event's median.{OFF}")


def print_verdict(v, thr):
    print(f"\n{BOLD}  PRE-COMMITTED ADOPTION GATE{OFF}")
    print(f"  {DIM}{RULES} - {RULE_NAME}\n"
          f"  Written before Field-3 existed. Nothing on the day may edit "
          f"it.{OFF}")
    print(f"\n    source stages                 {v['n_source']:3d}"
          f"      null stages {v['n_null']:3d}")
    print(f"    b  family-only detections     {v['b_family_only']:3d}"
          f"      (gains)")
    print(f"    c  sealed-only detections     {v['c_sealed_only']:3d}"
          f"      (regressions)")
    print(f"    new false alarms, null stages {len(v['fa_added']):3d}"
          f"      (also regressions, pre-committed)")
    print(f"    exact two-sided McNemar       p = {v['mcnemar_p']:.4f}"
          f"   {DIM}on the source stages{OFF}")
    if v["fa_removed"]:
        print(f"    false alarms REMOVED          "
              f"{len(v['fa_removed']):3d}      {DIM}reported, not counted: "
              f"the gate is about detection{OFF}")
    print()
    if v["verdict"] == "ADOPT":
        col, extra = GREEN, ""
    elif v["verdict"] == "DO NOT ADOPT":
        col = RED
        extra = "\n    losing stages: " + ", ".join(v["regressions"])
    else:
        col, extra = YELLOW, ""
    print(f"    {BOLD}{col}VERDICT: {v['verdict']}{OFF} - {v['why']}{extra}")
    if v["gain_stages"]:
        print(f"    {DIM}gains on: {', '.join(v['gain_stages'])}{OFF}")
    print(f"\n  {DIM}Threshold {thr:.4f}, the shipped v1 operating point. "
          f"Both arms at the SAME\n  threshold on purpose: this flag was "
          f"adopted as FREE - it does not move the\n  false-alarm rate, so "
          f"there is nothing to recalibrate. Any change that DID\n  move it "
          f"would have to be recalibrated first, which is the comb-hold "
          f"lesson.{OFF}")
    print(f"  {DIM}Exit status 0 means the analysis ran, NOT that the flag "
          f"passed. The verdict\n  is the word above.{OFF}")


# ---------------------------------------------------------------------------

class Refusal(Exception):
    """No verdict is possible, and saying so is the point. Carries the exit
    code, because a caller that cannot tell "nothing to analyse" from "the
    flag passed" is the failure mode this whole file exists to avoid."""

    def __init__(self, code, msg, unknown=()):
        super().__init__(msg)
        self.code = code
        self.unknown = list(unknown)


def analyse(sess, thr=None, use_cache=True, source=(), null=(), verbose=True):
    """Returns (rows, unknown, verdict_dict, cfg, thr) or raises Refusal."""
    cfg, thr_default = op.preset_config("HIGH_ALERT")
    thr = thr_default if thr is None else float(thr)
    caps = load_session(sess)
    if not caps:
        raise Refusal(2, f"no *_raw.npz captures in {sess}")
    cache = Cache(sess, enabled=use_cache)
    kw = family_kw()
    rows, unknown = collect(caps, cfg, thr, kw, cache, source, null, verbose)
    if not rows:
        raise Refusal(
            3,
            f"{len(caps)} capture(s) in {sess}, and not one of them is a "
            f"stage this analysis recognises:\n      "
            + ", ".join(sorted(caps))
            + "\n    Classify them with --source / --null, or rename the "
              "captures. An empty\n    table is not a pass.",
            unknown=unknown)
    return rows, unknown, verdict(rows), cfg, thr


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", nargs="?", default=None)
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--no-cache", action="store_true")
    ap.add_argument("--threshold", type=float, default=None,
                    help="EXPLORATION ONLY - stamps the output as not the "
                         "pre-committed analysis")
    ap.add_argument("--source", default="",
                    help="comma-separated stage names to force to 'source'")
    ap.add_argument("--null", default="",
                    help="comma-separated stage names to force to 'null'")
    a = ap.parse_args(argv)

    sess = Path(a.session) if a.session else PS.session_dir("field3")
    if not sess.exists():
        print(f"{RED}  no such session: {sess}{OFF}", file=sys.stderr)
        return 2
    src = tuple(s for s in a.source.split(",") if s)
    nul = tuple(s for s in a.null.split(",") if s)

    if not a.json:
        print(f"\n{BOLD}  FAMILY {{2,3}} FIELD ANALYSIS{OFF}")
        print(f"  session {sess}")
        print(f"  {DIM}rules: {RULES} - {RULE_NAME}, pre-committed "
              f"the corpus{OFF}")

    t0 = time.time()
    try:
        rows, unknown, v, cfg, thr = analyse(
            sess, a.threshold, not a.no_cache, src, nul, verbose=not a.json)
    except Refusal as e:
        print(f"\n{RED}{BOLD}  REFUSED{OFF}{RED} - {e}{OFF}", file=sys.stderr)
        return e.code

    if a.threshold is not None and abs(thr - SHIPPED_THR) > 1e-9:
        print(f"\n{YELLOW}  THIS IS NOT THE PRE-COMMITTED ANALYSIS: threshold "
              f"{thr:.4f}, not the shipped {SHIPPED_THR:.4f}. Nothing below "
              f"may promote the flag.{OFF}")
    elif abs(thr - SHIPPED_THR) > 1e-9:
        print(f"\n{YELLOW}  the HIGH_ALERT preset has MOVED to {thr:.4f}; C-3 "
              f"pre-committed {SHIPPED_THR:.4f}. Say which one this is in the "
              f"write-up.{OFF}")

    if a.json:
        print(json.dumps({"session": str(sess), "threshold": thr,
                          "rows": rows, "unclassified": unknown,
                          **v}, indent=2, default=float))
        return 0

    if src or nul:
        print(f"\n{YELLOW}  CLASSIFICATION OVERRIDDEN ON THE COMMAND LINE"
              f"{OFF} - source={list(src)} null={list(nul)}. Say so in the "
              f"write-up: it was not pre-committed.")
    if unknown:
        print(f"\n{YELLOW}  {len(unknown)} capture(s) NOT in the verdict "
              f"because this analysis cannot tell whether a source was "
              f"running:{OFF}")
        print(f"    {', '.join(unknown)}")
        print(f"  {DIM}Classify them with --source / --null if they belong in "
              f"it.{OFF}")

    print_table(rows)
    print_verdict(v, thr)
    print(f"\n  {DIM}done in {time.time() - t0:.1f} s. R5: no field "
          f"retuning - data, then offline analysis, then the next planning "
          f"session.{OFF}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
