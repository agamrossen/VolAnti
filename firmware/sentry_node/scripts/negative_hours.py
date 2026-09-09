#!/usr/bin/env python
"""
negative_hours.py - the drone-free-audio ledger.

    python scripts/negative_hours.py              the ledger
    python scripts/negative_hours.py --json       machine-readable
    python scripts/negative_hours.py --add-ring FILE   count a `V` dump

WHY THIS EXISTS. Tier-4 is calibrated, ported and OFF, and the ONLY thing
keeping it off is a number: the drone-free audio this project owns bounds its
false-alarm
rate at about 16/h against a 0.40/h allowance. Certifying it needs roughly
7.5 hours. Every session so far has re-derived that arithmetic by hand from a
different starting point, and nobody has ever known the running total.

So: one place that counts, one number that goes up, and a target beside it.

WHAT COUNTS AS AN HOUR, and this is the whole discipline of the file.

  * The audio must be DRONE-FREE and somebody must have said so. A capture
    nobody attested is not a negative, it is an unlabelled recording.
  * The hours are counted PER TIER-ELIGIBILITY, because a recording is a
    negative for the tiers that could have fired on it and says nothing about
    the others. A capture taken with Tier-4 disabled bounds nothing about
    Tier-4. See the evidence ledger for which tiers each recording is
    evidence for and which it is a hard negative for.
  * A `V` ring dump counts the device's UPTIME as negative hours only when
    the ring is empty of detections. One alert in four hours does not void
    the other three hours fifty-nine minutes - but it does mean the hours are
    no longer the same kind of evidence, so the tool reports both and refuses
    to add them together.

WHAT IT DOES NOT DO. It does not read the detector, it does not decide
anything, and NOTHING SELF-LEARNS from it. It counts what a human recorded.
"""
import argparse
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJ = HERE.parent                       # firmware/sentry_node
ROOT = PROJ.parent.parent                # repo root

# ---------------------------------------------------------------------------
# THE TARGETS. Both come from the same arithmetic: to bound a rate at R per
# hour with 95% confidence from ZERO events you need about 3/R hours (the
# rule-of-three), so an allowance of 0.40/h needs 7.5 h.
# ---------------------------------------------------------------------------
TARGET_H = {
    "T4": 7.5,     # 0.40/h allowance, rule of three
    "T3": 7.5,     # same allowance; T3's measured rate is 4.87/h and FAILED
    "v1": 7.5,     # v1's budget is 4.00 weighted, but the real-air rate is
                   # unmeasured and this is the honest bar for any of them
}

# ---------------------------------------------------------------------------
# THE LEDGER. One entry per body of real, attested, drone-free audio this
# project owns. ADDING A ROW IS A HUMAN ACT: it records that somebody listened
# to, or was present for, the audio and is willing to say there was no drone
# in it. Nothing populates this automatically.
# ---------------------------------------------------------------------------
LEDGER = [
    {
        # THE SECONDS ARE Measured, NOT WRITTEN DOWN. This row used to read a
        # hard-coded 667.0, and 667 was wrong: 180 s of the library was the
        # same four recordings listed twice under different names. A ledger
        # whose figures are typed rather than counted is a ledger that cannot
        # notice that. See DUPLICATES in src/real_negatives.py.
        "id": "real_negatives_lib",
        "what": "the assembled real-negative library",
        "seconds": None,                # measured below, deduplicated
        "provenance": "src/real_negatives.py",
        "attested": True,
        "eligible": ["v1", "T2", "T3", "T4"],
        "note": "fires ZERO at every v1 threshold 1.40-1.80. A gate that can "
                "reject a threshold and never certify one. CORRECTED "
                "667 s -> 487 s distinct, because 180 s was "
                "counted twice.",
    },
    {
        "id": "field1_selftest",
        "what": "Field-1 self-test capture, outdoors, no source",
        "seconds": 0.0,
        "provenance": "field/ - DURATION NOT YET EXTRACTED",
        "attested": True,
        "eligible": ["v1", "T2", "T3"],
        "note": "counted at zero until somebody reads the real duration off "
                "the capture. An unknown duration is not a small one.",
    },
    {
        "id": "lab_quiet_baseline_0813",
        "what": "the 08-13 lab room's quiet baseline",
        "seconds": 0.0,
        "provenance": "captures/ - DURATION NOT YET EXTRACTED",
        "attested": True,
        "eligible": ["v1", "T2"],
        "note": "NOT eligible for T3 or T4: the room has its own 245 Hz HVAC "
                "comb, which Tier-4 correctly finds - so it is not a negative "
                "for Tier-4, it is a positive for a building. See "
                "the evidence ledger.",
    },
]

TIERS = ("v1", "T2", "T3", "T4")


def ingested_livestock():
    """Rows for whatever `scripts/ingest_livestock.py` has registered.

    Livestock recordings are drone-free by the operator's own act of passing
    them to that tool, so they accrue here like any other attested negative -
    but they accrue to the COMB tiers only. The audio is phone-mono, and phone
    AGC rewrites the modulation depth Tier-3 measures, so a phone recording
    bounds nothing about Tier-3. `src/real_negatives.for_tier` enforces the
    same rule for the same reason; this list obeys it rather than restating
    it.

    Empty until livestock is recorded, so this changes nothing today."""
    import sys
    sys.path.insert(0, str(ROOT / "src"))
    try:
        import real_negatives as RN
    except Exception:                                          # noqa: BLE001
        return []
    rows = []
    for name, rel, prov, why, family in RN.registered():
        p = RN.CAP / rel
        if not p.exists():
            continue
        try:
            import soundfile as sf
            sec = sf.info(str(p)).duration
        except Exception:                                      # noqa: BLE001
            sec = 0.0
        rows.append({
            "id": Path(rel).stem,
            "what": f"ingested {family or 'real negative'} ({prov})",
            "seconds": float(sec),
            "provenance": f"scripts/ingest_livestock.py -> {rel}",
            "attested": True,
            "eligible": ["v1", "T2", "T4"],
            "note": "phone-mono: NOT eligible for T3, because AGC rewrites "
                    "the modulation depth T3 measures.",
        })
    return rows


def library_seconds():
    """The real-negative library, Measured and PER TIER.

    Two corrections live here, both found  and both of which made
    the ledger optimistic:

    (1) DUPLICATES. 180 s of the library was four recordings listed twice
        under different names - byte-identical, sha256-checked. for_tier() now
        drops a clip whose bytes it has already returned, so 667 s of listed
        audio is 487 s of distinct audio.

    (2) TIER-3 WAS NEVER ELIGIBLE FOR ALL OF IT. Phone audio has AGC, which
        rewrites the modulation depth Tier-3 measures, and the two rotor
        captures contain a rotor. `real_negatives.for_tier` has enforced that
        since the design - but this ledger applied ONE figure to all four tiers
        and so credited Tier-3 with audio it may not be priced on. Measured
        per tier, Tier-3 has 185 s, not 667.

    Returns {} when the library cannot be imported, so a ledger read never
    fails for want of numpy."""
    import sys
    sys.path.insert(0, str(ROOT / "src"))
    try:
        import real_negatives as RN
    except Exception:                                          # noqa: BLE001
        return {}
    out = {}
    for tier, key in (("v1", "v1"), ("T2", "t2"), ("T3", "t3"), ("T4", "t4")):
        try:
            out[tier] = sum(RN.seconds(c) for c in RN.for_tier(key))
        except Exception:                                      # noqa: BLE001
            return {}
    return out


def row_seconds(row, tier):
    """Seconds this row contributes TO THIS TIER."""
    if row.get("seconds") is None:
        return library_seconds().get(tier, 0.0)
    return float(row.get("seconds", 0.0))


def totals(extra=()):
    out = {t: 0.0 for t in TIERS}
    for row in list(LEDGER) + list(extra):
        if not row.get("attested"):
            continue
        for t in row.get("eligible", []):
            if t in out:
                out[t] += row_seconds(row, t)
    return out


# ---------------------------------------------------------------------------
# reading a `V` dump
# ---------------------------------------------------------------------------

RING_HDR = re.compile(r"EVENTS\s+(\d+)\s+total")
RING_ROW = re.compile(r"^\s+(\d+)\s+(\d+):(\d+)\.(\d)\s")


def read_ring(path):
    """Parse a saved `V` dump into (uptime_seconds, n_events, n_battery).

    The ring is the only recoverable record of a night on a power bank, and
    the uptime of its LAST entry is a lower bound on how long the device was
    guarding. An empty ring carries no uptime at all - which is exactly the
    case that matters most (a quiet night) and exactly the case this cannot
    measure. The status screen's `U<minutes>` overlay is the answer there, and
    it needs a human to read it off a photograph."""
    txt = Path(path).read_text(errors="replace")
    m = RING_HDR.search(txt)
    if not m:
        return None
    n_events = int(m.group(1))
    last_s = 0.0
    n_batt = 0
    for line in txt.splitlines():
        r = RING_ROW.match(line)
        if r:
            last_s = max(last_s, int(r.group(2)) * 60 + int(r.group(3))
                         + int(r.group(4)) / 10.0)
        if " batt " in line:
            n_batt += 1
    return {"uptime_s": last_s, "n_events": n_events, "n_battery": n_batt}


def report(rows_extra=(), as_json=False):
    rows_extra = list(ingested_livestock()) + list(rows_extra)
    tot = totals(rows_extra)
    if as_json:
        return json.dumps({
            "hours": {t: round(tot[t] / 3600.0, 4) for t in TIERS},
            "target_h": TARGET_H,
            "entries": LEDGER + list(rows_extra),
        }, indent=2)

    L = []
    A = L.append
    A("")
    A("  NEGATIVE-HOURS LEDGER - real, attested, drone-free audio")
    A("  " + "-" * 68)
    for row in LEDGER + list(rows_extra):
        # A row whose seconds differ per tier prints the widest tier it is
        # eligible for and says so, rather than printing one number that is
        # right for one tier and wrong for the others.
        per = {t: row_seconds(row, t) for t in row["eligible"]}
        sec = max(per.values()) if per else 0.0
        h = sec / 3600.0
        spread = "" if len(set(round(v, 1) for v in per.values())) <= 1 else \
                 "  (per tier: " + ", ".join(
                     f"{t} {v:.0f}s" for t, v in per.items()) + ")"
        A(f"  {row['id']:<26} {sec:8.1f} s  ({h:5.3f} h)  "
          f"{'/'.join(row['eligible'])}{spread}")
        A(f"      {row['what']}")
        A(f"      {row['provenance']}")
        if row.get("note"):
            A(f"      NOTE {row['note']}")
    A("")
    A("  tier   hours   target   bound at 95% from zero events")
    A("  " + "-" * 68)
    for t in TIERS:
        h = tot[t] / 3600.0
        target = TARGET_H.get(t)
        bound = (3.0 / h) if h > 0 else float("inf")
        b = "unbounded" if h <= 0 else f"{bound:.1f}/h"
        tgt = f"{target:.1f} h" if target else "-"
        A(f"  {t:<6} {h:6.3f}  {tgt:>7}   {b:>12}"
          + ("   MET" if target and h >= target else ""))
    A("")
    A("  THE RULE OF THREE: zero events in h hours bounds the rate at 3/h per")
    A("  hour, 95%. It BOUNDS; it never certifies. Tier-4's allowance is")
    A("  0.40/h, so it needs about 7.5 h - and it stays OFF until it has")
    A("  them.")
    A("")
    A("  THE CHEAPEST WAY TO ACCRUE THEM, and it costs nothing: leave the")
    A("  device guarding overnight at home on the power bank, and read `V`")
    A("  in the morning. A quiet night is 8 hours of exactly the evidence")
    A("  that is missing. Photograph the status screen for the uptime - an")
    A("  EMPTY ring has no timestamps in it to read.")
    A("")
    return "\n".join(L)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--add-ring", metavar="FILE",
                    help="a saved `V` dump to count (uptime, if the ring has "
                         "any entry to date it from)")
    a = ap.parse_args(argv)

    extra = []
    if a.add_ring:
        r = read_ring(a.add_ring)
        if not r:
            print(f"  not a `V` dump: {a.add_ring}", file=sys.stderr)
            return 2
        det = r["n_events"] - r["n_battery"]
        extra.append({
            "id": Path(a.add_ring).stem,
            "what": f"device ring dump ({det} detections, "
                    f"{r['n_battery']} battery notes)",
            "seconds": r["uptime_s"] if det == 0 else 0.0,
            "provenance": str(a.add_ring),
            "attested": det == 0,
            "eligible": list(TIERS),
            "note": ("counted" if det == 0 else
                     "NOT COUNTED: the ring holds detections, so this is not "
                     "drone-free audio unless a human attests each one was a "
                     "false alarm - and then it is not a ZERO-event bound "
                     "any more"),
        })
    print(report(extra, a.json))
    return 0


if __name__ == "__main__":
    sys.exit(main())
