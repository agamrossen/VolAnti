#!/usr/bin/env python
"""
stage_vocab.py - what a capture's NAME says about the capture. ONE copy.

WHY IT IS A FILE OF ITS OWN, AND NOT A TUPLE IN field_verdict.py. The same
fact was written down in three places before tonight:

  * `field_capture.py`'s CANONICAL, which decides whether the operator gets a
    "check the spelling" warning at the rig;
  * `family_field_analysis.py`'s SOURCE_/NULL_PREFIXES, which decides whether a
    stage can produce a gain, a regression, or neither in the pre-committed
    adoption gate;
  * five scattered `startswith` tests inside `field_verdict.py`, which decide
    which section of the verdict card a capture is printed in - or, for a name
    none of them matched, that it is printed nowhere at all.

They already disagreed. `guard_*` was an ambient stage to the verdict card and
UNCLASSIFIED to the adoption gate; `rotor_4m_0` - the name the only real rotor
audio this project owns is archived under - was a source stage to the gate and
invisible to the verdict card. Neither disagreement was visible from either
file. That is the shape this project keeps removing: two statements of one
fact, both right when written, one of them false by the time it matters.

A stage vocabulary is exactly the shape that rots, because it is only ever
edited when a NEW stage appears - which is the one moment the copies drift.

TWO QUESTIONS, NOT ONE, AND THEY ARE DIFFERENT QUESTIONS:

  stage_class(name)   was a SOURCE running?  source / null / None.
                      The adoption gate reads this. A null stage cannot score
                      a gain, only a new false alarm.
  section(name)       which section of the verdict card prints it? A name can
                      be classifiable and still belong to no section - see
                      `rotor` below - and the card must SAY SO rather than
                      drop it.

THE INVARIANT BETWEEN THEM, asserted by `tests/test_stage_vocab.py`: every
prefix that has a SECTION has a CLASS. The converse is deliberately false.

WHAT IT DOES NOT DO. It decides nothing. It does not know what a stage
measures, whether the capture is any good, or what any of it means. It maps a
string to a pigeonhole, and every consumer refuses loudly when the answer is
None.
"""
import difflib

# ---------------------------------------------------------------------------
# QUESTION 1: WAS A SOURCE RUNNING?
#
# Matched by prefix, because the Field-3 card is still being written and a
# frozen list of exact names would silently drop everything it did not
# anticipate. Anything matching neither list is UNCLASSIFIED, which every
# reader reports by name rather than skipping.
# ---------------------------------------------------------------------------
SOURCE_PREFIXES = ("solo_", "quad_", "dist_", "tilt_", "rotor", "drone",
                   "hover", "flyby", "punch", "approach", "spin", "descent")

# `guard` and `wind` joined this list tonight, and both are conservative:
#   guard - `field_verdict.py` the protocol has always replayed guard segments beside
#           ambient ones; the device standing guard has no rig running, so the
#           two vocabularies disagreed about a stage they both handled.
#   wind  - the design commissions site wind hours, and drone-free wind is
#           precisely the null that Tier-3's and Tier-4's thresholds stand on.
#           It is also the base name of the windscreen A/B (the protocol below).
# A fan or a wind machine is a SOUND SOURCE and not a DETECTION TARGET, and
# this list is about detection targets: `speech` has always been null for the
# same reason. An alert on the windscreen A/B is a false alarm, which is the
# entire point of running it.
NULL_PREFIXES = ("quiet", "baseline", "ambient", "background", "null",
                 "speech", "talk", "selftest", "guard", "wind")


def stage_class(name, source=(), null=()):
    """'source' (a rig was running), 'null' (nothing was), or None."""
    n = name.lower()
    if name in source:
        return "source"
    if name in null:
        return "null"
    # null wins a tie: `quiet_baseline_with_rotor_off` is a null stage, and a
    # stage whose name is ambiguous must never be scored as a detection.
    if n.startswith(NULL_PREFIXES):
        return "null"
    if n.startswith(SOURCE_PREFIXES):
        return "source"
    return None


# ---------------------------------------------------------------------------
# QUESTION 2: WHICH SECTION OF THE VERDICT CARD?
#
# First match wins, so the order here is the order of the card. The source
# families carry their underscore because SOURCE_PREFIXES does: a section
# prefix that is WIDER than the class prefix would produce a stage the card
# analyses and the adoption gate cannot classify, which is the invariant above.
# ---------------------------------------------------------------------------
SECTIONS = (
    ("quiet",    ("quiet", "baseline", "null")),      # 0. the gate
    ("solo",     ("solo_",)),                         # 1. R1, the band verdict
    ("quad",     ("quad_",)),                         # 2. R2/R4
    ("dist",     ("dist_",)),                         # 3. the distance ladder
    ("ambient",  ("ambient", "guard", "speech",       # 4. R3
                  "talk", "background", "wind")),
    ("tilt",     ("tilt_",)),                         # 6. the tilt ladder
    ("selftest", ("selftest",)),                      # printed by no section
)

# The sections that actually print a stage. `selftest` is recognised and
# analysed by nothing: it is the capture path proving itself, five seconds
# long, and putting it in R3's ambient table would be inventing evidence.
ANALYSED_SECTIONS = ("quiet", "solo", "quad", "dist", "ambient", "tilt")


def section(name):
    """The verdict-card section that prints this stage, or None."""
    n = name.lower()
    for sec, prefixes in SECTIONS:
        if n.startswith(prefixes):
            return sec
    return None


def split(names):
    """Three piles: (analysed, aside, unknown).

    `aside` is the pile that has never existed before and is the reason this
    function does: a name the adoption gate CAN classify and no section of the
    verdict card prints. `rotor_4m_0` is one. Reporting it as unknown would be
    a lie and dropping it silently is how a capture nobody looked at gets
    quoted as evidence six weeks later.
    """
    analysed, aside, unknown = [], [], []
    for n in names:
        sec = section(n)
        if sec in ANALYSED_SECTIONS:
            analysed.append(n)
        elif sec or stage_class(n):
            aside.append(n)
        else:
            unknown.append(n)
    return analysed, aside, unknown


# ---------------------------------------------------------------------------
# THE TILT LADDER - the design
#
# "The real beamforming of this device": the cone mouths become directive
# above roughly 4-5 kHz - Predicted by planning, not measured - which is
# Tier-3 and Tier-4 harmonic territory, so the MOUNTING ANGLE is the beam. The
# ladder is fixed source, device at 0 / 30 / 60 degrees face-up, plus one
# face-away control.
#
# Degrees are zero-padded to two digits so that lexicographic order IS ladder
# order. Every ladder on the verdict card is printed in `sorted()` order and
# the distance ladder takes its reference from the first key, so `tilt_5`
# sorting after `tilt_30` would not look like a bug, it would look like a
# measurement.
# ---------------------------------------------------------------------------
TILT_PREFIX = "tilt_"
TILT_REFERENCE = "tilt_00"       # the ladder's 0 dB / 0 deg row
TILT_CONTROL = "tilt_away"       # face-away: the control the claim needs


def tilt_angle(name):
    """Degrees of face-up tilt, or None - which covers both the face-away
    control and anything that is not a tilt stage at all. Callers that need to
    tell those two apart ask `section()` as well; the control is not an angle
    and must never be plotted as one."""
    n = name.lower()
    if not n.startswith(TILT_PREFIX):
        return None
    head = n[len(TILT_PREFIX):].split("_")[0]
    return float(head) if head.isdigit() else None


def tilt_sort_key(name):
    """Ladder order, control last: it answers a different question from the
    angles and reading it as the ladder's endpoint would overstate it."""
    a = tilt_angle(name)
    return (1, name) if a is None else (0, a)


# ---------------------------------------------------------------------------
# THE WINDSCREEN A/B - the design
#
# "Identical wind/fan exposure with and without the cone foam." A SUFFIX and
# not a family, because the field card asks for the A/B to be
# run over stages that already have families - repeat `quad_matched` and one
# distance stage with and without - so the arm must not overwrite what the
# stage already is. `quad_matched_ws_off` is a quad stage that answers R2 AND
# an arm of the A/B; it is printed in both places, and it is one capture.
#
# The bare pair is `wind_ws_on` / `wind_ws_off`: the protocol own stage, the wind or
# fan exposure with nothing else running.
# ---------------------------------------------------------------------------
WS_SUFFIX = {"_ws_on": "on", "_ws_off": "off"}


def windscreen_state(name):
    """'on', 'off', or None."""
    n = name.lower()
    for suf, state in WS_SUFFIX.items():
        if n.endswith(suf):
            return state
    return None


def windscreen_base(name):
    """The name the two arms share, or None. The class of an arm comes from
    this base - stripping the suffix must not be able to change it."""
    n = name.lower()
    for suf in WS_SUFFIX:
        if n.endswith(suf):
            return name[:-len(suf)]
    return None


def windscreen_pairs(names):
    """({base: {'on': name, 'off': name}}, [lone arm,...]).

    A lone arm is returned separately and never quietly averaged in with the
    complete pairs. An A/B with one arm is not a weak A/B, it is a level
    measurement with no control, and the difference is the whole experiment.
    """
    arms = {}
    for n in names:
        st = windscreen_state(n)
        if st is None:
            continue
        arms.setdefault(windscreen_base(n), {})[st] = n
    pairs = {b: a for b, a in arms.items() if len(a) == 2}
    lone = sorted(n for b, a in arms.items() if len(a) < 2
                  for n in a.values())
    return pairs, lone


# ---------------------------------------------------------------------------
# the names on the field card
# ---------------------------------------------------------------------------
# Only used to warn about a typo, never to refuse a capture: a capture that
# does not happen cannot be re-taken once the rig is packed.
CANONICAL = (
    "quiet_baseline",
    "solo_m1_hover", "solo_m2_hover", "solo_m3_hover", "solo_m4_hover",
    "quad_matched", "quad_spread",
    "dist_near", "dist_mid", "dist_far",
    "speech_2m", "ambient",
    # the design, the tilt ladder
    "tilt_00", "tilt_30", "tilt_60", "tilt_away",
    # the design, the windscreen A/B: the bare pair, plus the two repeats
    # the field card asks for
    "wind_ws_on", "wind_ws_off",
    "quad_matched_ws_on", "quad_matched_ws_off",
    "dist_mid_ws_on", "dist_mid_ws_off",
)


def nearest(name, n=1, cutoff=0.7):
    """Closest canonical name(s), for a typo warning that says which name it
    thinks you meant. `tilt_30` and `tilt_3O` differ by one keystroke and by a
    whole section of the card."""
    return difflib.get_close_matches(name, CANONICAL, n=n, cutoff=cutoff)
