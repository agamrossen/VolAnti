"""
site_profile.py - what the device is expected to hear at a deployment site.

The profile describes a rural, agricultural site on exposed high ground, with a
sparse civilian population and regular military activity in the area. It is not a city, and
the confuser library must not be weighted as though it were. Anyone deploying
somewhere different should write weights for their own site.

Why this file exists
--------------------
The operating threshold was first chosen as "the lowest threshold with zero
false events anywhere in the corpus". That is a max over families, so the
single worst family sets the sensitivity of the whole device - and for a long
time that family was motorbike_accel, an accelerating civilian motorbike,
which at this kind of site is close to absent. A source that is nearly never
present was setting the detection range for a life-safety device.

The fix is not to delete the family (it is a real acoustic phenomenon, and
deleting negatives to move a threshold is exactly the laundering the harness
exists to prevent). The fix is to weight each family by how much of the
deployment it actually occupies, and to budget false alarms per real hour.

Definition
----------
weight[f] = estimated fraction of deployment hours during which family f is
            audible at the sensor.

Weights deliberately do not sum to 1: several sources overlap constantly (wind
is audible almost always, birds overlap wind, a generator overlaps everything).
The weighted false-alarm rate is therefore

    FA/h_weighted = sum_f  weight[f] * (false events per hour of family-f audio)

which is the expected number of false alerts in a real deployment hour.

These weights are estimates
---------------------------
They are engineering judgement from a site description, not survey data, not
field logging and not measurement. No acoustic survey has been done. Every
number below could be wrong by a factor of two or more, and the ones that
matter most (wind, generators, rotorcraft) should be replaced with logged data
as soon as a device has been left running on site for a week. Until then,
treat every weighted FA/h figure as an estimate with the same uncertainty as
these weights.

Sensitivity to that uncertainty is reported by evaluate.py: it prints the
weighted rate and the unweighted rate side by side, so it is always visible
how much of the answer is coming from the weighting.
"""

# name -> (weight, rationale). The rationale is the argument, not decoration:
# if you disagree with a number, you should be able to see exactly what claim
# it rests on and argue with that claim instead.
PREVALENCE = {
    # ---------------- ambient, near-permanent -------------------------------
    "noise_only": (
        1.00, "Ambient bed. Something is always audible; this is the "
              "floor case and is present in 100% of deployment hours."),
    "wind_sustained": (
        0.55, "Exposed site. Steady wind is the dominant ambient. "
              "Estimate: over half of all hours have enough wind to set the "
              "noise floor."),
    "wind_gusting": (
        0.25, "Gust fronts are a subset of windy hours - the ones that stress "
              "the noise-floor estimator rather than merely raise it. "
              "Estimate: roughly half of windy hours are gusty."),
    "broadband": (
        0.15, "Rain, vegetation rustle, general road roar. Seasonal, with a "
              "wet winter and a dry summer. Estimate."),

    # ---------------- biological -------------------------------------------
    "birds": (
        0.35, "Dawn and dusk chorus plus daytime activity, year-round, rural. "
              "Estimate: ~8 h/day of meaningful bird activity."),
    "insects": (
        0.30, "Cicadas and crickets, warm months, mostly night. Strongly "
              "seasonal - closer to 0.6 in summer and ~0 in winter. The "
              "estimate is the annual average."),
    "dogs": (
        0.20, "Dogs, which bark at night, when the device matters most. "
              "Estimate of hours containing at least one bark sequence."),
    "livestock": (
        0.15, "Cattle, goat and sheep husbandry is normal in a farming "
              "community. Herds are not always within earshot of one sensor. "
              "Estimate."),

    # ---------------- fixed plant, runs for hours ---------------------------
    "diesel_generator": (
        0.12, "Standby generators at unoccupied buildings, pumping stations "
              "and equipment huts. Estimate, and the one most likely to be "
              "too low - if a generator is within earshot of the sensor its "
              "weight for that sensor is effectively 1.0."),
    "hvac_outdoor_unit": (
        0.10, "Air-conditioning condensers on building exteriors. Summer-"
              "weighted. Estimate."),
    "irrigation_pump": (
        0.10, "Scheduled irrigation, seasonal and typically overnight. "
              "Estimate."),
    "steady_machine": (
        0.05, "Generic grounds machinery: mowers, strimmers, generators of "
              "unknown type. Daylight only. Estimate."),
    "tonal": (
        0.03, "Alarms, resonances, electrical whine. Rare and short. Estimate."),

    # ---------------- impulsive noise, rotorcraft, heavy vehicles -----------
    "artillery_distant": (
        0.10, "Distant gunfire, demolition and impacts, weighted as a regular "
              "part of the soundscape rather than a rare event. Estimate, and "
              "highly situation-dependent."),
    "helicopter": (
        0.08, "Rotorcraft on station or orbiting. Deliberately raised from "
              "the generic-site assumption that a helicopter is a rare event. "
              "Estimate."),
    "helicopter_flyby": (
        0.06, "Rotorcraft transiting. Companion to the above. Estimate."),
    "apc_wheeled": (
        0.05, "Wheeled armoured vehicles on nearby roads. Estimate."),
    "diesel_truck": (
        0.04, "Heavy logistics traffic. Estimate."),
    "tracked_vehicle": (
        0.02, "Tracked vehicles. Present but far less frequent than wheeled "
              "traffic, and usually on transporters rather than on tracks. "
              "Estimate."),

    # ---------------- agricultural -----------------------------------------
    "tractor": (
        0.06, "Field and orchard work, daylight, seasonal peaks. Estimate."),
    "orchard_machinery": (
        0.03, "Air-blast sprayers and orchard fans. Campaign-based: near zero "
              "most of the year, then several hours a day for a fortnight. "
              "Estimate of the annual average."),

    # ---------------- civilian road traffic: low at this site ---------------
    "distant_traffic": (
        0.05, "A road exists, but the area is rural and sparsely populated. "
              "Lowered hard from any urban assumption. Estimate."),
    "passing_vehicle": (
        0.03, "Individual civilian vehicles passing close. Sparse. Estimate."),
    "prop_aircraft": (
        0.02, "Light fixed-wing overflight, rare in this airspace. Estimate."),
    "motorbike_accel": (
        0.005, "An accelerating civilian motorbike within earshot. This is "
               "the family that once set the threshold for the entire "
               "project. At a sparsely populated rural site it is close to "
               "absent: an estimate of roughly one audible occurrence per 200 "
               "deployment hours. Not deleted - the physics is real and it "
               "stays in the corpus - but it no longer sets the sensitivity "
               "of a life-safety device on its own."),
}

# ---------------------------------------------------------------------------
# The wash tier's confusers
#
# These four are not in PREVALENCE above. They are kept separate because every
# weighted-FA number for v1 and Tier 2 was computed over PREVALENCE exactly as
# it stands, and widening that dictionary would change those numbers without
# changing the detector. Tier 3's budget is computed over PREVALENCE +
# T3_PREVALENCE; v1's and Tier 2's remain over PREVALENCE alone.
# ---------------------------------------------------------------------------
T3_PREVALENCE = {
    "speech": (
        0.20, "Human voices near the device. In a small community a device on "
              "a perimeter mast hears conversation, children and a PA system "
              "for part of most days. This replaces the speech_like proxy, "
              "which was fitted to nothing and ran 15 dB hot at 500-707 Hz - "
              "inside the priority band - and on its own accounted for the "
              "difference between v1's true 1.96 wFA/h and its published "
              "3.57. Estimate."),
    "box_fan": (
        0.08, "Ventilation fans on farm buildings, shelters and equipment "
              "cabinets. The real near-enemy of the wash tier: identical "
              "mechanism, different rate. Weight is low because a fan has to "
              "be within a few tens of metres to matter. Estimate."),
    "rain": (
        0.12, "Wet-season rain on the enclosure. A control family as much as "
              "a confuser - it is broadband and high-band-heavy with no "
              "periodicity at all, so it tests whether the wash tier fires on "
              "the band rather than the modulation. Estimate."),
    "wind_at_capsule": (
        0.30, "Wind across the device's own microphone cones, shedding "
              "vortices at St*U/D. Weighted high because it is not a source "
              "in the environment that may or may not be present - it is the "
              "device's own housing, present in every windy hour, on an "
              "exposed site. Predicted from a Strouhal number and a caliper; "
              "never recorded. See detector_t3 for the shedding rates."),
}


def t3_weights():
    """Weights for a Tier-3 budget: the site profile plus the wash tier's own
    confusers. Never use these for a v1 or Tier-2 number."""
    w = {k: v[0] for k, v in PREVALENCE.items()}
    w.update({k: v[0] for k, v in T3_PREVALENCE.items()})
    return w


# Families that are not counted in the false-alarm budget, and why.
FRIENDLY = {
    "friendly_multirotor": (
        0.04, "Friendly quadcopters operating in the same airspace. Excluded "
              "from the FA budget on purpose. A friendly multirotor emits the "
              "same blade-pass comb as a hostile one; the only way to stop "
              "firing on it is to desensitise the detector, which is "
              "discrimination by blindness and would cost exactly the "
              "detection range this weighting exists to buy back. It is "
              "measured and disclosed as a hard limitation instead."),
}


def weights(include_friendly=False):
    w = {k: v[0] for k, v in PREVALENCE.items()}
    if include_friendly:
        w.update({k: v[0] for k, v in FRIENDLY.items()})
    return w


def rationale_table():
    lines = [f"{'family':<20}{'weight':>8}  rationale (all values are estimates)"]
    for k, (w, why) in sorted(PREVALENCE.items(), key=lambda kv: -kv[1][0]):
        lines.append(f"{k:<20}{w:>8.3f}  {why}")
    for k, (w, why) in FRIENDLY.items():
        lines.append(f"{k:<20}{w:>8.3f}  [excluded from FA budget] {why}")
    return "\n".join(lines)


if __name__ == "__main__":
    print(rationale_table())
