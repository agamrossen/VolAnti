# Test results

Everything below is real measured data, with date and conditions, in the order it happened. We publish conditions with every number because a range figure without its wind speed is fiction. Estimates and derivations live in [expected-performance.md](expected-performance.md), not here.

## Bench detection ladder: 2026-08-13

Single 2807-class motor with a 7″ three-blade propeller on a bench rig, dynamic throttle, quiet indoor conditions, through the full four-microphone array.

- **Detection at 25–30 m: the far wall of the longest space available, not the distance where detection failed.** The device never reached its limit indoors.
- First confirmed real-rotor detections locked at fundamentals of **423 Hz and 471 Hz**, inside the 375–475 Hz band predicted from the airframe's physics before any rotor had been heard. The physics-first band-setting approach held.

## Field-1: 2026-08-21, open ground, mild changing wind

Four, then two, **loaded** 7″ propellers on the rig (loaded props are several times louder than the free-spinning bench motor and are the honest stand-in for a carrying airframe).

- Detection to **~14 m** in changing wind with the fast tier + wash tier pair. Wind, not electronics, set the number.
- Close steady hovers were detected intermittently, and fast "punch-out" throttle spikes were mostly missed. Both traced to specific tracker mechanisms (background-floor absorption of steady tones; the two-comb-family alternation of three-blade props), both fixed by the slow-comb and family-logic tiers now in the stack. This session is *why* tiers 2 and 4 exist.
- **Zero false alarms** across the full field day and all lab hours before it.
- Loaded props measured **+5.9 dB in the 125–1000 Hz scoring band at 4 m** over site wind, and up to +50 dB in the high band the wash tier uses.

<!-- IMAGE: field1-rig.jpg: the propeller rig staked on open ground, unit in the foreground. -->

## Confuser testing

Synthetic confuser corpus, replayed through the full pipeline: helicopter, propeller aircraft, diesel truck, motorbike, distant traffic, HVAC. **No false alarms.** Real-air confusers found so far: **close sustained speech** at conversational distance can fire the comb tiers (disclosed to operators; genuinely comb-like), and idling engines are the hardest honest class (their combs sit still; a closing drone's slides). Real livestock, wind-machine, and generator hours are being accumulated and this section will grow.

## Production PCB verification: 2026-08-28

First bring-up of the production board (rev A2):

- Detector output **bit-identical to the reference implementation** on the golden test vectors, to the last decimal place. The algorithm you flash is provably the algorithm that produced every number on this page.
- Three tiers running together: **p99 frame time 29.4 ms against the 32 ms budget, zero overruns in 1,938 frames**, 7-minute sustained guard.
- All four microphones matched within **±1 dB** after per-channel calibration.
- One finding that shaped the build guide: with no battery fitted, simultaneous beeper + motor + radio transmit browns out a USB-only supply. **The battery is part of the power design, not an accessory.**

## Field measurements

Every measured detection, one row each, newest last. The table is empty until the current campaign produces rows: nothing is entered here that was not observed, and nothing is rounded up. Entries from other people's builds are added from their [build reports](#report-your-build) with the builder credited.

| Date | Build | Source | Distance | Result | Wind | Environment | Tier | Notes |
|---|---|---|---|---|---|---|---|---|
| | | | | | | | | |

**Columns.** *Distance* is the measured separation between the source and the unit. *Result* is detected or not detected, not a score. *Wind* is in m/s where measured and in the Beaufort description otherwise. *Tier* is which detector fired first. A "not detected" row is as valuable as a "detected" row and is never quietly dropped.

For what these numbers are expected to become in other conditions, see [expected-performance.md](expected-performance.md), which is derived rather than measured and says so on every row.

## Field sessions: upcoming

The next field campaign (distance ladders per motor count, wind captures, windscreen A/B) is running now; results, photos, and the raw event logs land here as they exist. This section is honest about its size: we will publish what we measure, and early on that will not be a large dataset.

---

## Report your build

Independent replications, including failures, are the most valuable data this project can receive. Open an issue titled `Build report: <location-ish, month>` containing:

```
Build type:        full PCB / breadboard
Firmware version:  (from the boot banner)
Mics healthy:      4/4? (boot report values)
Test source:       what made the sound (drone model / rig / speaker)
Distance(s):       detected at ___ m, not detected at ___ m
Wind:              calm / light (leaves move) / moderate (branches move) / strong
Environment:       open field / urban / indoor
False alarms:      count over ___ hours, and what caused them if known
Notes & photos:    (no sensor-position maps, please)
```

Every report gets folded into this page's aggregate table once enough exist to aggregate.
