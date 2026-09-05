# Expected performance in your conditions

[test-results.md](test-results.md) is what we measured. This page is what *you* should expect, derived from those measurements plus well-understood acoustics, so you can sanity-check your build and plan a deployment. Everything here is **derived, not measured**, and it says so per row. When your numbers disagree with this table, trust your numbers and please report them.

## How the derivation works (so you can redo it)

Acoustic detection range scales in dB, and dB stack:

- **Source loudness**: a full four-motor airframe under load is roughly **+6 dB (~2× range)** over our single loaded test rotor. Heavier airframes carrying load run louder still.
- **Integration**: the slow tiers gain ~**+3 dB per doubling of integration time** on a steady source (a flight controller holds RPM far tighter than the hand throttle we measured drifting 2.6 %/s).
- **Wind**: the dominator. Spherical spreading loses 6 dB per doubling of distance, so **every 6 dB of wind masking halves range**; measured upwind conditions can take 20 dB, which is the difference between 150 m and 15 m. This is physics; no processing recovers it.
- **Direction**: the array is omnidirectional at drone fundamentals; only body/mounting shadowing (3–6 dB) matters. Face-up mounting removes most of it.

## The table

Detection distance for a **full 7–9″-class multirotor under load, closing on the unit**, unit mounted face-up in the open with windscreen cloth fitted:

| Conditions | Wind (rough guide) | Expect detection at | Confidence |
|---|---|---|---|
| Ideal: still air, quiet rural site | < 2 m/s, leaves still | **~100–200 m** | Derived (single-rotor measurement + source and integration factors). The wide band is honest: nobody has measured the top end yet. |
| Typical: light breeze, ordinary background | 2–5 m/s, leaves moving | **~50–120 m** | Derived; consistent with the measured ladder scaled by measured wind penalties. |
| Breezy: moderate wind, or noisy site (traffic, machinery) | 5–8 m/s, branches moving | **~15–50 m** | Partially anchored: our 14 m loaded-prop measurement in changing wind sits at this band's bottom edge with a smaller source. |
| Strong wind | > 8 m/s | **Tens of metres at best; assume degraded** | Physics. Above ~11 m/s steady, wind over the mic ports adds its own noise mechanisms. Treat the device as impaired and say so to anyone relying on it. |
| Indoors / demo, speaker or small drone | n/a | Room-scale, reliably | Measured, routinely. |

Warning time = detection distance ÷ closing speed. At 27 m/s (a fast FPV run-in), 120 m is ~4.5 s and 50 m is under 2 s; slower profiles give proportionally more. **This is why multi-unit forward placement exists** ([deployment.md](deployment.md)): the units, not the algorithm, are how you buy tens of seconds.

## Expect these behaviours (they are features)

- **A steady distant hover takes longer to alarm than an approach** (the slow tiers carry it; 2–4 s rather than 0.23 s).
- **Loud steady noise near the unit shortens range without causing false alarms.** The floor rises; scores are relative to it. If your site is loud, your table shifts one row down.
- **Your own drone triggers it.** There is no friend-or-foe.
- **Close conversation can trigger the comb tiers.** Known, disclosed, and rare in outdoor mounting positions.

## The one measurement that beats this whole page

Fly or carry a real source past your installed unit once, at a known distance, in your typical wind, and write down what happened. Ten minutes, and you now know more about your site than this table can tell you. Then [report it](test-results.md#report-your-build).
