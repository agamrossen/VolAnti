# Running several units

A single unit is a complete device. It listens, and when it hears a multirotor it beeps, flashes red, buzzes, writes the time to the screen and puts every other unit in radio range into full alert. Nothing on this page is required to get that.

Several units do something a single unit cannot: they **cover more ground, and they move the alarm to where the people are**. A unit at the fence hears the aircraft. A unit inside the building is what wakes anybody up.

## Each unit stands on its own

Every unit runs the full detector independently and alerts on its own evidence. There is no voting, no quorum and no requirement for a second unit to agree before the alarm sounds. That is a deliberate choice, and it follows directly from the design priority: **a missed aircraft costs far more than a beep that turns out to be a strimmer.** Waiting for a second opinion costs seconds, and seconds are the entire product.

The false-alarm defence lives inside each unit instead, in the trackers: a detection has to be a real, stable, physically plausible rotor frequency that holds still across consecutive frames. Across every hour of field testing so far that has produced zero false alarms without any help from a neighbour.

What the radio link is for, then, is **spreading an alert, not confirming one**. One unit hears something, and everything in range starts making noise.

## Placing units

Think in terms of coverage, not of pairs.

| Role | Where it goes | Power | What it is doing |
|---|---|---|---|
| **Listening units** | Spread across the approach sides, spaced so their coverage areas touch or slightly overlap | Solar or battery | Hearing the aircraft as early as possible |
| **Indoor units** | Inside or beside the buildings people are in | Mains | Mostly repeating the alert where it matters. They still listen, and they double as an ambient-noise reference and an event recorder |
| **Cold spare** | On a shelf, assembled and flashed | | A swap takes two minutes. A repair takes a week. |

Spacing follows from range, and range follows from your wind. Work from the pessimistic end of [expected-performance.md](expected-performance.md) rather than the optimistic end, and check it with your own measurements.

### Planning arithmetic

Warning time = (detection distance + standoff from the thing being protected) divided by threat speed.

A unit that reliably hears an aircraft at 100 m, sited 300 m out from the building, gives roughly **15 seconds** of warning against something moving at 27 m/s. The same unit in a 6 m/s wind might only hear it at 30 m, and the same geometry then gives about 12 seconds. Wind is the variable that matters; run the arithmetic at both ends and site the listening units so the pessimistic answer is still enough time for people to act.

These are planning figures derived from measured factors, not guarantees.

## Siting each unit

- **Face up, parallel to the ground.** The array is omnidirectional at drone fundamentals, so the only orientation variable that matters is the unit's own body shadowing (3 to 6 dB at the fundamental, more at the harmonics the envelope tier uses). Face-up minimises it.
- **Shade.** Direct sun cooks the cell and ages the e-paper. Under an eave, on a north face in the northern hemisphere, or under a small printed hood.
- **A few metres clear of steady noise**: air conditioning, generators, transformers. They will not false-alarm the detector, but they raise the floor and cost detection range exactly where it is needed.
- **Height 2 to 4 m** is a good default. Above head height, below rooflines that shadow the direction of interest.
- **Do not publish unit positions.** Build reports and photographs are welcome. A map of where the sensors are is information an adversary can use. Blur it or leave it out.

## Mains-powered units

The USB input is the charger, and **the battery stays fitted**. It is the surge buffer that keeps the rail up when the beeper, motor and radio all fire together, which is a measured failure without it. See [how-it-works.md](how-it-works.md).

- A locally certified 5 V USB power supply, indoors or in a rated box.
- Braided USB-A to USB-C cable, **2 m or shorter at 22 AWG** conductors. Thin long cables drop enough voltage to matter at alert load.
- A **drip loop** below the unit's port, and a wrap of self-amalgamating silicone tape over the connector entry.
- Supply circuit on an RCD. A surge-protected strip is cheap insurance where lightning is a thing.

## Service schedule

| Interval | Action |
|---|---|
| Weekly | Glance: green blink present, screen sane, alert count noted |
| 3 months | Open one representative mains unit and inspect the cell for swelling. Permanently float-charged cells are the main long-run risk in a fleet. |
| 6 months | Replace cells. Check the silica sachets: a saturated sachet means that unit's seal has failed, so reseal it before it goes back up. |
| After any alert | Pull the event log and record it. Real events are the rarest data this project has. |

## Drills

An early-warning system nobody has rehearsed against is decoration. Decide in advance what an alert means, who moves where, who checks and who stands down. Walk it once with everybody. Then use the snooze and test behaviour to run a short drill each month.

Seconds of warning only help people who already know what to do with them.
