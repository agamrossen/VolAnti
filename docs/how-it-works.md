# How VolAnti works

This is the technical walkthrough: enough depth to judge whether the device is worth building, without reproducing the full engineering record. Every number here was measured on real hardware unless labelled otherwise.

## 1. The signal we hunt

Every electric multirotor emits a **blade-pass harmonic comb**: a fundamental at `BPF = (RPM ÷ 60) × blade count`, plus copies at exact integer multiples reaching past 8 kHz. Bigger props spin slower, so 7–9″ fibre-optic-class airframes sit low: hover fundamentals around 375–475 Hz for our reference airframe (7″ three-blade props on 2807-class motors), measured at 480–570 Hz at high throttle. A three-blade prop presents **two related comb families** (shaft rate and 3× blade rate); the detector treats ×2/×3 family hops as one source, because Field-1 measured exactly that alternation on real loaded props.

The detector scores live audio against this physics. It never compares against recordings, so scarce, compressed combat audio is not a limitation, and any airframe sharing the physics is in scope.

<!-- IMAGE: comb.png: synthetic blade-pass comb rising out of a pink-noise floor, annotated teeth vs gaps. -->

## 2. The detection stack

Audio path: 4 microphones → coherent sum → 2048-point FFT at 16 kHz, hop 512 samples, so **one full decision every 32 ms**. Four detector tiers then run together on the production board (measured p99 frame time 29.4 ms with three heavy tiers phase-scheduled onto disjoint frames; zero frame overruns in 1,938 consecutive frames).

| Tier | Mechanism | Catches | Latency |
|---|---|---|---|
| **1: fast comb** | Adaptive background floor (6 s rise) → spectral whitening → comb score 70 Hz–2 kHz (teeth minus gaps, normalised) → tracker demanding 6 consecutive in-tolerance frames | Approaching, manoeuvring, throttle-changing sources | 0.23 s |
| **2: slow comb** | 30 s floor, priority band, 44-of-63 half-rate M-of-N tracker | Steady and loitering sources that Tier 1's fast floor absorbs after ~15 s | 1.4–2.0 s |
| **3: envelope "wash"** | High-pass > 3 kHz → amplitude envelope → comb in the *envelope spectrum* over rotation rates 100–320 Hz | The broadband prop hiss; structurally immune to floor absorption; the only tier that caught our very first real rotor | ~1 s |
| **4: no-floor slow comb** | 2 s Welch PSD, whitened **across frequency** by local median (no temporal floor at all), comb 110–700 Hz with ×2/×3 family logic | A drone already hovering when the unit powers on, indefinitely | 2.5–4 s |

The alert is the OR of the enabled tiers, with per-tier attribution in the event log. Each tier exists because a measured failure mode of the others demanded it; none is speculative.

**Why zero false alarms is architectural, not lucky.** Loud steady noise (traffic, generators, HVAC, voices at distance) raises the floor estimate, and scoring happens *relative* to the floor, so noise costs detection range rather than credibility. The trackers then require a tone to hold frequency within 2 % across consecutive frames, which drops gunshots, shouts, and door slams. Across every lab and field hour logged to date: zero false alarms. The honest confusers we have measured are close sustained speech (fires at conversational distance, disclosed to operators) and idling engines (their combs sit still while a closing drone's slides; broader tones; the tracker discriminates but this is the hardest class).

## 3. The microphone array

Four ICS-43434 I2S MEMS microphones in a corner square, 56.00 mm sides, **79.2 mm corner to corner**, all clocked from one source so the streams stay sample-aligned. The array's job is **quiet, not direction**: coherently summing four channels adds the drone's sound in phase while microphone self-noise adds incoherently, worth about +6 dB, roughly a doubling of range.

We measured, rather than assumed, that the array cannot beamform at the comb frequencies: 79 mm is λ/10 at 425 Hz, and measured steering gain across the comb band is −0.02 to +0.03 dB. The coherent sum already *is* the only beam this aperture has. Above ~4–5 kHz the enclosure's cone mouths become mildly directive, so mounting tilt, not electronics, is the beam. Consequence for you: **the device is effectively omnidirectional at the frequencies that matter; mount it face-up** and let body shadowing (3–6 dB at the fundamental) be the only orientation variable.

## 4. Alerts and the operator surface

- **Beeper** (TMB12A03) and optional **vibration motor**: unmissable at belt distance.
- **RGB LED** through a light pipe: green blink every 2 s while guarding, solid blue in snooze, fast red in alert.
- **Waveshare 1.54″ e-paper (200 × 200)**: LISTENING screen with uptime, alert count, enabled tiers, threshold, and per-microphone health; the alert screen **persists with the power off**, so a unit found dead still tells you what it last heard. Three read-only menu pages; roughly 30 redraws a day, so the panel lasts.
- **One button**: tap to page through the menu while guarding; any press snoozes an active alert; long-press runs the clean shutdown ritual.
- **Self-interference safety**: while the beeper or motor is active, the envelope-based tiers freeze their trackers, because the device's own outputs are periodic sources sitting on the same box as the microphones. This is enforced in firmware for every tier, present and future.

## 5. Peer alerting (LoRa)

Ra-01H (SX1276) module, spring antenna, 868/915 MHz depending on region (**frequency is a configuration parameter; verify your local allocation**: see [configuration.md](configuration.md)). Protocol v0 is deliberately minimal: an 18-byte versioned packet, MAC-derived unit identity, all-units broadcast, no relaying. A unit that confirms a detection **puts every unit in range into full alert mode**, beeper and all, not merely a notification. The link spreads an alert; it does not vote on one. Every unit decides for itself (see [deployment.md](deployment.md)).

## 6. Power

USB-C in → BQ24074 power-path charger → protected 1S LiPo → TPS63020 buck-boost → 3.3 V rail. The unit runs while charging and guards from power-on in about 2 seconds with zero interaction.

**The battery is mandatory even on mains power.** Measured on the production board: the alert load (beeper + motor + LoRa transmit simultaneously) browns out a USB-only supply. The cell is the rail's surge buffer, not just backup. Runtime on the 2500 mAh reference cell: ~18–22 h at typical guard draw.

## 7. Honest limits

- **Wind is the binding constraint.** Above ~5 m/s, upwind sound can lose 20 dB before it arrives; no array recovers that. Windscreen cloth over the mic ports is the cheapest range purchase available and is part of the standard build.
- **Quiet-by-design aircraft defeat the method outright.**
- **No bearing.** The device tells you something is coming, not from where.
- **Friend or foe is indistinguishable.** Your own drone triggers it.
- **This is a short-range early-warning aid.** Seconds of warning, not minutes. Plan around that.

For measured performance under stated conditions, see [test-results.md](test-results.md) and [expected-performance.md](expected-performance.md).
