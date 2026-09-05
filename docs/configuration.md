# Configuration

The shipped image is the validated field configuration: flash it and the unit guards correctly with zero setup. Everything below is runtime-adjustable over the serial console and persists across power cycles. Change things deliberately; every default here was set by measurement, not preference.

## Detection threshold

Two named operating points for the fast-comb tier:

| Setting | Value | Character |
|---|---|---|
| `HIGH_ALERT` (**deployment default**) | 1.70 | Maximum sensitivity that the false-alarm budget allows. Chosen because for this application a missed drone costs vastly more than a false beep. |
| `NORMAL` | 2.14 | Quieter life for benign environments (workshop, demo, indoor testing). |

The threshold is not the main false-alarm defence; the trackers are. Dropping below 1.70 buys little real sensitivity (the trackers gate detections, and they were the measured limit in field testing) while spending the alarm budget. If you believe your environment justifies a change, measure first: run at the candidate threshold for a full day and count.

## Tiers

Each tier enables independently (`t1=1`, `t2=1`, `t3=1`, `t4=1` style flags, shown on the LISTENING screen). Shipped state: **all four on**. Tier 4, the no-floor comb, is the most sensitive of the four and the one that keeps working where the background never really goes quiet. Turn a tier off only if a specific site gives you a reason to, and record what changed and why in your build report.

## Radio region: you must set this

The LoRa alert frequency is a **configuration parameter with no universal legal default**:

| Region | Band |
|---|---|
| EU / UK | 868 MHz |
| Israel | 917–920 MHz (verify current allocation before deploying) |
| US | 915 MHz |

The Ra-01H module covers all of these. Transmitting on the wrong band in your country is illegal; check your allocation, set it, and note that all units at a site must match. Duty cycle is naturally tiny (an 18-byte packet per alert event), well inside every region's limits.

## Alert behaviour

- **Snooze**: any button press during an alert silences outputs for 10 s; detection continues underneath and the display keeps the warning.
- **LED language**: green blink 150 ms every 2 s = guarding; solid blue = snoozed; fast red = alert.
- **Shutdown**: long-press runs the honest-OFF ritual (state saved, screen updated, then power down), so a unit is never ambiguous about whether it is guarding.

## What is deliberately not configurable

Sample rate, FFT geometry, the tracker rules, and the whitening chain are fixed. They are the calibrated instrument; the golden-vector test suite pins their behaviour bit-for-bit, and a "small tweak" there silently invalidates every published number. If you want to experiment with the algorithms, do it in the offline Python pipeline in [tools/](../tools/) where the same golden vectors will tell you exactly what changed.
