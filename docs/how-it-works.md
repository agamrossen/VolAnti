# How it works

The device measures how strongly the sound spectrum looks like an evenly spaced comb of harmonics, and refuses to alert until that pattern has held still long enough not to be an accident. This page is the numbers behind that sentence. Every stage below runs live in [the simulator](https://volantitech.com/#simulator).

## The signal

A propeller with B blades at R revolutions per second chops the air B×R times a second. That is the fundamental, f0. The chopping is not a sine, so energy lands at 2×, 3×, 4× and so on: a harmonic comb.

<img src="images/comb.png" width="720" alt="Measured spectrum of a loaded 7 in three-blade propeller">

For a three-blade propeller the detector tracks two comb families, at the 2× and 3× spacing, so a detection survives one family being masked. The predicted hover band for the airframe this was built against was 375 to 475 Hz. The first real-rotor locks landed at 423 and 471 Hz. Loaded flight sits higher, 480 to 570 Hz, which is what a propeller doing work should do.

## Capture

| Stage | Setting |
|---|---|
| Microphones | 4 × ICS-43434, digital MEMS, one shared I2S clock |
| Array | Plus pattern, 79 mm corner to corner, ±28 mm on each axis |
| Sample rate | 16 kHz per channel |
| Combination | The four channels are summed |
| FFT | 2048 points, every 512 samples, so one frame every 32 ms |
| Bin width | 7.8 Hz |

At the frequencies that matter, 79 mm is about a tenth of a wavelength, so the array is effectively a point. Sound from any direction adds in phase, uncorrelated capsule noise does not, and the sum gains about 6 dB of signal to noise while staying nearly omnidirectional. Measured grazing loss across the comb band was between −0.02 and +0.03 dB.

That measurement closed a design question. Beamforming was tried and dropped. At 79 mm the ratio of aperture to wavelength at 450 Hz is about 0.10, and steering does nothing until around 2 kHz. The sum already is the beam. The array buys sensitivity, not direction.

## Floor

The detector keeps a running estimate of the site's normal spectrum in every bin and subtracts it. Quiet changes are learned with a 6 s time constant, so a new tone stands out for seconds. Loud broadband bursts, wind gusts, a lorry, are learned in 0.8 s, so they cannot park themselves in the model. The whitened value is capped at 2.5 so nothing can dominate.

The floor has a known weakness. A drone that arrives and hovers is learned into the fast floor after about 6 s and disappears from tier 1. Tiers 2 and 4 exist because of that.

## Score

For every candidate f0 from 70 to 2000 Hz in 1 Hz steps, 1931 candidates a frame:

    score = (energy on the comb teeth − energy in the gaps) / √K

with K the number of teeth in band. A high score means the spectrum is combed at that spacing, not just loud. The best candidate has to win six frames in a row within 2 % before anything is allowed to happen.

## Four detectors

| Tier | Floor | Band | Decision | Threshold | Latency |
|---|---|---|---|---|---|
| 1 Fast comb | 6 s rise, 0.8 s fall | 70 to 2000 Hz, alerts 200 to 810 Hz | 6 frame chain, family continuity across 1, 2, 3, ½, ⅓ | 1.70 | 0.23 s |
| 2 Slow comb | 30 s | 200 to 800 Hz | 44 of 63 frames | 1.40 | 1.4 s and up |
| 3 Envelope wash | none, whitened across frequency | modulation rate 100 to 650 Hz on the envelope above 3 kHz | 16 of 23 | 20 | 1 to 3 s |
| 4 No-floor comb | none, 64 frame Welch median, whitened across frequency | f0 110 to 700 Hz | 5 of 8 updates | 30.5 | 5 to 15 s |

Tier 1 catches arrivals and changes. Tier 2 catches an arrival that then hovers, because its floor has not caught up. Tier 3 catches loaded, close, high thrust flight, where the giveaway is a roar modulated above 3 kHz rather than a clean comb. Tier 4 catches a long hover in a place that never goes quiet, because nothing is ever learned into a floor. It is the most sensitive and the slowest, and it is the one that fired at 104 m.

The alert is the first tier to be sure. Outputs hold for 5 s with a countdown on the screen, and the slow accumulators freeze while the outputs are active so the unit's own beeper cannot feed the detector.

## Why the thresholds are low

For this use a missed aircraft costs far more than a false beep. So the thresholds sit low and the false alarm defence is the trackers: a real, stable, physically plausible rotor rate that persists. That rejects noise better than a high threshold does, because a high threshold also rejects quiet real drones.

## Sealed

Tier 1 is pinned by golden test vectors. A given input file must produce the same score to the last decimal place on a laptop and on the board, and on the production PCB it does, bit for bit. No future change can move it silently.
