<div align="center">

<img src="docs/images/logo.svg" width="64" alt="">

# VolAnti

<img src="docs/images/assembled-iso.png" width="400" alt="VolAnti, assembled">

### An open-source acoustic drone detector that hears propellers, not radio.

A 91 mm box with four microphones, an ESP32-S3, an e-paper screen and a long-range radio link.
It listens for the harmonic pattern a multirotor makes, and when it hears one it beeps, flashes red, buzzes,
writes the time to a screen that survives a power cut, and tells every other unit on the network.

<br>

<table>
<tr>
<td align="center" width="20%"><h3>100 to 200 m</h3>expected detection range<br>in still air</td>
<td align="center" width="20%"><h3>0.23 s</h3>from first sound<br>to alarm</td>
<td align="center" width="20%"><h3>zero</h3>false alarms across<br>field testing to date</td>
<td align="center" width="20%"><h3>18 to 22 h</h3>on one<br>battery charge</td>
<td align="center" width="20%"><h3>£50 to £80</h3>parts, per unit,<br>at small quantity</td>
</tr>
</table>

<sub><b>Range is derived, not measured.</b> It comes from applying measured scaling factors to a measured bench result, and it collapses in wind.<br>
The single outdoor measurement so far is <b>14 m</b> with loaded propellers in a mild breeze. Both numbers, and how they were arrived at, are below.</sub>

<br>

**[Website](https://agamrossen.github.io/VolAnti/)** · **[Try it on a breadboard](docs/breadboard-build.md)** · **[Build the full unit](docs/build-guide.md)** · **[Parts and prices](#parts)** · **[How it works](#how-it-works)** · **[Build photos](docs/gallery.md)**

**Detection and alert only.** No jamming, no interception, no countermeasures, ever. [Why](#scope).

</div>

---

## Contents

<table>
<tr>
<td valign="top" width="33%">

**Understand it**

[The problem it solves](#problem)
[What the device does](#what-it-does)
[How it works](#how-it-works)
[Range and performance](#performance)
[What it gets wrong](#limits)

</td>
<td valign="top" width="33%">

**Build it**

[Two build routes](#routes)
[Parts and prices](#parts)
[The PCB](#pcb)
[The enclosure](#enclosure)
[Firmware and flashing](#firmware)
[Assembly, step by step](#assembly)

</td>
<td valign="top" width="33%">

**Run it**

[Configuration](#configuration)
[Running several units](#several)
[Service and upkeep](#service)
[Test results](#results)
[Report a build](#contributing)

</td>
</tr>
</table>

**Also:** [the website](https://agamrossen.github.io/VolAnti/), a shorter illustrated version of this page · [build photo archive](docs/gallery.md)

**Files:** [firmware](firmware/) · [gerbers](hardware/pcb/gerbers/) · [bill of materials](hardware/pcb/bom.csv) · [pick and place](hardware/pcb/pick-and-place.csv) · [schematics](hardware/schematics/) · [printable enclosure](hardware/enclosure/stl/) · [offline tools](tools/)

**Reference:** [full build guide](docs/build-guide.md) · [breadboard guide](docs/breadboard-build.md) · [flashing](docs/flashing.md) · [configuration](docs/configuration.md) · [deployment](docs/deployment.md) · [test results](docs/test-results.md) · [expected performance](docs/expected-performance.md) · [disclaimer](DISCLAIMER.md) · [licensing](LICENSE.md)

---

<a name="problem"></a>
## The problem this was built for

Most drone detection on the market is **radio-frequency detection**. It listens for the control link between a pilot and an aircraft, decodes the protocol, and reports the drone's position and often the pilot's. It is accurate, it is fast, and against a large category of threat it is the right tool.

It has one hard failure mode: **it needs the drone to transmit.**

The aircraft this project was built against does not transmit. A **fibre-optic FPV drone** trails a hair-thin spool of glass fibre back to its operator. Video comes down the fibre, control goes up the fibre, and the radio spectrum stays silent. There is nothing to detect and nothing to decode. The same is true of an aircraft flying a route loaded before take-off with its transmitter switched off.

What the aircraft cannot hide is that it is **four motors spinning propellers**. A 7-inch three-blade propeller on a 2807-class motor at hover produces an acoustic signature that is not a vague buzz. It is a **precisely structured comb of harmonics**, spaced by the blade-passage frequency, and that spacing is set by physics the airframe cannot switch off.

VolAnti listens for that structure.

**Where this fits.** It is not a replacement for radar, for RF detection, or for a national air-defence picture. It is a **local, cheap, quiet tripwire for the last few hundred metres** that keeps working when the radio picture is empty, and that a school or a farm or a small community can build, own and repair themselves.

---

<a name="what-it-does"></a>
## What the device actually does

<img src="docs/images/assembled-face.png" align="right" width="290" alt="The face of the unit: four microphone grilles, e-paper screen, light pipe, snooze button">

It runs continuously and does one job. Every **32 milliseconds** it takes a new slice of sound, folds the four microphone channels together, measures how comb-like the spectrum is, and compares the result against four independent detectors.

When one of those detectors fires, five things happen at once.

| Channel | What happens | Why it is there |
|---|---|---|
| **Beeper** | A loud, patterned tone from the piezo beeper | The channel that wakes people and carries through a wall |
| **RGB LED** | Solid **red** through the light pipe in the lid | Readable across a yard at a glance, day or night |
| **Vibration motor** | Continuous haptic buzz | Works when the beeper cannot be heard: noisy sites, hearing loss, a unit being held |
| **E-paper screen** | Writes the alert and the time, and **holds it with the power off** | Somebody arriving ten minutes later still learns what happened, and when |
| **Radio** | An 18-byte packet to every other unit in range over **LoRa** | One unit hearing something puts everything in range into full alert |

Between alerts the LED blinks slow green, roughly once every few seconds. That blink is deliberate: a silent detector and a dead detector otherwise look identical, so the device never goes fully dark.

A button on the front is the **snooze**. Pressing it stops the outputs while the detector keeps running and keeps logging. It is for the case where somebody is running a strimmer next to the unit and the beeper is not helping anyone.

<br clear="right">

---

<a name="how-it-works"></a>
## How it works

The short version: the device measures how strongly the sound spectrum looks like an evenly spaced comb of harmonics, and refuses to alert until that pattern has held still long enough to not be an accident. The long version follows.

### 1. The signal it is looking for

A propeller with **B blades** turning at **R revolutions per second** chops the air B×R times per second. That is the **blade-passage frequency**, and it is the fundamental of the sound. Because the chopping is not a pure sine wave, energy also lands at 2×, 3×, 4× and so on, producing a **harmonic comb**: peaks at regular spacing with quiet gaps between them.

<img src="docs/images/comb.png" width="720" alt="Measured spectrum showing the harmonic comb of a loaded propeller">

Two properties of that comb make it worth building a detector around.

- **It is sparse and regular.** Almost nothing else in an outdoor soundscape is. Wind is broadband, traffic is broadband, rain is broadband. A comb is a strong statement.
- **Its spacing is the rotor speed.** The detector does not have to guess whether a sound is drone-like. It measures a number, `f0`, and checks whether that number holds still.

For a three-blade propeller the design tracks two comb families, at 2× and 3× spacing, so detection survives one family being masked. On the specific airframe this was built against, the predicted hover band was **375 to 475 Hz**; the first real-rotor detections landed at **f0 = 423 Hz and 471 Hz**. Loaded flight measured higher, **480 to 570 Hz**, which is what a propeller doing work should do.

### 2. From four microphones to one number

**Array capture.** Four **ICS-43434** digital MEMS microphones sit in a plus pattern, **79 mm corner to corner** (±28 mm on each axis), clocked from the same source so their samples line up.

**Coherent sum.** The four channels are added. At the frequencies that matter, 79 mm is a tenth of a wavelength, so the array is effectively a point: sound from any direction adds in phase, and noise that is uncorrelated between microphones does not. That is roughly **+6 dB of signal-to-noise for free**, and it is nearly omnidirectional. Worst-case grazing loss across the comb band measured **-0.02 to +0.03 dB**.

That measurement closed a design question. Beamforming was investigated and **rejected on evidence**: at this array size and these frequencies the coherent sum already is the beam, and steering buys nothing. The array is a sensitivity device, not a direction finder. Full reasoning in [docs/how-it-works.md](docs/how-it-works.md).

**Spectrum.** A 2048-point FFT every 512 samples at 16 kHz, giving a **32 ms** frame rate with overlap.

**Adaptive floor.** The detector continuously learns what the site normally sounds like, per frequency bin, and subtracts it. Quiet things are learned slowly (6 s), loud things quickly (0.8 s), which is what stops a passing lorry parking itself in the noise model.

**Comb score.** For a candidate `f0`: add the energy sitting on the comb teeth, subtract the energy sitting in the gaps between them, normalise. A high score means the spectrum genuinely looks combed at that spacing, not just loud.

**Track and hold.** A single high-scoring frame means nothing. The detector demands the same `f0` across **six consecutive frames** with less than 2% drift. Noise does not do that. A rotor does.

### 3. Four detectors, not one threshold

A single detector has to choose between catching a drone that hovers and catching a drone that charges. This design does not choose. Four tiers run in parallel on the same audio, each blind to the others' weaknesses.

| | Tier | What it catches | Mechanism | Typical latency |
|---|---|---|---|---|
| **1** | **Fast comb** | An aircraft that arrives, approaches or changes | Comb score against the fast adaptive floor | **0.23 s** |
| **2** | **Slow comb** | An aircraft that arrives and then **hovers** | Same score against a 30 s floor, so a steady hover is not absorbed | 1.4 to 4 s |
| **3** | **Envelope wash** | Loaded, close, high-thrust flight | Broadband amplitude modulation above 3 kHz | 1 to 3 s |
| **4** | **No-floor comb** | Slow approach into a site that is already noisy | Comb score with no floor subtraction at all | 2 to 5 s |

Tier 1 is **sealed**. Its behaviour is pinned by golden test vectors so no future change can silently alter it: the same input file must produce the same score to the last decimal place, on a laptop and on the board. It currently does, bit for bit.

All four tiers are **enabled by default**. Tier 4 is the most sensitive, and it is the one that earns its place at a site where the background never really goes quiet. Each tier can still be turned off independently if a particular site calls for it. See [docs/configuration.md](docs/configuration.md).

**Why there is no single sensitivity slider.** For this application a **missed drone costs vastly more than a false beep**. So the thresholds sit low and the false-alarm defence is carried by the trackers instead: the requirement that a real, stable, physically plausible rotor frequency persists. That is a better filter than a high threshold, because a high threshold rejects quiet real drones and a tracker does not.

### 4. Radio

Each unit carries an **Ra-01H LoRa module** and a spring antenna in the wall of the case. The alert packet is 18 bytes: identity, tier, `f0`, score, sequence. There is no relay and no mesh in v0, deliberately: every unit hears every other unit directly or it does not, and that is a property that can be tested in an afternoon rather than trusted.

The link **spreads an alert, it does not vote on one**. Every unit decides for itself and alerts on its own evidence. There is no quorum, because waiting for a second opinion costs seconds and seconds are the entire product.

**The frequency is a configuration item that must be set for the country of use.** 868 MHz in the UK and EU, 915 MHz in the US, and local allocations elsewhere must be verified before transmitting. Details in [docs/configuration.md](docs/configuration.md).

### 5. Power

USB-C in, a **BQ24074** charger with power-path, a **TPS63020** buck-boost holding 3.3 V, and a 1S lithium-polymer cell.

**The cell is not optional, even on a unit that lives on mains.** During bring-up the board browned out when the beeper, the vibration motor and the radio transmitted at the same moment on USB power alone. The cell is what absorbs that current spike. A mains unit with no battery fitted is a unit that reboots at the exact moment it matters. For anybody planning a permanent installation, this is the single most important sentence on this page.

Runtime on battery alone is roughly **18 to 22 hours** with a 2500 mAh cell.

---

<a name="performance"></a>
## Range and performance

Nobody should quote a single range figure for an acoustic detector, including this one. Detection distance depends on the aircraft, the wind, the background noise and the terrain, and it moves by an order of magnitude across those. What follows separates **what has been measured** from **what the physics predicts**, and never mixes them.

### Measured, with the conditions attached

| Date | Setup | Result |
|---|---|---|
| 2026-08-13 | Single motor and propeller, quiet indoor lab | Detected at **25 to 30 m**, which was the room limit, not the device limit |
| 2026-08-13 | Same | First real-rotor lock at **f0 = 423 Hz** and **471 Hz** |
| 2026-08-21 | Field session, loaded propellers, mild wind, outdoors | Detections to **14 m**, tracked hovers and approaches |
| 2026-08-21 | Same, all session hours | **Zero false alarms** |
| 2026-08-21 | Signal margin at 4 m | **+5.9 dB** in the 125 to 1000 Hz band |
| 2026-08-28 | PCB rev A2 bring-up | Golden vectors **bit-identical** to the laptop reference |
| 2026-08-28 | Same | Worst-case frame time **29.4 ms** against the 32 ms budget, over 1938 frames, zero overruns |

Every measured detection is logged one row at a time in the **[field measurement table](docs/test-results.md#field-measurements)**. The full record, including what went wrong and what it taught, is in **[docs/test-results.md](docs/test-results.md)**.

### Predicted, from the measured factors

These are **derived numbers, not measurements**. They are built by taking the measured single-motor result and applying known scaling factors. They are here to be argued with and then replaced by real measurements.

| Conditions | Expected detection distance | Basis |
|---|---|---|
| Still air, quiet rural site, four motors under load | 100 to 200 m | Derived |
| Light breeze 2 to 5 m/s, typical rural background | 50 to 120 m | Derived, closest to the measured point |
| Breezy 5 to 8 m/s | 15 to 50 m | Derived, anchored by the measured 14 m in mild wind |
| Strong wind above 8 m/s, upwind of the aircraft | Assume badly degraded, tens of metres | Derived, wind costs up to -20 dB |

The factors behind those rows, so the arithmetic can be redone independently:

- **Four loaded motors instead of one bench motor: about +6 dB.**
- **Four-microphone coherent sum: about +6 dB.**
- **Every doubling of integration time: about +3 dB.**
- **Wind noise upwind above 5 m/s: up to -20 dB.** This is the dominant term, and it is why no honest range figure exists without a wind figure beside it.
- Every 6 dB lost roughly halves the distance.

The full derived table, the reasoning behind each band and the warning-time arithmetic: **[docs/expected-performance.md](docs/expected-performance.md)**

<a name="limits"></a>
### What it gets wrong

Stated plainly, because a detector nobody understands is a detector nobody can trust.

- **Wind is the enemy.** Above roughly 5 m/s, upwind, performance falls off a cliff. No algorithm fixes moving air across a microphone.
- **It has no idea whose drone it is.** No identification, no friend-or-foe, no transponder. Your own drone will set it off. That is correct behaviour.
- **Close speech can fire the comb tiers.** A voice a metre from the microphones has harmonic structure. This is a disclosed, understood confuser.
- **Idling engines are the hardest honest confuser class.** A diesel at idle is a rotating machine producing a harmonic comb. That is not a bug in the algorithm, it is the algorithm doing exactly what it says.
- **It is not a certified life-safety product.** It has been through no safety, EMC or reliability certification. It must never be the only thing between people and harm. [Full disclaimer](DISCLAIMER.md).

---

<a name="routes"></a>
## Two ways to build it

<table>
<tr>
<th width="50%">Start here: breadboard</th>
<th width="50%">Then this: the full unit</th>
</tr>
<tr>
<td valign="top">

**About £35 to £45. An evening. No soldering beyond header pins.**

A dev board and four microphone breakouts on a solderless breadboard. It runs the **same firmware** and the **same detector** as the finished product, and it will detect a drone.

What it gives up: the tuned enclosure, the sealed microphone ports, the radio link, battery operation and the array geometry that provides the +6 dB.

This is the right place to start for anyone who has never built the device, wants to check the algorithm against their own recordings, or is teaching it.

**→ [docs/breadboard-build.md](docs/breadboard-build.md)**

</td>
<td valign="top">

**About £50 to £80 per unit. A weekend. Requires a PCB order.**

A four-layer board assembled by the fab, four hand-soldered joints, four printed parts and a handful of fit parts.

This is the unit that was tested, the unit that goes on a wall, and the unit every number on this page came from.

The step from the breadboard to this one is a board order. Nothing is thrown away.

**→ [docs/build-guide.md](docs/build-guide.md)**

</td>
</tr>
</table>

Cost figures are estimates at quantities of a few units, excluding shipping and tax. They fall sharply at ten and rise sharply at one. Get a real quote before planning a budget.

---

<a name="parts"></a>
## Parts

Every part number below links to a supplier. Every file name links into this repository.

### The parts that define the design

Substituting any of these changes the behaviour of the device and invalidates the numbers on this page.

| Part | Designator | Side | What it does | Buy |
|---|---|---|---|---|
| **ESP32-S3-WROOM-1-N16R8** | U3 | Bottom | The processor. 16 MB flash, 8 MB PSRAM. Runs the whole detector in real time. | [LCSC C2913202](https://www.lcsc.com/search?q=C2913202) |
| **ICS-43434** × 4 | M1 to M4 | Bottom | Digital MEMS microphones, bottom-port, I2S, 24-bit. **The single most important part.** | [LCSC C5656610](https://www.lcsc.com/search?q=C5656610) |
| **Ra-01H** | U4 | Top | LoRa radio module for the peer alert link | [LCSC C503593](https://www.lcsc.com/search?q=C503593) |
| **BQ24074RGTR** | U1 | Bottom | Battery charger with power path, so the unit runs while charging | [LCSC search](https://www.lcsc.com/search?q=BQ24074RGTR) |
| **TPS63020DSJR** | U2 | Bottom | Buck-boost. Holds 3.3 V whether the cell is at 4.2 V or 3.2 V. | [JLCPCB C15483](https://jlcpcb.com/parts/componentSearch?searchTxt=C15483) |
| **WS2812B-V6** | LED1 | Top | The addressable RGB LED behind the light pipe | [LCSC C52917433](https://www.lcsc.com/search?q=C52917433) |
| **TMB12A03** | BZ1 | Top | The beeper. 3 V active, 12 mm body, 7.6 mm pin pitch. Hand-soldered. | [LCSC C96222](https://www.lcsc.com/search?q=C96222) |

The microphones sit at ±28 mm on both axes, which is where the 79 mm corner-to-corner figure comes from. Their exact placement is in [pick-and-place.csv](hardware/pcb/pick-and-place.csv).

<details>
<summary><b>The complete bill of materials, all 36 lines</b></summary>

<br>

These are placed by the fab. They do not need to be bought individually unless the board is being assembled by hand. "Side" is the layer as given in the pick-and-place file; the **Top** side is the one that faces the lid.

| Part | Value | Designator | Side | LCSC |
|---|---|---|---|---|
| TMB12A03 | beeper, 3 V active | BZ1 | Top | [C96222](https://www.lcsc.com/search?q=C96222) |
| CL10A475KP8NNNC | 4.7 µF 0603 | C1, C2, C3 | Bottom | [C1705](https://www.lcsc.com/search?q=C1705) |
| GRM188R61A226ME15L | 22 µF 0603 | C6, C7, C8 | Bottom | [C167429](https://www.lcsc.com/search?q=C167429) |
| CL10A106KP8NNNC | 10 µF 0603 | C4, C5, C9, C12 | Bottom | [C19702](https://www.lcsc.com/search?q=C19702) |
| CL10A105KB8NNNC | 1 µF 0603 | C13, C17, C19, C21, C23 | Bottom | [C15849](https://www.lcsc.com/search?q=C15849) |
| CL10B104KB8NNNC | 100 nF 0603 | C10, C11, C15, C16, C18, C20, C22 | Bottom | [C1591](https://www.lcsc.com/search?q=C1591) |
| CL10B104KB8NNNC | 100 nF 0603 | C25 | Top | [C1591](https://www.lcsc.com/search?q=C1591) |
| MMASL168BB5106MTNA01 | 0603 | C14 | Bottom | [C6594205](https://www.lcsc.com/search?q=C6594205) |
| 6SVPC330M | 330 µF polymer | C24 | Top | [C139569](https://www.lcsc.com/search?q=C139569) |
| OCV101M0JTR-0606 | 100 µF | C26 | Top | [C134854](https://www.lcsc.com/search?q=C134854) |
| USBLC6-2SC6 | USB ESD array | D1 | Bottom | [C7519](https://www.lcsc.com/search?q=C7519) |
| 1N4148W | flyback diodes | D2, D3 | Top | [C112342](https://www.lcsc.com/search?q=C112342) |
| BLM18AG601SN1D | ferrite bead | FB1 | Bottom | [C19330](https://www.lcsc.com/search?q=C19330) |
| TYPE-C-31-M-12 | USB-C receptacle | J1 | Bottom | [C165948](https://www.lcsc.com/search?q=C165948) |
| B2B-PH-SM4-TB | battery socket, JST-PH | J2 | Bottom | [C160352](https://www.lcsc.com/search?q=C160352) |
| XFL4020-152MEC | 1.5 µH inductor | L1 | Bottom | [C3033018](https://www.lcsc.com/search?q=C3033018) |
| WS2812B-V6 | RGB LED | LED1 | Top | [C52917433](https://www.lcsc.com/search?q=C52917433) |
| **ICS-43434** | MEMS microphone | M1, M2, M3, M4 | Bottom | [C5656610](https://www.lcsc.com/search?q=C5656610) |
| MMBT2222ALT1G | NPN, beeper and motor drive | Q1, Q2 | Top | [C82460](https://www.lcsc.com/search?q=C82460) |
| 0603WAF5101T5E | 5.1 kΩ, USB-C CC | R1, R2 | Bottom | [C23186](https://www.lcsc.com/search?q=C23186) |
| ERJ3EKF1781V | 1.78 kΩ | R3 | Bottom | [C403029](https://www.lcsc.com/search?q=C403029) |
| RC0603FR-073K01L | 3.01 kΩ | R4 | Bottom | [C137732](https://www.lcsc.com/search?q=C137732) |
| RC0603FR-0790K9L | 90.9 kΩ | R5 | Bottom | [C185305](https://www.lcsc.com/search?q=C185305) |
| 0603WAF1002T5E | 10 kΩ | R6, R14 | Bottom | [C25804](https://www.lcsc.com/search?q=C25804) |
| CL0603FN3K48P | 3.48 kΩ | R7 | Bottom | [C52209912](https://www.lcsc.com/search?q=C52209912) |
| 0603WAF1003T5E | 100 kΩ | R8, R12, R13 | Bottom | [C25803](https://www.lcsc.com/search?q=C25803) |
| 0603WAF1004T5E | 1 MΩ | R9, R11 | Bottom | [C22935](https://www.lcsc.com/search?q=C22935) |
| ERJ-U03F1783V | 178 kΩ | R10 | Bottom | [C1859202](https://www.lcsc.com/search?q=C1859202) |
| 0603WAF1001T5E | 1 kΩ, base resistors | R15, R16 | Top | [C21190](https://www.lcsc.com/search?q=C21190) |
| 0603WAF3300T5E | 330 Ω, LED series | R17 | Top | [C23138](https://www.lcsc.com/search?q=C23138) |
| MK-12C02-G015 | slide switch, power | S1 | Top | [C2911519](https://www.lcsc.com/search?q=C2911519) |
| K2-1114SA-A4SW-06 | tactile, boot and reset | SW2, SW3 | Top | [C136662](https://www.lcsc.com/search?q=C136662) |
| TS-1187A-B-A-B | tactile, snooze | SW4 | Top | [C318884](https://www.lcsc.com/search?q=C318884) |
| BQ24074RGTR | charger, power path | U1 | Bottom | [search](https://www.lcsc.com/search?q=BQ24074RGTR) |
| TPS63020DSJR | buck-boost | U2 | Bottom | [C15483](https://jlcpcb.com/parts/componentSearch?searchTxt=C15483) |
| ESP32-S3-WROOM-1-N16R8 | processor module | U3 | Bottom | [C2913202](https://www.lcsc.com/search?q=C2913202) |
| Ra-01H | LoRa module | U4 | Top | [C503593](https://www.lcsc.com/search?q=C503593) |

**Machine-readable copies:** [hardware/pcb/bom.csv](hardware/pcb/bom.csv) · [hardware/pcb/pick-and-place.csv](hardware/pcb/pick-and-place.csv)
Both upload directly to JLCPCB. Between them they define every populated position on the board. Any silkscreen reference not appearing in either file is an unpopulated position and is meant to stay empty.

</details>

### What gets soldered by hand

Four joints. That is the entire hand-soldering job on the finished board, and all four are on the **top** side, the side that faces the lid.

| # | Part | Where | Notes | Buy |
|---|---|---|---|---|
| 1 | **TMB12A03 beeper** | BZ1 | Pins arrive at 6.5 mm and must be splayed gently to the 7.6 mm pads. Left pin is **+**, right pin is **−**. | [LCSC C96222](https://www.lcsc.com/search?q=C96222) |
| 2 | **8-pin right-angle 2.54 mm header** | J3 | Pins point forward. Order is VCC · GND · DIN · CLK · CS · DC · RST · BUSY. Check squareness before the second joint. | [Amazon UK B01F558ASI](https://www.amazon.co.uk/sourcingmap%C2%AE-40-pin-2-54mm-Single-Header-Black-Silver-Tone/dp/B01F558ASI) |
| 3 | **Spring antenna, 868 or 915 MHz** | ANT1 pad, east edge | Lies flat and parallel in the wall channel. **The 17 mm type, not the 34 mm.** Nothing metallic goes near it. | [The Pi Hut](https://thepihut.com/products/simple-spring-antenna-915mhz) |
| 4 | **ERM vibration motor, Ø10 coin** | MP1 and MP2 pads | Red wire to MP1, blue or black to MP2. Bench-test before bedding the body in neutral-cure silicone so vibration does not fatigue the joints. | Any 10 mm 3 V coin ERM |

The board carries the driver transistor and flyback diode for the motor already, so this is a solder-and-go part rather than a modification. Leaving it off costs the haptic channel and nothing else.

Full orientation detail, including the traps: **[docs/build-guide.md](docs/build-guide.md)**

### Connected, but not soldered

These plug in. They are not on the bill of materials because the fab does not fit them.

| Part | What it is | Notes | Buy |
|---|---|---|---|
| **Waveshare 1.54″ e-paper, V2** | The screen | The **black and white** version, not the three-colour: three-colour refreshes far too slowly. The 200 mm stock cable is longer than the case needs and wants coiling, or replacing with roughly 100 mm. | [Waveshare](https://www.waveshare.com/1.54inch-e-paper-module.htm) |
| **1S LiPo, protected, JST-PH 2.0** | The cell | Must be **protected**, must have the JST-PH plug, must fit 61 × 51 × 8 mm. **Meter the plug before first connection**, red to the VBAT pin: some suppliers ship reversed polarity. | [PKCELL LP785060 2500 mAh](https://www.welectron.com/PKCell-LP785060-LiPo-2500-mAh-PH-Connector) · fallback [Pimoroni 2000 mAh](https://shop.pimoroni.com/products/lipo-battery-pack) |
| **USB-C cable, data rated** | Power and flashing | Charge-only cables will power the board and refuse to flash it. This costs somebody an hour at least once. | Any data cable |

### Enclosure fit parts

The small things that make it a device rather than a print. Quantities are per unit.

| Part | Qty | What it does | Buy |
|---|---|---|---|
| **M2.5 × 8 machine screws** | 4 | Board down onto its four posts | Any fastener supplier |
| **ST2.9 × 9.5 self-tapping, DIN 7981C A2** | 4 | Lid to base. **Use these, not M3 × 12.** M3 was tried and splits the Ø2.4 mm pilot bosses. | [Bolt Base](https://www.boltbase.co.uk) |
| **2 mm closed-cell foam, self-adhesive** | 1 sheet | Microphone gaskets. Fills the deliberate 1.5 mm gap under the lid's microphone tubes. Cut ~8 mm discs with a ~5 mm hole. | [Amazon UK B09JHN7NVM](https://www.amazon.co.uk/Adhesive-Closed-Polyethylene-Waterproof-Thickness/dp/B09JHN7NVM) |
| **Speaker grille cloth, waterproof** | 4 caps | Covers the microphone ports. Must be **woven and 0.4 mm or thinner**. A solid membrane costs about **-14 dB** and ruins the device. | [Amazon UK B0BPCLXZLB](https://www.amazon.co.uk/Speaker-Covered-Acoustic-Breathable-Waterproof-black/dp/B0BPCLXZLB) |
| **Permatex 22072 Ultra Black silicone** | 1 tube | Grille cloth, antenna anchor, light-pipe seal, motor bedding. **Neutral cure only. Never acetoxy**, which releases acetic acid and corrodes copper. | [Amazon UK B000HBIBOY](https://www.amazon.co.uk/Permatex-22072-Maximum-Resistance-Silicone/dp/B000HBIBOY) |
| **5 mm clear acrylic rod** | 9 mm cut | The light pipe over the RGB LED. Cut with a razor saw, finish 400 then 800 then 1200 grit. | Any plastics supplier |
| **Ø9.5 × 3.8 mm clear bumpon** | 1 | Snooze button cap, sticks to the tactile switch actuator | Amazon UK |
| **1 g silica gel sachets, Tyvek** | 2 | Taped flat to the base floor, fitted last. They double as a **passive leak detector**: a saturated sachet at service time means the seal has failed. | Amazon UK |
| **Silicone USB-C dust plug** | 1 | Closes the charge port | [Amazon UK B0773KLPBB](https://www.amazon.co.uk/Outstanding-Silicone-Anti-Dust-Protector-Smartphone/dp/B0773KLPBB) |

Rationale for each of these, including what was tried and rejected: **[hardware/enclosure/fit-parts.md](hardware/enclosure/fit-parts.md)**

### Breadboard route parts

| Part | Qty | Notes |
|---|---|---|
| **ESP32-S3-DevKitC-1**, N16R8 variant | 1 | Must be the S3. Not the original ESP32, not the C3. |
| **ICS-43434 breakout boards** | 4 | Or SPH0645 if different gain is acceptable. I2S digital, not analogue electret. |
| Solderless breadboard and jumpers | 1 | |
| Piezo beeper, NPN transistor, 1 kΩ resistor | 1 | **Never drive the beeper straight from a GPIO pin.** |
| LED and 330 Ω resistor | 1 | Optional |
| Waveshare 1.54″ e-paper | 1 | Optional |

Full wiring tables, the pins that must not be used, and bring-up: **[docs/breadboard-build.md](docs/breadboard-build.md)**

---

<a name="pcb"></a>
## The PCB

<img src="docs/images/pcb-top.jpg" align="right" width="300" alt="VolAnti rev A2 board, assembled">

**84 × 84 mm, four layers, ENIG finish.** Designed in Flux, fabricated and assembled by JLCPCB.

The board geometry is not cosmetic. The four microphones sit at fixed positions **±28 mm on each axis**, giving the 79 mm corner-to-corner spacing the firmware assumes. Move them and the coherent sum stops matching the model.

- **Interactive schematic and board, in a browser:** [VolAnti v1 on Flux](https://www.flux.ai/agamrossen/sentry-node-v1~wa)
- **Gerbers for fabrication:** [hardware/pcb/gerbers/](hardware/pcb/gerbers/)
- **Bill of materials:** [hardware/pcb/bom.csv](hardware/pcb/bom.csv)
- **Pick and place:** [hardware/pcb/pick-and-place.csv](hardware/pcb/pick-and-place.csv)
- **Schematic PDF:** [hardware/schematics/](hardware/schematics/)
- **Ordering, click by click:** [hardware/pcb/README.md](hardware/pcb/README.md)

<br clear="right">

**Do not allow substitutions** for U1, U2, U3, U4, the four ICS-43434 microphones, or the connectors. Everything else is a passive and an equivalent is fine. Exclude BZ1 from assembly: it is a hand-soldered part.

<a name="enclosure"></a>
## The enclosure

<img src="docs/images/exploded-iso.png" align="right" width="230" alt="Exploded view">

**Four printed parts**, 91 × 91 × 29 mm assembled, designed for FDM in **PETG**. The unit sits face up on its stand, and the same stand clips onto a belt if it is being carried.

| Part | Qty | Bed orientation | Notes |
|---|---|---|---|
| **Base** | 1 | As drawn | Holds the board, cell and silica |
| **Lid** | 1 | **Top face down** | Carries the microphone cones, screen window, light-pipe port, button and the four screw posts |
| **GrilleRing** | 4 | Flat side down | Retains the grille cloth over each microphone port. 0.1 mm layers. |
| **Mount** | 1 | As drawn | A wedge stand: a horizontal tray with two rails the base slides between, a triangular body underneath, and a **belt clip** on the back plate so the unit can also be carried |

**Print settings that matter:** 0.4 mm nozzle, 3 perimeters, 20 to 25% infill, elephant-foot compensation on. PETG for UV and temperature; PLA will sag on a sunlit wall.

The microphone ports are **45° cones opening from Ø4.5 to Ø23 mm**. That geometry is acoustic, not decorative. A remixed lid should keep it.

- **Printable files:** [hardware/enclosure/stl/](hardware/enclosure/stl/)
- **Print notes and fit checks:** [hardware/enclosure/README.md](hardware/enclosure/README.md)
- **Consumables, with reasoning:** [hardware/enclosure/fit-parts.md](hardware/enclosure/fit-parts.md)

<br clear="right">

**Three rules for a remix:** weld thicknesses stay at or above 1 mm, joins refill cuts rather than leaving voids, and undersides stay flat or ramped so they print without support.

<a name="firmware"></a>
## Firmware and flashing

Two board targets, one codebase: the **PCB** and the **DevKitC breadboard**.

The architecture is a deliberate seam: `front_end → combiner → back_end`, with all state in a single struct and **zero globals**. The combiner is an identity function at one channel, which is what leaves room to insert array processing later without touching the detector.

- **Source:** [firmware/](firmware/)
- **Flash from a browser**, no toolchain: [docs/flashing.md](docs/flashing.md)
- **Build from source with ESP-IDF:** [docs/flashing.md](docs/flashing.md)
- **Offline Python reference detector and test tools:** [tools/](tools/)

Three lessons that save an evening, learned the hard way.

1. **Never hardcode the serial port.** It re-enumerates on reset and between boot modes. Read it from the terminal every time.
2. **Erase the flash before the first write.** `idf.py erase-flash`, then `idf.py flash monitor`.
3. **Use a data-rated cable.** See above. It is always the cable.

<a name="assembly"></a>
## Assembly

Nine steps, in an order that matters: things that are hard to reach go in first, and the silica goes in last because everything after it would let moisture back in.

1. Bumpon onto the snooze switch. **Press it 20 times before committing** to the adhesive.
2. Foam gasket rings onto the four microphone pads. Dry-fit, lift the lid, and check for a **witness ring** on the solder mask. No ring means no seal.
3. Cell on its foam pad. **Meter the plug first.** Red to VBAT.
4. Board into the base on 4 × M2.5 × 8.
5. Screen taped into the lid tray, cable onto J3, cable coiled.
6. Light pipe pressed into the lid port. Silicone on the **side wall only**, never the end faces, or it goes cloudy.
7. Grille cloth and retaining rings over the four microphone ports.
8. Two silica sachets flat on the base floor.
9. Lid on with 4 × **ST2.9 × 9.5** self-tappers.

Each step in full, with torque notes and the mistakes worth avoiding: **[docs/build-guide.md](docs/build-guide.md)**

---

<a name="configuration"></a>
## Configuration

Most of the detector is deliberately not configurable. The core is pinned by golden vectors, and if it could be tuned per site the numbers on this page would mean nothing. Experiments belong in [tools/](tools/), where nothing depends on the result.

What does get set:

| Setting | Default | Why it would change |
|---|---|---|
| Alert threshold | **1.70**, high alert | 2.14 is the quieter "normal" setting. 1.70 is the default because a miss costs more than a beep. |
| All four tiers | **Enabled** | Turn one off only if a specific site gives you a reason to |
| **Radio frequency** | none | **Must be set for the country of use before transmitting** |
| Snooze duration | | How long the outputs stay quiet |

**→ [docs/configuration.md](docs/configuration.md)**

<a name="several"></a>
## Running several units

A single unit is a complete device. Mount it face up, plug it in, and it beeps when it hears a multirotor.

Several units cover more ground and, just as importantly, move the alarm to where the people are. **Every unit decides for itself**: there is no voting and no quorum, because waiting for a second unit to agree costs seconds and seconds are the entire product. The radio link spreads an alert; it does not confirm one.

| Role | Where | Power | Job |
|---|---|---|---|
| **Listening units** | Spread across the approach sides, spaced so coverage areas touch | Solar or battery | Hear the aircraft as early as possible |
| **Indoor units** | Inside or beside the buildings people are in | Mains | Repeat the alert where it matters. They still listen, and they double as a noise reference and event recorder. |
| **Cold spare** | On a shelf, assembled and flashed | | A swap takes two minutes, a repair takes a week |

Siting, mains hardening, the service schedule and the drill routine: **[docs/deployment.md](docs/deployment.md)**

> **One security note.** Do not publish the positions of deployed units, and do not post photographs that show where they are mounted relative to a real site. A public map of a detection network is a map of its gaps.

<a name="service"></a>
## Service

| Interval | Task |
|---|---|
| Weekly | Glance at the LED. Slow green blink means alive. |
| 3 months | Open, inspect the cell for swelling, check the silica |
| 6 months | Replace the cell. Float charge is the main long-run failure mode of a permanently powered unit. |
| After any alert | Read the screen, pull the log |

---

<a name="results"></a>
## Test results, and how to add yours

The measured record lives in **[docs/test-results.md](docs/test-results.md)**, dated, with conditions, including the sessions where things did not work.

The most valuable thing anybody can contribute is **a build report with real numbers**: what was built, where it was tested, the wind, the distance, the aircraft, and whether it fired. Second most valuable is **quiet negative audio**, hours of a real site sounding like itself, which is what justifies turning on the sensitive tier.

There is a template at the bottom of [docs/test-results.md](docs/test-results.md). Open an issue with it filled in.

<a name="contributing"></a>
## Contributing

Ranked by how much it helps.

1. **Build reports with measured numbers and stated conditions**
2. **Quiet negative audio hours** from real sites
3. **Translations** of the build guide
4. **Documentation fixes**, especially anywhere the docs and the hardware disagree
5. **Code**, which must pass the golden vector suite unchanged

A measured number with its conditions attached beats a plausible argument every time. That rule runs the whole project.

**→ [CONTRIBUTING.md](CONTRIBUTING.md)**

<a name="scope"></a>
## Scope

This project detects drones and raises alarms. That is the whole of it.

It will never include jamming, spoofing, RF interference, interception, kinetic response, targeting, or any other form of countermeasure. Contributions in that direction will be rejected regardless of framing, jurisdiction or intent. This is not a licensing position that can be forked around. It is the reason the project exists in the form it does: a device that only listens and shouts is a device a school can own.

<a name="licensing"></a>
## Licensing

| What | Licence | Meaning |
|---|---|---|
| Hardware: board, enclosure, schematics | **[CERN-OHL-W-2.0](LICENSES/CERN-OHL-W-2.0.txt)** | Modify and sell it; publish modifications to the hardware |
| Firmware and tools | **[Apache-2.0](LICENSES/Apache-2.0.txt)** | Use anywhere, including in closed products |
| Documentation | **[CC-BY-SA-4.0](LICENSES/CC-BY-SA-4.0.txt)** | Copy, translate and adapt, keeping the same licence |

Commercial builds are allowed. Details and the practical questions answered: **[LICENSE.md](LICENSE.md)**

---

<div align="center">

**Read the disclaimer before relying on this device for anything. → [DISCLAIMER.md](DISCLAIMER.md)**

[Website](https://agamrossen.github.io/VolAnti/) · [Build photos](docs/gallery.md) · [Report a build](docs/test-results.md#report-your-build)

Designed and tested at the **University of York**, Department of Electronic Engineering.

</div>
