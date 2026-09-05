# Build guide: breadboard version

The full detector, on a dev board, with no custom PCB and no printed parts. This is how VolAnti itself was developed for its first months: every algorithm in the stack was proven on exactly this setup before the PCB existed. **It runs the same firmware** (the codebase carries a `devkit` board target alongside the production `pcb_rev_a2` target), so you get the real detector, the real thresholds, the real serial console.

What you give up against the full build: the enclosure's weather sealing and wind performance, the e-paper's held-with-power-off alert, the tidy alert loudness, and a few dB of consistency from fixed mic geometry. What you keep: the entire detection stack.

## Parts

| Full build (PCB) | Breadboard equivalent | Notes |
|---|---|---|
| ESP32-S3-WROOM-1-**N16R8** module | **ESP32-S3-DevKitC-1 N16R8** dev board | The N16R8 memory variant matters: firmware assumes 16 MB flash. Clone boards work; genuine boards from Espressif distributors avoid surprises. |
| 4 × ICS-43434 on the board | 4 × **ICS-43434 breakout boards** | Any I2S breakout for this part. (INMP441 breakouts also work electrically but that part is end-of-life.) |
| Waveshare 1.54″ e-paper via J3 | Same module, jumper wires | Optional on the bench; the serial console shows everything. |
| WS2812B LED on board | Any WS2812/NeoPixel + 330 Ω series resistor | Optional. |
| TMB12A03 beeper via driver | Same beeper + NPN (2N2222) + 1 kΩ base resistor | Optional. **Do not drive a beeper straight from a GPIO pin.** |
| LoRa Ra-01H | Skip it | Peer alerting is a multi-unit feature; bring it up later. |
| Battery + charger + regulator | **USB power from your computer** | Fine for detection work. If you later add beeper + motor + radio, expect brown-outs on USB alone; the firmware has a no-battery bench mode for exactly this. |

Plus a breadboard, jumper wires, and a piece of stiff card.

## Wiring

All four microphones share one clock pair; they split across two data lines, two mics each, using the ICS-43434's L/R select pin to share a line.

**Clock pair (to all four mics):**

| DevKit GPIO | Mic pin | 
|---|---|
| GPIO4 | SCK (bit clock), all four |
| GPIO5 | WS (word select), all four |

**Data lines and channel select:**

| Mic | Position (reference) | SD → GPIO | L/R pin |
|---|---|---|---|
| M1 | West | **GPIO6** (bus A) | **GND** |
| M2 | East | **GPIO6** (bus A) | **3V3** |
| M3 | North | **GPIO7** (bus B) | **GND** |
| M4 | South | **GPIO7** (bus B) | **3V3** |

Every mic: VDD → 3V3, GND → GND. The two mics on each data line must have opposite L/R levels, or they fight for the same slot and you get one garbled channel.

**Outputs and the button:**

| DevKit GPIO | Function | Wiring |
|---|---|---|
| GPIO10 | e-paper CS | direct |
| GPIO11 | e-paper DIN | direct |
| GPIO12 | e-paper CLK | direct |
| GPIO13 | e-paper DC | direct |
| GPIO14 | e-paper RST | direct |
| GPIO15 | e-paper BUSY | direct (panel drives it) |
| GPIO16 | WS2812 data | through 330 Ω |
| GPIO17 | Beeper | GPIO → 1 kΩ → NPN base; beeper between 3V3 and collector |
| GPIO18 | Vibration motor | same driver pattern as the beeper, flyback diode across the motor |
| GPIO21 | Snooze button | button to GND; internal pull-up is enabled in firmware |

<!-- IMAGE: breadboard-photo.jpg: the actual dev-era breadboard rig, labelled. -->
<!-- Schematic: hardware/schematics/ contains the breadboard wiring diagram split by subsystem. -->

## Pins you must not use

If you extend the build, these are off-limits on the DevKitC-1 N16R8, and two of them are traps:

- **GPIO33–37**: bonded to the module's PSRAM die **even when PSRAM is disabled in software**. Using them causes intermittent corruption, not a clean failure. (Many pinout diagrams wrongly show 33–34 as free.)
- **GPIO26–32**: SPI flash. **GPIO19/20**: native USB, your console and flashing link. **GPIO0, 3, 45, 46**: boot strapping.

Free pins for your own additions: **1, 2, 8, 9, 47**.

## Geometry

Tape the four breakouts to stiff card at the reference geometry: a square, 56 mm sides, ports facing up, in the M1-west / M2-east / M3-north / M4-south arrangement above. Exact spacing is not critical for detection (the array's job is noise averaging, and it is omnidirectional at drone fundamentals); matching the reference layout just keeps your numbers comparable to everyone else's.

## Bring-up

1. Flash the `devkit` target: [flashing.md](flashing.md).
2. Open the serial console. Send `I`: the firmware prints the I2S pin map it was compiled with. If your wiring and that printout disagree, the printout wins.
3. Check the boot report shows four healthy channels with quiet-RMS values in the same order of magnitude (a healthy bench reading looks like 250–420 per channel). One silent channel is almost always a swapped L/R pin or the two-mics-one-slot conflict above.
4. **Tap test**: tap next to each mic and watch its channel respond. This catches position swaps that all electrical checks pass.
5. Play a drone comb from a phone speaker at half a metre (any FPV flight video with clean audio, or the synthetic clips in [tools/](../tools/)) and watch the score cross the threshold on the console.

From here, the [configuration](configuration.md), [test-results](test-results.md), and [expected-performance](expected-performance.md) pages all apply to your build unchanged.
