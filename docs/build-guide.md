# Build guide: full unit

The complete build: custom PCB, printed enclosure, roughly two evenings of work once parts arrive. The board arrives from the fab with everything machine-placed except four through-hole parts; you solder those, verify on the bench, then the whole thing screws together. No step here requires more than a basic soldering iron and a screwdriver.

If you want to try the detector *tonight* with parts you may already own, go to [breadboard-build.md](breadboard-build.md) instead. Same firmware, no custom hardware.

## 0. What you are ordering

1. **The PCB, assembled**: gerbers, [BOM](../hardware/pcb/bom.csv), and [pick-and-place](../hardware/pcb/pick-and-place.csv) files are in [hardware/pcb/](../hardware/pcb/). Ordering walkthrough: [hardware/pcb/README.md](../hardware/pcb/README.md). 84 × 84 mm, 4-layer, ENIG finish.
2. **Four printed parts**: Base, Lid, Mount, GrilleRing (×4 per unit). Files and print settings: [hardware/enclosure/README.md](../hardware/enclosure/README.md).
3. **The fit-parts kit**: battery, display, antenna, screws, and the small consumables. Complete list with exact sizes: [hardware/enclosure/fit-parts.md](../hardware/enclosure/fit-parts.md).

## 1. Hand-soldering: four joints

The fab places everything else, including the USB-C connector, battery socket, and slide switch. You solder these, in this order:

### 1.1 Beeper: BZ1 (required)
TMB12A03, top side. The pins come at 6.5 mm pitch; **splay them gently to 7.6 mm** to match the pads. Polarity matters: **left pad = + (3V3_SYS), right pad = − (switched)**. The + mark on the beeper body faces the left pad.

<!-- IMAGE: bz1-orientation.jpg: beeper in place, + mark visible, before soldering. -->

### 1.2 Display header: J3 (required)
8-pin 2.54 mm **right-angle** male header, top side, **pins pointing toward the board's +Y edge** (toward the display bay). Pin order along the row: VCC · GND · DIN · CLK · CS · DC · RST · BUSY. Solder one end pin, check the header sits flat and square, then do the rest. A leaning header is the single most common cause of a dead display, because the panel's BUSY line then makes intermittent contact.

### 1.3 Antenna: ANT1 (required)
868/915 MHz solder-type spring antenna onto the ANT1 pad on the +X edge. Mount it **flat, parallel to the board**, lying into the east-wall channel of the enclosure. Do not trim the spring.

### 1.4 Vibration motor: MP1/MP2 (optional)
The board carries the full driver (transistor + flyback diode) and two pads, so this is a solder-and-go option, not a modification. Ø10 ERM coin motor, lying **flat on the board's top face** beside its pads: **red wire → MP1 (3V3_SYS), blue/black wire → MP2**. Tack the wires first, bench-test it (section 2), and only then bed the motor body in a small bead of neutral-cure silicone so vibration does not fatigue the joints. It is the fifth alert channel alongside the beeper, the LED, the screen and the radio, and it is the one that works when the beeper cannot be heard. If it is left off, leave MP1/MP2 bare and the unit still runs.

## 2. Bench verification: before anything goes in a box

Power over USB-C **with the battery connected** (see the polarity check in step 3.3 first: do it now, out of order, before the first plug-in). The unit boots to guarding in ~2 s and runs an output parade: beeper chirp, motor pulse, LED, display refresh.

Then over the USB serial console:

1. Send `I`: the firmware prints its own I2S pin map. Confirm four microphones report healthy (per-channel RMS in the same order of magnitude; the LISTENING footer also shows MIC health continuously).
2. Continuity checks worth 60 seconds each: radio ANT pin → ANT1 pad; MP1 → 3V3; MP2 → driver collector.
3. **Tap test**: tap next to each mic port in turn and watch the per-channel levels respond in the right positions. This catches a swapped mic pair that every electrical test passes.
4. Press the snooze button; confirm the console registers it.

Do not proceed to assembly until all four pass. Everything is reachable now; nothing is once the lid is on.

## 3. Enclosure assembly

Work top of this list to the bottom; the order is load-bearing (several parts block access to earlier ones).

1. **Snooze dome**: stick the Ø9.5 × 3.8 mm clear hemispherical bumpon centred on the tactile switch actuator. Press 20 times before committing to the build: a mis-centred dome binds in the lid's well.
2. **Mic gaskets**: one ring of 2 mm self-adhesive closed-cell foam around each of the four microphone ports (≈8 mm disc with a ≈5 mm centre hole; match the hole to your printed skirt-tube OD). These fill the deliberate 1.5 mm gap under the lid's cones. After a dry lid fit, lifting the lid should leave a faint witness ring on the solder mask at each port; no ring means no seal.
3. **Battery**: lay the 1 mm foam pad in the bay, meter the plug (**red must reach the VBAT pin of J2**: if reversed, lift the two crimp tabs with a pin and swap them, no parts needed), seat the cell, coil the lead in the east channel, plug in. Reference cell: PKCELL LP785060 2500 mAh protected, JST-PH; the Pimoroni 2000 mAh is a drop-in alternative. **Protected cells only.**
4. **PCB down**: 4 × **M2.5 × 8** machine screws into the four bosses. Snug, not gorilla: the bosses are printed plastic.
5. **Display into the lid**: the panel tapes into the lid tray (thin double-sided tape at the corners); its cable plugs onto J3. Note: the stock Waveshare cable is 200 mm and must be coiled once in the bay, or replaced with a ~100 mm JST-PH 2.0 8-pin to Dupont-female lead if you can source or crimp one.
6. **Light pipe**: cut 5 mm clear acrylic rod to **9 mm** (razor saw + mitre box), finish both faces through 400 → 800 → 1200 wet-or-dry, press-fit into the Ø5.15 lid port. If your rod runs oversize, one wrap of PTFE tape at mid-rod restores the press fit. If it runs loose, a bead of neutral-cure silicone **around the side wall only, never on the end faces**: silicone on an optical face kills the LED's brightness.
7. **Grille cloth**: for each cone mouth, a Ø24–25 mm disc of thin acoustically-transparent woven cloth into the recess, bonded with a thin ring of **neutral-cure silicone** (Permatex Ultra Black 22072 or equivalent) applied like grouting tile: small dabs placed with a cocktail stick, cloth pressed in, edges captured. **Never acetoxy-cure silicone** (vinegar smell): it corrodes copper. Then glue a GrilleRing bezel over each, flat side down, aligned by eye to the mouth.
8. **Desiccant**: two 1 g silica-gel sachets (Tyvek), taped flat to the base floor. Fit these **last before the lid closes**. They double as a passive leak detector: saturated sachets at a service interval mean the seal has failed.
9. **Lid on**: engage the corner posts and the N/S tabs, then 4 × **ST2.9 × 9.5 mm pan-head self-tapping screws** (DIN 7981C, stainless) from below. *Do not substitute M3 machine screws here*: their core is too wide for the Ø2.4 mm pilot bores and will split the posts; we learned this the hard way.

<!-- IMAGE: assembly-exploded.jpg: exploded render of Base / PCB / Battery / Lid / GrilleRings from the CAD. -->
<!-- IMAGE: assembly-sequence.jpg: grid of steps 1–9 as photos. -->

## 4. First power-up in the box

Hold the unit at arm's length, power on, and confirm: LED green blink through the light pipe, LISTENING screen with all four MIC-health marks, one clean beeper chirp. Tap the snooze dome through the lid and confirm the menu pages. That is a complete unit.

Now do the one test that matters: [test-results.md](test-results.md) describes the minimal live-rotor verification and the log format for reporting your build back.
