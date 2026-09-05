# The enclosure

91 × 91 × 29 mm, four printed parts, all designed to print **support-free**. The lid's four 45° acoustic cones are the signature feature: they run unbroken from a Ø4.5 mm aperture above each microphone port to Ø23 mm mouths, buying acceptance angle for low-elevation sound and taming wind at the port, and 45° is exactly the self-supporting limit for a face-down print. That is not a coincidence; the acoustics and the printability were designed into each other.

<!-- IMAGE: enclosure-cad.jpg: exploded CAD view: Base, PCB, Battery, Lid, GrilleRings. -->

## The four parts

| Part | File | Material | Orientation on the bed |
|---|---|---|---|
| **Base** | `stl/VolAnti_Base.stl` | PETG | As modelled, cavity up |
| **Lid** | `stl/VolAnti_Lid.stl` | PETG | **Top face down, flat on the bed** (the cones are then 45° self-supporting funnels) |
| **Mount** | `stl/VolAnti_Mount.stl` | PETG | Wedge stand: tray with two guide rails, triangular body, belt clip on the back plate |
| **GrilleRing** (×4 per unit) | `stl/VolAnti_GrilleRing.stl` | PLA is fine | Flat side down, ramp up |

STEP files for every part are alongside the STLs for anyone remixing in CAD.

## Print settings (what every unit so far was printed with)

- 0.4 mm nozzle, **3 perimeters**, **20–25 % infill**
- PETG for Base / Lid / Mount (outdoor UV and heat tolerance); PLA acceptable for the small GrilleRings
- GrilleRings at **0.1 mm layers** so the bezel ramp comes out smooth; 0.2 mm is fine everywhere else
- **Elephant-foot compensation on**: several fits are first-layer-critical
- No supports anywhere, by design; if your slicer wants supports, your orientation is wrong

**Fit checks after printing:** the corner posts (0.4 mm/side clearance into the lid bores) and the Ø5.15 light-pipe port (ream to 5.1 mm if your printer runs tight). Everything else has generous clearance.

## For remixers: three rules this design learned the hard way

1. **Weld every hanging feature ≥ 1 mm into its parent.** A face-kissing 0.05 mm overlap is legal in CAD and becomes a disconnected floating body in the STL export; that is how an early revision's display rails snapped off.
2. **Any feature joined after a cut refills that cut.** Re-check every hole downstream of a new join before exporting.
3. **Printed undersides are flat or ramped, never stepped.** The GrilleRing took three revisions to reach a fully flat bottom, and it prints perfectly ever since.

## Fit-parts

Everything that goes *into* the printed parts (screws, cell, display, cloth, foam, light pipe, desiccant), with exact sizes and the reasons behind them, lives in [fit-parts.md](fit-parts.md). Assembly order and technique are in the [build guide](../../docs/build-guide.md).
