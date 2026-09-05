# Maintainer checklist

**Delete this file before the repository goes public.** It is the only file here addressed to the maintainer rather than to a builder.

## Files still to add

| Path | What goes there | How to produce it |
|---|---|---|
| `hardware/pcb/gerbers/VolAnti_revA2_gerbers.zip` | Gerber and drill set | Flux, export fabrication outputs, zip as-is |
| `hardware/schematics/VolAnti_revA2_schematic.pdf` | Multi-sheet schematic | Flux, print to PDF |
| `hardware/schematics/breadboard-*.svg` | Breadboard wiring diagrams | To draw |
| `hardware/enclosure/stl/VolAnti_{Base,Lid,GrilleRing,Mount}.{stl,step}` | Printable parts | Fusion, export each component |
| `firmware/volanti/**` | The firmware tree | Repo copy, minus any site-specific config. Delete the `.gitkeep` files as real folders fill. |
| `tools/**` | Reference detector, synth generator, golden vectors, parity harness | Repo copy |
| `docs/images/bz1-orientation.jpg` | Close-up of the beeper joint showing which pin is positive | To shoot |
| `docs/images/breadboard-photo.jpg` | The breadboard build wired up | To shoot |
| `docs/images/field1-rig.jpg` | The field test setup | To shoot |

## Links to fill in

- **Website.** Every link currently points at `https://agamrossen.github.io/VolAnti/`, which is what GitHub Pages will serve if Pages is enabled on this repo. If the site lives anywhere else, replace that string across `README.md`, `docs/gallery.md` and `CITATION.cff`.
- **Photo archive.** `docs/gallery.md` has a marked slot for the Google Drive link. Set the folder to "anyone with the link can view".
- **ESP Web Tools flasher** in `docs/flashing.md`, once the first release is tagged.
- **JLCPCB shared project links** in `hardware/pcb/README.md`.
- **The £50 to £80 estimate** in `README.md`, once a real ten-unit quote exists.
- The Flux project link is public and already correct.

## Repository settings

- **Description:** Open-source acoustic drone detection. It hears the propellers, not the radio, so it works against fibre-optic FPV aircraft that emit no signal at all.
- **Website:** the Pages URL above
- **Topics:** `drone-detection`, `acoustics`, `dsp`, `esp32`, `esp32-s3`, `open-hardware`, `lora`, `microphone-array`, `early-warning`, `kicad-alternative`, `pcb`, `3d-printing`
- Enable Issues. Enable Discussions only if there is capacity to answer them.
- Enable Pages if the site is served from this repo.

## Checks before pushing

- [ ] Every measured number in `docs/test-results.md` still matches the logs
- [ ] No image shows a deployed unit's position relative to a real site
- [ ] Israeli 917 to 920 MHz allocation verified and reflected in `docs/configuration.md`
- [ ] Firmware ships with all four tiers enabled and no site-specific defaults
- [ ] `git log` contains no site names, addresses or coordinates
- [ ] This file deleted
