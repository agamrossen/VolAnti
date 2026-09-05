# Fit-parts and consumables

Everything the printed parts and the PCB need to become a finished unit. Exact sizes matter here; the printed cutouts were designed to these parts. Where a part is optional, it says so. Generic descriptions are given so you can source locally; a part number in brackets is simply the one we used.

## Fasteners

| Item | Qty/unit | Use |
|---|---|---|
| **M2.5 × 8 mm** machine screw | 4 | PCB onto its bosses |
| **ST2.9 × 9.5 mm** pan-head self-tapping, stainless (DIN 7981C) | 4 | Lid to base, from below. **These, not M3 machine screws**: an M3's core splits the Ø2.4 mm printed pilots. Field-proven correction. |

## Power

| Item | Spec | Notes |
|---|---|---|
| **Battery** | Protected 1S LiPo, JST-PH 2.0 connector, ≤ 61 × 51 × 8 mm | Reference: PKCELL LP785060, 2500 mAh (sold under Adafruit and many maker brands). Drop-in alternative: the common 2000 mAh 60 × 42.5 × 8.3 pack. **Protected cells only. Meter the plug polarity before first connection**: red must land on the VBAT pin; JST leads from different vendors are wired both ways. |
| Battery foam | 1 mm EVA sheet, bay-sized pad | Anti-rattle, under the cell |
| USB PSU + cable (mains units) | Certified local 5 V PSU; braided cable ≤ 2 m, 22 AWG | See [deployment.md](../../docs/deployment.md) |

## Display and indicators

| Item | Spec | Notes |
|---|---|---|
| **E-paper** | Waveshare 1.54″ V2, black/white, 200 × 200 | **Not** the 3-colour variant (its refresh is far too slow for alerts). Comes with a 200 mm JST-PH-to-Dupont cable; it coils once in the bay, or substitute a ~100 mm lead. |
| **Light pipe** | 5 mm clear acrylic rod, cut to 9 mm | Faces finished through 400 → 800 → 1200 grit. Press-fits the Ø5.15 lid port. |
| **Snooze dome** | Ø9.5 × 3.8 mm clear self-adhesive hemispherical bumpon | Sticks on the tactile switch; the lid's well is sized to it. |

## Radio

| Item | Spec | Notes |
|---|---|---|
| **Antenna** | Solder-type spring, 868/915 MHz (~17 mm coil) | Lies flat in the east-wall channel. **Nothing metallic near it**: metal mesh, foil tape, or a metal mic grille within a few mm detunes it. |

## Acoustics and sealing

| Item | Spec | Notes |
|---|---|---|
| **Grille cloth** | Thin (≤ 0.4 mm) acoustically transparent woven speaker cloth | Cut Ø24–25 mm discs, 4/unit. Open weave, **not** membrane material: a filtration membrane costs ~14 dB at 1 kHz; open weave costs nothing measurable and still sheds drizzle and dust. Must be non-metallic (antenna, above). |
| **Adhesive/sealant** | **Neutral-cure** silicone (e.g. Permatex Ultra Black 22072) | The one adhesive in the whole build: bonds cloth, bezels, antenna anchor, light-pipe backup. **Never acetoxy-cure** (vinegar smell): it outgasses acetic acid and corrodes copper in a sealed box. |
| **Mic gaskets** | 2 mm self-adhesive closed-cell foam | ~Ø8 mm rings with ~Ø5 mm holes, 4/unit; seal the lid cones to the PCB ports. |
| **Desiccant** | 2 × 1 g silica gel sachets (Tyvek) | Taped flat to the base floor, fitted last. Doubles as a leak detector at service time. |
| PTFE tape | 1 roll | One wrap restores the light-pipe press fit on oversize rod. |

## Optional finish (nice, not needed)

- Silicone USB-C dust plug (plug-in type) for the port when unpowered.
- A 22 × 6 mm rectangle of the same grille cloth over the antenna slot, stuck with thin tape: matches the mic caps and is non-metallic by construction.

## Deliberate omissions

No conformal coating (the mic ports must stay open, and the sealed box + desiccant strategy is the moisture design). No threaded inserts (self-tappers into printed pilots are the tested joint). No metal anywhere on the lid's top face (antenna).
