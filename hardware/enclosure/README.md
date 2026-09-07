# The enclosure

91 × 91 × 29 mm, four printed parts, no supports anywhere. The lid's four 45 degree cones are the point of the design: they run unbroken from the grille mouth down to the microphone port and print as self-supporting funnels when the lid is face down.

<img src="renders/all-parts.png" width="100%" alt="Base, lid, grille ring and mount">

| Part | STL | STEP | Material | On the bed |
|---|---|---|---|---|
| Base | [VolAnti_Base.stl](stl/VolAnti_Base.stl) | [step](step/VolAnti_Base.step) | PETG | As modelled, cavity up |
| Lid | [VolAnti_Lid.stl](stl/VolAnti_Lid.stl) | [step](step/VolAnti_Lid.step) | PETG | Face down, flat on the bed |
| Grille ring, four per unit | [VolAnti_GrilleRing.stl](stl/VolAnti_GrilleRing.stl) | [step](step/VolAnti_GrilleRing.step) | PLA is fine | Flat side down |
| Mount | [VolAnti_Mount.stl](stl/VolAnti_Mount.stl) | [step](step/VolAnti_Mount.step) | PETG | Wedge, tray with two rails, belt clip on the back |

All STLs are in millimetres and print at 100 %. STEP files are there for anyone changing the design.

## Print settings

Every unit so far: 0.4 mm nozzle, 3 perimeters, 20 to 25 % infill, 0.2 mm layers, black PETG. Grille rings at 0.1 mm layers so the ramp comes out smooth. Elephant-foot compensation on, several fits are first-layer critical. If your slicer wants supports, the part is the wrong way up.

After printing, check the corner posts drop into the lid bores and the light pipe port takes a 5 mm rod. Ream to 5.1 mm if your printer runs tight.

## For anyone remixing

Three things this design learned the hard way. Weld every hanging feature at least 1 mm into its parent, a face-kissing overlap exports as a floating body. Any feature joined after a cut refills that cut, so recheck every hole downstream of a new join. Printed undersides are flat or ramped, never stepped. The grille ring took three revisions to get a fully flat bottom and has printed cleanly ever since.
