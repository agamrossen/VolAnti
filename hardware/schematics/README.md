# Schematics

Two forms, for two different jobs.

## The board schematic

`VolAnti_revA2_schematic.pdf`, multi-sheet, in this order:

1. Power: USB-C input, BQ24074 charger and power path, TPS63020 buck-boost, cell connector
2. Processor: ESP32-S3-WROOM-1, strapping, boot and reset, test points
3. Microphone array: four ICS-43434, the two shared I2S clock lines and the two data buses
4. Radio: Ra-01H, antenna feed and keepout
5. Outputs and controls: beeper and motor drivers, RGB LED, e-paper header, switches

The interactive version, where nets can be clicked through, is on [Flux](https://www.flux.ai/agamrossen/sentry-node-v1~wa).

## The breadboard wiring diagrams

Drawn per subsystem so a single sheet stays readable on a bench: clocking and microphones, outputs, e-paper. These match the tables in [docs/breadboard-build.md](../../docs/breadboard-build.md).

## Where the truth lives

Where a schematic and the board files disagree, **the board files win**. They are what was fabricated, and they are what the golden test vectors were captured against.
