# Firmware

ESP32-S3 firmware built with ESP-IDF v6. The source lands here with the first tagged release, in this layout. Until then [docs/how-it-works.md](../docs/how-it-works.md) describes exactly what it does.

    firmware/volanti/
      main/            entry, task setup, boot self-test
      components/
        board/         pin maps for the two targets, PCB and DevKitC
        front_end/     I2S capture, per-channel calibration, framing
        combiner/      channel combination
        back_end/      spectrum, adaptive floor, comb scoring, the four tiers

## One image, two boards

The same image runs on the four-layer board and on a DevKitC breadboard build. The board target selects the pin map and nothing else. That is what lets a breadboard result mean something for a boxed unit.

## Flashing

Toolchain, commands and the three mistakes that cost an evening: [docs/flashing.md](../docs/flashing.md).

## Sealed

Tier 1 is pinned by golden test vectors. A given input file must produce the same score to the last decimal place on a laptop and on the board, and on the production board it does, bit for bit. Any change to the detection chain has to pass those unchanged, or bring new vectors and a reason.
