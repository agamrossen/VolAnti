# Firmware

ESP32-S3 firmware, built with ESP-IDF. **The source is not published yet.** The directory tree below is the real one and the code lands into it at the first tagged release; the `.gitkeep` files disappear when it does.

Until then, [tools/](../tools/) contains the offline reference implementation of the same detector, and [docs/how-it-works.md](../docs/how-it-works.md) describes the algorithm in enough detail to reimplement it.

```
firmware/volanti/
  main/                     application entry, task setup, boot self-test
  components/
    board/                  pin maps and the two board targets, PCB and DevKitC
    front_end/              I2S capture, per-channel calibration, framing
    combiner/               channel combination, an identity function at one channel
    back_end/               spectrum, adaptive floor, comb scoring, the four tiers
```

## Two board targets, one codebase

The same image runs on the four-layer board and on a DevKitC breadboard build. The target selects the pin map and nothing else: the detector is identical, which is what makes a breadboard result meaningful.

## The seam

`front_end -> combiner -> back_end`, with all state in a single struct and **zero globals**. The combiner is deliberately an identity function while one summed channel is used. It exists so array processing can be inserted later without touching the detector, and so the golden vectors keep meaning when it is.

## What "sealed" means

The tier 1 detector path is pinned by golden test vectors. A given input file must produce the same score to the last decimal place, on a laptop and on the board. It currently does, bit for bit. Any change that moves those numbers is a change to the instrument, not a refactor, and needs a measurement attached.

## Building

Toolchain, flashing, and the three mistakes that cost an evening: [docs/flashing.md](../docs/flashing.md).
