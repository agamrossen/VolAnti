# Firmware

ESP32-S3 firmware, built with ESP-IDF v6.0. Setup, build commands for both boards and the three mistakes that cost an evening are in [docs/flashing.md](../docs/flashing.md). [docs/how-it-works.md](../docs/how-it-works.md) describes what the detector does.

    firmware/sentry_node/
      main/              the application: capture, the four detector tiers, alerts, display, radio
        boards/          pin maps for the two targets, the DevKitC breadboard and the PCB
        epaper/          e-paper panel drivers
        generated/       headers written by the build, not kept in git
      scripts/           host tools: header generator, golden vector capture and comparison, parity checks
      PORTING_NOTES.md   how the C port is kept identical to the Python reference

    src/                 the Python reference detector, and the synthetic audio used to test it
    data/                detector constants, operating points and the golden test vectors
    tests/test_golden.py checks the golden vectors against the reference detector
    environment.yml      the Python environment the build and the tools use

## One codebase, two boards

The four-layer board and the DevKitC breadboard build are two build targets of the same source. The target selects the pin map and the hardware the board has, such as the battery monitor and the LoRa radio, and nothing else. The detector is the same code on both, which is what lets a breadboard result mean something for a boxed unit.

## Generated headers

The detector's constants, lookup tables and golden vectors are not written by hand. On every build `scripts/gen_headers.py` writes them into `main/generated/` from `src/` and `data/`, which is why the build needs Python with numpy as well as ESP-IDF.

## Sealed

Tier 1 is pinned by golden test vectors. A given input file must produce the same score to the last decimal place on a laptop and on the board, and on the production board it does, bit for bit. Any change to the detection chain has to pass those unchanged, or bring new vectors and a reason.

On a laptop, `python tests/test_golden.py` checks the vectors against the reference detector. On a board, `scripts/capture_trace.py` records the replay of each vector and `scripts/compare_trace.py` judges it against the reference trace.
