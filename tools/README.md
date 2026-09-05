# Offline tools

The detector, the test rig and the analysis scripts that run on a laptop rather than on the board. **Not published yet.** The tree below is the real one and the code lands into it at the first tagged release.

```
tools/
  environment.yml           conda environment, published now so it can be built ahead of time
  detector/                 the reference implementation the firmware is checked against
  synth/                    synthetic rotor and confuser generator
  golden/                   golden input vectors and their expected traces
  tests/                    the parity harness, including test_golden
```

## Why an offline reference exists

Every number this project publishes came out of a detector that runs in two places: once here in Python, and once on the microcontroller. The firmware is correct when the two agree exactly on the golden vectors. That check is what lets a claim like "bit-identical to the reference" mean something, and it is what stops a well-meaning optimisation from quietly changing the instrument.

## Environment

```bash
conda env create -f tools/environment.yml
conda activate volanti
```

## Working here

This is the right place to experiment. Thresholds, tier weights and new detector ideas can be tried against recorded audio without touching anything that a deployed unit depends on. When an experiment produces a number worth keeping, it belongs in a [build report](../docs/test-results.md#report-your-build) with its conditions attached.
