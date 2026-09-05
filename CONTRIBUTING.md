# Contributing

The contributions this project needs most, in order:

1. **Build reports**: successes and failures, in the format at the bottom of [docs/test-results.md](docs/test-results.md). This is the highest-value contribution available; it needs no code.
2. **Real negative audio hours**: logged drone-free guarding hours (livestock, wind, machinery environments) advance tier certification directly.
3. **Translations** of the build docs (CC-BY-SA; keep numbers and warnings intact).
4. **Documentation fixes**: anywhere a builder stumbled, the doc is wrong.
5. **Code and hardware changes**: welcome, with two hard rules: anything touching the detector core must pass the golden-vector suite or be explicitly proposed as a new (flagged, opt-in) variant with its own calibration evidence; and nothing outside the project scope (detection + peer alert only: see [DISCLAIMER.md](DISCLAIMER.md)) will be merged, regardless of framing.

Engineering culture, in one line: **measured numbers with stated conditions beat plausible reasoning**, here as in the project's own history. If you claim a change helps, bring the measurement.
