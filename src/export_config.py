"""
export_config.py - one source of truth for every constant the firmware needs.

Writes data/device_config.json. The firmware header generator consumes this
file and nothing else: if a constant is not in here, the firmware must not
have it. Regenerate after any change to operating_point.py or detector.Config.

Deliberately includes the disabled features (comb-hold, re-anchoring,
harmonic-extent gate) with their state and the reason they are off. A
firmware author who cannot see that a knob exists and was measured will
eventually re-invent it.

Run: conda run -n acoustic-detector python src/export_config.py
"""

import json
from pathlib import Path

import operating_point as op
from detector import Config

DATA_DIR = Path(__file__).resolve().parent.parent / "data"

# Why each measured-and-disabled feature is off. Kept next to the values so
# the two cannot drift apart.
DISABLED_NOTES = {
    "hold_mode": "comb-hold. "
                 "Set to 'chain' or 'cand' only with new measurements.",
    "reanchor": "subharmonic re-anchoring: measured zero change in FA and "
                "P(d); frame-level octave slip rose 16.9%->17.8%.",
    "min_teeth": "harmonic-extent gate: no separation against livestock, "
                 "which saturates at 12/12 supported teeth exactly as drones "
                 "do. That is why it is off. It is not a general result: a "
                 "piano is inharmonic (stiff string, partials at "
                 "n*f0*sqrt(1+B*n^2)) and measures 8/11/12 teeth at "
                 "p10/median/p90 where every drone class measures 12/12/12, "
                 "so the same gate at 11 is the veto's second term. Rejected "
                 "against the wrong adversary; see veto_voice.",
    "veto_voice": "the voice and struck-note veto, calibrated against "
                  "synthetic piano and held-vowel families. Recalibrated and "
                  "paired over 486 positives it is 0 lost / 92 gained, "
                  "McNemar p < 0.0001, and it removes a held vowel entirely.",
    "thr_off_prio": "two-tier band offset: measured no effect - the threat "
                    "and the binding confuser both sit in the priority band, "
                    "so any offset is cancelled by threshold recalibration.",
}


def build():
    base, _ = op.preset_config(op.DEFAULT_PRESET)
    d = base.to_dict()
    presets = {name: dict(p) for name, p in op.PRESETS.items()}
    return {
        "schema": "acoustic-detector/device_config/1",
        "site": "rural",
        "corpus_tag": op.CORPUS_TAG,
        "default_preset": op.DEFAULT_PRESET,
        "presets": presets,
        "budgets_weighted_fa_per_hour_nongust": {
            "NORMAL": op.NORMAL_BUDGET_WEIGHTED_PER_HOUR,
            "HIGH_ALERT": op.HIGH_ALERT_BUDGET_WEIGHTED_PER_HOUR,
        },
        "detector": d,
        "committed_overrides": dict(op.COMMITTED_DETECTOR),
        "disabled_features": {
            k: {"value": d.get(k), "why": v} for k, v in DISABLED_NOTES.items()
        },
        "frame": {
            "fs_hz": d["fs"], "n_fft": d["n_fft"], "hop": d["hop"],
            "frame_period_s": d["hop"] / d["fs"],
            "n_bins": d["n_fft"] // 2 + 1,
            "bin_width_hz": d["fs"] / d["n_fft"],
            "cycles_at_240mhz": 240e6 * d["hop"] / d["fs"],
        },
        "golden_vectors": ["golden_strong_pos", "golden_marginal_neg",
                           "golden_marginal_pos"],
        "trace_fields": ["frame", "t_s", "score", "f0_bin", "f0_hz",
                         "f0_raw_hz", "teeth", "floor_fast", "reanch",
                         "n_held_bins", "above_thr", "cont_accepted", "chain",
                         "fired"],
    }


def main():
    DATA_DIR.mkdir(exist_ok=True)
    cfg = build()
    p = DATA_DIR / "device_config.json"
    p.write_text(json.dumps(cfg, indent=2))
    print(f"wrote {p}")
    print(f"  default preset : {cfg['default_preset']} "
          f"(threshold {cfg['presets'][cfg['default_preset']]['threshold']})")
    print(f"  floor mode     : {cfg['detector']['floor_mode']}")
    print(f"  comb-hold      : {cfg['detector']['hold_mode']}")
    print(f"  frame budget   : {cfg['frame']['cycles_at_240mhz']:.0f} cycles")
    return cfg


if __name__ == "__main__":
    main()
