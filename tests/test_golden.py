"""
test_golden.py - checks the golden vectors against the reference detector.

For each golden vector, data/<name>.h (the C array the ESP32 replays from
flash) and data/<name>_trace.csv (the per-frame output the ESP32 must
reproduce) are written from what is supposed to be the same clip. If they ever
drifted apart, the firmware would be compared against a trace belonging to a
signal it was never given, and the mismatch would look like a porting bug for
as long as it took to notice. This test re-derives every trace from its header
and requires it to match the CSV.

It checks the whole trajectory - score, argmax bin, continuity decision, chain
counter, latch - not just the final verdict, because the two marginal vectors
exist precisely to catch a port that reaches the right answer by the wrong
route.

Also asserts the corpus is bit-reproducible: same seed -> same samples.

Run: conda run -n acoustic-detector python tests/test_golden.py
"""

import json
import re
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT / "src"))

import synth                                              # noqa: E402
from detector import CombDetector, Config, track_frames   # noqa: E402
import operating_point as op                              # noqa: E402

DATA = _ROOT / "data"
# The three parity vectors are cut at NORMAL (2.14). golden_high_alert_pos is
# cut at HIGH_ALERT (1.70) because that is the deployment default and nothing
# else in the golden set exercises it. Each vector is replayed at its own
# preset threshold, read from golden_vectors.json - never at a single global
# one, or the HIGH_ALERT vector would be judged against the wrong operating
# point and would look like a porting bug.
VECTORS = ["golden_strong_pos", "golden_marginal_neg",
           "golden_marginal_pos", "golden_high_alert_pos"]


def read_c_header(path):
    txt = path.read_text()
    body = txt.split("{", 1)[1].rsplit("}", 1)[0]
    vals = np.array([int(v) for v in re.findall(r"-?\d+", body)], np.int16)
    fs = int(re.search(r"#define \w+_FS (\d+)", txt).group(1))
    n = int(re.search(r"#define \w+_LEN (\d+)", txt).group(1))
    assert len(vals) == n, f"header LEN={n} but {len(vals)} samples parsed"
    return vals, fs


def read_trace_csv(path):
    txt = path.read_text().splitlines()
    hdr = next(l for l in txt if l.startswith("frame,"))
    cols = hdr.split(",")
    rows = [l for l in txt if l and not l.startswith("#") and l is not hdr
            and not l.startswith("frame,")]
    a = np.array([[float(v) for v in l.split(",")] for l in rows])
    return {c: a[:, i] for i, c in enumerate(cols)}


def main():
    fails = []

    cfg, THRESHOLD = op.preset_config("NORMAL")

    # ---- 1. the vectors were cut at the current preset -------------------
    gvp = DATA / "golden_vectors.json"
    if not gvp.exists():
        fails.append("data/golden_vectors.json missing - run make_golden.py")
        info = None
    else:
        info = json.loads(gvp.read_text())
        if abs(info["threshold"] - THRESHOLD) > 1e-9:
            fails.append(f"vectors cut at threshold {info['threshold']} but "
                         f"the NORMAL preset is {THRESHOLD} - rerun "
                         f"`python src/make_golden.py`")
        for k, v in info["config"].items():
            if k in ("floor_mode", "max_jitter", "reanchor", "min_teeth",
                     "f_prio_lo", "f_prio_hi", "thr_off_prio", "thr_off_gen"):
                if getattr(cfg, k) != v:
                    fails.append(f"vectors were cut with {k}={v} but the "
                                 f"committed config has {getattr(cfg, k)}")
        if not any("rerun" in f or "cut with" in f for f in fails):
            print(f"OK  vectors match the committed NORMAL preset "
                  f"(thr {THRESHOLD:.4f}, floor {cfg.floor_mode}, "
                  f"max_jitter {cfg.max_jitter})")

    def preset_of(name):
        """The preset a vector was cut at, per golden_vectors.json."""
        if info:
            for v in info["vectors"]:
                if v["name"] == name:
                    return v.get("preset", "NORMAL")
        return "NORMAL"

    # ---- 2. every trace reproduces FROM its own header --------------------
    for name in VECTORS:
        preset = preset_of(name)
        thr = op.PRESETS[preset]["threshold"]
        hp, cp = DATA / f"{name}.h", DATA / f"{name}_trace.csv"
        if not hp.exists() or not cp.exists():
            fails.append(f"{name}: missing .h or _trace.csv - run "
                         f"`python src/make_golden.py`")
            continue
        q, _ = read_c_header(hp)
        c = read_trace_csv(cp)
        tr = CombDetector(cfg).analyze(q.astype(np.float32) / 32767.0)
        ev, fr = track_frames(tr, thr, cfg)
        if len(c["frame"]) != len(tr["t"]):
            fails.append(f"{name}: trace length {len(c['frame'])} != detector "
                         f"frames {len(tr['t'])}")
            continue
        f0_bin = np.round((tr["f0"] - cfg.f_search_lo) / cfg.f_step)
        checks = {
            "score": (np.max(np.abs(c["score"] - tr["score"])), 2e-6),
            "f0_hz": (np.max(np.abs(c["f0_hz"] - tr["f0"])), 0.05),
            "f0_bin": (np.max(np.abs(c["f0_bin"] - f0_bin)), 0),
            "above_thr": (np.max(np.abs(c["above_thr"] - fr["above_thr"])), 0),
            "cont_accepted": (np.max(np.abs(c["cont_accepted"]
                                            - fr["accepted"])), 0),
            "chain": (np.max(np.abs(c["chain"] - fr["chain"])), 0),
            "fired": (np.max(np.abs(c["fired"] - fr["active"])), 0),
            # v2 state - if these drift the floors or the gates have diverged
            "f0_raw_hz": (np.max(np.abs(c["f0_raw_hz"] - tr["f0_raw"])), 0.05),
            "teeth": (np.max(np.abs(c["teeth"] - tr["teeth"])), 0),
            "floor_fast": (np.max(np.abs(c["floor_fast"]
                                         - tr["fast"].astype(float))), 0),
            "reanch": (np.max(np.abs(c["reanch"]
                                     - tr["reanch"].astype(float))), 0),
            "n_held_bins": (np.max(np.abs(c["n_held_bins"]
                                          - tr["n_held"].astype(float))), 0),
        }
        bad = [f"{k}(max diff {v:.3g})" for k, (v, tol) in checks.items()
               if v > tol]
        if bad:
            fails.append(f"{name}: trajectory mismatch vs header-derived run: "
                         f"{', '.join(bad)} - rerun `python src/make_golden.py`")
        else:
            print(f"OK  {name:<22} {len(tr['t']):4d} frames, chain peak "
                  f"{int(fr['chain'].max())}/{cfg.track_need}, "
                  f"{'FIRES' if ev else 'NO FIRE':<7} @ {preset:<10} "
                  f"thr {thr:.4f} (score/bin/continuity/chain/latch match)")

    # ---- 3. the marginal vectors really are marginal -----------------------
    # Each is judged against its own preset threshold.
    if info:
        for v in info["vectors"]:
            if v["role"] == "strong positive":
                continue
            own = op.PRESETS[v.get("preset", "NORMAL")]["threshold"]
            ft = v.get("flip_threshold")
            if ft is None:
                fails.append(f"{v['name']} has no flip threshold - it is not "
                             f"marginal and cannot catch a float32 bug")
            elif abs(ft - own) > 0.25:
                fails.append(f"{v['name']} flips at {ft:.3f}, which is "
                             f"{abs(ft - own):.3f} from its operating point "
                             f"{own:.4f} - too far to be a marginal vector")
        if not any("marginal" in f or "flip" in f for f in fails):
            print("OK  every marginal vector flips within 0.25 of its own "
                  "preset threshold")

    # ---- 3b. the HIGH_ALERT vector must separate the two presets ----------
    # Its whole reason to exist: it fires at the deployment default and is
    # silent at the parity preset. If it fired at both it would test nothing
    # the other three do not already test.
    if info:
        ha = [v for v in info["vectors"]
              if v.get("preset") == "HIGH_ALERT"]
        if not ha:
            fails.append("no HIGH_ALERT vector - the deployment default "
                         "(1.70) has no golden evidence")
        for v in ha:
            lo = op.HIGH_ALERT["threshold"]
            hi = op.NORMAL["threshold"]
            ft = v.get("flip_threshold")
            if v["verdict"] != "FIRES":
                fails.append(f"{v['name']} does not fire at HIGH_ALERT")
            elif ft is None or not (lo <= ft < hi):
                fails.append(f"{v['name']} flips at {ft} - it must fire at "
                             f"{lo:.2f} and NOT at {hi:.2f}, i.e. flip in "
                             f"[{lo:.2f}, {hi:.2f})")
            else:
                print(f"OK  {v['name']} fires at HIGH_ALERT {lo:.4f} and is "
                      f"silent at NORMAL {hi:.4f} (flips at {ft:.3f}, "
                      f"cls={v.get('cls')})")

    # ---- 4. bit-reproducibility of the generators -------------------------
    kinds = ["drone_flyby", "drone_static", "prop_aircraft", "motorbike_accel",
             "helicopter", "dogs", "birds", "livestock", "diesel_generator",
             "irrigation_pump", "orchard_machinery", "tracked_vehicle",
             "wind_gusting", "artillery_distant", "friendly_multirotor"]
    bad = []
    for kind in kinds:
        a, _ = synth.make_clip(kind, snr_db=3.0, noise="wind", seed=7)
        b, _ = synth.make_clip(kind, snr_db=3.0, noise="wind", seed=7)
        if not np.array_equal(a, b):
            bad.append(kind)
    if bad:
        fails.append(f"not reproducible from seed: {', '.join(bad)}")
    else:
        print(f"OK  all {len(kinds)} generators bit-reproducible from seed")

    # ---- 5. a regression that once broke the corpus -----------------------
    # prop_aircraft / motorbike fundamentals must stay below f_alert_lo so the
    # band gate can do its job. This is deliberately not asserted for the
    # whole library: irrigation_pump, orchard_machinery, dogs and the dove
    # coos in birds have genuine in-band fundamentals, which is a real
    # property of those sources and a documented limitation, not a bug.
    for blades in (2, 3):
        for rpm in (2200.0, 2700.0):
            bpf = synth.bpf_from_rpm(rpm, blades)
            if bpf >= cfg.f_alert_lo:
                fails.append(f"prop_aircraft BPF {bpf:.0f} Hz ({blades} blades "
                             f"@ {rpm:.0f} rpm) is inside the alert band")
    if not any("alert band" in f for f in fails):
        print(f"OK  prop_aircraft BPF range stays below "
              f"f_alert_lo={cfg.f_alert_lo:.0f} Hz")

    print()
    if fails:
        for f in fails:
            print(f"FAIL  {f}")
        return 1
    print("all golden/reproducibility checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
