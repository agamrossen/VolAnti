#!/usr/bin/env python
"""
beamform_offline.py - delay-and-sum steering, measured on DEVICE captures.

    python scripts/beamform_offline.py

the design OFFLINE ONLY. Nothing here goes near the firmware, and nothing
in the detector consults it. The point is to close a question with a number
instead of leaving it as an intuition that keeps coming back.

THE TWO QUESTIONS

(a) In the comb band, does steering beat the plain broadside sum?
    Physics says no, and says why: a 61 mm array against a 0.86 m wavelength
    at 400 Hz is 0.07 of a wavelength across. Steering to any direction
    changes the inter-channel delays by at most 0.18 ms, which at 400 Hz is
    26 degrees of phase - the four capsules are effectively co-located and
    every steering vector is nearly the same vector. The measurement here is
    the number that lets the question be retired.

(b) In the Tier-3 band, is a STEERED coherent sum better than the incoherent
    envelope average Tier-3 actually uses? Above 3.2 kHz the array is bigger
    than half a wavelength, so steering is meaningful - but the measured
    inter-capsule coherence there is 0.21-0.33, and you cannot coherently sum
    what is not coherent. Both are computed on the same rotor captures.

Delays are FRACTIONAL. A sample at 16 kHz is 21.4 mm of path, and the whole
array is 61 mm, so integer-sample steering would quantise the entire steering
range into three steps. Implemented as a linear phase ramp in the frequency
domain, which is exact for a band-limited signal.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "src"))

import geometry as G                                            # noqa: E402

FS = 16000
CAP = ROOT / "firmware/sentry_node/captures"


def load(rel):
    d = np.load(str(CAP / rel))
    return np.stack([c.astype(np.float64) / 32767.0 for c in d["audio"]])


def frac_delay(x, samples):
    """Delay by a fractional number of samples via a linear phase ramp."""
    n = len(x)
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(n, 1.0)
    return np.fft.irfft(X * np.exp(-2j * np.pi * f * samples), n)


def steer(X, g, az, el=0.0):
    """Align the four channels for a source at (az, el) and sum."""
    arr = g.arrival_samples(az, el)
    keys = sorted(arr)
    out = np.zeros(X.shape[1])
    for i, k in enumerate(keys):
        out += frac_delay(X[i], -arr[k])
    return out


def band_power_db(x, lo, hi, fs=FS):
    X = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    m = (f >= lo) & (f < hi)
    return 10.0 * np.log10(float(X[m].mean()) + 1e-30)


def coherence(a, b, lo, hi, nper=2048, fs=FS):
    from scipy.signal import coherence as coh
    f, c = coh(a, b, fs=fs, nperseg=nper)
    m = (f >= lo) & (f < hi)
    return float(np.mean(c[m]))


def sweep(X, g, bands, az_step=15.0, seconds=8.0):
    """Best steered band power over an azimuth sweep, against the plain sum."""
    n = int(seconds * FS)
    X = X[:, :n]
    plain = X.sum(axis=0)
    out = {}
    for name, (lo, hi) in bands.items():
        base = band_power_db(plain, lo, hi)
        best, best_az = -1e30, None
        for az in np.arange(0.0, 360.0, az_step):
            p = band_power_db(steer(X, g, float(az)), lo, hi)
            if p > best:
                best, best_az = p, float(az)
        out[name] = {"plain_db": base, "best_steered_db": best,
                     "gain_db": best - base, "best_az_deg": best_az}
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--geometry", default="breadboard_plus_v1")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)

    doc = json.loads((ROOT / "data/geometry_profiles.json").read_text())
    g = G.Geometry(doc, a.geometry)
    bands = {"comb 200-800": (200.0, 800.0),
             "mid 800-2000": (800.0, 2000.0),
             "T3 3200-7800": (3200.0, 7800.0)}

    print(f"geometry {a.geometry}: longest baseline "
          f"{g.longest_baseline()['mm']:.1f} mm, half-wavelength at "
          f"{g.half_wavelength_hz():.0f} Hz, spatial aliasing above "
          f"{g.alias_free_hz():.0f} Hz\n")

    clips = [("rotor 4 m", "2026-08-13_1445_ldtest/rotor_4m_0_raw.npz"),
             ("rotor 10 m", "2026-08-13_1445_ldtest/rotor_10m_1_raw.npz"),
             ("quiet", "2026-08-13_1445_ldtest/quiet_raw.npz")]
    res = {}
    for name, rel in clips:
        if not (CAP / rel).exists():
            continue
        X = load(rel)
        r = sweep(X, g, bands, seconds=a.seconds)
        res[name] = r
        print(f"{name}")
        for b, v in r.items():
            print(f"   {b:<14} plain {v['plain_db']:7.2f} dB   "
                  f"best steered {v['best_steered_db']:7.2f} dB   "
                  f"gain {v['gain_db']:+5.2f} dB at az {v['best_az_deg']:.0f}")
        cs = [coherence(X[i, :int(a.seconds * FS)], X[j, :int(a.seconds * FS)],
                        3200.0, 7800.0)
              for i in range(4) for j in range(i + 1, 4)]
        cl = [coherence(X[i, :int(a.seconds * FS)], X[j, :int(a.seconds * FS)],
                        200.0, 800.0)
              for i in range(4) for j in range(i + 1, 4)]
        res[name]["coherence"] = {"comb": (min(cl), max(cl)),
                                  "t3": (min(cs), max(cs))}
        print(f"   inter-capsule coherence  200-800 Hz "
              f"{min(cl):.2f}-{max(cl):.2f}   3.2-7.8 kHz "
              f"{min(cs):.2f}-{max(cs):.2f}")
        print()
    if a.json:
        Path(a.json).write_text(json.dumps(res, indent=1, default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())
