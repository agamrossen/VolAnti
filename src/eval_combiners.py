"""
eval_combiners.py - the combiner experiments: instrument, mini-synth, verdicts.

Three parts, in the order they have to be trusted
-------------------------------------------------
1. A floor-independent instrument (tooth SNR). The adaptive floor is a
   nonlinear filter with seconds of memory. Judging combiners by the comb score
   alone would confound "this combiner passes more of the tooth" with "this
   combiner made the floor settle somewhere else", and the second effect is
   larger. Tooth SNR measures a harmonic against its own neighbourhood in the
   same frame, so the floor cannot enter the comparison.

2. A spatial mini-synth, whose answers are known by construction. This is
   validation machinery, not a corpus: no P(d), no false-alarm rate and no
   operating point may be derived from it. Its only job is to prove that the
   combiners do what the algebra says before they are pointed at real audio.

3. Verdicts on the four real captures. Whatever the mini-synth says, the
   decision belongs to real audio.

The shipped default stays CX-A whatever this file finds. Changing it needs
recorded evidence, and this file's output is one input to that decision.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

import combiners as CX
import detector as D
import detector_t2 as T2
import operating_point as op

_HERE = Path(__file__).resolve().parent
ROOT = _HERE.parent
DATA_DIR = ROOT / "data"
CAPS = ROOT / "firmware" / "sentry_node" / "captures" / "2026-08-13_1445_ldtest"
sys.path.insert(0, str(ROOT / "firmware" / "sentry_node" / "scripts"))

import geometry as G                                            # noqa: E402

FS = 16000
N_FFT = 2048
HOP = 512
C_SOUND = 343.0


# ===========================================================================
# 2. the spatial mini-synth  (validation only - not a corpus)
# ===========================================================================

def fractional_delay(x, d, n_fft=None):
    """Delay x by d samples (d may be fractional, and may be negative).

    Implemented as an exact phase ramp on the whole-signal rFFT, which IS
    ideal sinc interpolation - the sinc kernel is what an ideal band-limited
    delay convolves with, and a phase ramp applies it without truncation.
    Wraparound is circular; callers pad. Kept exact on purpose: the CX-D test
    asserts recovery to 1e-5 relative, and a truncated FIR sinc would fail that
    for reasons that have nothing to do with the combiner.
    """
    n = len(x)
    f = np.fft.rfftfreq(n, 1.0)
    return np.fft.irfft(np.fft.rfft(x) * np.exp(-2j * np.pi * f * d), n)


def diffuse_field(n, n_ch, d_bar_m, seed=0, fs=FS, c=C_SOUND):
    """A diffuse ambient with the right inter-channel coherence.

        X_i(f) = sqrt(rho(f)) * N0(f) + sqrt(1 - rho(f)) * N_i(f)
        rho(f)  = |sinc(2 pi f d_bar / c)|

    Approximations (this is why it is validation machinery and not a
    corpus):
      - One mean spacing d_bar is used for every pair. A real array has
        several distinct spacings and therefore several coherence curves.
      - The true diffuse-field coherence sin(x)/x goes negative above the
        first zero; taking the magnitude discards that sign. Above the first
        zero this model is wrong in phase, right in magnitude.
      - The construction is exact in the one property it is built for: the
        magnitude-squared coherence between any two channels is rho(f).
    """
    rng = np.random.default_rng(seed)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    x = 2.0 * np.pi * f * d_bar_m / c
    rho = np.abs(np.sinc(x / np.pi))            # np.sinc(t) = sin(pi t)/(pi t)
    N0 = np.fft.rfft(rng.standard_normal(n))
    out = []
    for _ in range(n_ch):
        Ni = np.fft.rfft(rng.standard_normal(n))
        out.append(np.fft.irfft(np.sqrt(rho) * N0
                                + np.sqrt(1.0 - rho) * Ni, n))
    return np.array(out)


def spatial_quad(sig, geom, az_deg=0.0, el_deg=0.0, self_noise=0.0,
                 diffuse=0.0, bus_lag_samples=0.0, bus="B", seed=0, fs=FS):
    """A point source at (az, el) recorded by `geom`'s four capsules.

    bus_lag_samples is an electrical start offset applied to every channel on
    `bus` - the thing CX-D compensates. It is applied after the acoustic
    delays and identically to both channels on that bus, because two capsules
    hanging off one I2S RX engine start together whatever the sound is doing.
    """
    sig = np.asarray(sig, float)
    n = len(sig)
    arr = geom.arrival_samples(az_deg, el_deg)
    rng = np.random.default_rng(seed + 991)
    chans = []
    for k in G.CH_ORDER:
        d = arr[k] + (bus_lag_samples if geom.bus[k] == bus else 0.0)
        chans.append(fractional_delay(sig, d))
    x = np.array(chans)
    if diffuse > 0:
        d_bar = float(np.mean([geom.baseline_mm(a, b) * 1e-3
                               for i, a in enumerate(G.CH_ORDER)
                               for b in G.CH_ORDER[i + 1:]]))
        x = x + diffuse * diffuse_field(n, 4, d_bar, seed=seed, fs=fs)
    if self_noise > 0:
        x = x + self_noise * rng.standard_normal(x.shape)
    return x


# ===========================================================================
# 1. the instrument
# ===========================================================================

def frame_spectra(x, i, n_fft=N_FFT, window=None):
    w = window if window is not None else np.hanning(n_fft).astype(np.float32)
    return [np.fft.rfft(x[ch, i:i + n_fft] * w) for ch in range(x.shape[0])]


def comb_f0_floor_free(x, lo=200.0, hi=2000.0, k_max=12, skip_s=1.0,
                       stride=4, n_fft=N_FFT, hop=HOP, fs=FS):
    """f0 from a comb search on the time-median power spectrum, with no floor
    anywhere in it.

    This exists because the obvious estimator does not work on the captures
    that matter. v1's argmax is computed on a whitened spectrum, and the floor
    that whitens it is initialised from the first frame - so on a capture where
    the source was already running when recording started, the source is baked
    into the floor, the whitened spectrum is flat, and the argmax wanders. That
    is the exact shape of a rig capture (the motor is spinning before anyone
    presses record), and a tooth-SNR table read at a wandering f0 measures the
    gaps and reports that the comb is not there.

    The statistic is the comb score's own arithmetic - teeth minus gaps, in dB,
     1/sqrt(k) weighted - on log power, which needs no floor because the median
    over time already is one.
    """
    w = np.hanning(n_fft).astype(np.float64)
    rows = []
    for i in range(int(skip_s * fs), x.shape[1] - n_fft, hop * stride):
        rows.append(np.abs(np.fft.rfft(x[:, i:i + n_fft].astype(np.float64)
                                       * w, axis=1)).mean(axis=0))
    if not rows:
        return float("nan"), float("nan")
    P = np.median(np.array(rows), axis=0) ** 2
    L = 10.0 * np.log10(P + 1e-30)
    n_bins = len(L)
    bw = fs / n_fft
    kw = 1.0 / np.sqrt(np.arange(1, k_max + 1))
    best_f, best_s = float("nan"), -1e30
    for f0 in np.arange(lo, hi + 1e-9, 1.0):
        t = g = tw = gw = 0.0
        for k in range(1, k_max + 1):
            it = int(round(k * f0 / bw))
            ig = int(round((k + 0.5) * f0 / bw))
            if it >= n_bins or ig >= n_bins:
                break
            t += kw[k - 1] * L[it]
            g += kw[k - 1] * L[ig]
            tw += kw[k - 1]
            gw += kw[k - 1]
        if tw <= 0:
            continue
        s = t / tw - g / gw
        if s > best_s:
            best_s, best_f = s, float(f0)
    return best_f, best_s


def _dominant_f0(f0, lo, hi, frac=0.10):
    """The in-band f0 that the most frames agree with to within +-frac.

    A median in-band winner is right when most frames carry the source.
    It is wrong when they do not: a capture that is
    silent for a third of its length has a bimodal argmax distribution -
    wandering noise, and the source - and the median lands in the gap between
    them, at a frequency nothing was ever at. Measured on an 8 s clip with a
    430 Hz comb arriving at t=4 s: the argmax locks at 429-431 Hz, and the
    median of the in-band frames is 722 Hz.

    So the estimator is the value the most frames cluster around, which is what
    "the winner" was always trying to mean. It reduces to the median when the
    distribution is unimodal.
    """
    v = np.asarray(f0, float)
    v = v[(v >= lo) & (v <= hi)]
    if not len(v):
        return float("nan")
    edges = np.arange(lo, hi + 2.0, 1.0)
    hist, _ = np.histogram(v, bins=edges)
    cum = np.concatenate([[0], np.cumsum(hist)])
    centres = edges[:-1]
    j_lo = np.clip(np.searchsorted(edges, centres * (1.0 - frac)),
                   0, len(cum) - 1)
    j_hi = np.clip(np.searchsorted(edges, centres * (1.0 + frac)),
                   0, len(cum) - 1)
    counts = cum[j_hi] - cum[j_lo]
    best = int(np.argmax(counts))
    win = v[np.abs(v - centres[best]) <= frac * centres[best]]
    return float(np.median(win)) if len(win) else float(centres[best])


def estimate_f0_track(x, cfg=None, lo=200.0, hi=2000.0, agree_frac=0.10):
    """Per-frame f0 from the v1 argmax, restricted to +-10% of the dominant
    in-band winner, with an arbiter.

    The restriction matters on its own terms: the raw argmax octave-slips on
    16.9% of frames, and a tooth-SNR table computed at a slipped f0 measures
    the gap, not the tooth.

    The arbiter. v1's argmax is computed on a whitened spectrum, so it inherits
    the floor - and on a capture where the source was already running when
    recording started, the floor was initialised from a frame that already
    contained the source. The whitened spectrum is then flat, the argmax
    wanders, and a table read at that f0 reports confidently that the comb is
    not there. That is the shape of every rig capture: the motor is spinning
    before anyone presses record.

    So a floor-free estimate is computed too, and it arbitrates:

      they agree (within agree_frac)  -> the v1 track is describing the real
                                         source; use it per frame where it
                                         agrees, and the dominant value
                                         elsewhere. A source whose f0 genuinely
                                         moves keeps its per-frame detail here.
      they disagree                   -> the floor cannot fool the floor-free
                                         estimate, so it wins and the track is
                                         held at that value.

    Returns (per-frame track, dominant f0, mask of frames the v1 argmax
    supplied). `keep.any() == False` means the whole track came from the
    floor-free estimate, which is worth knowing when reading the table.
    """
    cfg = cfg or op.preset_config("HIGH_ALERT")[0]
    det = D.CombDetector(cfg)
    st = D.DetectorState(det.n_bins, det.cfg, None)
    c = det.cfg
    nf = max(0, 1 + (x.shape[1] - c.n_fft) // c.hop)
    f0 = np.zeros(nf)
    for i in range(nf):
        s0 = i * c.hop
        spec = det.combiner(frame_spectra(x, s0, c.n_fft, det.window))
        rec = det.back_end(spec, st, (s0 + c.n_fft) / c.fs, None)
        f0[i] = rec["f0"]

    med = _dominant_f0(f0, lo, hi)
    ff, _ = comb_f0_floor_free(x, lo=lo, hi=hi)
    if np.isfinite(ff) and (not np.isfinite(med)
                            or abs(med - ff) > agree_frac * ff):
        med = ff
    if not np.isfinite(med):
        return f0, med, np.zeros(nf, bool)
    keep = np.abs(f0 - med) <= 0.10 * med
    track = np.where(keep, f0, med)
    return track, float(med), keep


def tooth_snr_table(x, combiner, f0, frames=None, k_max=12, skip_s=1.0,
                    n_fft=N_FFT, hop=HOP, fs=FS):
    """Median per-harmonic tooth SNR (dB) over the analysed frames.

    Skips the first `skip_s` of every capture: the INMP441 startup transient
    is ~500 ms and the floor needs longer, and this instrument is supposed to
    measure the combiner, not the power-on."""
    w = np.hanning(n_fft).astype(np.float32)
    nf = max(0, 1 + (x.shape[1] - n_fft) // hop)
    i0 = int(skip_s * fs / hop)
    idx = range(i0, nf) if frames is None else frames
    rows = []
    for i in idx:
        s0 = i * hop
        spec = combiner(frame_spectra(x, s0, n_fft, w))
        psd = np.abs(spec) ** 2
        f = f0[i] if hasattr(f0, "__len__") else f0
        if not np.isfinite(f) or f <= 0:
            continue
        rows.append(CX.tooth_snr_db(psd, f, k_max=k_max, fs=fs, n_fft=n_fft))
    if not rows:
        return [float("nan")] * k_max
    A = np.array(rows, float)
    with np.errstate(invalid="ignore"):
        return [float(np.nanmedian(A[:, k])) for k in range(k_max)]


# ===========================================================================
# 3. verdicts
# ===========================================================================

VARIANTS = [("a", None, None), ("b", None, None),
            ("c", 1000.0, None), ("c", 1500.0, None), ("c", 2000.0, None),
            ("c", 2800.0, None), ("d", None, 0.0)]


def variant_combiners(delta=0.0):
    out = []
    for name, split, dl in VARIANTS:
        dl = delta if name == "d" else dl
        out.append((CX.label(name, split, dl),
                    CX.make_combiner(name, f_split=split or 1500.0,
                                     delta_samples=dl or 0.0)))
    return out


def load_capture(name):
    p = CAPS / f"{name}_raw.npz"
    if not p.exists():
        return None
    a = np.load(p)["audio"]
    return np.stack([np.asarray(ch, np.int16).astype(np.float32) / 32767.0
                     for ch in a])


def verdicts_on_captures(names=("quiet", "talk", "rotor_4m_0", "rotor_10m_1"),
                         delta=0.0, seconds=None):
    """Tooth-SNR tables plus full v1+T2 replays with the combiner swapped."""
    cfg, thr = op.preset_config("HIGH_ALERT")
    out = {}
    for nm in names:
        x = load_capture(nm)
        if x is None:
            continue
        if seconds:
            x = x[:, :int(seconds * FS)]
        f0, med, _ = estimate_f0_track(x, cfg)
        rows = {}
        for label, cf in variant_combiners(delta):
            t0 = time.time()
            ts = tooth_snr_table(x, cf, f0)
            r = T2.analyze_quad_t2(x, cfg, T2.T2Config(tau2_rise_s=60.0),
                                   thr_ref=thr, combiner=cf)
            ev1, _ = D.track_frames(r, thr, cfg)
            rows[label] = {
                "tooth_snr_db": ts,
                "v1_events": len(ev1),
                "v1_score_med": float(np.median(r["score"])),
                "v1_score_p99": float(np.percentile(r["score"], 99)),
                "t2_events": len(r["t2_events"]),
                "t2_score_med": float(np.median(r["t2"]["score2"])),
                "t2_score_p99": float(np.percentile(r["t2"]["score2"], 99)),
                "kappa_med": float(np.nanmedian(r["t2"]["kappa"])),
                "secs": time.time() - t0}
        out[nm] = {"f0_median_hz": med, "variants": rows}
        print(f"  {nm}: f0 median {med:.0f} Hz", flush=True)
    return out


def print_verdicts(res):
    for nm, blob in res.items():
        print(f"\n--- {nm}   (f0 track median {blob['f0_median_hz']:.0f} Hz) ---")
        print(f"  {'combiner':<22} {'v1ev':>4} {'v1med':>6} {'T2ev':>4} "
              f"{'T2med':>6} {'kappa':>6}   tooth SNR dB by harmonic k=1..12")
        for label, r in blob["variants"].items():
            ts = "  ".join("  na" if not np.isfinite(v) else f"{v:4.1f}"
                           for v in r["tooth_snr_db"])
            print(f"  {label:<22} {r['v1_events']:>4} {r['v1_score_med']:>6.2f} "
                  f"{r['t2_events']:>4} {r['t2_score_med']:>6.2f} "
                  f"{r['kappa_med']:>6.3f}   {ts}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--delta", type=float, default=0.0,
                    help="bus-B lag in samples for CX-D (0 until the bench "
                         "proves the offset is constant across resets)")
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--out", default=str(DATA_DIR / "combiner_verdicts.json"))
    a = ap.parse_args(argv)
    res = verdicts_on_captures(delta=a.delta, seconds=a.seconds)
    print_verdicts(res)
    Path(a.out).write_text(json.dumps(res, indent=2))
    print(f"\nwrote {a.out}")
    return res


if __name__ == "__main__":
    main()
