"""
combiners.py - drop-in replacements for CombDetector.combiner.

The seam. `detector.py` is split so that everything an array can do
differently from one microphone happens in exactly one function:

    front_end(block)        -> complex spectrum, one per channel
    combiner([spec, ...])   -> one spectrum          <-- this file
    back_end(spec, state)   -> per-frame decision

Nothing downstream of the combiner knows how many microphones there were,
which means a combiner experiment is a real experiment - the same sealed
detector, the same constants, the same tracker - and not a fork.

Why this is not beamforming
---------------------------
Every combiner here is geometry-free. None of them steers, none computes a
bearing, none needs to know where a microphone is. CX-D's delta is a measured
electrical constant - the start offset between two I2S RX engines - not a
steering delay. Steering weights would need a PCB with verified alignment.

The physics that motivates the experiment
-----------------------------------------
The shipped combiner is an unweighted complex sum, i.e. a broadside beam. That
is exactly right while the array is small compared with the wavelength: at
380 Hz the wavelength is ~90 cm against a 61 mm maximum baseline, so the four
channels add near-coherently from any direction and the sum buys the full
coherent +6 dB.

It stops being right as the wavelength approaches the baseline. lambda/2 at
60.96 mm is ~2.8 kHz. Above that an off-axis source can arrive at two
microphones in antiphase and the complex sum cancels the very teeth it is
supposed to be adding - and the comb score reaches to 7.8 kHz, so those teeth
are scored. Any uncompensated start offset between the two I2S buses makes it
worse and makes it start lower in frequency. Measured on the real quad
captures, inter-microphone coherence falls from 0.97 below 800 Hz to 0.23 above
3.2 kHz: the high teeth are already not adding coherently.

An incoherent (power) sum cannot cancel anything, at the cost of the coherent
gain. So the hypothesis worth testing is a hybrid: add complex where the array
is small compared with the wavelength, add power where it is not.

The shipped default is CX-A.
"""

import numpy as np


def cx_a_complex_sum(spectra):
    """CX-A - the shipped behaviour, byte for byte.

    Deliberately delegates nothing: this is a copy of detector.CombDetector's
    own combiner body, so an experiment harness that swaps combiners can use
    the same call path for the baseline as for the variants and never compare
    a swapped path against an unswapped one.
    """
    return spectra[0] if len(spectra) == 1 else np.sum(spectra, axis=0)


def cx_b_incoherent_rms(spectra):
    """CX-B - incoherent (power) sum: X(b) = sqrt(sum_c |X_c(b)|^2).

    Returned as a real spectrum, which is legal at this seam because back_end
    only ever takes |.| of it. Phase is discarded, so nothing can cancel; the
    price is the coherent gain, sqrt(N) instead of N in amplitude.

    Scale caveat, for the report: against four coherent channels CX-A gives 4x
    and CX-B gives 2x. In steady state the per-bin adaptive floor absorbs that
    factor completely and the whitened spectrum is unchanged. It is not
    absorbed during the first few seconds after a switch, nor at a CX-C split
    boundary before the floor has settled - so a combiner comparison made in
    the first ~5 s of a clip is measuring floor transients, not combiners.
    """
    X = np.asarray(spectra)
    return np.sqrt((np.abs(X) ** 2).sum(axis=0))


def cx_c_hybrid(spectra, f_split=1500.0, fs=16000, n_fft=2048):
    """CX-C - complex below f_split, incoherent above.

    f_split = inf is exactly CX-A; f_split = 0 is exactly CX-B. Both identities
    are asserted in the tests, because a hybrid that is not a strict
    generalisation of its two endpoints is a third algorithm pretending to be a
    knob.
    """
    n_bins = np.asarray(spectra[0]).shape[-1]
    f = np.arange(n_bins) * (fs / n_fft)
    lowband = f <= f_split
    if lowband.all():
        return cx_a_complex_sum(spectra)          # f_split = inf, bit-exact
    if not lowband.any():
        return cx_b_incoherent_rms(spectra)       # f_split = 0,   bit-exact
    hi = ~lowband
    out = np.zeros(n_bins, complex)
    # slice first, then reduce with the same function the baseline uses, so a
    # bin below the split is bit-identical to what CX-A would have produced
    # for it. Reducing the full array and masking afterwards would go through
    # a different numpy accumulation order and differ in the last bits.
    out[lowband] = cx_a_complex_sum([s[lowband] for s in spectra])
    out[hi] = cx_b_incoherent_rms([s[hi] for s in spectra])
    return out


def cx_d_offset_compensated(spectra, delta_samples=0.0, n_fft=2048,
                            bus_b=(2, 3)):
    """CX-D - complex sum with the inter-bus start offset removed.

        X = X_M1 + X_M2 + exp(+j*theta(b)) * (X_M3 + X_M4),
        theta(b) = 2*pi*b*delta/n_fft

    SIGN, stated once so it cannot be got backwards: `delta_samples` is how far
    bus B lags bus A. A signal delayed by d samples has spectrum
    X(b)*exp(-j*2*pi*b*d/N), so undoing it multiplies by exp(+j*2*pi*b*d/N).
    The tests prove this two ways - a synthetic capture with an injected lag
    must recover the delta=0 spectrum to within 1e-5 relative, and a broadband
    clap's compensated peak must sharpen, not smear.

    delta may be fractional host-side; the firmware takes an integer.

    This is not steering. delta is a property of two DMA engines starting at
    slightly different times, identical for every direction of arrival. It
    stays 0 until the bench proves the offset is constant across resets.
    """
    X = np.asarray(spectra)
    if X.shape[0] < max(bus_b) + 1:
        return cx_a_complex_sum(spectra)
    if float(delta_samples) == 0.0:
        # The device default. Not "close to CX-A", identical to it: regrouping
        # the sum as (M1+M2) + 1.0*(M3+M4) changes the accumulation order and
        # therefore the last bits, and `H` with its default arguments has to be
        # provably the same arithmetic as `G`.
        return cx_a_complex_sum(spectra)
    n_bins = X.shape[-1]
    b = np.arange(n_bins)
    rot = np.exp(1j * 2.0 * np.pi * b * float(delta_samples) / n_fft)
    a_idx = [i for i in range(X.shape[0]) if i not in bus_b]
    return X[a_idx].sum(axis=0) + rot * X[list(bus_b)].sum(axis=0)


# ---------------------------------------------------------------------------
# the registry the experiment harness and the firmware's `cx` argument share
# ---------------------------------------------------------------------------

CX_SPLITS = (1000.0, 1500.0, 2000.0, 2800.0, float("inf"))


def make_combiner(name="a", f_split=1500.0, delta_samples=0.0, fs=16000,
                  n_fft=2048):
    """name in {a, b, c, d}. Returns a callable(list of spectra) -> spectrum.

    The letters are the device's runtime `cx` argument, so a variant that wins
    offline can be given device time without a rebuild.
    """
    n = str(name).lower()
    if n == "a":
        return cx_a_complex_sum
    if n == "b":
        return cx_b_incoherent_rms
    if n == "c":
        return lambda S: cx_c_hybrid(S, f_split=f_split, fs=fs, n_fft=n_fft)
    if n == "d":
        return lambda S: cx_d_offset_compensated(S,
                                                 delta_samples=delta_samples,
                                                 n_fft=n_fft)
    raise ValueError(f"unknown combiner {name!r}; expected one of a, b, c, d")


def label(name, f_split=None, delta_samples=None):
    n = str(name).lower()
    if n == "c":
        return f"CX-C(f_split={f_split:.0f})" if np.isfinite(f_split) \
            else "CX-C(inf) == CX-A"
    if n == "d":
        return f"CX-D(delta={delta_samples:g})"
    return {"a": "CX-A complex sum", "b": "CX-B incoherent RMS"}[n]


# ---------------------------------------------------------------------------
# the floor-independent instrument
# ---------------------------------------------------------------------------

def tooth_snr_db(psd, f0, k_max=12, fs=16000, n_fft=2048, half_width=8,
                 skip=1):
    """Per-harmonic tooth SNR on one combined power spectrum.

        toothSNR_k = 10*log10( P(k*f0) / median(P in k*f0 +- half_width bins,
                                                excluding +- skip) )

    Deliberately floor-independent. The adaptive floor is a strong nonlinear
    filter with seconds-long memory, so comparing combiners through the comb
    score alone confounds "this combiner passes more of the tooth" with "this
    combiner made the floor settle somewhere else". This measures the tooth
    against its own local neighbourhood in the same frame, so the floor cannot
    enter the comparison at all.

    Returns a list of length k_max; entries are NaN where the harmonic is out
    of range.
    """
    P = np.asarray(psd, float)
    n_bins = len(P)
    bw = fs / n_fft
    out = []
    for k in range(1, k_max + 1):
        pos = k * f0 / bw
        i = int(round(pos))
        if i < half_width + 1 or i >= n_bins - half_width - 1:
            out.append(float("nan"))
            continue
        lo, hi = i - half_width, i + half_width + 1
        idx = np.arange(lo, hi)
        idx = idx[np.abs(idx - i) > skip]
        ref = float(np.median(P[idx]))
        out.append(10.0 * np.log10((P[i] + 1e-30) / (ref + 1e-30)))
    return out
