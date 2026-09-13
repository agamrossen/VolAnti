"""
wash_model.py - the broadband rotor "wash", measured and then made synthesisable.

Why this file exists
--------------------
Every drone generator in synth.py is a harmonic stack: energy at the blade-pass
fundamental and its multiples, and nothing else. That was a reasonable model
until a real propeller was recorded, at which point it became the single
largest known error in the corpus.

The first rotor captures showed that the real rotor puts almost nothing where
the model puts everything, and almost everything where the model puts nothing:

    band            rotor(4 m) - quiet room
    125 - 1000 Hz      0.0 to  0.7 dB     <-- where v1 and Tier 2 look
    1.4 - 2.0 kHz              1.6 dB
    2.0 - 2.8 kHz              3.9 dB
    2.8 - 4.0 kHz              7.2 dB
    4.0 - 5.7 kHz             17.1 dB
    5.7 - 8.0 kHz             24.4 dB     <-- where the rotor actually is

That high-band energy is broadband, not tonal, and it is amplitude modulated at
the shaft rate because it is made by a blade passing a fixed point. Tier 3
exists to demodulate it. But a corpus of comb-only positives cannot evaluate
Tier 3 at all: it would score ~0 on every "drone" in the corpus and the sweep
would be measuring nothing. So the wash has to go into the synthesiser, and it
has to go in fitted rather than invented.

What is fitted, and from how much data
--------------------------------------
Two captures: one rotor, two ranges, one room, one afternoon. That is nowhere
near enough to call these constants right. They are called measured, which is
a weaker and truer word: they are what those two files contain.

  Shape   the excess spectrum above the quiet room as a 1/3-octave curve,
          interpolated in log-frequency. Rising ~24 dB from 1 kHz to 7 kHz.

  Depth   the modulation index m_k at the first four harmonics of the shaft
          rate, fitted through the instrument: a synthetic wash is generated,
          run through detector_t3's own front end, and m is adjusted until the
          per-update prominence at k*r matches the real capture's. Result:

                        prom k=1..4 (dB)        fitted m_k
            4 m      22.3 18.1 16.6 12.9    0.455 0.271 0.207 0.128
            10 m     21.5 17.5 13.1 11.1    0.400 0.250 0.130 0.100

Fitting through the instrument was not optional
-----------------------------------------------
The obvious fit - take the envelope spectrum of the whole 60 s file and read
the harmonic amplitudes - returns m = [0.12, 0.09, 0.04, 0.03]. That is wrong
by a factor of four, and wrong in the direction that makes the drone look
weaker than it is. The rotor rate wanders a few percent over a minute, so a
whole-file transform smears each harmonic across several bins and reports the
smear. The detector never sees that smear: it transforms 0.5 s at a time, over
which the rate is stationary. Measure a quantity the way the thing that
consumes it measures it, or do not quote the number.

Range does not work the way the first fit suggested
---------------------------------------------------
The whole-file fit said depth collapses 4x between 4 m and 10 m. The
instrument-matched fit says it falls by about 10 percent at k=1 and 35 percent
at k=3 - close to flat. That is the physically sensible answer: modulation
depth is a property of the source, not of how far away you stand.

What range actually costs is level. The high band ran +24.4 dB over the quiet
room at 4 m and +18.2 dB at 10 m, i.e. about -6 dB for the 4x - close to
spherical spreading. Tier 3 therefore holds its score until the wash sinks
toward the ambient floor, and then falls off a cliff rather than fading. So
range in the corpus is applied the way it is applied to every other family -
as a signal-to-noise ratio in the mix - and not as a depth reduction.

The 4 m and 10 m captures score 33.4 and 31.9 through Tier 3. Two points 2.5x
apart in range differ by 1.5 in W, against an empirical null of 11.0. Nothing
here bounds the range at which Tier 3 stops working, because neither capture
came close to stopping it.

The null, measured
------------------
The quiet room and a 6-minute real-speech recording both return a median W
of 11.0 +- 0.5, and the reason is exact rather than empirical: the statistic
takes an argmax over 611 candidate rates, so pure noise yields a selection-
biased prominence of about [5.2, 4.0, 3.4, 2.9] dB at the winning rate, which
the weights sum to 11.4. The measured medians are 10.7 (quiet) and 11.7
(speech). tau3 = 20 therefore sits 9 dB of weighted prominence above a null
that was measured and not assumed.

Defaults off. No existing generator changes behaviour. The wash is reachable
only through the *_wash families, so every v1 and Tier 2 number measured
without it still describes the same corpus.
"""

import numpy as np

FS = 16000

# 1/3-octave band centres and the measured excess of rotor(4 m) over the quiet
# room, dB. Below 1 kHz the excess is within the measurement's own scatter
# (+-0.5 dB) and is written as the measured value rather than rounded to zero.
WASH_BANDS_HZ = np.array([150., 210., 300., 425., 600., 850., 1200., 1700.,
                          2400., 3400., 4800., 6800.])
WASH_EXCESS_DB = np.array([0.0, 0.9, 0.2, 0.5, 0.3, 0.7, 0.7, 1.6,
                           3.9, 7.2, 17.1, 24.4])

# Modulation index at k * shaft rate, fitted through detector_t3's own front
# end against the per-update prominences of the real captures. See the header:
# these are 4x the naive whole-file estimate, and the whole-file estimate is
# the one that is wrong.
WASH_M_4M = np.array([0.455, 0.271, 0.207, 0.128])
WASH_M_10M = np.array([0.400, 0.250, 0.130, 0.100])
WASH_M = WASH_M_4M                      # the default source model

# W returned by pure noise, because the statistic maximises over 611 rates.
# Measured: 10.7 quiet room, 11.7 six minutes of real speech; predicted 11.4
# from the selection-biased prominence vector. Quote this beside every tau3.
W_NULL = 11.0

# The wash's rms relative to the tonal comb's, in the *_wash families. A real
# rotor is nothing like -6 dB down: it is +24 dB up in the band where the wash
# lives and +0.5 dB in the band where the comb lives. This constant exists to
# let a corpus family carry both signatures at a stated ratio; the ratio itself
# is a corpus design choice, not a measurement, and is labelled as one.
WASH_LEVEL_DB = 0.0


def wash_shape(freqs, bands=WASH_BANDS_HZ, excess_db=WASH_EXCESS_DB):
    """Excess (dB) at arbitrary frequencies, log-interpolated between the
    measured 1/3-octave points and held flat outside them. Flat rather than
    extrapolated on purpose: extrapolating a +7 dB/octave slope past 6.8 kHz
    would invent 30 dB of energy at Nyquist that nothing measured."""
    lf = np.log10(np.maximum(np.asarray(freqs, float), 1.0))
    return np.interp(lf, np.log10(bands), excess_db,
                     left=excess_db[0], right=excess_db[-1])


def modulation_depths(range_m=None, n_harm=4):
    """m_k for the source.

    range_m is accepted and deliberately has almost no effect: between the only
    two ranges ever measured, 4 m and 10 m, depth changed by 10 percent at k=1.
    Passing a range interpolates between those two vectors and clamps outside
    them; it does not extrapolate a decay, because no decay was observed and
    inventing one would make Tier-3's range look bounded by evidence that does
    not exist. Range belongs in the mix SNR - see the header."""
    m4, m10 = WASH_M_4M[:n_harm], WASH_M_10M[:n_harm]
    if range_m is None:
        return m4.copy()
    r = float(np.clip(range_m, 4.0, 10.0))
    t = np.log(r / 4.0) / np.log(10.0 / 4.0)
    return np.exp((1 - t) * np.log(m4) + t * np.log(m10))


def wash(dur, shaft_hz, range_m=None, n_harm=4, seed=0, fs=FS,
         depths=None, phases=None, hp_hz=1000.0):
    """Broadband noise with the measured spectral shape, amplitude modulated at
    the shaft rate.

    shaft_hz may be a scalar or a per-sample array, so a rotor that spins up
    modulates at a rate that spins up with it. The modulation phase is the
    integral of the rate, which is what makes a swept AM coherent rather than a
    sequence of phase jumps.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    # shape white noise in the frequency domain, once
    X = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1.0 / fs)
    g = 10.0 ** (wash_shape(f) / 20.0)
    g = g * (f >= hp_hz)                    # the wash is a high-band object
    x = np.fft.irfft(X * g, n)

    m = modulation_depths(range_m, n_harm) if depths is None \
        else np.asarray(depths, float)[:n_harm]
    ph = rng.uniform(0, 2 * np.pi, len(m)) if phases is None \
        else np.asarray(phases, float)
    t = np.arange(n) / fs
    if np.isscalar(shaft_hz):
        theta = 2 * np.pi * float(shaft_hz) * t
    else:
        theta = 2 * np.pi * np.cumsum(np.asarray(shaft_hz, float)) / fs
    env = np.ones(n)
    for k in range(1, len(m) + 1):
        env += m[k - 1] * np.cos(k * theta + ph[k - 1])
    y = x * env
    r = np.sqrt(np.mean(y * y))
    return y / r if r > 0 else y


def add_wash(sig, shaft_hz, range_m=None, level_db=WASH_LEVEL_DB, seed=0,
             fs=FS, n_harm=4):
    """sig + a wash at level_db relative to sig's rms. Returns the sum,
    renormalised to sig's original rms so that adding a wash does not change
    the clip's level and therefore does not change its nominal SNR."""
    sig = np.asarray(sig, float)
    r0 = np.sqrt(np.mean(sig * sig)) + 1e-30
    w = wash(len(sig) / fs, shaft_hz, range_m=range_m, n_harm=n_harm,
             seed=seed, fs=fs)[:len(sig)]
    y = sig + w * r0 * 10.0 ** (level_db / 20.0)
    r1 = np.sqrt(np.mean(y * y)) + 1e-30
    return y * (r0 / r1)
