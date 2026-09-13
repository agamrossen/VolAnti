"""
synth.py - synthetic acoustic signal generator for acoustic-detector.

Produces drone-like blade-pass harmonic combs and the confusers that must NOT
trigger the detector. Everything is seeded, so any clip is reproducible from
(name, seed) alone. That property is what makes golden test vectors possible.

Physics implemented:
  BPF   = (RPM / 60) * blades
  Doppler shift from radial velocity during a flyby
  1/r amplitude spreading
  Frequency-dependent atmospheric absorption (high harmonics die first)

Run directly to write demo clips + plots into ../data and ../figures.
"""

from pathlib import Path

import math
import numpy as np

FS = 16000
C_SOUND = 343.0

_HERE = Path(__file__).resolve().parent
DATA_DIR = _HERE.parent / "data"
FIG_DIR = _HERE.parent / "figures"


# ----------------------------------------------------------------------
# physics helpers
# ----------------------------------------------------------------------

def bpf_from_rpm(rpm, blades):
    """Blade-pass frequency in Hz. This is the whole basis of the detector."""
    return rpm / 60.0 * blades


def rpm_from_bpf(bpf, blades):
    """Inverse - what throttle to aim for on the bench rig."""
    return bpf * 60.0 / blades


# ISO 9613-1 / -2 atmospheric attenuation, 20 C / 70 % RH / 1 atm, dB per km.
# These are the standard octave-band table values, not a fitted curve: the
# absorption exponent is not constant with frequency (it steepens from ~0.9 in
# the hundreds of Hz to ~1.6 in the kHz), so a single power law cannot fit it.
# The old 0.0005*(f/kHz)^1.7 fit was 6x too weak at 1-8 kHz and 20x too weak
# below 500 Hz, which let distant sources keep tonal harmonics out to 3 kHz
# that the real atmosphere would have removed.
_ABS_F_HZ = np.array([63.0, 125.0, 250.0, 500.0,
                      1000.0, 2000.0, 4000.0, 8000.0])
_ABS_DB_PER_KM = np.array([0.1, 0.4, 1.0, 1.9, 3.7, 9.7, 32.8, 117.0])


def absorption_db_per_m(freq_hz):
    """
    Atmospheric absorption in dB per metre at 20 C / 70 % RH.

    Log-log interpolation of the ISO 9613 octave-band table. This is the term
    that decides how many harmonics a distant source still has: at 400 m a
    3 kHz harmonic loses ~7.7 dB and at 900 m ~18 dB, so a light aircraft a
    few hundred metres off genuinely arrives with its comb truncated to the
    low orders. Getting this wrong does not perturb the near-field drone
    (at 40 m even 3 kHz loses under 1 dB) - it perturbs the confusers.
    """
    f = np.clip(np.asarray(freq_hz, dtype=float), 1.0, None)
    db_per_km = np.exp(np.interp(np.log(f), np.log(_ABS_F_HZ),
                                 np.log(_ABS_DB_PER_KM)))
    return db_per_km / 1000.0


# ----------------------------------------------------------------------
# noise
# ----------------------------------------------------------------------

def white_noise(n, rng):
    return rng.standard_normal(n)


def pink_noise(n, rng):
    """1/f noise - closer to real outdoor ambience than white."""
    spec = np.fft.rfft(rng.standard_normal(n))
    freqs = np.fft.rfftfreq(n, 1 / FS)
    scale = np.ones_like(freqs)
    scale[1:] = 1.0 / np.sqrt(freqs[1:])
    out = np.fft.irfft(spec * scale, n)
    return out / (np.std(out) + 1e-12)


def wind_noise(n, rng, corner_hz=200.0):
    """
    Wind is heavily low-frequency weighted and it is the dominant real-world
    masker. Fundamentals at 240-360 Hz sit right at the edge of this.
    """
    spec = np.fft.rfft(rng.standard_normal(n))
    freqs = np.fft.rfftfreq(n, 1 / FS)
    scale = 1.0 / (1.0 + (freqs / corner_hz) ** 2)
    out = np.fft.irfft(spec * scale, n)
    return out / (np.std(out) + 1e-12)


def wind_gusty(n, rng, corner_hz=120.0):
    """
    Gusting wind as a bed. On an exposed site the ambient is not a
    stationary noise floor, it steps.

    Why this is its own noise kind and not a cosmetic variation: the detector
    whitens against an asymmetric per-bin floor (tau_rise 6 s, tau_fall 0.8 s).
    A gust front arriving in ~1 s rises far faster than the floor can follow,
    so for a second or two afterwards every bin reads several dB "above floor"
    at once. That is a broadband lift, which the teeth-minus-gaps score is
    designed to be immune to - but only if the lift really is broadband. This
    bed is the test of that immunity, and it is the single most common thing
    the deployed device will actually hear.
    """
    base = wind_noise(n, rng, corner_hz=corner_hz)
    t = np.arange(n) / FS
    env = np.ones(n)
    n_gusts = max(1, int(round(t[-1] / float(rng.uniform(4.0, 12.0)))))
    for _ in range(n_gusts):
        t0 = float(rng.uniform(0.0, max(t[-1], 1e-6)))
        rise = float(rng.uniform(0.5, 2.5))
        hold = float(rng.uniform(1.0, 6.0))
        gain_db = float(rng.uniform(5.0, 16.0))
        w = np.clip((t - t0) / rise, 0.0, 1.0) * \
            np.clip((t0 + rise + hold + rise - t) / rise, 0.0, 1.0)
        env = env + (10.0 ** (gain_db / 20.0) - 1.0) * (0.5 - 0.5 * np.cos(np.pi * w)) ** 2
    out = base * env
    return out / (np.std(out) + 1e-12)


NOISE_KINDS = {"white": white_noise, "pink": pink_noise, "wind": wind_noise,
               "wind_gusty": wind_gusty}


def mix_at_snr(signal, noise, snr_db):
    """Scale noise so signal sits snr_db above it. Returns the mixture."""
    p_sig = np.mean(signal ** 2)
    p_noi = np.mean(noise ** 2)
    if p_sig <= 0 or p_noi <= 0:
        return signal + noise
    target = p_sig / (10.0 ** (snr_db / 10.0))
    return signal + noise * np.sqrt(target / p_noi)


# ----------------------------------------------------------------------
# core comb synthesis
# ----------------------------------------------------------------------

def comb(f0_track, dur, n_harm=14, rolloff_db=8.0, fs=FS,
         amp_track=None, distance_track=None, rng=None, jitter_hz=0.0):
    """
    Phase-continuous harmonic comb.

    f0_track  : scalar Hz, or per-sample array (lets f0 drift with Doppler)
    rolloff_db: level drop per harmonic. 6-10 is typical for a rotor.
    jitter_hz : slow random wobble on f0. Real motors are never perfectly
                steady, and a detector tuned to a mathematically pure comb
                will disappoint on real audio.
    """
    n = int(round(dur * fs))
    t = np.arange(n) / fs

    f0 = np.full(n, float(f0_track)) if np.isscalar(f0_track) else np.asarray(f0_track, float)

    if jitter_hz > 0:
        if rng is None:
            rng = np.random.default_rng(0)
        wobble = np.cumsum(rng.standard_normal(n)) / fs
        wobble = wobble - np.mean(wobble)
        f0 = f0 + jitter_hz * wobble / (np.std(wobble) + 1e-12)

    amp = np.ones(n) if amp_track is None else np.asarray(amp_track, float)

    out = np.zeros(n)
    for k in range(1, n_harm + 1):
        fk = f0 * k
        if np.max(fk) >= fs / 2:          # never synthesise above Nyquist
            continue
        level = 10.0 ** (-rolloff_db * (k - 1) / 20.0)
        if distance_track is not None:
            att = absorption_db_per_m(fk) * np.asarray(distance_track, float)
            level = level * 10.0 ** (-att / 20.0)
        phase = 2 * np.pi * np.cumsum(fk) / fs
        out += level * amp * np.sin(phase)

    # broadband rotor "whoosh" - a real prop is not only tones
    if rng is not None:
        out += 0.12 * np.std(out) * pink_noise(n, rng) * amp

    return out / (np.max(np.abs(out)) + 1e-12)


# ----------------------------------------------------------------------
# drone models
# ----------------------------------------------------------------------

def drone_static(dur=3.0, rpm=6000, blades=3, n_harm=14, rolloff_db=8.0,
                 seed=0, fs=FS):
    """Hovering / bench-rig case: steady f0, no Doppler, no range change."""
    rng = np.random.default_rng(seed)
    f0 = bpf_from_rpm(rpm, blades)
    return comb(f0, dur, n_harm, rolloff_db, fs, rng=rng, jitter_hz=2.0), f0


def drone_loiter(dur=45.0, rpm=8500.0, blades=3, n_harm=14, rolloff_db=8.0,
                 wander_frac=0.015, wander_tau_s=6.0, seed=0, fs=FS):
    """
    Loiter - the primary platform holding station for tens of seconds.

    Every positive class that existed before this one is 8-10 s long, which is
    comparable to, or shorter than, the v1 noise floor's 6 s rise time. That is
    not a modelling choice, it is a blind spot: the corpus could not express "a
    source that has been running for forty-five seconds", so no measurement
    made on it could ever have shown the floor absorbing a steady source. Real
    audio showed it; this class is what lets the corpus show it too.

    The rpm wander is a bounded random walk of at most `wander_frac` (default
    +-1.5%), correlated over `wander_tau_s`. Both numbers matter:

      magnitude  a real quadcopter holding altitude trims rpm continuously; a
                 mathematically constant f0 would let any tracker integrate
                 forever and would flatter every long-integration scheme,
                 including this one.
      rate       the walk must be slow. The tracker's continuity rule allows
                 2% per frame, so wander correlated over seconds never breaks
                 continuity - which is precisely the property Tier 2 relies
                 on. A drone that wandered fast enough to break continuity
                 would not be detectable by integration and should not be
                 modelled as though it were.

    Returns (signal, f0_nominal). f0_nominal is the blade-pass at `rpm`; the
    realised track wanders around it.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    f0 = bpf_from_rpm(rpm, blades)
    # control points every wander_tau_s, linearly interpolated: a random walk
    # generated at audio rate and then smoothed would cost a 96k-tap
    # convolution for nothing.
    n_ctrl = max(2, int(round(dur / wander_tau_s)) + 1)
    ctrl = np.cumsum(rng.standard_normal(n_ctrl))
    ctrl = ctrl - ctrl.mean()
    ctrl = ctrl / (np.max(np.abs(ctrl)) + 1e-12)
    track = f0 * (1.0 + wander_frac * np.interp(
        np.arange(n) / fs, np.linspace(0.0, dur, n_ctrl), ctrl))
    return comb(track, dur, n_harm, rolloff_db, fs, rng=rng, jitter_hz=0.0), f0


def drone_flyby(dur=6.0, rpm=6000, blades=3, speed_ms=25.0, closest_m=40.0,
                n_harm=14, rolloff_db=8.0, seed=0, fs=FS):
    """
    Straight-line pass. Returns (signal, f0_source, r_track).

    Doppler is the discriminator that separates a drone from a lawnmower:
    an approaching drone's whole comb slides, a fixed machine's does not.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    t = np.arange(n) / fs

    t_ca = dur / 2.0
    dx = speed_ms * (t - t_ca)
    r = np.sqrt(closest_m ** 2 + dx ** 2)
    v_radial = speed_ms ** 2 * (t - t_ca) / r      # +ve receding

    f0_src = bpf_from_rpm(rpm, blades)
    f0_obs = f0_src * C_SOUND / (C_SOUND + v_radial)

    amp = closest_m / r                            # 1/r spreading, normalised

    sig = comb(f0_obs, dur, n_harm, rolloff_db, fs,
               amp_track=amp, distance_track=r, rng=rng, jitter_hz=2.0)
    return sig, f0_src, r


def drone_throttle(dur=8.0, blades=3, rpm_hover=8500.0, rpm_peak=12000.0,
                   mode="punch", seed=0, fs=FS):
    """
    Throttle transient on the exact threat airframe.

    A hovering 7" 3-blade sits at 7.5-9.5k rpm (blade pass 375-475 Hz). An
    attack run is not a hover: the pilot punches out, and blade pass sweeps to
    ~600 Hz in a few hundred milliseconds, then settles. A descent drops it
    the other way.

    This matters for the tracker, not just the band: `cont_frac` is 2% per
    frame, and a punch from 425 -> 600 Hz in 0.25 s is ~4.4%/frame, which the
    continuity rule cannot follow. Transients are in the corpus so that cost is
    measured rather than discovered in the field.

    Returns (signal, f0_median_source).
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    t = np.arange(n) / fs
    if mode == "punch":
        t0 = float(rng.uniform(0.25, 0.55)) * dur
        rise = float(rng.uniform(0.15, 0.40))
        hold = float(rng.uniform(0.5, 1.6))
        fall = float(rng.uniform(0.4, 0.9))
        up = np.clip((t - t0) / rise, 0.0, 1.0)
        dn = np.clip((t0 + rise + hold + fall - t) / fall, 0.0, 1.0)
        prof = rpm_hover + (rpm_peak - rpm_hover) * np.minimum(up, dn)
    else:                                   # descent: revs bleed off
        t0 = float(rng.uniform(0.2, 0.45)) * dur
        prof = rpm_hover * (1.0 - float(rng.uniform(0.25, 0.40))
                            * np.clip((t - t0) / (dur * 0.55), 0.0, 1.0))
    f0 = prof / 60.0 * blades
    sig = comb(f0, dur, 14, 8.0, fs, rng=rng, jitter_hz=2.0)
    return sig, float(np.median(f0))


def drone_approach(dur=10.0, blades=3, rpm=8500.0, r_start=300.0, r_end=45.0,
                   seed=0, fs=FS):
    """
    Slow approach - the critical guard for any faster noise floor.

    A drone flying in from 300 m to 45 m over ~10 s is a slow, monotonic,
    broadband-ish amplitude rise. That is exactly the shape a gust-tracking
    floor is built to absorb. If a faster floor eats this, it has traded the
    thing the device exists for against the thing that annoys it, and it must
    be rejected however good its gust numbers look.

    Returns (signal, f0_source).
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    t = np.arange(n) / fs
    r = r_start + (r_end - r_start) * (t / dur)
    v_radial = (r_end - r_start) / dur              # negative = closing
    f0_src = bpf_from_rpm(rpm, blades)
    f0 = np.full(n, f0_src * C_SOUND / (C_SOUND + v_radial))
    amp = r_end / r                                 # 1/r, rises through clip
    sig = comb(f0, dur, 14, 8.0, fs, amp_track=amp, distance_track=r,
               rng=rng, jitter_hz=2.0)
    return sig, f0_src


# ----------------------------------------------------------------------
# confusers - the hard part of the problem
# ----------------------------------------------------------------------

def confuser_steady_machine(dur=6.0, f0=180.0, n_harm=16, rolloff_db=5.0,
                            seed=0, fs=FS):
    """Lawnmower / generator / strimmer. A real comb that must NOT fire."""
    rng = np.random.default_rng(seed)
    return comb(f0, dur, n_harm, rolloff_db, fs, rng=rng, jitter_hz=1.0)


def confuser_passing_vehicle(dur=6.0, f0=95.0, speed_ms=13.0, closest_m=15.0,
                             n_harm=12, rolloff_db=4.0, seed=0, fs=FS):
    """
    Motorbike or car passing. This one has BOTH a comb and Doppler drift,
    which makes it the nastiest confuser in the set. Separating it relies on
    fundamental band and harmonic spacing, not on drift alone.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    t = np.arange(n) / fs
    t_ca = dur / 2.0
    dx = speed_ms * (t - t_ca)
    r = np.sqrt(closest_m ** 2 + dx ** 2)
    v_radial = speed_ms ** 2 * (t - t_ca) / r
    f0_obs = f0 * C_SOUND / (C_SOUND + v_radial)
    amp = closest_m / r
    return comb(f0_obs, dur, n_harm, rolloff_db, fs,
                amp_track=amp, rng=rng, jitter_hz=3.0)


def confuser_tonal(dur=6.0, freq=310.0, seed=0, fs=FS):
    """Single tone, no harmonics - alarm, whine, resonance."""
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    t = np.arange(n) / fs
    sig = np.sin(2 * np.pi * freq * t)
    return sig + 0.05 * pink_noise(n, rng)


def confuser_broadband(dur=6.0, seed=0, fs=FS):
    """Rain, rustling, road roar. No comb structure at all."""
    rng = np.random.default_rng(seed)
    return pink_noise(int(round(dur * fs)), rng)


def confuser_insects(dur=6.0, seed=0, fs=FS):
    """
    Cicadas / crickets: strong narrowband energy high in the band, amplitude
    modulated. Genuinely relevant for a device deployed outdoors in a warm
    climate, and easy to forget until it ruins a field test.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    t = np.arange(n) / fs
    carrier = np.sin(2 * np.pi * 4200 * t) + 0.6 * np.sin(2 * np.pi * 5100 * t)
    env = 0.5 + 0.5 * np.sign(np.sin(2 * np.pi * 40 * t))
    return carrier * env + 0.1 * pink_noise(n, rng)


# ----------------------------------------------------------------------
# deployment-realistic confusers
#
# The device goes outdoors, near roads and around the outdoor plant of
# buildings. The negatives that matter there are not kitchen appliances - they
# are aircraft, road traffic and building services. Every one of these is a
# genuine harmonic comb thrown off by a rotating machine, i.e. exactly the
# structure the detector keys on. This is the real adversary set.
#
# The hard one is the helicopter. Its main-rotor blade-pass fundamental is
# 15-25 Hz, below the detector's 70 Hz search floor, so the detector can
# never see the true fundamental and score it down. What it can see are the
# in-band harmonics - and an odd multiple of a comb is itself a clean-looking
# comb: teeth land on true harmonics, gaps land on half-integer multiples
# which are empty. 13 x 18 Hz = 234 Hz sits squarely in the alert band.
# That masquerade route is the reason this model exists.
# ----------------------------------------------------------------------

def _unit(x):
    """Unit-RMS. Lets component weights below mean what they look like."""
    return x / (np.std(x) + 1e-12)


def _mic_highpass(x, f_c=60.0, order=2, fs=FS):
    """
    Butterworth-magnitude high-pass, applied to the low-fundamental sources.

    Rationale, because this is not cosmetic: the INMP441 is about -3 dB at
    60 Hz and the detector's search floor is 70 Hz. A helicopter or a heavy
    diesel radiates most of its acoustic power below both. Corpus SNR is
    defined on total clip power, so without modelling that roll-off the SNR
    knob would be set almost entirely by energy the sensor cannot hear, and
    every such clip would become a trivially easy negative for the wrong
    reason. Modelling the sensor keeps the confuser hard where it must be.
    """
    n = len(x)
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    h = 1.0 / np.sqrt(1.0 + (f_c / np.maximum(f, 1e-9)) ** (2 * order))
    return np.fft.irfft(X * h, n)


def harmonic_levels(n_harm, slope_db_per_oct=8.0, first_db=0.0,
                    f0=None, knee_hz=None, tail_db_per_oct=16.0):
    """
    Per-harmonic amplitudes on a power-law (dB per octave) slope.

    comb() uses dB per harmonic index, which collapses to nothing by k=30.
    A rotorcraft needs 100+ live harmonics, so it needs a slope that is
    linear in log-frequency instead.

    knee_hz is the tonal ceiling, and it matters more than the slope. Real
    rotating machinery is only discretely tonal at low orders: blade-passage
    and cylinder-firing tones dominate for roughly the first ten orders, and
    above a knee the radiated spectrum becomes broadband (vortex shedding,
    turbulent boundary layers, exhaust and intake flow noise). Without a knee
    a pure dB/octave law keeps synthesising resolvable tones to the 40th
    order, which manufactures a comb far deeper than any real engine makes -
    and a comb that deep can be sampled every third tooth and still look like
    a clean comb, which is exactly how a 90 Hz aero-engine acquired a
    convincing 270 Hz identity. Above knee_hz the level rolls off by an extra
    tail_db_per_oct; the broadband that physically replaces those tones is
    added by the caller as its own component.
    """
    k = np.arange(1, n_harm + 1, dtype=float)
    db = first_db - slope_db_per_oct * np.log2(k)
    if f0 is not None and knee_hz is not None:
        octaves_over = np.maximum(0.0, np.log2(np.maximum(f0 * k, 1e-9)
                                               / knee_hz))
        db = db - tail_db_per_oct * octaves_over
    return 10.0 ** (db / 20.0)


def harmonic_stack(f0_track, levels, fs=FS, amp_track=None,
                   distance_track=None, f_max=7800.0):
    """
    Phase-continuous harmonic stack with an explicit per-harmonic level array.

    Same idea as comb(), but the k=1 phase is integrated ONCE and scaled by k,
    so a 130-harmonic rotor costs one cumsum instead of 130. Absorption uses
    the nominal harmonic frequency (Doppler moves it by ~1%, absorption by
    far less than that).
    """
    f0 = np.asarray(f0_track, float)
    n = len(f0)
    f0_mean = float(np.mean(f0))
    phi1 = 2.0 * np.pi * np.cumsum(f0) / fs
    ceiling = min(fs / 2.0, f_max)

    out = np.zeros(n)
    for k, lv in enumerate(levels, start=1):
        if k * np.max(f0) >= ceiling:
            break
        g = lv
        if distance_track is not None:
            att = absorption_db_per_m(k * f0_mean) * np.asarray(distance_track, float)
            g = g * 10.0 ** (-att / 20.0)
        out += g * np.sin(k * phi1)

    if amp_track is not None:
        out = out * np.asarray(amp_track, float)
    return out


def _f0_track(f0, n, rng, jitter_hz=0.0, fs=FS):
    """Constant f0 plus a slow random-walk wobble. Real machines wander."""
    f = np.full(n, float(f0))
    if jitter_hz > 0:
        w = np.cumsum(rng.standard_normal(n)) / fs
        w = w - np.mean(w)
        f = f + jitter_hz * w / (np.std(w) + 1e-12)
    return f


def _flyby_geometry(n, dur, speed_ms, closest_m, fs=FS):
    """Straight-line pass -> (slant range, Doppler factor, 1/r amplitude)."""
    t = np.arange(n) / fs
    dx = speed_ms * (t - dur / 2.0)
    r = np.hypot(closest_m, dx)
    v_radial = speed_ms ** 2 * (t - dur / 2.0) / r
    return r, C_SOUND / (C_SOUND + v_radial), closest_m / r


def _blade_slap(n, rate_hz, rng, fs=FS, decay_s=0.012, band=(150.0, 2500.0)):
    """
    The "wop-wop": one near-identical broadband transient per blade passage.

    Because the transient repeats, its spectrum is a comb at the blade-pass
    rate spanning the whole mid band - the same structure a drone makes, only
    with far finer spacing. Deliberately the nastiest part of the helicopter.
    """
    klen = int(round(0.06 * fs))
    kern = pink_noise(klen, rng) * np.exp(-np.arange(klen) / fs / decay_s)
    K = np.fft.rfft(kern)
    kf = np.fft.rfftfreq(klen, 1.0 / fs)
    K = K * ((kf >= band[0]) & (kf <= band[1]))
    kern = np.fft.irfft(K, klen)
    kern = kern / (np.max(np.abs(kern)) + 1e-12)

    out = np.zeros(n + klen)
    period = fs / rate_hz
    pos = 0.0
    while pos < n:
        i = int(round(pos))
        out[i:i + klen] += (1.0 + 0.15 * rng.standard_normal()) * kern
        pos += period * (1.0 + 0.01 * rng.standard_normal())
    return out[:n]


def confuser_helicopter(dur=6.0, seed=0, fs=FS, flyby=False,
                        main_bpf=None, tail_bpf=None):
    """
    Rotorcraft - the priority confuser at this kind of site.

    Three stacked mechanisms, all tied to the same rotor:
      main rotor  15-25 Hz blade pass, 2-5 blades, ~100 harmonics on a shallow
                  (5.5-8 dB/octave) slope so the comb still has teeth at 1 kHz
      tail rotor  65-150 Hz blade pass, its own comb, in the search band
      blade slap  a repeating impulsive transient at the main blade-pass rate,
                  giving a broadband comb across the whole alert band

    The main-rotor fundamental is unreachable: 15-25 Hz is below f_search_lo
    (70 Hz), so unlike a car engine it cannot be put "in evidence" and scored
    down. Note also that a 2048-point FFT at 16 kHz has 7.8 Hz bins, so an
    18 Hz comb is only ~2.3 bins apart - it is unresolved and smears toward a
    continuum. Whether that helps or not is an empirical question, which
    is what evaluate.py is for.

    flyby=False is a hover/orbit (slow range cycle); flyby=True is a transit
    at 40-75 m/s passing 150-400 m off.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    t = np.arange(n) / fs

    blades = int(rng.integers(2, 6))
    main_bpf = float(rng.uniform(15.0, 25.0)) if main_bpf is None else float(main_bpf)
    tail_bpf = float(rng.uniform(65.0, 150.0)) if tail_bpf is None else float(tail_bpf)

    if flyby:
        r, dopp, amp = _flyby_geometry(n, dur, float(rng.uniform(40.0, 75.0)),
                                       float(rng.uniform(150.0, 400.0)), fs)
    else:
        r0 = float(rng.uniform(200.0, 600.0))
        period = float(rng.uniform(20.0, 40.0))
        r = r0 * (1.0 + 0.25 * np.sin(2 * np.pi * t / period
                                      + float(rng.uniform(0, 2 * np.pi))))
        dopp = C_SOUND / (C_SOUND + np.gradient(r, 1.0 / fs))
        amp = r0 / r

    # governed rotors are steady; the tail wanders a little more
    f_main = _f0_track(main_bpf, n, rng, jitter_hz=0.08, fs=fs) * dopp
    f_tail = _f0_track(tail_bpf, n, rng, jitter_hz=0.30, fs=fs) * dopp

    main = harmonic_stack(
        f_main, harmonic_levels(int(min(130, 2400.0 / main_bpf)),
                                float(rng.uniform(5.5, 8.0))),
        fs=fs, amp_track=amp, distance_track=r, f_max=2400.0)
    tail = harmonic_stack(
        f_tail, harmonic_levels(int(min(40, 6000.0 / tail_bpf)),
                                float(rng.uniform(7.0, 10.0))),
        fs=fs, amp_track=amp, distance_track=r, f_max=6000.0)
    slap = _blade_slap(n, main_bpf, rng, fs=fs) * amp
    bb = pink_noise(n, rng) * amp                     # turbine + rotor hiss

    out = (1.00 * _unit(_mic_highpass(main, fs=fs))
           + 0.55 * _unit(tail)
           + 0.50 * _unit(slap)
           + 0.20 * _unit(bb))
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_helicopter_flyby(dur=6.0, seed=0, fs=FS, **kw):
    """Transiting rotorcraft: the whole comb Doppler-slides, like a drone."""
    return confuser_helicopter(dur=dur, seed=seed, fs=fs, flyby=True, **kw)


def confuser_prop_aircraft(dur=6.0, seed=0, fs=FS):
    """
    Light fixed-wing (Cessna class) transit at 300-900 m slant range.

    Two nearly-coincident combs: propeller blade pass (2-3 blades at
    2200-2700 rpm) and engine firing order (4-cyl 4-stroke, = 2 x rev/s).
    For the 2-blade case those coincide exactly - which is not a modelling
    slip but what a direct-drive 4-cylinder with a 2-blade prop actually
    does - and for 3 blades they are close but not equal, so the teeth beat
    against each other and neither comb's gaps are fully dark.

    Blade-pass fundamental is 73-135 Hz (a 2-blade at 2400 rpm gives 80 Hz),
    comfortably below the 220 Hz alert floor, so this confuser is rejected by
    the fundamental-band gate - provided the detector's argmax actually locks
    onto the fundamental. Two things decide whether it does, and both are
    physical:
      tonal ceiling  a GA propeller radiates discrete tones for roughly the
                     first eight orders and is broadband above ~700 Hz. Left
                     unbounded, the comb stayed tonal to the 40th order, and
                     a 40-tooth comb still looks clean sampled every 3rd
                     tooth - so argmax rode the 3rd harmonic to ~270 Hz,
                     inside the alert band. That was an early regression in the corpus.
      absorption     at 300-900 m the atmosphere removes 8-18 dB at 3 kHz,
                     which is what truncates the comb to its low orders.
                     The old absorption fit was ~6x too weak to do that.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))

    blades = int(rng.integers(2, 4))
    rpm = float(rng.uniform(2200.0, 2700.0))
    prop_bpf = bpf_from_rpm(rpm, blades)
    fire_hz = 2.0 * rpm / 60.0

    r, dopp, amp = _flyby_geometry(n, dur, float(rng.uniform(45.0, 70.0)),
                                   float(rng.uniform(300.0, 900.0)), fs)

    # tonal ceiling: discrete prop/engine orders die into broadband here
    knee = float(rng.uniform(600.0, 900.0))

    prop = harmonic_stack(
        _f0_track(prop_bpf, n, rng, jitter_hz=0.5, fs=fs) * dopp,
        harmonic_levels(int(4000.0 / prop_bpf), float(rng.uniform(6.0, 9.0)),
                        f0=prop_bpf, knee_hz=knee),
        fs=fs, amp_track=amp, distance_track=r, f_max=4000.0)
    engine = harmonic_stack(
        _f0_track(fire_hz, n, rng, jitter_hz=0.8, fs=fs) * dopp,
        harmonic_levels(int(4000.0 / fire_hz), float(rng.uniform(7.0, 10.0)),
                        f0=fire_hz, knee_hz=knee),
        fs=fs, amp_track=amp, distance_track=r, f_max=4000.0)
    # slipstream / airframe / exhaust: the broadband that physically replaces
    # the tonal orders above the knee, so the band stays occupied
    slip = pink_noise(n, rng) * amp

    out = 1.00 * _unit(prop) + 0.70 * _unit(engine) + 0.45 * _unit(slip)
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_diesel_truck(dur=6.0, seed=0, fs=FS):
    """
    Heavy diesel passing at 15-25 m/s, 20-80 m off.

    Firing frequency 55-95 Hz (6-cyl 4-stroke, 1100-1900 rpm), but the real
    teeth are at HALF that spacing: cylinder-to-cylinder imbalance puts a
    half-order family between every firing harmonic. That is the specific
    thing that ruins an octave-blind matcher, because the half-order teeth
    sit exactly where a firing-frequency candidate expects dark gaps.
    Tyre roar and exhaust broadband on top; turbo whine at 2-4 kHz.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    t = np.arange(n) / fs

    rev_hz = float(rng.uniform(1100.0, 1900.0)) / 60.0
    fire_hz = 3.0 * rev_hz                            # 6-cyl 4-stroke
    half_hz = fire_hz / 2.0

    r, dopp, amp = _flyby_geometry(n, dur, float(rng.uniform(15.0, 25.0)),
                                   float(rng.uniform(20.0, 80.0)), fs)

    # one stack at half-order spacing: even k = true firing harmonics (strong),
    # odd k = imbalance components (weaker but present)
    n_h = int(3000.0 / half_hz)
    lv = harmonic_levels(n_h, float(rng.uniform(6.0, 9.0)))
    lv[::2] *= float(rng.uniform(0.25, 0.5))          # odd k -> half-orders
    engine = harmonic_stack(
        _f0_track(half_hz, n, rng, jitter_hz=0.6, fs=fs) * dopp, lv,
        fs=fs, amp_track=amp, f_max=3000.0)

    turbo_f = float(rng.uniform(2000.0, 4000.0)) * dopp
    turbo = np.sin(2 * np.pi * np.cumsum(turbo_f) / fs) * amp
    tyres = wind_noise(n, rng, corner_hz=900.0) * amp

    out = (1.00 * _unit(_mic_highpass(engine, fs=fs))
           + 0.85 * _unit(tyres) + 0.10 * _unit(turbo))
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_motorbike_accel(dur=6.0, seed=0, fs=FS):
    """
    Twin-cylinder bike accelerating away, with one gear change.

    Firing frequency sweeps ~40 -> 170 Hz, so the whole comb slides. That
    kills any "combs that drift must be drones" shortcut, and the sweep rate
    at the fundamental (~20 Hz/s) is well inside the tracker's 2%-per-frame
    continuity tolerance, so the tracker cannot reject it either. Only the
    fundamental-band gate can.

    Exhaust and intake orders are tonal only to ~1 kHz; above that a bike is
    broadband (exhaust flow, mechanical, tyre). Without that ceiling the comb
    stayed tonal past the 30th order, deep enough that every 3rd tooth still
    read as a clean comb and argmax rode the 3rd harmonic of a ~100 Hz firing
    frequency up to ~300 Hz, inside the alert band. See prop_aircraft.

    Known limit: a 4-cylinder sports bike at redline has a firing frequency
    above 400 Hz - inside the alert band - and the f0 gate alone would not
    reject that. Not modelled here; recorded as a known gap in the docs.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    t = np.arange(n) / fs

    f_lo = float(rng.uniform(35.0, 55.0))
    f_hi = float(rng.uniform(140.0, 175.0))
    shift_at = float(rng.uniform(0.45, 0.65))         # gear change, fraction
    ramp = t / dur
    f0 = f_lo + (f_hi - f_lo) * ramp
    f0 = np.where(ramp > shift_at, f0 * 0.72, f0)     # revs drop on the change

    speed = float(rng.uniform(12.0, 28.0))
    r = np.hypot(float(rng.uniform(15.0, 50.0)), speed * t)   # receding
    dopp = C_SOUND / (C_SOUND + np.gradient(r, 1.0 / fs))
    amp = r[0] / r

    wobble = _f0_track(0.0, n, rng, jitter_hz=1.0, fs=fs)   # zero-mean wander
    knee = float(rng.uniform(900.0, 1400.0))          # tonal ceiling
    # The ceiling is a property of radiated frequency, not of harmonic index,
    # so it must be referred to the fundamental actually being radiated. This
    # source sweeps 40 -> 170 Hz, so keying the knee to f_lo would let the
    # tonal orders scale up with the revs - the opposite of the truth, which
    # is that a revving engine keeps its ceiling near 1 kHz and therefore has
    # fewer tonal orders the harder it works.
    f0_obs_mean = float(np.mean(f0 * dopp))
    eng = harmonic_stack(
        f0 * dopp + wobble,
        harmonic_levels(int(3000.0 / f_lo), float(rng.uniform(4.0, 7.0)),
                        f0=f0_obs_mean, knee_hz=knee),
        fs=fs, amp_track=amp, distance_track=r, f_max=3000.0)
    # broadband exhaust/mechanical/tyre: replaces the tonal orders above knee
    out = 1.00 * _unit(eng) + 0.55 * _unit(pink_noise(n, rng) * amp)
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_distant_traffic(dur=6.0, seed=0, fs=FS):
    """
    A road a few hundred metres off: 3-5 vehicles overlapping at once, each
    with its own fundamental (45-120 Hz) and its own slow Doppler, over a bed
    of road roar. Several independent combs at once is a distinct stress from
    one clean comb: they interleave, and any candidate f0 finds some tooth
    lit by something. This is the everyday background near a road.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    t = np.arange(n) / fs

    out = np.zeros(n)
    for _ in range(int(rng.integers(3, 6))):
        f0 = float(rng.uniform(45.0, 120.0))
        r = np.hypot(float(rng.uniform(150.0, 500.0)),
                     float(rng.uniform(15.0, 30.0)) * (t - dur * rng.uniform(0.2, 0.8)))
        dopp = C_SOUND / (C_SOUND + np.gradient(r, 1.0 / fs))
        amp = float(rng.uniform(0.4, 1.0)) * np.min(r) / r
        out += _unit(harmonic_stack(
            _f0_track(f0, n, rng, jitter_hz=0.8, fs=fs) * dopp,
            harmonic_levels(int(2500.0 / f0), float(rng.uniform(6.0, 10.0))),
            fs=fs, amp_track=amp, distance_track=r, f_max=2500.0))

    roar = wind_noise(n, rng, corner_hz=600.0)
    out = _unit(_mic_highpass(out, fs=fs)) + 1.20 * _unit(roar)
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_hvac_outdoor_unit(dur=6.0, seed=0, fs=FS):
    """
    Air-conditioner / heat-pump condenser on the outside wall of the house -
    the "outdoor bits" confuser, and structurally the closest thing to a
    drone in the whole set, because it IS a multi-blade fan.

    Two stacked combs: an axial fan blade pass (4-6 blades at 800-1150 rpm ->
    53-115 Hz) and a mains-locked compressor at 2x line frequency (100 Hz)
    with strong harmonics. It runs dead steady for hours, so the temporal
    tracker offers no protection whatsoever: if this ever scores above
    threshold it scores above threshold all night. Only the fundamental-band
    gate and the gap penalty stand in the way.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))

    blades = int(rng.integers(4, 7))
    fan_bpf = bpf_from_rpm(float(rng.uniform(800.0, 1150.0)), blades)
    comp_hz = 2.0 * float(rng.uniform(49.8, 50.2))    # 2 x mains

    fan = harmonic_stack(
        _f0_track(fan_bpf, n, rng, jitter_hz=0.4, fs=fs),
        harmonic_levels(int(3500.0 / fan_bpf), float(rng.uniform(5.0, 8.0))),
        fs=fs, f_max=3500.0)
    comp = harmonic_stack(
        _f0_track(comp_hz, n, rng, jitter_hz=0.05, fs=fs),
        harmonic_levels(int(2500.0 / comp_hz), float(rng.uniform(6.0, 9.0))),
        fs=fs, f_max=2500.0)
    airflow = pink_noise(n, rng)

    out = 1.00 * _unit(fan) + 0.80 * _unit(comp) + 0.45 * _unit(airflow)
    return out / (np.max(np.abs(out)) + 1e-12)


# ----------------------------------------------------------------------
# Site confuser set - a rural, agricultural site with military activity nearby.
# Everything below is a model, not a recording; the prevalence weights that
# say how much each one matters live in site_profile.py.
# ----------------------------------------------------------------------

def confuser_wind_sustained(dur=6.0, seed=0, fs=FS):
    """
    Steady wind on an exposed site. A masker, not a comb source - it has no periodic
    structure at all, so the only way it can produce a false event is by
    lifting the whole spectrum and letting noise win argmax somewhere.

    Included as a confuser as well as a bed because the deployed device will
    spend more hours listening to this than to anything else on the list, and
    "the score never goes anywhere in plain wind" is a claim worth measuring
    rather than assuming.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    t = np.arange(n) / fs
    corner = float(rng.uniform(60.0, 180.0))
    base = wind_noise(n, rng, corner_hz=corner)
    # slow breathing of the mean wind speed, a few dB, tens of seconds
    drift = 1.0 + 0.25 * np.sin(2 * np.pi * t / float(rng.uniform(15.0, 45.0))
                                + float(rng.uniform(0, 2 * np.pi)))
    # mic-body buffeting: very low frequency, mostly removed by the INMP441
    buffet = wind_noise(n, rng, corner_hz=25.0) * float(rng.uniform(0.3, 0.9))
    out = _mic_highpass(base * drift + buffet, fs=fs)
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_wind_gusting(dur=6.0, seed=0, fs=FS):
    """
    Gust fronts. The specific stress is on the noise floor estimator, not on
    the comb score: the floor rises with tau 6 s and a gust arrives in ~1 s,
    so immediately after a gust every bin reads high at once and immediately
    after it passes the floor is left stranded above the true level for
    several seconds (tau_fall 0.8 s helps, but only downward).

    A broadband lift should cancel in teeth-minus-gaps. If it does not, this
    family is where it will show, and it is the most common thing at the site.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    out = wind_gusty(n, rng, corner_hz=float(rng.uniform(60.0, 160.0)))
    out = _mic_highpass(out, fs=fs)
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_apc_wheeled(dur=6.0, seed=0, fs=FS):
    """
    Wheeled armoured vehicle (8x8 class) on a road, 50-300 m off.

    Large multi-cylinder diesel at low revs: 8-cyl 4-stroke at 1200-2000 rpm
    gives a firing frequency of 80-133 Hz, comfortably below the 220 Hz alert
    floor - which means, exactly as for prop_aircraft and motorbike_accel,
    that it is rejected only if argmax locks onto the fundamental and not onto
    its 2nd or 3rd harmonic. Transmission gear mesh adds a genuine mid-band
    tone that is not harmonically related to the firing order, which is the
    part that can push a candidate's teeth around.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    rev_hz = float(rng.uniform(1200.0, 2000.0)) / 60.0
    fire_hz = 4.0 * rev_hz                              # 8-cyl 4-stroke
    r, dopp, amp = _flyby_geometry(n, dur, float(rng.uniform(5.0, 15.0)),
                                   float(rng.uniform(50.0, 300.0)), fs)
    knee = float(rng.uniform(700.0, 1100.0))
    eng = harmonic_stack(
        _f0_track(fire_hz, n, rng, jitter_hz=0.5, fs=fs) * dopp,
        harmonic_levels(int(3500.0 / fire_hz), float(rng.uniform(5.0, 8.0)),
                        f0=fire_hz, knee_hz=knee),
        fs=fs, amp_track=amp, distance_track=r, f_max=3500.0)
    # gear mesh: tooth-count x shaft rate, unrelated to the firing order
    mesh_f = float(rng.uniform(700.0, 2200.0)) * dopp
    mesh = np.sin(2 * np.pi * np.cumsum(mesh_f) / fs) * amp
    tyres = wind_noise(n, rng, corner_hz=800.0) * amp
    out = (1.00 * _unit(_mic_highpass(eng, fs=fs)) + 0.70 * _unit(tyres)
           + 0.18 * _unit(mesh))
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_tracked_vehicle(dur=6.0, seed=0, fs=FS):
    """
    Tracked AFV. The dangerous part is NOT the engine, it is the track.

    Every track pad striking the road is a near-identical broadband transient,
    repeating at (speed / pad pitch) = 20-80 Hz for a 0.15 m pitch at 3-12
    m/s. A repeating impulse is a comb spanning the entire band - structurally
    the same threat as helicopter blade slap, and for the same reason: the
    teeth are everywhere, so a candidate at almost any multiple of the slap
    rate finds its teeth lit. Road-wheel and sprocket rates add two more
    incommensurate periodicities on top.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    speed = float(rng.uniform(3.0, 12.0))
    pitch = float(rng.uniform(0.13, 0.18))
    slap_hz = speed / pitch
    r, dopp, amp = _flyby_geometry(n, dur, speed,
                                   float(rng.uniform(60.0, 400.0)), fs)
    rev_hz = float(rng.uniform(1400.0, 2200.0)) / 60.0
    fire_hz = 6.0 * rev_hz                              # 12-cyl 4-stroke
    eng = harmonic_stack(
        _f0_track(fire_hz, n, rng, jitter_hz=0.6, fs=fs) * dopp,
        harmonic_levels(int(3000.0 / fire_hz), float(rng.uniform(5.0, 8.0)),
                        f0=fire_hz, knee_hz=float(rng.uniform(700.0, 1100.0))),
        fs=fs, amp_track=amp, distance_track=r, f_max=3000.0)
    slap = _blade_slap(n, slap_hz, rng, fs=fs, decay_s=0.008,
                       band=(120.0, 4000.0)) * amp
    sprocket = _blade_slap(n, slap_hz / float(rng.uniform(4.0, 7.0)), rng,
                           fs=fs, decay_s=0.020, band=(80.0, 1200.0)) * amp
    rumble = wind_noise(n, rng, corner_hz=400.0) * amp
    out = (0.85 * _unit(slap) + 0.45 * _unit(sprocket)
           + 0.80 * _unit(_mic_highpass(eng, fs=fs)) + 0.55 * _unit(rumble))
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_tractor(dur=6.0, seed=0, fs=FS):
    """
    Agricultural tractor working a field or orchard, 30-250 m off, slow.

    3- or 4-cylinder 4-stroke at 1000-2200 rpm: firing 25-73 Hz, well below
    the alert band. A tractor under load is much more impulsive than a car -
    the exhaust pulses are separate events - so its comb is deep and reaches
    high, which is what makes the 3rd-5th harmonic masquerade available. The
    PTO-driven implement adds an unrelated rate.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    cyl = int(rng.integers(3, 5))
    rev_hz = float(rng.uniform(1000.0, 2200.0)) / 60.0
    fire_hz = (cyl / 2.0) * rev_hz
    r, dopp, amp = _flyby_geometry(n, dur, float(rng.uniform(1.5, 6.0)),
                                   float(rng.uniform(30.0, 250.0)), fs)
    knee = float(rng.uniform(600.0, 1000.0))
    eng = harmonic_stack(
        _f0_track(fire_hz, n, rng, jitter_hz=0.7, fs=fs) * dopp,
        harmonic_levels(int(3000.0 / fire_hz), float(rng.uniform(4.0, 7.0)),
                        f0=fire_hz, knee_hz=knee),
        fs=fs, amp_track=amp, distance_track=r, f_max=3000.0)
    # PTO shaft at 540 or 1000 rpm -> 9 or 16.7 Hz, driving a rattling implement
    pto_hz = float(rng.choice([540.0, 1000.0])) / 60.0
    implement = _blade_slap(n, pto_hz * float(rng.integers(2, 7)), rng, fs=fs,
                            decay_s=0.03, band=(200.0, 3000.0)) * amp
    out = (1.00 * _unit(_mic_highpass(eng, fs=fs)) + 0.35 * _unit(implement)
           + 0.30 * _unit(pink_noise(n, rng) * amp))
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_irrigation_pump(dur=6.0, seed=0, fs=FS):
    """
    Electric irrigation pump. Runs unattended for hours on a schedule.

    This one puts a fundamental inside the alert band, and it is not a
    modelling accident: a 2-pole induction motor at ~2900 rpm driving a 6-vane
    impeller has a vane-pass frequency of 290 Hz, sitting squarely in the
    240-1600 Hz drone band. There is no band gate defence against that. The
    only things standing in the way are the gap penalty (a pump's comb is
    shallower than a rotor's) and the fact that it does not move.

    Because it is steady for hours, the temporal tracker gives no protection:
    if it clears threshold once it clears it all night. Same structural
    problem as hvac_outdoor_unit, but with the fundamental in-band.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    poles = int(rng.choice([2, 4]))
    rpm = (3000.0 / (poles / 2)) * float(rng.uniform(0.955, 0.985))   # slip
    vanes = int(rng.integers(5, 9))
    vane_hz = vanes * rpm / 60.0
    hum_hz = 2.0 * float(rng.uniform(49.8, 50.2))          # magnetostriction
    vane = harmonic_stack(
        _f0_track(vane_hz, n, rng, jitter_hz=0.15, fs=fs),
        harmonic_levels(max(2, int(3500.0 / vane_hz)),
                        float(rng.uniform(7.0, 11.0)),
                        f0=vane_hz, knee_hz=float(rng.uniform(1500.0, 2500.0))),
        fs=fs, f_max=3500.0)
    hum = harmonic_stack(
        _f0_track(hum_hz, n, rng, jitter_hz=0.02, fs=fs),
        harmonic_levels(int(2000.0 / hum_hz), float(rng.uniform(8.0, 12.0))),
        fs=fs, f_max=2000.0)
    flow = wind_noise(n, rng, corner_hz=1200.0)            # cavitation / flow
    out = 1.00 * _unit(vane) + 0.60 * _unit(hum) + 0.50 * _unit(flow)
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_orchard_machinery(dur=6.0, seed=0, fs=FS):
    """
    Air-blast orchard sprayer / orchard fan - and this is the closest acoustic
    relative of a drone in the entire library.

    It is a multi-blade axial fan: 8-12 blades at 1500-2500 rpm gives a
    blade-pass frequency of 200-500 Hz, inside the alert band, with the same
    physical mechanism (blade passage) generating the same kind of harmonic
    comb. It is driven off a tractor PTO, so a tractor engine comb is present
    underneath at a completely unrelated rate.

    If anything in this library is going to produce a genuine false alert at a
    sensitive operating point, the best prior is that it is this.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    blades = int(rng.integers(8, 13))
    fan_rpm = float(rng.uniform(1500.0, 2500.0))
    bpf = bpf_from_rpm(fan_rpm, blades)
    r, dopp, amp = _flyby_geometry(n, dur, float(rng.uniform(1.0, 4.0)),
                                   float(rng.uniform(40.0, 250.0)), fs)
    fan = harmonic_stack(
        _f0_track(bpf, n, rng, jitter_hz=1.5, fs=fs) * dopp,
        harmonic_levels(max(2, int(6000.0 / bpf)), float(rng.uniform(6.0, 9.0)),
                        f0=bpf, knee_hz=float(rng.uniform(2000.0, 3500.0))),
        fs=fs, amp_track=amp, distance_track=r, f_max=6000.0)
    rev_hz = float(rng.uniform(1600.0, 2200.0)) / 60.0
    fire_hz = 2.0 * rev_hz
    eng = harmonic_stack(
        _f0_track(fire_hz, n, rng, jitter_hz=0.7, fs=fs) * dopp,
        harmonic_levels(int(2500.0 / fire_hz), float(rng.uniform(5.0, 8.0)),
                        f0=fire_hz, knee_hz=float(rng.uniform(600.0, 1000.0))),
        fs=fs, amp_track=amp, distance_track=r, f_max=2500.0)
    airflow = wind_noise(n, rng, corner_hz=1500.0) * amp   # the "blast"
    out = (1.00 * _unit(fan) + 0.55 * _unit(_mic_highpass(eng, fs=fs))
           + 0.85 * _unit(airflow))
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_diesel_generator(dur=6.0, seed=0, fs=FS):
    """
    Standby diesel genset running near-field (5-50 m) - unoccupied buildings,
    pumping stations, equipment huts. Common at this site and it runs all night.

    Two properties make it harder than diesel_truck:
      governed     a 50 Hz genset is locked to 1500 rpm (4-pole), so the
                   firing frequency is 50-75 Hz and does not wander. There is
                   no Doppler and no rev drift, so nothing about the f0
                   trajectory distinguishes it from a hovering rotor.
      near-field   at 5-50 m atmospheric absorption removes almost nothing, so
                   the comb survives to high order instead of being truncated
                   the way a 400 m prop_aircraft is. Deep comb, many teeth.
    Steady for hours, so the temporal tracker offers no protection.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    cyl = int(rng.choice([4, 6]))
    rev_hz = 1500.0 / 60.0                                  # governed, 4-pole
    fire_hz = (cyl / 2.0) * rev_hz * float(rng.uniform(0.998, 1.002))
    r_m = float(rng.uniform(5.0, 50.0))
    r = np.full(n, r_m)
    knee = float(rng.uniform(900.0, 1500.0))
    eng = harmonic_stack(
        _f0_track(fire_hz, n, rng, jitter_hz=0.05, fs=fs),
        harmonic_levels(int(4000.0 / fire_hz), float(rng.uniform(4.0, 7.0)),
                        f0=fire_hz, knee_hz=knee),
        fs=fs, distance_track=r, f_max=4000.0)
    alt_hz = 2.0 * float(rng.uniform(49.9, 50.1))           # alternator hum
    alt = harmonic_stack(
        _f0_track(alt_hz, n, rng, jitter_hz=0.02, fs=fs),
        harmonic_levels(int(2500.0 / alt_hz), float(rng.uniform(7.0, 10.0))),
        fs=fs, f_max=2500.0)
    rad_fan = harmonic_stack(
        _f0_track(bpf_from_rpm(1500.0, int(rng.integers(6, 10))), n, rng,
                  jitter_hz=0.3, fs=fs),
        harmonic_levels(12, float(rng.uniform(7.0, 10.0))), fs=fs, f_max=3000.0)
    out = (1.00 * _unit(_mic_highpass(eng, fs=fs)) + 0.45 * _unit(alt)
           + 0.35 * _unit(rad_fan) + 0.30 * _unit(pink_noise(n, rng)))
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_artillery_distant(dur=6.0, seed=0, fs=FS):
    """
    Distant artillery, gunfire or demolition.

    Not a comb at all - sparse, impulsive, very low-frequency-weighted after
    a few kilometres of propagation (the atmosphere has removed everything
    above ~1 kHz), with a long ground-reflected tail. It cannot look like a
    drone spectrally.

    The reason it is in the library is the noise floor, not the score: each
    detonation slams the per-bin floor upward, and the floor then decays with
    tau_fall 0.8 s. In the seconds after a blast the detector is effectively
    desensitised, and that is a miss risk on a life-safety device, not a false
    alarm risk. Worth measuring even though it will never fire.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    out = np.zeros(n)
    rate = float(rng.uniform(0.03, 0.5))                     # events per second
    n_ev = max(1, int(round(rate * dur)))
    for _ in range(n_ev):
        i0 = int(rng.integers(0, max(1, n - 1)))
        dec = float(rng.uniform(0.15, 0.8))
        klen = min(n - i0, int(round(6.0 * dec * fs)))
        if klen < 64:
            continue
        k = np.arange(klen) / fs
        # low-passed shock with a long tail; distance sets how low-passed
        shock = wind_noise(klen, rng, corner_hz=float(rng.uniform(40.0, 200.0)))
        env = np.exp(-k / dec) * (1.0 - np.exp(-k / 0.01))
        out[i0:i0 + klen] += shock * env * float(rng.uniform(0.5, 1.0))
        # ground reflection / hillside echo
        d = int(round(float(rng.uniform(0.08, 0.5)) * fs))
        if i0 + d + klen <= n:
            out[i0 + d:i0 + d + klen] += 0.45 * shock * env
    out = _mic_highpass(out, fs=fs) + 0.05 * pink_noise(n, rng)
    return out / (np.max(np.abs(out)) + 1e-12)


def _voice_stack(f0_arr, n_harm, rolloff_db_oct, rng, fs=FS,
                 jitter_frac=0.02, hnr_db=12.0, f_max=7000.0):
    """
    A biological voice source, as opposed to a machine comb.

    Two properties separate an animal call from a rotor, and both were missing
    from the first cut of the livestock/birds/dogs models - which made all
    three of them fire far harder than any machine in the library:

    jitter   Phonation has cycle-to-cycle fundamental perturbation of roughly
             1-3% (much more in a dog bark, which shows outright nonlinear
             behaviour). Harmonic k inherits k times that deviation, so with
             2% jitter on a 300 Hz call the 10th harmonic wanders +-60 Hz -
             eight FFT bins - while a governed engine's 10th harmonic sits
             still. Jitter is what stops a voice from reading as a deep comb,
             and it is the biological analogue of the tonal ceiling that fixed
             the earlier machinery regression.
    HNR      Voiced calls are 5-20 dB harmonics-to-noise; a substantial part
             of the energy is turbulent and aperiodic. A pure harmonic stack
             is 100% periodic, which no animal is.

    Modelling these is a correction of a named physical error, not a knob:
    the values are set from the phonation literature's typical ranges and
    then left alone whatever they do to the threshold.
    """
    n = len(f0_arr)
    w = rng.standard_normal(n)
    k = max(1, int(round(0.005 * fs)))            # ~5 ms correlation time
    w = np.convolve(w, np.ones(k) / k, mode="same")
    w = w / (np.std(w) + 1e-12)
    f = np.asarray(f0_arr, float) * (1.0 + jitter_frac * w)
    tonal = harmonic_stack(f, harmonic_levels(n_harm, rolloff_db_oct),
                           fs=fs, f_max=f_max)
    aper = wind_noise(n, rng, corner_hz=float(max(np.mean(f0_arr) * 4.0, 200.0)))
    return _unit(tonal) + (10.0 ** (-hnr_db / 20.0)) * _unit(aper)


def confuser_livestock(dur=6.0, seed=0, fs=FS):
    """
    Cattle / goats / sheep: vocalisations plus neck bells.

    The vocalisations are genuinely harmonic (f0 80-250 Hz with 10-20 orders)
    but they are short - one to three seconds - so they cannot sustain the
    six consecutive frames the tracker needs. The bells are metallic tonal
    clanks at 800-2500 Hz: single teeth, which the saturation cap exists to
    stop from carrying a candidate on their own.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    out = np.zeros(n)
    for _ in range(int(rng.integers(2, 9))):                 # calls
        f0 = float(rng.uniform(80.0, 250.0))
        ln = int(round(float(rng.uniform(0.6, 2.5)) * fs))
        i0 = int(rng.integers(0, max(1, n - ln)))
        t = np.arange(ln) / fs
        # calls glide in pitch and are strongly amplitude-shaped
        f_tr = f0 * (1.0 + float(rng.uniform(-0.25, 0.25)) * t / (t[-1] + 1e-9))
        call = _voice_stack(f_tr, int(rng.integers(6, 13)),
                            float(rng.uniform(7.0, 11.0)), rng, fs=fs,
                            jitter_frac=float(rng.uniform(0.015, 0.035)),
                            hnr_db=float(rng.uniform(8.0, 15.0)), f_max=5000.0)
        env = np.sin(np.pi * np.arange(ln) / ln) ** 1.5
        out[i0:i0 + ln] += _unit(call) * env * float(rng.uniform(0.4, 1.0))
    for _ in range(int(rng.integers(0, 40))):                # bell strikes
        f = float(rng.uniform(800.0, 2500.0))
        ln = int(round(float(rng.uniform(0.15, 0.5)) * fs))
        i0 = int(rng.integers(0, max(1, n - ln)))
        t = np.arange(ln) / fs
        ring = (np.sin(2 * np.pi * f * t) + 0.5 * np.sin(2 * np.pi * f * 2.76 * t))
        out[i0:i0 + ln] += ring * np.exp(-t / 0.12) * float(rng.uniform(0.2, 0.8))
    out = out + 0.25 * pink_noise(n, rng)
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_birds(dur=6.0, seed=0, fs=FS):
    """
    Dawn/dusk bird chorus. Mostly 2-8 kHz FM chirps, which is above the
    fundamental search band but squarely inside the harmonic range: a candidate
    f0 whose 8th-12th teeth land in the chorus can pick up lit teeth from
    something with no relationship to it at all. Doves are the low-frequency
    exception (300-600 Hz coo with real harmonics), and they sit in the alert
    band, so they are modelled separately.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    out = np.zeros(n)
    for _ in range(int(rng.integers(20, 120))):              # chirps
        ln = int(round(float(rng.uniform(0.04, 0.25)) * fs))
        i0 = int(rng.integers(0, max(1, n - ln)))
        t = np.arange(ln) / fs
        f_a = float(rng.uniform(2000.0, 7000.0))
        f_b = f_a * float(rng.uniform(0.6, 1.7))
        f_tr = np.linspace(f_a, f_b, ln)
        ch = np.sin(2 * np.pi * np.cumsum(f_tr) / fs)
        out[i0:i0 + ln] += ch * np.sin(np.pi * np.arange(ln) / ln) \
            * float(rng.uniform(0.3, 1.0))
    for _ in range(int(rng.integers(0, 8))):                 # dove coos
        # A coo is close to a pure tone: 2-3 weak harmonics, not a comb. The
        # first cut gave it 8 harmonics on a shallow slope, which turned a
        # dove into a small drone and made birds the loudest family in the
        # library. Corrected, not tuned.
        f0 = float(rng.uniform(300.0, 600.0))
        ln = int(round(float(rng.uniform(0.3, 0.9)) * fs))
        i0 = int(rng.integers(0, max(1, n - ln)))
        coo = _voice_stack(np.full(ln, f0), 3, float(rng.uniform(12.0, 18.0)),
                           rng, fs=fs,
                           jitter_frac=float(rng.uniform(0.008, 0.020)),
                           hnr_db=float(rng.uniform(10.0, 18.0)), f_max=5000.0)
        out[i0:i0 + ln] += coo * np.sin(np.pi * np.arange(ln) / ln) * 0.5
    out = out + 0.2 * pink_noise(n, rng)
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_dogs(dur=6.0, seed=0, fs=FS):
    """
    Dogs barking. Every farm has them and they bark at night, which is when
    the device matters most.

    This is a real threat to the tracker and not an obvious one. A bark is a
    harmonic stack with f0 typically 250-900 Hz - inside the alert band - and
    a dog barks repeatedly at 1-3 Hz with an f0 that barely changes between
    barks. Each bark is 0.1-0.3 s, i.e. 3-9 frames at a 32 ms hop, and the
    continuity rule is checked against the last accepted frame, not the last
    frame, so a gap between barks does not by itself reset f0. A run of barks
    is therefore the closest thing in the library to a legitimate chain.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    out = np.zeros(n)
    for _ in range(int(rng.integers(1, 4))):                 # 1-3 dogs
        f0_dog = float(rng.uniform(250.0, 900.0))
        rate = float(rng.uniform(1.0, 3.0))
        t_pos = float(rng.uniform(0.0, dur))
        while t_pos < dur:
            ln = int(round(float(rng.uniform(0.10, 0.30)) * fs))
            i0 = int(round(t_pos * fs))
            if i0 + ln >= n:
                break
            f0 = f0_dog * float(rng.uniform(0.93, 1.07))     # bark to bark
            t = np.arange(ln) / fs
            f_tr = f0 * (1.0 - 0.15 * t / (t[-1] + 1e-9))    # falls through bark
            # Barks are the least periodic voiced sound in this library: high
            # jitter and low harmonics-to-noise, with outright nonlinear
            # (chaotic / biphonic) episodes. Modelled as such - the first cut
            # used a clean 16-order stack and produced 31 false events in a
            # 1.3 h quick corpus, more than every machine combined.
            bark = _voice_stack(f_tr, int(rng.integers(5, 11)),
                                float(rng.uniform(6.0, 10.0)), rng, fs=fs,
                                jitter_frac=float(rng.uniform(0.030, 0.065)),
                                hnr_db=float(rng.uniform(4.0, 10.0)),
                                f_max=7000.0)
            env = (1.0 - np.exp(-t / 0.008)) * np.exp(-t / float(rng.uniform(0.05, 0.15)))
            out[i0:i0 + ln] += bark * env * float(rng.uniform(0.5, 1.0))
            t_pos += (1.0 / rate) * (1.0 + 0.25 * rng.standard_normal())
            if rng.random() < 0.12:                          # pauses
                t_pos += float(rng.uniform(1.0, 5.0))
    out = out + 0.2 * pink_noise(n, rng)
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_friendly_multirotor(dur=6.0, seed=0, fs=FS):
    """
    A friendly multirotor - friendly quadcopters operate in the same airspace.

    This is not really a confuser. It is the same physical source as the
    threat: rotating blades producing a blade-pass comb. It is built by
    calling the same generator the positives use, with a different parameter
    draw, because that is the plain statement of the problem - there is no
    modelling trick here, only different numbers.

    Fleet mix modelled (an assumption, and the measured firing rate is only as
    good as it):
      heavy lift        2 blades, 2400-3600 rpm  ->  BPF  80-120 Hz  (below band)
      medium quad       2 blades, 4000-6000 rpm  ->  BPF 133-200 Hz  (below band)
      small quad        2 blades, 6000-9000 rpm  ->  BPF 200-300 Hz  (in band)
      hex/octo          2 blades, 4800-7200 rpm  ->  BPF 160-240 Hz  (edge)
    The small-quad class is acoustically indistinguishable from the threat and
    No attempt is made to separate them. See the "friendly drones" limitation.
    """
    rng = np.random.default_rng(seed)
    cls = int(rng.integers(0, 4))
    rpm = [float(rng.uniform(2400.0, 3600.0)), float(rng.uniform(4000.0, 6000.0)),
           float(rng.uniform(6000.0, 9000.0)), float(rng.uniform(4800.0, 7200.0))][cls]
    if rng.random() < 0.5:
        sig, _, _ = drone_flyby(dur=dur, rpm=rpm, blades=2,
                                speed_ms=float(rng.uniform(8.0, 20.0)),
                                closest_m=float(rng.uniform(60.0, 250.0)),
                                seed=seed, fs=fs)
    else:
        sig, _ = drone_static(dur=dur, rpm=rpm, blades=2, seed=seed, fs=fs)
    return sig / (np.max(np.abs(sig)) + 1e-12)


def friendly_multirotor_f0(seed):
    """The blade-pass fundamental confuser_friendly_multirotor will produce,
    without synthesising it. Replays the RNG draws in the same order."""
    rng = np.random.default_rng(seed)
    cls = int(rng.integers(0, 4))
    rpm = [float(rng.uniform(2400.0, 3600.0)), float(rng.uniform(4000.0, 6000.0)),
           float(rng.uniform(6000.0, 9000.0)), float(rng.uniform(4800.0, 7200.0))][cls]
    return bpf_from_rpm(rpm, 2), cls


def confuser_sustained_vowel(dur=6.0, seed=0, fs=FS):
    """
    One vowel, held. Added for the voice veto.

    A synthetic proxy, like confuser_speech_like.

    Why it is separate from the talker, and why it is the harder case. Every
    structural thing that lets a detector reject speech is a thing speech does
    between syllables: the fundamental walks, the level gaps, the comb breaks.
    A held vowel does none of it. It is a steady harmonic comb at a drone-like
    fundamental with a dozen live orders, and the only features that separate
    it from a rotor are the ones that do not depend on time: the missing high
    band, and the level (a drone loud enough to look like this from a metre
    away would have a high band).

    So this family exists to make the veto prove it is not relying on jitter
    and gaps alone. If a veto only catches confuser_speech_like, it will not
    catch somebody holding a note next to the unit.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    out = np.zeros(n)
    i = 0
    while i < n:
        seg = min(int(round(float(rng.uniform(1.0, 3.0)) * fs)), n - i)
        if seg < int(0.3 * fs):
            break
        f0 = float(rng.uniform(95.0, 240.0))
        # +/- 1 %: a human holding a note is steady, not a signal generator
        drift = f0 * (1.0 + 0.01 * np.sin(
            2 * np.pi * float(rng.uniform(0.2, 0.8))
            * np.arange(seg) / fs + float(rng.uniform(0, 2 * np.pi))))
        v = _voice_stack(drift, int(rng.integers(10, 18)),
                         float(rng.uniform(7.0, 11.0)), rng, fs=fs,
                         jitter_frac=float(rng.uniform(0.002, 0.006)),
                         hnr_db=float(rng.uniform(14.0, 22.0)), f_max=7000.0)
        env = np.ones(seg)
        r = int(0.05 * fs)
        env[:r] = np.linspace(0.0, 1.0, r)
        env[-r:] = np.linspace(1.0, 0.0, r)
        out[i:i + seg] += _unit(v) * env
        i += seg + int(round(float(rng.uniform(0.2, 0.6)) * fs))
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_piano(dur=8.0, seed=0, fs=FS):
    """
    A piano in the room. Added because the device fired on one in testing and
    nothing in the corpus modelled it.

    Why a piano is harder than speech:

      It does not jitter. A struck string is mechanically periodic. The pitch
      wander that separates a talker from a rotor is simply absent, so the J
      term of the veto is worth nothing here.
      It sustains. A pedalled note rings for seconds, so the syllabic gaps
      that the G term looks for are absent too.
      Its fundamental sits where the threat does. A2 to A4 is 110 to 440 Hz,
      straddling the 375 to 475 Hz blade-pass band outright.
      It has twenty live orders at a low fundamental, which is exactly the
      comb the detector hunts.

    What separates it, and what the veto must therefore use: the attack is a
    broadband hammer transient, and the partials are inharmonic. A stiff
    string gives f_n = n*f0*sqrt(1 + B*n^2), so by the tenth order the teeth
    have walked measurably off the exact multiples a rotor produces. B is
    2e-4 to 8e-4 for real piano wire in this register.

    Chords and pedalling are modelled because a single note is the easy case:
    two notes a fifth apart put energy on each other's harmonics and make the
    comb look deeper than either.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    out = np.zeros(n)
    t_all = np.arange(n) / fs
    # a rough equal-tempered scale from A2 to A5
    semis = np.arange(0, 37)
    notes = 110.0 * (2.0 ** (semis / 12.0))
    i = 0
    while i < n:
        # a note, or a two- or three-note chord under the pedal
        n_note = int(rng.integers(1, 4))
        hold = float(rng.uniform(1.0, 4.0))
        seg = min(int(round(hold * fs)), n - i)
        if seg < int(0.3 * fs):
            break
        tt = t_all[:seg]
        for _ in range(n_note):
            f0 = float(rng.choice(notes))
            B = float(rng.uniform(2e-4, 8e-4))          # string stiffness
            decay = float(rng.uniform(0.8, 3.0))
            note = np.zeros(seg)
            for k in range(1, 21):
                fk = k * f0 * math.sqrt(1.0 + B * k * k)
                if fk >= 0.45 * fs:
                    break
                # higher partials die faster, as they do on a real string
                a = (1.0 / k ** 1.2) * np.exp(-tt * (decay * (1.0 + 0.35 * k)))
                note += a * np.sin(2 * np.pi * fk * tt
                                   + float(rng.uniform(0, 2 * np.pi)))
            # the hammer: a short broadband transient at the strike
            nh = int(0.008 * fs)
            hammer = np.zeros(seg)
            hammer[:nh] = rng.normal(0.0, 1.0, nh) * np.linspace(1.0, 0.0, nh)
            out[i:i + seg] += _unit(note) + 0.12 * hammer
        i += seg + int(round(float(rng.uniform(0.0, 0.4)) * fs))
    return out / (np.max(np.abs(out)) + 1e-12)


def confuser_speech_like(dur=6.0, seed=0, fs=FS):
    """
    Conversational speech near the device. Added for Tier 2.

    This is a synthetic proxy, not speech. A real speech capture is the harder
    gate and it stays the gate; this family exists so speech-shaped audio can
    be priced in hours, which no single 60 s capture can do.

    Why it has to exist at all: on real audio, human speech beat the real rotor
    in the detector's own metric - 26-28 frame chains against the rotor's 3,
    peak score 3.23. Voiced speech is a harmonic comb with f0 110-230 Hz and
    ten to twenty live orders, which is the same mechanism by which `livestock`
    binds. Tier 2's entire selectivity claim is that speech cannot hold one
    fundamental for three seconds. That claim is only worth anything if it is
    tested against something that tries.

    So the structure here is chosen to be as hard as speech really is:
      segments   200-400 ms voiced runs separated by 100-300 ms pauses. Long
                 enough to build a chain, short enough that the fundamental
                 changes before an M-of-N window closes.
      f0 walk    110-230 Hz, re-drawn per syllable with a within-syllable
                 glide. A speaker's f0 moves across a phrase; a rotor's does
                 not.
      syllabic   3-6 Hz amplitude modulation, the rate that makes speech
                 sound like speech.
    plus `_voice_stack`'s jitter and HNR, which are what stop a voice reading
    as a machine-deep comb.
    """
    rng = np.random.default_rng(seed)
    n = int(round(dur * fs))
    out = np.zeros(n)
    f0 = float(rng.uniform(110.0, 230.0))
    i = 0
    while i < n:
        seg = min(int(round(float(rng.uniform(0.20, 0.40)) * fs)), n - i)
        if seg < int(0.05 * fs):
            break
        # the fundamental walks between syllables and glides inside one
        f0 = float(np.clip(f0 * (1.0 + rng.normal(0.0, 0.06)), 110.0, 230.0))
        glide = f0 * (1.0 + float(rng.uniform(-0.12, 0.12))
                      * np.linspace(0.0, 1.0, seg))
        v = _voice_stack(glide, int(rng.integers(10, 20)),
                         float(rng.uniform(6.0, 10.0)), rng, fs=fs,
                         jitter_frac=float(rng.uniform(0.010, 0.025)),
                         hnr_db=float(rng.uniform(8.0, 16.0)), f_max=7000.0)
        t = np.arange(seg) / fs
        am = 0.55 + 0.45 * np.sin(2 * np.pi * float(rng.uniform(3.0, 6.0)) * t
                                  + float(rng.uniform(0.0, 2 * np.pi)))
        env = np.sin(np.pi * np.arange(seg) / seg) ** 0.6
        out[i:i + seg] += _unit(v) * am * env * float(rng.uniform(0.6, 1.0))
        i += seg + int(round(float(rng.uniform(0.10, 0.30)) * fs))
    out = out + 0.10 * wind_noise(n, rng, corner_hz=3000.0)   # fricatives/room
    return out / (np.max(np.abs(out)) + 1e-12)


CONFUSERS = {
    "steady_machine": confuser_steady_machine,
    "passing_vehicle": confuser_passing_vehicle,
    "tonal": confuser_tonal,
    "broadband": confuser_broadband,
    "insects": confuser_insects,
    # deployment-realistic set
    "helicopter": confuser_helicopter,
    "helicopter_flyby": confuser_helicopter_flyby,
    "prop_aircraft": confuser_prop_aircraft,
    "diesel_truck": confuser_diesel_truck,
    "motorbike_accel": confuser_motorbike_accel,
    "distant_traffic": confuser_distant_traffic,
    "hvac_outdoor_unit": confuser_hvac_outdoor_unit,
    # site confuser set (see site_profile.py for prevalence weights)
    "wind_sustained": confuser_wind_sustained,
    "wind_gusting": confuser_wind_gusting,
    "apc_wheeled": confuser_apc_wheeled,
    "tracked_vehicle": confuser_tracked_vehicle,
    "tractor": confuser_tractor,
    "irrigation_pump": confuser_irrigation_pump,
    "orchard_machinery": confuser_orchard_machinery,
    "diesel_generator": confuser_diesel_generator,
    "artillery_distant": confuser_artillery_distant,
    "livestock": confuser_livestock,
    "birds": confuser_birds,
    "dogs": confuser_dogs,
    # not a confuser in the scoring sense - see site_profile.FRIENDLY
    "friendly_multirotor": confuser_friendly_multirotor,
}

# ----------------------------------------------------------------------
# Tier 2 additions - deliberately not members of `CONFUSERS`.
#
# evaluate.build_corpus() does `for conf in synth.CONFUSERS` and draws seeds
# from one RNG in that order, so adding a key above would shift every seed
# drawn after it and silently rewrite the sealed corpus - the negatives, the
# noise-only clips, and with them every published number. New families get
# their own registry and are composed only by evaluate_t2.py.
# ----------------------------------------------------------------------
# ----------------------------------------------------------------------
# Tier 3 families - the wash tier's world
#
# Tier 3 does not listen where v1 and Tier 2 listen, so its positives and its
# confusers are different objects. Everything below is reachable only through
# the T3_* registries; nothing above this line changes, which is what keeps
# every v1 and Tier 2 number valid.
# ----------------------------------------------------------------------

# real speech, 1/3-octave, measured on the 6-minute home recording, dB
# referenced to the 63-89 Hz band. The old confuser_speech_like proxy sits
# 15 dB hot at 500-707 Hz - inside the priority band - which is most of why it
# alone drove v1's published false-alarm rate from 1.96 to 3.57 wFA/h.
_SPEECH_BANDS = np.array([75., 106., 150., 210., 300., 425., 600., 850.,
                          1200., 1700., 2400., 3400., 4800., 6800.])
_SPEECH_SHAPE_DB = np.array([0.0, 3.4, -2.9, -3.8, -6.6, -9.2, -15.5, -23.2,
                             -26.5, -27.2, -27.6, -27.3, -23.2, -31.2])


def _shaped_noise(n, bands, shape_db, rng, fs=FS):
    """Noise with an arbitrary 1/3-octave shape, log-interpolated. One rFFT,
    no filter design, exactly reproducible from the seed."""
    X = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1.0 / fs)
    g = 10.0 ** (np.interp(np.log10(np.maximum(f, 1.0)), np.log10(bands),
                           shape_db, left=shape_db[0],
                           right=shape_db[-1]) / 20.0)
    return _unit(np.fft.irfft(X * g, n))


def confuser_speech(dur=6.0, seed=0, fs=FS):
    """Speech fitted to a 6-minute home recording, replacing the proxy.

    Two things the proxy got wrong and this does not: the spectral shape above
    500 Hz (the proxy was 15 dB hot exactly where the detector looks), and the
    activity profile (real conversational speech was above p90-20 dB on 0.99 of
    half-second frames - close to continuous, not the bursty on/off the proxy
    assumed). A voiced comb is still laid on top, because speech is a harmonic
    comb and pretending otherwise would make this family free."""
    rng = np.random.default_rng(seed + 91000)
    n = int(round(dur * fs))
    base = _shaped_noise(n, _SPEECH_BANDS, _SPEECH_SHAPE_DB, rng, fs)
    # voiced segments: f0 wandering over a speaker's range, ~60% of the time
    t = np.arange(n) / fs
    f0 = 105.0 * np.exp(0.25 * np.sin(2 * np.pi * 0.7 * t + rng.uniform(0, 6))
                        + 0.10 * rng.standard_normal(n).cumsum() / np.sqrt(n))
    voiced = (np.sin(2 * np.pi * 0.55 * t + rng.uniform(0, 6)) > -0.3).astype(float)
    voiced = np.convolve(voiced, np.ones(400) / 400, "same")
    lv = harmonic_levels(18, slope_db_per_oct=12.0)
    comb_v = harmonic_stack(f0, lv, fs=fs) * voiced
    y = base + 0.9 * _unit(comb_v)
    return _unit(_mic_highpass(y, fs=fs))


def confuser_box_fan(dur=6.0, seed=0, fs=FS):
    """A domestic box fan, and the closest near-enemy of the wash tier.

    It is the same mechanism as the threat: a blade chopping broadband flow
    noise, so it amplitude-modulates its own high band at its blade-pass rate.
    Nothing about the Tier-3 statistic distinguishes a fan from a rotor. The
    only separation is rate - a 5-blade fan at 350-1100 rpm blade-passes at
    30-90 Hz, against 240-475 Hz for the threat - and that separation is the
    entire justification for the 100 Hz firing floor.

    Because it is the near-enemy, its modulation depth is set to the rotor's
    own measured value rather than something conveniently smaller. If the fan
    is easy to reject here it is because of where it sits, never because it was
    modelled weak."""
    import wash_model as _wm
    rng = np.random.default_rng(seed + 92000)
    n = int(round(dur * fs))
    blades = int(rng.integers(3, 6))
    rpm = float(rng.uniform(350.0, 1100.0))
    shaft = rpm / 60.0
    bpf = shaft * blades
    y = _wm.wash(dur, bpf, depths=_wm.WASH_M_4M, seed=seed + 92001, fs=fs,
                 hp_hz=400.0)
    # a fan also has a motor hum and a broad low-frequency rumble
    t = np.arange(n) / fs
    hum = 0.25 * np.sin(2 * np.pi * 100.0 * t + rng.uniform(0, 6))
    rumble = 0.8 * _shaped_noise(n, np.array([50., 200., 800., 3000.]),
                                 np.array([0., -6., -18., -30.]), rng, fs)
    return _unit(_mic_highpass(y[:n] + hum + rumble, fs=fs))


def confuser_rain(dur=6.0, seed=0, fs=FS):
    """Rain on the enclosure: broadband, high-band-heavy, and aperiodic.

    This is the control that asks whether the wash tier fires on anything with
    energy where it listens. Rain has more 4-8 kHz energy than the rotor does
    and no periodicity whatsoever, so a Tier-3 that fires on rain is detecting
    the band and not the modulation."""
    rng = np.random.default_rng(seed + 93000)
    n = int(round(dur * fs))
    bg = _shaped_noise(n, np.array([100., 500., 2000., 5000., 8000.]),
                       np.array([-12., -6., 0., 3., 2.]), rng, fs)
    # drop impacts: Poisson, ~2000/s, each a short high-frequency click
    n_drops = int(dur * 2000)
    idx = rng.integers(0, n, n_drops)
    imp = np.zeros(n)
    np.add.at(imp, idx, rng.exponential(1.0, n_drops))
    k = np.exp(-np.arange(96) / 12.0) * np.sin(2 * np.pi * 4200 *
                                               np.arange(96) / fs)
    y = bg + 1.4 * _unit(np.convolve(imp, k, "same"))
    return _unit(_mic_highpass(y, fs=fs))


def confuser_wind_at_capsule(dur=6.0, seed=0, fs=FS, wind_ms=None):
    """Wind blowing across the enclosure's own microphone cones.

    Not the same object as confuser_wind_sustained, which models wind noise in
    the far field. This is wind interacting with the device: a cylinder in a
    flow sheds vortices alternately from each side at the Strouhal frequency,
    f = St * U / D with St ~ 0.2. The cone mouths are 23 mm, so

        2 m/s -> 17 Hz    5 m/s -> 43 Hz    10 m/s -> 87 Hz

    and the shedding both radiates a tone at f and modulates the broadband
    self-noise at f, which is precisely what Tier-3 hunts. At 10 m/s the rate
    lands within 6 percent of the 82 Hz shaft rate the real rotor showed.

    This family is predicted, not observed: it comes from a Strouhal number and
    a caliper, and nothing has yet blown across this enclosure with a
    microphone recording. It is in the corpus because a false-alarm mechanism
    that is specific to the detector's own housing is worth being wrong about
    early rather than surprised by in the field."""
    import wash_model as _wm
    rng = np.random.default_rng(seed + 94000)
    n = int(round(dur * fs))
    U = float(rng.uniform(2.0, 12.0)) if wind_ms is None else float(wind_ms)
    D = 0.023
    f_s = 0.2 * U / D
    # gustiness wanders the shedding rate; a cylinder in real flow is not a
    # crystal oscillator, and the +-10% wander is what a tracker has to survive
    t = np.arange(n) / fs
    f_t = f_s * (1.0 + 0.10 * np.sin(2 * np.pi * 0.3 * t + rng.uniform(0, 6)))
    y = _wm.wash(dur, f_t, depths=0.6 * _wm.WASH_M_4M, seed=seed + 94001,
                 fs=fs, hp_hz=300.0)[:n]
    tone = 0.5 * np.sin(2 * np.pi * np.cumsum(f_t) / fs + rng.uniform(0, 6))
    bg = 2.5 * wind_noise(n, rng, corner_hz=200.0)
    return _unit(_mic_highpass(y + tone + bg, fs=fs))


def drone_quad_wash(dur=6.0, rpm=8500.0, blades=3, spread_pct=1.5, seed=0,
                    fs=FS, range_m=None, wash_db=0.0, n_harm=14,
                    rolloff_db=8.0):
    """Four motors, each at its own rpm, each with its own wash.

    Models inter-motor beating, which the corpus did not have before. Four
    motors within +-1.5 percent of a common rpm beat against each other at
    2-8 Hz, which smears the tonal comb's teeth and - the part that matters
    here - puts a slow amplitude wander on the wash.

    Whether that helps or hurts is not asserted: it is a corpus family so that
    the sweep can answer it. What can be said in advance is that Tier 3's
    tracker allows +-3 percent rate continuity, and a +-1.5 percent spread
    across four motors fits inside that by construction, so the failure mode to
    watch for is not the tracker breaking but the four rates blurring into a
    single wide peak that the prominence statistic scores down because its
    local median rises with it."""
    import wash_model as _wm
    rng = np.random.default_rng(seed + 95000)
    n = int(round(dur * fs))
    y = np.zeros(n)
    for i in range(4):
        r = rpm * (1.0 + spread_pct / 100.0 * rng.uniform(-1, 1))
        shaft = r / 60.0
        f0 = bpf_from_rpm(r, blades)
        lv = harmonic_levels(n_harm, slope_db_per_oct=rolloff_db)
        y += _unit(harmonic_stack(np.full(n, f0), lv, fs=fs)) / 4.0
        y += (10.0 ** (wash_db / 20.0) / 4.0) * _wm.wash(
            dur, shaft, range_m=range_m, seed=seed + 95001 + i, fs=fs)[:n]
    return _unit(_mic_highpass(y, fs=fs))


def _with_wash(fn):
    """Wrap an existing positive so it carries a wash at the shaft rate implied
    by its own rpm. The tonal comb is byte-identical to the un-washed family
    for the same seed, so a paired comparison isolates the wash and nothing
    else."""
    import wash_model as _wm

    def gen(dur=6.0, seed=0, fs=FS, rpm=8500.0, blades=3, range_m=None,
            wash_db=0.0, **kw):
        x = fn(dur=dur, seed=seed, fs=fs, rpm=rpm, blades=blades, **kw)
        if isinstance(x, tuple):        # drone_static returns (samples, f0)
            x = x[0]
        return _unit(_wm.add_wash(x, rpm / 60.0, range_m=range_m,
                                  level_db=wash_db, seed=seed + 96000, fs=fs))
    gen.__name__ = fn.__name__ + "_wash"
    gen.__doc__ = (fn.__doc__ or "") + "\n\nPlus the measured broadband wash."
    return gen


drone_static_wash = _with_wash(drone_static)
drone_loiter_wash = _with_wash(drone_loiter)


T2_CONFUSERS = {
    "speech_like": confuser_speech_like,
}

# Kept under both names on purpose. "speech_like" is the identifier every
# earlier Tier 2 and Tier 3 measurement was indexed by, and renaming it would
# silently invalidate them; "speech_proxy_harsh" is what it should have been
# called, and is the name to use from here on so nobody mistakes it for speech.
T2_CONFUSERS["speech_proxy_harsh"] = confuser_speech_like

T3_CONFUSERS = {
    "speech": confuser_speech,
    "box_fan": confuser_box_fan,
    "rain": confuser_rain,
    "wind_at_capsule": confuser_wind_at_capsule,
}

T3_POSITIVES = {
    "drone_static_wash": drone_static_wash,
    "drone_loiter_wash": drone_loiter_wash,
    "drone_quad_wash": drone_quad_wash,
}


# ----------------------------------------------------------------------
# The veto families - not members of CONFUSERS.
#
# Same reason the Tier 2 and Tier 3 registries exist: evaluate.build_corpus()
# draws seeds from one rng while iterating CONFUSERS, so a new key there shifts
# every seed after it and silently rewrites the sealed corpus. These are
# reachable only through synth.generator().
#
# They are not priced into the v1 false-alarm budget either, and that is a
# statement about evidence rather than about risk. PREVALENCE weights are
# survey-free estimates, and nobody has counted how often a piano is audible
# at a deployment site. Charging the threshold for a made-up prevalence would
# move the operating point on a guess. They are measured, reported and vetoed
# instead.
# ----------------------------------------------------------------------
VETO_CONFUSERS = {
    "piano": confuser_piano,
    "vowel": confuser_sustained_vowel,
}


# ======================================================================
# The second-edition positive families. Not members of any existing registry,
# for the same reason confuser_piano and confuser_sustained_vowel are not:
# evaluate.build_corpus() draws seeds from one rng while iterating CONFUSERS,
# so a new key there would shift every seed after it and silently rewrite the
# sealed corpus. Everything below is reachable only through POSITIVES_V2 and
# generator().
#
# Why they exist
# --------------
# Three gates were fitted against a corpus that does not contain the physics
# they were meant to discriminate on, and each one shipped and then failed in
# the field:
#
#   the jitter gate  fitted to an f0 that is a straight line plus 2 Hz of
#                    wobble; real rigs measure 0.005 to 0.34 per frame.
#   the R gate       rejected on corpus evidence because the corpus's own
#                    drones read R = -29.6 dB. They read that because
#                    propeller broadband was never put in the synthesiser.
#   min_teeth        rejected against livestock, which saturates at 12/12
#                    exactly as a drone does. There was no piano to test on.
#
# The sealed families are a harmonic stack with one f0, no broadband, and a
# step envelope. A quadcopter is four motors at four rpms, is a broadband
# source in its own right, and arrives as a ramp. These families are that.
#
# What is measured and what is fitted - read before quoting any of it
# ------------------------------------------------------------------
#   measured   the broadband shape (PROP_BB_DB below), from three near-field
#              rig recordings.
#   measured   the argmax structure being reproduced: the 4 m clip's argmax
#              runs 158-190 Hz on the shaft family and 468-571 Hz on the
#              blade family, a ~20 % spread within each family.
#   fitted     PROP_BB_LEVEL_DB, the one free parameter, set so that a
#              near-field clip's R lands on the measured -5 +- 3 dB.
#   inferred   the tonal slope, the tonal knee, and the shaft-family depth.
#              Physics-shaped, not measured.
#
# There is exactly one recording chain behind the shape (an AAC phone with
# AGC), and phone AGC is why only ratio statistics across frequency may be
# taken from it. R is such a ratio; an absolute level is not.
# ======================================================================

# Propeller broadband, per-BIN power in dB re the 500 Hz third-octave band.
# Measured: the median of the FFT bins inside each third-octave band, averaged
# over every frame, of the three near-field rig recordings, then averaged
# across the three. The median across bins is what removes the blade-pass
# tones, in the same way Tier-4 whitens across frequency by a local median -
# a mean would have carried the comb into the "broadband" shape and the model
# would have double-counted its own tones.
#
# The shape above 1 kHz is the finding: a propeller is nearly flat from 1 kHz
# to 6.3 kHz. That is why R is near 0 dB in the near field, and why an
# instrument, whose energy stops an octave or two above its fundamental,
# cannot look like one.
PROP_BB_BANDS_HZ = np.array([80., 100., 125., 160., 200., 250., 315., 400.,
                             500., 630., 800., 1000., 1250., 1600., 2000.,
                             2500., 3150., 4000., 5000., 6300., 8000.])
PROP_BB_DB = np.array([11.4, 11.1, 13.3, 10.4, 7.8, 4.6, 2.5, -0.4, 0.0,
                       -6.1, -7.2, -4.2, -8.9, -6.5, -8.2, -10.2, -10.9,
                       -8.5, -5.4, -6.2, -15.6])

# The sub-1 kHz end of that curve is not used, and this is the most important
# paragraph in the file.
#
# The de-toning instrument fails exactly where the drone lives. A third-octave
# band at 125 Hz holds four fft bins and a shaft-order tone sits in it, so its
# median is the tone; the 500 and 630 Hz bands hold the blade pass itself.
# Using the measured curve below 1 kHz would put the recording's own comb into
# the "broadband" model and then add a synthetic comb on top of it: the same
# double-count the wash model was written to avoid.
#
# Two pieces of evidence bracket the truth and neither is the phone curve:
#   upper bound  the phone's +13 dB rise at 125 Hz. Those clips were made
#                outdoors on a handheld phone; two of the five files from the
#                same session are recordings of wind. Handling and wind noise
#                live exactly there.
#   lower bound  the rotor captures made on the device's own microphones with
#                a quiet-room baseline to subtract. Measured excess over that
#                baseline: +0.3 dB at 250-500 Hz, +17.5 dB above 3.2 kHz. On
#                the sensor that matters, a rotor adds essentially nothing
#                below 1 kHz.
#
# The shipped model holds the curve flat below `PROP_BB_FLAT_BELOW_HZ`. That
# is a model choice inside those bounds, not a measurement, and it is the
# choice that makes the family's v1 score match the real near-field rig
# recordings while keeping R on target. P(d) of this family moves from 0.00
# to 0.5 across a calibration the evidence cannot yet pin down. What would pin
# it down is the rig recorded at 2, 3, 5 and 8 m through the device's own
# microphones, with a quiet-room baseline.
PROP_BB_FLAT_BELOW_HZ = 1000.0

# Broadband rms in dB relative to the unit-rms sum of the tonal stacks. Fitted
# jointly against two measurements of the same three near-field recordings:
#   R              -5 +- 3 dB (the target, and the measured range)
#   v1 score p95   at least the weakest real rig clip's 1.91, so that the
#                  corpus family is never harder to detect than the recording
#                  it was built from
# At -3.0 the generated hover reads R -5.3 and score p95 1.91. Raising it to
# +6 puts R at -5.2 as well but drops score p95 to 1.51 and P(d) to zero, and
# nothing in the evidence distinguishes the two: that is the finding, not a
# choice to be hidden in a constant.
PROP_BB_LEVEL_DB = -3.0


def prop_broadband(n, rng, fs=FS, range_m=None, level_db=0.0,
                   flat_below_hz=PROP_BB_FLAT_BELOW_HZ):
    """
    The propeller's own broadband: blade-vortex shedding, turbulent boundary
    layer over the blade, tip vortices, and inflow turbulence chopped at the
    blade rate. Unit rms before `level_db`.

    This is the term the sealed corpus does not have. comb() adds
    `0.12 * std` of pink noise, which is 18 dB down and falls at 3 dB/octave,
    so it contributes essentially nothing above 3 kHz - the sealed drones read
    R = -29.6 dB, indistinguishable from a piano.

    `flat_below_hz` holds the shape flat below that frequency. Pass 0 or None
    to use the raw measured phone curve instead, which is the upper bound of
    the low band and is what the block above explains should not be shipped.

    `range_m` applies ISO 9613 atmospheric absorption in the frequency domain,
    which is the only range term that changes R. Spherical spreading is
    frequency-flat and changes level, not ratio. That distinction matters:
    absorption alone cannot take R from -5 dB to -33 dB over 4 to 14 m.
    """
    X = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1.0 / fs)
    db = np.interp(np.log(np.clip(f, 1.0, None)),
                   np.log(PROP_BB_BANDS_HZ), PROP_BB_DB,
                   left=PROP_BB_DB[0], right=PROP_BB_DB[-1])
    if flat_below_hz:
        db = np.where(f < flat_below_hz,
                      float(np.interp(np.log(flat_below_hz),
                                      np.log(PROP_BB_BANDS_HZ), PROP_BB_DB)),
                      db)
    if range_m is not None:
        # `db` is per-bin power, and an ISO attenuation in dB is the same
        # number for power and for pressure, so it is subtracted once. The
        # amplitude response is then 10**(db/20), which squares to the power
        # asked for. Writing 2*att here would double the absorption.
        db = db - absorption_db_per_m(np.clip(f, 20.0, None)) * range_m
    y = np.fft.irfft(X * 10.0 ** (db / 20.0), n)
    return y / (np.std(y) + 1e-12) * 10.0 ** (level_db / 20.0)


def _shaft_track(n, rate_hz, rng, jitter_pct_100ms=1.0, drift_pct=2.5,
                 drift_tau_s=4.0, fs=FS):
    """
    One motor's shaft rate over time, in Hz.

    Two components, both absent from the sealed families, whose f0 is a
    straight line plus `jitter_hz = 2.0`:

      jitter   a random walk scaled so that its standard deviation over
               100 ms is `jitter_pct_100ms` percent of the nominal rate.
               The modelled range is 0.3 to 2 %. At the 32 ms hop that is
               0.17 to 1.1 % per frame. Measured real-rig medians of
               |df0|/f0 per frame are 0.0049, 0.0163 and 0.3418, and
               0.0054-0.0091 in a later session - so the model brackets the
               two slow ones instead of sitting two orders of magnitude
               under all three. The 0.34 clip is the argmax hopping between
               motors, which is the cluster and not this term.
      drift    slow, control-driven: a quadcopter trims rpm continuously to
               hold attitude and station. Correlated over `drift_tau_s`, so
               it never breaks a 2 %-per-frame continuity rule by itself.

    The walk is scaled by sqrt(k) with k the number of samples in 100 ms, so
    the stated percentage is the standard deviation of the 100 ms increment
    and not of the whole track. Getting that normalisation wrong is how a
    "1 % jitter" model ends up with 30 % excursions by the end of a clip.
    """
    k = max(1, int(round(0.1 * fs)))
    w = np.cumsum(rng.standard_normal(n)) / math.sqrt(k)
    w = w - np.mean(w)
    f = float(rate_hz) * (1.0 + jitter_pct_100ms / 100.0 * w)

    n_ctrl = max(2, int(round(n / fs / max(drift_tau_s, 1e-3))) + 1)
    c = np.cumsum(rng.standard_normal(n_ctrl))
    c = c - c.mean()
    c = c / (np.max(np.abs(c)) + 1e-12)
    f = f * (1.0 + drift_pct / 100.0
             * np.interp(np.arange(n) / fs,
                         np.linspace(0.0, n / fs, n_ctrl), c))
    return np.maximum(f, 10.0)


def _motor_levels(shaft_hz, blades, shaft_db, slope_db_per_oct,
                  knee_hz, tail_db_per_oct, n_orders):
    """
    Levels for one motor, indexed by shaft order k = 1, 2, 3, ...

    A motor is not a blade-pass comb sitting alone. It radiates at every
    shaft order; the blade orders (k a multiple of `blades`) dominate because
    that is where the loading pulse repeats. That is the physically right
    shape, and it is why `shaft_db` is a level and not a separate oscillator.

    `shaft_db` is the level of the shaft fundamental relative to the blade
    fundamental (typically -10 to -20 dB).

    What this does not reproduce, measured: the real 4 m recording's argmax
    sits below 200 Hz on 32-36 % of its loud frames, on the shaft family.
    These families reach only 1-3 %, and sweeping `shaft_db` from -4 to -20
    moves that fraction by 0.04 - the shaft comb loses the argmax to the
    blade comb at every depth in the modelled range and well outside it. So
    the cluster is reproduced (the argmax hops across the four motors' blade
    lines, a 13-37 % spread within the family) and the family split is not.
    Do not quote this generator as evidence about the sub-200 Hz half of the
    tracker problem.
    """
    k = np.arange(1, n_orders + 1, dtype=float)
    is_blade = (k % blades) == 0
    blade_order = np.maximum(k / blades, 1.0)
    db = np.where(is_blade,
                  -slope_db_per_oct * np.log2(blade_order),
                  shaft_db - slope_db_per_oct * np.log2(k))
    # The tonal ceiling. Above the knee a real rotor is broadband, not
    # discretely tonal; prop_broadband() supplies what replaces these tones.
    db = db - tail_db_per_oct * np.maximum(
        0.0, np.log2(np.maximum(shaft_hz * k, 1e-9) / knee_hz))
    lv = 10.0 ** (db / 20.0)
    live = np.nonzero(lv > 1e-4)[0]
    return lv[:int(live[-1]) + 1] if len(live) else lv[:1]


def quad_source(dur=8.0, seed=0, fs=FS, rpm=8500.0, blades=3,
                spread_pct=None, shaft_db=None, jitter_pct_100ms=None,
                drift_pct=2.5, slope_db_per_oct=6.0, knee_hz=2500.0,
                tail_db_per_oct=12.0, n_orders=48, range_m=4.0,
                amp_track=None, doppler=None, bb_level_db=PROP_BB_LEVEL_DB,
                bb_flat_below_hz=PROP_BB_FLAT_BELOW_HZ,
                with_broadband=True, n_motors=4):
    """
    Four motors at four rpms, each a shaft-order comb, plus the propeller's
    own broadband. Unit rms. This is the shared core of every v2 family.

    `spread_pct`      3 to 20 %, drawn per clip. Measured justification: the
                      4 m recording's argmax runs 158-190 Hz within the shaft
                      family and 468-571 Hz within the blade family, and two
                      bench motors produced that. Four will not be tighter.
    `range_m`         absorption only. Level belongs to `amp_track`.
    `doppler`         per-sample multiplier on every rate, or None.
    `with_broadband`  False reproduces the sealed families' physics (tones
                      only) with the new tracker structure, so that the two
                      changes can be separated in a paired comparison.
    `n_motors`        1 is the other half of that pair: one rpm, no cluster.
                      Between them the two switches decompose any result into
                      "the broadband did it" and "the cluster did it", which
                      is why both exist as parameters.
    """
    rng = np.random.default_rng(seed + 770000)
    n = int(round(dur * fs))
    if spread_pct is None:
        spread_pct = float(rng.uniform(3.0, 20.0))
    if shaft_db is None:
        shaft_db = float(rng.uniform(-20.0, -10.0))
    if jitter_pct_100ms is None:
        jitter_pct_100ms = float(rng.uniform(0.3, 2.0))

    shaft0 = rpm / 60.0
    # The rates span `spread_pct`, in a random order so the loudest motor is
    # not always the fastest.
    n_motors = max(1, int(n_motors))
    offs = (np.linspace(-0.5, 0.5, n_motors)[rng.permutation(n_motors)]
            if n_motors > 1 else np.zeros(1))
    rates = shaft0 * (1.0 + spread_pct / 100.0 * offs)

    dist = None if range_m is None else np.full(n, float(range_m))
    y = np.zeros(n)
    for r0 in rates:
        tr = _shaft_track(n, r0, rng, jitter_pct_100ms=jitter_pct_100ms,
                          drift_pct=drift_pct, fs=fs)
        if doppler is not None:
            tr = tr * np.asarray(doppler, float)
        lv = _motor_levels(r0, blades, shaft_db, slope_db_per_oct,
                           knee_hz, tail_db_per_oct, n_orders)
        y = y + harmonic_stack(tr, lv, fs=fs, distance_track=dist)
    y = y / (np.std(y) + 1e-12)

    if with_broadband:
        y = y + prop_broadband(n, rng, fs=fs, range_m=range_m,
                               level_db=bb_level_db,
                               flat_below_hz=bb_flat_below_hz)
        y = y / (np.std(y) + 1e-12)
    if amp_track is not None:
        y = y * np.asarray(amp_track, float)
    return y, float(np.median(rates) * blades)


def drone_v2_hover(dur=8.0, seed=0, fs=FS, rpm=None, blades=3, range_m=4.0,
                   **kw):
    """
    Hover, with the spread. The primary use case, and the class the sealed
    corpus scores 0.59 on with one f0 and no broadband.
    """
    rng = np.random.default_rng(seed + 771000)
    if rpm is None:
        rpm = float(rng.uniform(7500.0, 9500.0))
    return quad_source(dur=dur, seed=seed, fs=fs, rpm=rpm, blades=blades,
                       range_m=range_m, **kw)


def drone_v2_approach(dur=20.0, seed=0, fs=FS, rpm=None, blades=3,
                      r_start=None, r_end=None, **kw):
    """
    A true approach: level rising as 1/r^2 over 5 to 30 s, not a step.

    What this does to v1 can be said in advance: for a source rising with time
    constant tau against a floor with tau_rise = 6 s the quasi-steady ratio
    is 1 + 6/tau, so a 20 s rise reaches about 8 % of a step's per-tooth
    contrast and never crosses threshold. v1 is a change detector by
    construction. The point of this family is not to make v1 pass it - it is
    to stop the corpus pretending v1 was ever asked.

    Doppler is included and is small on purpose: a head-on closer at
    (r_start - r_end) / dur metres per second is a near-constant frequency
    offset, not a sweep. That is the quasi-static claim in the threat model,
    made explicit in the generator instead of asserted in prose.
    """
    rng = np.random.default_rng(seed + 772000)
    if rpm is None:
        rpm = float(rng.uniform(7500.0, 9500.0))
    if r_start is None:
        r_start = float(rng.uniform(60.0, 160.0))
    if r_end is None:
        r_end = float(rng.uniform(6.0, 20.0))
    n = int(round(dur * fs))
    t = np.arange(n) / fs
    r = r_start + (r_end - r_start) * (t / max(dur, 1e-9))
    amp = r_end / r                                   # power as 1/r^2
    v_radial = (r_end - r_start) / max(dur, 1e-9)     # negative = closing
    dop = np.full(n, C_SOUND / (C_SOUND + v_radial))
    # Absorption is applied at the mean range: the whole point of the family
    # is the level ramp, and absorption over this span moves R by well under
    # a decibel.
    return quad_source(dur=dur, seed=seed, fs=fs, rpm=rpm, blades=blades,
                       range_m=0.5 * (r_start + r_end), amp_track=amp,
                       doppler=dop, **kw)


def drone_v2_spinup(dur=8.0, seed=0, fs=FS, rpm=None, blades=3, range_m=4.0,
                    **kw):
    """
    Spin-up at close range: step, then steady. This is the shape of every rig
    test run so far - a machine that was not there and then is, a few metres
    away - and it is the only shape v1's floor is built for.

    Reported separately from hover for exactly that reason. A device that
    detects a spin-up at 3 m and misses an approach at 100 m has been
    measured on the easy half of its own job, and the sealed corpus contains
    only the easy half.
    """
    rng = np.random.default_rng(seed + 773000)
    if rpm is None:
        rpm = float(rng.uniform(7500.0, 9500.0))
    n = int(round(dur * fs))
    t = np.arange(n) / fs
    t0 = float(rng.uniform(0.15, 0.35)) * dur
    rise = float(rng.uniform(0.25, 0.8))
    # rpm climbs from idle to hover over `rise`; level follows it.
    frac = np.clip((t - t0) / rise, 0.0, 1.0)
    idle = float(rng.uniform(0.25, 0.45))
    rate = idle + (1.0 - idle) * frac
    amp = np.clip((t - t0) / rise, 0.0, 1.0) ** 1.5
    y, f0 = quad_source(dur=dur, seed=seed, fs=fs, rpm=rpm, blades=blades,
                        range_m=range_m, amp_track=amp,
                        doppler=rate, **kw)
    return y, f0


def drone_v2_transit(dur=10.0, seed=0, fs=FS, rpm=None, blades=3,
                     speed_ms=None, closest_m=None, **kw):
    """
    Straight-line pass with the spread and the broadband. The v2 counterpart
    of p7_flyby, kept so that the one class the sealed corpus scores well on
    (transit, 0.83) can be compared like for like.
    """
    rng = np.random.default_rng(seed + 774000)
    if rpm is None:
        rpm = float(rng.uniform(7500.0, 9500.0))
    if speed_ms is None:
        speed_ms = float(rng.uniform(12.0, 30.0))
    if closest_m is None:
        closest_m = float(rng.uniform(8.0, 40.0))
    n = int(round(dur * fs))
    r, dop, amp = _flyby_geometry(n, dur, speed_ms, closest_m, fs=fs)
    return quad_source(dur=dur, seed=seed, fs=fs, rpm=rpm, blades=blades,
                       range_m=float(np.mean(r)), amp_track=amp,
                       doppler=dop, **kw)


def drone_v2_hover_tonal(dur=8.0, seed=0, fs=FS, **kw):
    """The SAME clip with the broadband switched off. Paired control: it
    isolates the broadband from the multi-motor spread, so a result can say
    which of the two moved a number instead of asserting it."""
    return drone_v2_hover(dur=dur, seed=seed, fs=fs, with_broadband=False,
                          **kw)


# The second-edition positives.
#
# Two dicts, and the reason is a footgun rather than taste. Every other
# registry in this file returns samples, and evaluate.compose() reaches a
# registry through generator() and immediately does `src_full[:k]`. Slicing a
# (samples, f0) tuple that way truncates the tuple, does not raise, and feeds
# the corpus two elements of garbage. So the registry generator() sees returns
# samples like everything else, and the (samples, f0) form keeps its own name.
POSITIVES_V2_WITH_F0 = {
    "v2_hover": drone_v2_hover,
    "v2_approach": drone_v2_approach,
    "v2_spinup": drone_v2_spinup,
    "v2_transit": drone_v2_transit,
    "v2_hover_tonal": drone_v2_hover_tonal,
}


def _samples_only(fn):
    def gen(dur=8.0, seed=0, fs=FS, **kw):
        return fn(dur=dur, seed=seed, fs=fs, **kw)[0]
    gen.__name__ = fn.__name__ + "_samples"
    gen.__doc__ = (fn.__doc__ or "") + "\n\nSamples only; see POSITIVES_V2_WITH_F0."
    return gen


POSITIVES_V2 = {k: _samples_only(v) for k, v in POSITIVES_V2_WITH_F0.items()}


def generator(kind):
    """Look a family up in either registry. The only sanctioned way to reach
    a Tier 2 or later family by name."""
    for reg in (CONFUSERS, T2_CONFUSERS, T3_CONFUSERS, T3_POSITIVES,
                VETO_CONFUSERS, POSITIVES_V2):
        if kind in reg:
            return reg[kind]
    raise KeyError(kind)


# ----------------------------------------------------------------------
# clip builder + export
# ----------------------------------------------------------------------

def make_clip(kind="drone_flyby", snr_db=0.0, noise="pink", seed=0,
              dur=6.0, fs=FS, **kw):
    """
    One labelled clip. Returns (samples, meta dict).
    meta carries ground truth, which evaluate.py needs to score anything.
    """
    rng = np.random.default_rng(seed + 9973)
    meta = {"kind": kind, "snr_db": snr_db, "noise": noise,
            "seed": seed, "fs": fs, "dur": dur, "drone_present": False,
            "f0_true": None}

    if kind == "drone_flyby":
        sig, f0, _ = drone_flyby(dur=dur, seed=seed, fs=fs, **kw)
        meta.update(drone_present=True, f0_true=f0)
    elif kind == "drone_static":
        sig, f0 = drone_static(dur=dur, seed=seed, fs=fs, **kw)
        meta.update(drone_present=True, f0_true=f0)
    elif kind == "noise_only":
        sig = np.zeros(int(round(dur * fs)))
    elif kind in CONFUSERS:
        sig = CONFUSERS[kind](dur=dur, seed=seed, fs=fs, **kw)
    else:
        raise ValueError(f"unknown kind: {kind}")

    bed = NOISE_KINDS[noise](len(sig), rng)
    out = bed if kind == "noise_only" else mix_at_snr(sig, bed, snr_db)
    peak = np.max(np.abs(out))
    return (out / peak * 0.95 if peak > 0 else out), meta


def to_c_header(samples, name, path, fs=FS, comment=""):
    """
    Export as a C array for ESP32 flash, the format of the golden vectors.
    int16 to keep flash usage sane: 6 s at 16 kHz is ~192 KB.
    """
    q = np.clip(np.round(np.asarray(samples) * 32767), -32768, 32767).astype(np.int16)
    lines = [f"// {comment}", f"// generated by synth.py - do not edit by hand",
             f"// fs = {fs} Hz, n = {len(q)}", "#pragma once", "#include <stdint.h>", "",
             f"#define {name.upper()}_FS {fs}",
             f"#define {name.upper()}_LEN {len(q)}", "",
             f"static const int16_t {name}[{len(q)}] = {{"]
    for i in range(0, len(q), 16):
        lines.append("  " + ", ".join(str(v) for v in q[i:i + 16]) + ",")
    lines.append("};")
    Path(path).write_text("\n".join(lines))
    return path


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    DATA_DIR.mkdir(exist_ok=True)
    FIG_DIR.mkdir(exist_ok=True)

    print(f"3-blade 7in prop, 240-360 Hz target band:")
    for f in (240, 300, 360):
        print(f"   {f:3d} Hz  ->  {rpm_from_bpf(f, 3):5.0f} rpm")

    demos = ["drone_flyby", "drone_static", "helicopter", "prop_aircraft",
             "diesel_truck", "hvac_outdoor_unit", "distant_traffic",
             "noise_only"]

    fig, axes = plt.subplots(len(demos), 1, figsize=(9, 2.0 * len(demos)),
                             sharex=True)
    for ax, kind in zip(axes, demos):
        x, meta = make_clip(kind, snr_db=6.0, noise="pink", seed=1)
        ax.specgram(x, NFFT=2048, Fs=FS, noverlap=1536, cmap="magma")
        ax.set_ylim(0, 3000)
        ax.set_ylabel("Hz", fontsize=8)
        tag = f"  (f0={meta['f0_true']:.0f} Hz)" if meta["f0_true"] else ""
        ax.set_title(f"{kind}{tag}", fontsize=9, loc="left")
    axes[-1].set_xlabel("time (s)")
    plt.tight_layout()
    plt.savefig(FIG_DIR / "synth_overview.png", dpi=130)
    print(f"\nwrote {FIG_DIR / 'synth_overview.png'}")

    x, meta = make_clip("drone_flyby", snr_db=6.0, noise="pink", seed=42)
    to_c_header(x, "golden_flyby", DATA_DIR / "golden_flyby.h",
                comment=f"drone_flyby seed=42 snr=6dB f0={meta['f0_true']:.2f} Hz")
    print(f"wrote {DATA_DIR / 'golden_flyby.h'}  (golden vector)")
