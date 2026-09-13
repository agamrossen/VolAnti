"""
detector_t3.py - Tier 3: the wash tier. Find the rotor where the rotor is.

What this is, and why it is a tier rather than an instrument
------------------------------------------------------------
v1 and Tier 2 hunt a harmonic comb of the blade-pass fundamental, in 200-800 Hz.
In the first real propeller recordings, the rotor put +0.3 dB there and
+14.9 dB above 3.2 kHz. Its periodicity is real and in band - blade-pass
246 Hz - but it is carried as amplitude modulation of high-band noise rather
than as energy at 246 Hz, and a comb detector looks for energy.

Broadband rotor noise is made by a blade, once per blade per revolution. So
demodulate the high band and the periodicity is right there in the envelope.
Measured on both rotor captures: a four-harmonic envelope comb, W = 31.8 at 4 m
and 30.9 at 10 m, against 1.1 for the quiet room and 0.6 for real speech.

Three properties make this a tier and not a curiosity:

  It does not absorb. The statistic whitens across the envelope spectrum's own
  frequency axis at each update, with no temporal memory at all. A source that
  has been running for an hour scores exactly what it scored in its first
  second. v1 absorbs a steady source in ~15 s and Tier 2 absorbs it more
  slowly; Tier 3 never does, so a loitering drone is covered structurally
  rather than by tuning.

  It is geometry-free. Envelopes are averaged across channels incoherently, so
  nothing depends on 40.64 or 60.96 mm and everything transfers to the 56 mm
  PCB unchanged. (This is also forced: at 5 kHz the wavelength is 68.6 mm
  against a 60.96 mm baseline, so a coherent sum has direction-dependent nulls,
  and the measured inter-capsule coherence at 3.2-7.8 kHz is 0.21-0.33.)

  Its confusers are not v1's. Speech makes in-band combs and scores 0.6 here.
  Wind fires Tier 2 zero times in 1.83 h. A fan is the real near-enemy, and it
  lives at 30-90 Hz, which is why the firing floor is 100 Hz.

enabled_default is False in every config this module writes. Whether the tier
runs on a unit is decided by the firmware's runtime settings.
"""

from dataclasses import dataclass, asdict

import numpy as np

FS = 16000
HOP = 512

# ---------------------------------------------------------------------------
# front end
# ---------------------------------------------------------------------------
HP_FC = 3000.0
ENV_LP_FC = 800.0

# Three cascaded one-poles, not two. With two, a 2.1 kHz amplitude modulation
# folds past the 1 kHz envelope Nyquist into ~100 Hz - inside the firing grid -
# at a score of 4.8-6.7, around the detection threshold. Measured attenuation
# at 2100 Hz: 17.45 dB with two poles, 26.17 dB with three. The cost at the
# signal is 0.09 -> 0.14 dB at 82 Hz. Nine dB of alias rejection for five
# hundredths of a dB of signal.
ENV_LP_POLES = 3

DECIM = 8
FS_ENV = FS // DECIM              # 2000 Hz
ENV_NFFT = 1024                   # 0.512 s
ENV_HOP = 256                     # 0.128 s per update
WELCH_N = 4

# ---------------------------------------------------------------------------
# the rate grid, from physics
#
#   threat at loaded hover, 3 blades   shaft 125-158 Hz   blade-pass 375-475
#   2-blade variants                   shaft 120-180      blade-pass 240-360
#   the bench test motor               shaft  82          blade-pass 246
#   box fan                            blade-pass 30-90
#   vortex shedding off a 23 mm cone   17 Hz at 2 m/s, 43 at 5, 87 at 10
#
# The grid is computed from 40 Hz so the bench sees fans and shedding in the
# telemetry. Only 100 Hz and above may fire, because at 10 m/s the enclosure's
# own cone mouths shed vortices at 87 Hz - within 6% of the rotor rate the
# bench capture showed. That is a false-alarm mechanism specific to this
# detector and this enclosure, predicted from Strouhal number rather than
# observed, and the floor is where it is because of it.
#
# 100 Hz is not settled. It excludes the 82 Hz bench motor by design; validate
# against those captures with fire_lo = 60 and say so. The deployment floor
# should come from measured fan and shedding rates.
# ---------------------------------------------------------------------------
GRID_LO, GRID_HI = 40.0, 650.0
FIRE_LO, FIRE_HI = 100.0, 320.0

# Why FIRE_HI is 320 and not 650, and why that is not a tuning choice.
#
# The envelope band is usable to about 950 Hz (2 kHz sample rate, an 800 Hz
# 3-pole smoother above it). A candidate rate r is scored on its harmonics
# k*r, and every harmonic past 950 Hz is dropped. So a candidate at 600 Hz is
# scored on one line while a candidate at 200 Hz is scored on four, and the
# statistic is therefore biased toward low rates by construction: a pure 600 Hz
# modulation is reported at 200 Hz, because 200's third harmonic lands on it
# and 200 gets to use the other three bins as well.
#
# That bias is not a defect to be corrected, because the low rate is the one
# that matters. Tier 3 finds the shaft rate, not the blade-pass rate: the bench
# capture put prominences of 22.3 / 18.1 / 16.6 / 12.9 dB at 82, 164, 246 and
# 328 Hz, and the strongest line was the shaft. For the target airframe -
#
#     7" 3-blade, blade-pass 375-475 Hz   ->  shaft 125-158 Hz
#     2-blade variants, 240-360 Hz        ->  shaft 120-180 Hz
#
# - the shaft rate is always between 120 and 180 Hz. So the band that has to
# fire is 100-320: it covers every threat shaft rate with margin, it covers a
# 2-blade blade-pass outright, and it stops below the region where a candidate
# has too few harmonics inside the envelope band to be told apart from its own
# subharmonic. Rates above 320 Hz are still computed, and appear in telemetry
# as r_any; they are simply not allowed to fire under their own name.

RATE_STEP = 1.0
HARM_CAP_DB = 12.0
LOCAL_HALF, LOCAL_SKIP = 8, 1


@dataclass
class T3Config:
    """Tier-3's constants. Same conventions as T2Config: a frozen record that
    goes to JSON, to the generated header, and into the cache key."""

    # --- front end -----------------------------------------------------
    hp_fc: float = HP_FC
    env_lp_fc: float = ENV_LP_FC
    env_lp_poles: int = ENV_LP_POLES
    decim: int = DECIM

    # --- the statistic --------------------------------------------------
    grid_lo: float = GRID_LO          # computed, for telemetry
    grid_hi: float = GRID_HI
    fire_lo: float = FIRE_LO          # may fire only inside this
    fire_hi: float = FIRE_HI
    n_harm: int = 4                   # swept in {2, 3, 4}
    weight_exp: float = 0.5           # 0.5 -> 1/sqrt(k); 1.0 -> 1/k
    harm_cap_db: float = HARM_CAP_DB
    cluster_frac: float = 0.0         # sum envelope power over r +- this

    # --- the tracker ----------------------------------------------------
    tau3: float = 20.0                # threshold on W
    n3: int = 23                      # ring length in updates (2.94 s)
    m3: int = 16                      # hits to fire
    cont_frac: float = 0.03           # rate continuity, +-3%
    n3_gap: int = 6
    warmup_s: float = 2.0
    release_updates: int = 8

    # --- self-interference (see freeze()) ---------------------------------
    freeze_tail_s: float = 0.5

    # --- default state; the firmware settings decide at runtime ----------
    enabled_default: bool = False

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, d):
        known = {f for f in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in d.items() if k in known})


def sos_highpass(fc=HP_FC, fs=FS):
    """4th-order Butterworth as SOS. Computed here and frozen into the header
    by the generator, so the firmware copies numbers rather than a designer."""
    from scipy.signal import butter
    return butter(4, fc, "high", fs=fs, output="sos")


class T3State:
    """Every mutable Tier-3 quantity, in one object. A second band would be a
    second state, not a second import."""

    def __init__(self, cfg: T3Config, n_ch=1):
        self.sos_z = None              # filter state, per channel
        self.lp_z = None
        self.env = []                  # decimated envelope samples, pending
        self.ring = np.zeros(int(cfg.n3), np.uint8)
        self.ri = 0
        self.hits = 0
        self.track_r = None
        self.track_age = 0
        self.gap = 0
        self.latched = False
        self.quiet = 0
        self.t_on = None
        self.r_on = None
        self.n_events = 0
        self.t_off = None
        self.n_latched = 0
        self.frozen_until = -1.0       # self-interference freeze
        self.n_frozen = 0
        self.last = None

    def push(self, v, n3):
        self.hits += int(v) - int(self.ring[self.ri])
        self.ring[self.ri] = v
        self.ri = (self.ri + 1) % n3
        self.track_age += 1

    def clear_track(self, n3):
        self.ring[:] = 0
        self.ri = 0
        self.hits = 0
        self.track_r = None
        self.track_age = 0
        self.gap = 0


class Tier3:
    """Stateless given a T3State: constants, the front end, the statistic and
    the tracker."""

    def __init__(self, cfg: T3Config = None, n_ch=1, fs=FS):
        self.cfg = cfg or T3Config()
        self.fs = fs
        self.n_ch = n_ch
        self.sos = sos_highpass(self.cfg.hp_fc, fs)
        self.a_lp = float(np.exp(-2.0 * np.pi * self.cfg.env_lp_fc / fs))
        self.fs_env = fs // self.cfg.decim
        self.freqs = np.fft.rfftfreq(ENV_NFFT, 1.0 / self.fs_env)
        self.rates = np.arange(self.cfg.grid_lo, self.cfg.grid_hi + 1e-9,
                               RATE_STEP)
        self.win = np.hanning(ENV_NFFT)
        self._pidx = None              # gather tables, built on first use
        self._sidx = None

    def state(self, n_ch=None):
        return T3State(self.cfg, n_ch or self.n_ch)

    # -- front end -------------------------------------------------------
    def envelope_block(self, X, st):
        """One hop of raw audio (n_ch, hop) -> decimated envelope samples.

        Streaming and stateful: filter states persist across calls, so the
        result is identical to filtering the whole signal at once. That is what
        makes this the same computation the firmware does.

        The time-domain path is a requirement, not a preference. Per-frame band
        energy from the existing FFT is sampled at 31.25 Hz (hop 512), Nyquist
        15.6 Hz - it cannot see an 82 Hz modulation, let alone 650 Hz. Do not
        "optimise" this into the existing transform; it cannot work.
        """
        from scipy.signal import sosfilt, lfilter
        X = np.atleast_2d(np.asarray(X, np.float64))
        n_ch = X.shape[0]
        if st.sos_z is None:
            st.sos_z = [np.zeros((self.sos.shape[0], 2)) for _ in range(n_ch)]
            st.lp_z = [[np.zeros(1) for _ in range(self.cfg.env_lp_poles)]
                       for _ in range(n_ch)]
        outs = []
        a = self.a_lp
        for c in range(n_ch):
            y, st.sos_z[c] = sosfilt(self.sos, X[c], zi=st.sos_z[c])
            e = y * y
            for p in range(self.cfg.env_lp_poles):
                e, st.lp_z[c][p] = lfilter([1.0 - a], [1.0, -a], e,
                                           zi=st.lp_z[c][p])
            outs.append(e)
        # Envelopes are averaged, never the high-band waveforms - see the
        # module docstring. Envelopes are non-negative and add incoherently.
        e = np.mean(outs, axis=0)
        return np.sqrt(np.maximum(e[::self.cfg.decim], 0.0))

    # -- the statistic ---------------------------------------------------
    def _prom_tables(self, n):
        """Precompute the gather index table for the local-median reference,
        once per (n, half, skip). The firmware does exactly this: a constant
        index table in flash and a gather, never an index computation per bin.

        Edge bins reflect rather than truncate. Truncating shrinks the sample
        the median is taken over at the ends, which raises its variance exactly
        where the grid's lowest rates read - and the lowest rates are where the
        fan and the vortex shedding live."""
        half, skip = LOCAL_HALF, LOCAL_SKIP
        off = np.array([d for d in range(-half, half + 1) if abs(d) > skip])
        idx = np.arange(n)[:, None] + off[None, :]
        idx = np.abs(idx)                       # reflect at 0
        idx = np.where(idx > n - 1, 2 * (n - 1) - idx, idx)
        return idx

    def _prominence(self, psd, half=LOCAL_HALF, skip=LOCAL_SKIP):
        psd = np.asarray(psd, float)
        n = len(psd)
        if getattr(self, "_pidx", None) is None or self._pidx.shape[0] != n:
            self._pidx = self._prom_tables(n)
        ref = np.median(psd[self._pidx], axis=1)
        return 10.0 * np.log10((psd + 1e-30) / (ref + 1e-30))

    def score(self, psd):
        """W(r) over the whole grid, plus the winner restricted to the firing
        band. The grid is wider than the firing band on purpose: the telemetry
        must show a fan at 45 Hz even though a fan may not fire."""
        c = self.cfg
        prom = self._prominence(psd)
        if getattr(self, "_sidx", None) is None:
            self._build_score_tables(len(prom))
        v = np.where(self._svalid, prom[self._sidx], 0.0)
        if c.cluster_frac > 0:
            # Four motors within a percent of each other put their harmonics
            # in adjacent bins. Taking the best bin within +-cluster_frac
            # recovers a peak that rate spread has walked off centre. This is
            # a max, not a sum, because the prominence is already a ratio to a
            # local median and summing ratios means nothing.
            v = np.where(self._svalid,
                         np.max(prom[self._cidx], axis=2), 0.0)
        W = (self._skw * np.minimum(v, c.harm_cap_db)).sum(axis=1)
        fire = self._sfire
        jf = int(np.argmax(np.where(fire, W, -1e30))) if fire.any() \
            else int(np.argmax(W))
        jall = int(np.argmax(W))
        return {"W": W, "prom": prom,
                "r_fire": float(self.rates[jf]), "W_fire": float(W[jf]),
                "r_any": float(self.rates[jall]), "W_any": float(W[jall])}

    def _build_score_tables(self, nb):
        """The rate x harmonic bin table, built once. Same object the firmware
        holds in flash, which is why it is a table and not arithmetic."""
        c = self.cfg
        df = self.freqs[1] - self.freqs[0]
        k = np.arange(1, c.n_harm + 1)
        f = self.rates[:, None] * k[None, :]
        idx = np.rint(f / df).astype(np.int64)
        valid = (f <= 950.0) & (f <= self.freqs[-1]) & (idx < nb)
        # a harmonic past the end stops the sum there, so mask everything
        # after the first invalid k rather than only the invalid ones
        valid = np.cumprod(valid, axis=1).astype(bool)
        self._sidx = np.clip(idx, 0, nb - 1)
        self._svalid = valid
        self._skw = np.where(valid, 1.0 / k[None, :] ** c.weight_exp, 0.0)
        self._sfire = (self.rates >= c.fire_lo) & (self.rates <= c.fire_hi)
        if c.cluster_frac > 0:
            w = max(1, int(np.ceil(c.cluster_frac / 100.0 * f.max() / df)))
            d = np.arange(-w, w + 1)
            cidx = self._sidx[:, :, None] + d[None, None, :]
            self._cidx = np.clip(cidx, 0, nb - 1)

    # -- self-interference -------------------------------------------------
    def freeze(self, st, t, active):
        """While any of the device's own outputs is running - and for a tail
        afterwards - Tier 3's tracker is frozen: the update counts as neither a
        hit nor a miss.

        Why this exists at all: the vibration motor is an ERM at roughly
        100-200 Hz, mechanically coupled to the microphone cones, radiating
        broadband noise modulated at exactly the rate this tier searches for.
        The buzzer drives at 2-4 kHz, inside the high band. Unhandled, the
        failure is a self-latching loop - fire, motor runs, tier sees a strong
        envelope periodicity, alert never clears. That would break the device
        in the field, so it is closed by construction here rather than hoped
        away by measurement.

        Freezing, not inhibiting. An inhibit would count misses and break an
        established chain, losing a genuine detection during the alert burst.
        Freezing preserves the detection state and pauses only accumulation.
        The burst is 3 s on / 5 s off, so the tier still accumulates 5 of every
        8 seconds during an active alert; since it has by then already fired
        and the latch holds, this affects re-confirmation only.
        """
        if active:
            st.frozen_until = t + self.cfg.freeze_tail_s
        return t <= st.frozen_until

    # -- the update -------------------------------------------------------
    def step(self, psd, t, st, output_active=False):
        """One envelope-spectrum update (every ENV_HOP/FS_ENV = 0.128 s)."""
        c = self.cfg
        sc = self.score(psd)
        r, W = sc["r_fire"], sc["W_fire"]

        if self.freeze(st, t, output_active):
            st.n_frozen += 1
            rec = dict(st.last) if st.last else self._blank(t)
            rec.update(t=float(t), frozen=True, hit=False,
                       fired3=st.latched, r=r, W=W,
                       r_any=sc["r_any"], W_any=sc["W_any"])
            st.last = rec
            return rec

        above = (W >= c.tau3 and t >= c.warmup_s
                 and c.fire_lo <= r <= c.fire_hi)
        hit = False
        if above:
            if st.track_r is None:
                st.clear_track(c.n3)
                st.track_r = r
                hit = True
            elif abs(r - st.track_r) <= c.cont_frac * st.track_r:
                hit = True
                st.track_r = r
            else:
                # A rate jump breaks the chain and starts a new one. It does
                # not end a latched event - see _close's docstring. Measured
                # on the 4 m rotor capture: the argmax hops 82 <-> 163 Hz (the
                # octave) and 82 <-> 70 Hz, 106 times in 914 updates, so
                # identifying "event" with "unbroken chain" reported one
                # continuously running rotor as twelve alerts.
                st.clear_track(c.n3)
                st.track_r = r
                hit = True

        if st.track_r is not None:
            st.push(1 if hit else 0, c.n3)
            st.gap = 0 if hit else st.gap + 1
            if st.gap > c.n3_gap:
                st.clear_track(c.n3)

        if not st.latched:
            if st.track_r is not None and st.hits >= c.m3:
                st.latched = True
                st.t_on = float(t)
                st.r_on = float(st.track_r)
                st.quiet = 0
        else:
            # Release is driven by absence, not by track identity. An event is
            # a latched interval; it ends when the tier stops seeing anything,
            # not when the thing it sees changes rate. Same convention as
            # Tier 2's merge_events(), and the only one under which "events per
            # hour" means what an operator would mean by it.
            st.quiet = 0 if above else st.quiet + 1
            if st.quiet >= c.release_updates:
                self._close(st, t)

        st.n_latched += int(st.latched)
        rec = {"t": float(t), "r": r, "W": W, "r_any": sc["r_any"],
               "W_any": sc["W_any"], "hit": hit, "hits": int(st.hits),
               "n3": int(c.n3), "track_age": int(st.track_age),
               "fired3": bool(st.latched), "frozen": False}
        st.last = rec
        return rec

    def _blank(self, t):
        return {"t": float(t), "r": float("nan"), "W": 0.0,
                "r_any": float("nan"), "W_any": 0.0, "hit": False, "hits": 0,
                "n3": int(self.cfg.n3), "track_age": 0, "fired3": False,
                "frozen": False}

    def _close(self, st, t):
        """End a latched event. Called only on release or at end of signal -
        never on a chain break, because a chain break is a rate jump and a rate
        jump is still the same source."""
        if st.latched:
            st.n_events += 1
            st.t_off = float(t)
        st.latched = False
        st.quiet = 0

    def finish(self, st, t_last):
        self._close(st, t_last)
        return st.n_events


# ---------------------------------------------------------------------------
# whole-signal driver
# ---------------------------------------------------------------------------

def analyse(X, cfg: T3Config = None, fs=FS, hop=HOP, skip_s=2.0,
            output_active=None):
    """Stream a whole capture through Tier-3, hop by hop, exactly as the
    device would. Returns per-update records plus the event count."""
    cfg = cfg or T3Config()
    X = np.atleast_2d(np.asarray(X))
    if X.dtype not in (np.float32, np.float64):
        X = np.stack([np.asarray(c, np.int16).astype(np.float64) / 32767.0
                      for c in X])
    X = X[:, int(skip_s * fs):]
    t3 = Tier3(cfg, n_ch=X.shape[0], fs=fs)
    st = t3.state(X.shape[0])
    env_buf = np.zeros(0)
    per = []
    recs = []
    n_hops = X.shape[1] // hop
    for h in range(n_hops):
        blk = X[:, h * hop:(h + 1) * hop]
        env_buf = np.concatenate([env_buf, t3.envelope_block(blk, st)])
        while len(env_buf) >= ENV_NFFT:
            seg = env_buf[:ENV_NFFT]
            s = seg - seg.mean()
            per.append(np.abs(np.fft.rfft(s * t3.win)) ** 2)
            env_buf = env_buf[ENV_HOP:]
            if len(per) >= WELCH_N:
                psd = np.mean(per[-WELCH_N:], axis=0)
                t = skip_s + (h + 1) * hop / fs
                oa = bool(output_active(t)) if output_active else False
                recs.append(t3.step(psd, t, st, output_active=oa))
    t3.finish(st, recs[-1]["t"] if recs else 0.0)
    return {"records": recs, "n_events": st.n_events, "n_frozen": st.n_frozen,
            "n_latched": st.n_latched, "config": cfg.to_dict()}
