"""
detector_t2.py - Tier 2: the slow-floor, long-integration detection tier.

What this is for
----------------
v1 is a change detector. Its per-bin floor rises with tau = 6 s, so a source
that keeps running is absorbed into its own noise floor: the whitened comb
collapses toward log1p(1), the score returns to ambient, and the tracker never
gets its six frames. Measured on real audio, a 7" rotor at 4 m lifted the
median score 0.87 -> 0.99 and then decayed while the rotor kept turning; over
120 s the 4 m and 10 m runs became indistinguishable.

A hovering or loitering drone is the primary case for this device. Tier 2 is a
second, parallel tier that trades latency for integration so that case has a
detector at all:

    same front_end, same combiner, same magnitude
    -> a second per-bin floor with a rise constant of tens of seconds
    -> a second whitening
    -> the same comb arithmetic, restricted to the priority band
    -> an M-of-N tracker over seconds, not a 6-frame chain

v1 is untouched. Nothing here feeds back into v1's floor, score, tracker or
threshold. The device alert is the OR of the tiers.

Why this is not comb-hold, and not the rejected minstat floor
-------------------------------------------------------------
Comb-hold (rejected, p < 0.0001 worse) inflated v1's own scores and did so
non-selectively, so recalibrating v1's threshold to the same false-alarm budget
ate the whole gain and more. The minstat and slow floors (rejected) replaced
the primary floor and destroyed approach detection, because a closing drone is
a rise. Tier 2 inflates nothing and replaces nothing, and it is priced
separately: tau2 is recalibrated against the combined (v1 OR T2) false-alarm
budget, so any gain it reports has already paid for itself.

Its selectivity comes from one property: stability over seconds. A rotor holds
one f0 for as long as it flies. Speech, livestock and dog barks do not - they
are bursts of a few hundred milliseconds with a wandering fundamental. M-of-N
over 3 s with v1's continuity rule is the cheapest test that separates them.

Known blind spots
-----------------
1. A steady source already running when the device arms is baked into floor2
   by the first-frame init, exactly as it is into v1's floor. T2 does not fix
   that. What T2 buys is the source that arrives or grows while the device is
   armed and settled, which is the deployment case.
2. Loud broadband noise saturates S2 at sat_log in teeth and gaps alike, so
   teeth-minus-gaps collapses and T2 goes quiet. That is v1's territory anyway.
3. Steady in-band machinery (irrigation pump, generator) is T2's real
   false-alarm load. It is priced by the long-form negatives and, at a site,
   by the persistent-source exclusion list (empty by default).

Numerics - read before porting
------------------------------
This file deliberately mirrors v1's accidental dtype behaviour rather than
improving on it. `np.where(up, a2_up, a2_dn)` builds a float64 array from two
numpy scalars, so floor2 - and with it the whitening and the log1p - is float64
from frame 0 onward, exactly as `detector.py::_update_floor` is. Only S2, the
gather tables and the comb accumulation are float32.

That is not an oversight. The firmware whitens v1 in float32 against this same
float64 reference, and the divergence was measured and accepted (median 4e-7 to
2.6e-6 on scored frames). Making T2 float32 in Python would give the T2 port a
different numerical relationship to its reference than the v1 port has to its
own, for no measured benefit. One convention, one gate.

Starting a track
----------------
A rule under which any in-band frame that is not continuous starts a new track
would let every sub-threshold frame restart it: the argmax of a quiet spectrum
wanders freely, so no track could ever accumulate. So only a frame that clears
tau2 may start a track, exactly as only such a frame may extend one. A frame
above tau2 whose f0 is discontinuous is evidence of a different comb, so it
closes the current track and opens a new one at its own f0.
"""

from dataclasses import dataclass, asdict, field

import numpy as np

import detector as D

# The T2 grid is a slice of v1's tables, never a second set. f0s = 70 + j Hz,
# so 200 Hz is row 130 and 800 Hz is row 730: 601 candidates. Recomputing the
# tooth/gap tables here would create a second source of truth for the one
# quantity the C port copies verbatim out of flash.
T2_BAND_DEFAULT = (200.0, 800.0)


@dataclass
class T2Config:
    """Tier-2 constants. Mirrors detector.Config's style: a frozen record that
    goes to JSON, to the generated C header, and into the analysis cache key."""

    # --- search band: the priority band only, on v1's own 1 Hz grid -------
    f2_lo: float = 200.0
    f2_hi: float = 800.0

    # --- floor2: asymmetric exponential, no tonality gate, no fast path ---
    tau2_rise_s: float = 60.0      # calibrated from {30, 60, 120}
    tau2_fall_s: float = 0.8       # same fall as v1 - see the note in _whiten
    sat_log: float = 2.5           # same saturation as v1

    # --- decision: M-of-N with v1's continuity rules, then latch ----------
    tau2: float = 1.70             # calibrated against the combined FA budget
    n2: int = 94                   # ring length,  94 frames ~ 3.008 s
    m2: int = 66                   # hits to fire, 66/94 = 70%
    n2_gap: int = 16               # track dies after this many consecutive
    #                                non-hits (~0.51 s)
    t2_warmup_s: float = 4.0       # floor2 must outlive the startup transient
    release_frames: int = 31       # ~1 s of hits < m2/2 closes the event
    cont_frac: float = 0.02        # v1's continuity constants, deliberately
    cont_min_hz: float = 12.0      # the same two numbers

    # --- persistent-source exclusion, empty by default --------------------
    # Up to 4 (centre_hz, tol_hz) bands inside which a T2 hit is NOT counted.
    # Telemetry still logs the frame; only the decision ignores it. There is no
    # self-learning: a human puts a number here or it stays empty.
    excluded_f0: tuple = ()

    # --- rate: half-rate is the shipped default until the device shows ----
    # full-rate p99 fits the 32 ms hop. Halving the rate halves every frame
    # count below, so the integration in seconds is unchanged.
    half_rate: bool = False

    # --- coherence telemetry: measured, never gated on --------------------
    kappa_f_max: float = 1500.0

    def to_dict(self):
        d = asdict(self)
        d["excluded_f0"] = [list(b) for b in self.excluded_f0]
        return d

    @classmethod
    def from_dict(cls, d):
        d = dict(d)
        d["excluded_f0"] = tuple(tuple(float(v) for v in b)
                                 for b in d.get("excluded_f0", ()))
        known = {f for f in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in d.items() if k in known})


class T2State:
    """Every mutable Tier-2 quantity, in exactly one object - v1's rule. A
    second band or a second beam is a second T2State, not a second import."""

    def __init__(self, n_bins, t2):
        self.floor2 = None
        # ring of hit flags for the current track only
        self.ring = np.zeros(int(t2.n2_eff), np.uint8)
        self.ri = 0
        self.hits = 0
        self.track_f0 = None
        self.track_age = 0
        self.gap = 0
        self.latched = False
        self.below = 0
        self.t_on = None
        self.ev_f0s = []
        self.ev_peak = 0.0
        self.ev_kappa = []
        self.events = []
        self.frame_i = -1
        self.last = None

    # -- ring -------------------------------------------------------------
    def push(self, v):
        self.hits += int(v) - int(self.ring[self.ri])
        self.ring[self.ri] = v
        self.ri = (self.ri + 1) % len(self.ring)
        self.track_age += 1

    def clear_track(self):
        self.ring[:] = 0
        self.ri = 0
        self.hits = 0
        self.track_f0 = None
        self.track_age = 0
        self.gap = 0


class Tier2:
    """Stateless given a T2State: tables, constants, and the three steps.

    Constructed from a CombDetector so the tooth/gap/weight/znorm tables are
    the same arrays v1 uses, sliced - not rebuilt.
    """

    def __init__(self, det: D.CombDetector, t2: T2Config = None):
        self.det = det
        self.cfg = t2 or T2Config()
        c = self.cfg
        j0, j1 = det._idx(c.f2_lo), det._idx(c.f2_hi)
        if j0 < 0 or j1 < 0 or j1 < j0:
            raise ValueError(f"T2 band {c.f2_lo}-{c.f2_hi} Hz is not inside "
                             f"the v1 search grid")
        self.row_lo, self.row_hi = j0, j1
        sl = slice(j0, j1 + 1)
        self.f0s = det.f0s[sl]
        self.t_i0, self.t_fr = det.t_i0[sl], det.t_fr[sl]
        self.g_i0, self.g_fr = det.g_i0[sl], det.g_fr[sl]
        self.tw, self.gw = det.tw[sl], det.gw[sl]
        self.znorm = det.znorm[sl]
        self.valid = det.valid[sl]
        self.tooth_ok = det.tooth_ok[sl]
        self.n_bins = det.n_bins
        self.bin_w = det.bin_w

        # kappa weights: the tooth weights, restricted to teeth below
        # kappa_f_max and renormalised. Above ~1.5 kHz an uncompensated
        # inter-bus start offset rotates phases enough to poison C(b).
        tf = self.f0s[:, None] * np.arange(1, det.cfg.n_harm_max + 1)[None, :]
        kok = self.tooth_ok & (tf <= c.kappa_f_max)
        kw = np.where(kok, self.tw, 0.0)
        self.kw = (kw / np.maximum(kw.sum(axis=1, keepdims=True), 1e-12)
                   ).astype(np.float32)
        self.k_any = kok.any(axis=1)

        # rate-dependent constants, all derived so seconds are preserved
        dec = self.decim
        dt = det.cfg.hop / det.cfg.fs * dec
        self.a2_up = np.exp(-dt / c.tau2_rise_s)
        self.a2_dn = np.exp(-dt / c.tau2_fall_s)
        self.dt2 = dt

    # -- rate-scaled constants -------------------------------------------
    @property
    def decim(self):
        return 2 if self.cfg.half_rate else 1

    def state(self):
        """A fresh T2State sized for this configuration."""
        return T2State(self.n_bins, self)

    @property
    def n2_eff(self):
        return max(1, int(round(self.cfg.n2 / self.decim)))

    @property
    def m2_eff(self):
        return max(1, int(round(self.cfg.m2 / self.decim)))

    @property
    def gap_eff(self):
        return max(1, int(round(self.cfg.n2_gap / self.decim)))

    @property
    def rel_eff(self):
        return max(1, int(round(self.cfg.release_frames / self.decim)))

    # -- stage 1: floor2 + whitening --------------------------------------
    def _whiten(self, mag, st):
        """Asymmetric exponential floor, then log1p, saturated.

        The fall constant is v1's 0.8 s on purpose, and it is the reason T2 is
        not a fix for a source that was already running at arm: a floor that
        falls fast converges down onto whatever is present, teeth included.
        tau2_rise only protects against a rise. Slowing the fall as well would
        turn floor2 into a min-statistics floor, which this project measured
        and rejected (approach P(d) 0.27 -> 0.04).
        """
        if st.floor2 is None:
            st.floor2 = mag.copy()
        prev = st.floor2
        up = mag > prev
        a = np.where(up, self.a2_up, self.a2_dn)
        st.floor2 = a * prev + (1.0 - a) * mag
        return np.minimum(np.log1p(mag / (st.floor2 + 1e-9)),
                          self.cfg.sat_log).astype(np.float32)

    # -- stage 2: the comb score, v1's arithmetic on the T2 rows ----------
    def _score(self, S2):
        tv = S2[self.t_i0] * (1.0 - self.t_fr) + S2[self.t_i0 + 1] * self.t_fr
        gv = S2[self.g_i0] * (1.0 - self.g_fr) + S2[self.g_i0 + 1] * self.g_fr
        sc = ((tv * self.tw).sum(axis=1)
              - (gv * self.gw).sum(axis=1)) * self.znorm
        sc[~self.valid] = -1e9          # no-op on 200-800 Hz; kept for parity
        return sc, tv

    # -- coherence telemetry ----------------------------------------------
    def kappa(self, spectra, b):
        """Tooth-weighted mean of C(b) over the winner's teeth below
        kappa_f_max.  C(b) = |sum_c X_c|^2 / (n_ch * sum_c |X_c|^2), so a
        perfectly coherent, perfectly aligned array gives 1.0 and n independent
        channels give ~1/n. With one channel it is identically 1.0, which is
        why mono golden replays carry kappa = 1.0 rather than a missing field.
        """
        if spectra is None or len(spectra) == 0:
            return 1.0
        X = np.asarray(spectra)
        num = np.abs(X.sum(axis=0)) ** 2
        den = len(X) * (np.abs(X) ** 2).sum(axis=0)
        C = (num / np.maximum(den, 1e-30)).astype(np.float32)
        if not self.k_any[b]:
            return float("nan")
        i0, fr = self.t_i0[b], self.t_fr[b]
        cv = C[i0] * (1.0 - fr) + C[i0 + 1] * fr
        return float((cv * self.kw[b]).sum())

    # -- decision helpers --------------------------------------------------
    def _excluded(self, f0):
        for centre, tol in self.cfg.excluded_f0:
            if abs(f0 - centre) <= tol:
                return True
        return False

    def _continuous(self, f0, lf):
        """v1's octave-tolerant continuity, verbatim. Returns (ok, is_octave);
        an octave match holds the track frequency rather than moving it, so one
        argmax slip costs one frame instead of two."""
        c = self.cfg
        if abs(f0 - lf) <= max(c.cont_frac * lf, c.cont_min_hz):
            return True, False
        if (abs(f0 - 2.0 * lf) <= max(c.cont_frac * 2.0 * lf, c.cont_min_hz)
                or abs(f0 - 0.5 * lf) <= max(c.cont_frac * 0.5 * lf,
                                             c.cont_min_hz)):
            return True, True
        return False, False

    def _close_event(self, st, t):
        if st.latched:
            st.events.append({
                "t_on": st.t_on, "t_off": float(t),
                "f0": float(np.median(st.ev_f0s)) if st.ev_f0s else float("nan"),
                "peak_score2": float(st.ev_peak),
                "kappa": (float(np.nanmedian(st.ev_kappa))
                          if st.ev_kappa else float("nan"))})
        st.latched = False
        st.below = 0
        st.t_on = None
        st.ev_f0s, st.ev_kappa, st.ev_peak = [], [], 0.0

    # -- the frame ---------------------------------------------------------
    def _hold(self, st, t):
        """A frame T2 skips at half rate: repeat the last record with
        ran=False, so callers never have to branch on the rate."""
        rec = dict(st.last) if st.last else self._blank(t)
        rec.update(t=float(t), ran=False, hit=False, fired2=st.latched)
        return rec

    def step(self, mag_or_spec, t, st, spectra=None):
        """One frame. `mag_or_spec` may be a complex spectrum (the combiner's
        output) or an already-computed real magnitude."""
        st.frame_i += 1
        if st.frame_i % self.decim:
            return self._hold(st, t)

        x = np.asarray(mag_or_spec)
        mag = (np.abs(x) if np.iscomplexobj(x) else x).astype(np.float32)
        S2 = self._whiten(mag, st)
        sc, tv = self._score(S2)
        b = int(np.argmax(sc))
        f02, s2 = float(self.f0s[b]), float(sc[b])
        kap = self.kappa(spectra, b)
        teeth2 = int(((tv[b] >= self.det.cfg.teeth_level)
                      & self.tooth_ok[b]).sum())
        return self._decide_core(s2, f02, t, st, kap, teeth2)

    def _decide(self, s2, f02, t, st, kappa=1.0, teeth2=0):
        """Decision half only, for replay over a cached (score2, f02) series.
        Shares _decide_core with step(), so the sweep can never drift from the
        rule the reference actually applies."""
        st.frame_i += 1
        if st.frame_i % self.decim:
            return self._hold(st, t)
        return self._decide_core(s2, f02, t, st, kappa, teeth2)

    def _decide_core(self, s2, f02, t, st, kap, teeth2):
        excluded = self._excluded(f02)
        above = (s2 >= self.cfg.tau2 and t >= self.cfg.t2_warmup_s
                 and not excluded)

        hit = False
        if above:
            if st.track_f0 is None:
                st.clear_track()
                st.track_f0 = f02
                hit = True
            else:
                ok, is_oct = self._continuous(f02, st.track_f0)
                if ok:
                    hit = True
                    if not is_oct:
                        st.track_f0 = f02
                else:
                    # a different comb: the old track ends here and a new one
                    # opens at this frequency with an empty ring.
                    self._close_event(st, t)
                    st.clear_track()
                    st.track_f0 = f02
                    hit = True

        if st.track_f0 is not None:
            st.push(1 if hit else 0)
            st.gap = 0 if hit else st.gap + 1
            if st.gap > self.gap_eff:
                self._close_event(st, t)
                st.clear_track()

        if hit:
            st.ev_f0s.append(st.track_f0)
            st.ev_kappa.append(kap)
            st.ev_peak = max(st.ev_peak, s2)

        m2 = self.m2_eff
        if st.track_f0 is not None:
            if not st.latched and st.hits >= m2:
                st.latched = True
                st.t_on = float(t)
                st.below = 0
            elif st.latched:
                if st.hits * 2 >= m2:
                    st.below = 0
                else:
                    st.below += 1
                    if st.below >= self.rel_eff:
                        self._close_event(st, t)

        rec = {"t": float(t), "score2": s2, "f02": f02, "teeth2": teeth2,
               "hit": hit, "hits": int(st.hits), "n2": int(self.n2_eff),
               "track_age": int(st.track_age), "track_f0": st.track_f0,
               "kappa": kap, "fired2": bool(st.latched),
               "excluded": bool(excluded), "ran": True}
        st.last = rec
        return rec

    def _blank(self, t):
        return {"t": float(t), "score2": 0.0, "f02": float(self.f0s[0]),
                "teeth2": 0, "hit": False, "hits": 0, "n2": int(self.n2_eff),
                "track_age": 0, "track_f0": None, "kappa": 1.0,
                "fired2": False, "excluded": False, "ran": True}

    def finish(self, st, t_last):
        """Close a still-open event at the end of a clip, as v1's tracker does.
        Without this a detection that runs to the last sample is not an event
        and silently scores as a miss."""
        self._close_event(st, t_last)
        return st.events


# --------------------------------------------------------------------------
# whole-signal drivers, in the quad_reference.py pattern: the reference's own
# front_end / combiner / back_end, never a lookalike
# --------------------------------------------------------------------------

_T2_KEYS = ("t", "score2", "f02", "teeth2", "hit", "hits", "track_age",
            "kappa", "fired2", "excluded", "ran")


def _alloc(n):
    out = {k: np.zeros(n) for k in ("t", "score2", "f02", "kappa")}
    out["teeth2"] = np.zeros(n, np.int16)
    out["hits"] = np.zeros(n, np.int32)
    out["track_age"] = np.zeros(n, np.int32)
    for k in ("hit", "fired2", "excluded", "ran"):
        out[k] = np.zeros(n, bool)
    return out


def _store(out, i, rec):
    for k in _T2_KEYS:
        out[k][i] = rec[k]


def analyze_quad_t2(channels, cfg=None, t2cfg=None, thr_ref=None,
                    combiner=None, tier1=True):
    """Run both tiers over N channels of real audio.

    channels : (n_ch, n_samples) int16 or float32
    combiner : optional drop-in replacement for CombDetector.combiner, for the
               combiner experiments. None = the shipped unweighted complex sum.

    Returns one dict holding v1's per-frame arrays (the same keys
    CombDetector.analyze produces, so every existing tool understands it) plus
    the T2 arrays and the T2 events.
    """
    import operating_point as op

    cfg = cfg or op.preset_config(op.DEFAULT_PRESET)[0]
    det = D.CombDetector(cfg)
    c = det.cfg
    t2 = Tier2(det, t2cfg or T2Config())

    x = np.asarray(channels)
    if x.dtype != np.float32:
        x = np.stack([np.asarray(ch, np.int16).astype(np.float32) / 32767.0
                      for ch in x])
    if x.ndim == 1:
        x = x[None, :]
    n_ch, n = x.shape

    st1 = D.DetectorState(det.n_bins, c, thr_ref)
    st2 = t2.state()
    n_frames = max(0, 1 + (n - c.n_fft) // c.hop)

    out = {k: np.empty(n_frames) for k in ("t", "f0", "f0_raw", "score")}
    out["teeth"] = np.zeros(n_frames, np.int16)
    out["reanch"] = np.zeros(n_frames, bool)
    out["flat"] = np.zeros(n_frames, np.float32)
    out["fast"] = np.zeros(n_frames, bool)
    out["n_held"] = np.zeros(n_frames, np.int16)
    t2out = _alloc(n_frames)

    comb_fn = combiner or det.combiner
    for i in range(n_frames):
        s0 = i * c.hop
        t = (s0 + c.n_fft) / c.fs                 # frame end time: causal
        spectra = [det.front_end(x[ch, s0:s0 + c.n_fft]) for ch in range(n_ch)]
        spec = comb_fn(spectra)
        if tier1:
            rec = det.back_end(spec, st1, t, thr_ref)
            out["t"][i] = t
            for k in ("f0", "f0_raw", "score", "teeth", "reanch", "flat",
                      "fast", "n_held"):
                out[k][i] = rec[k]
        else:
            out["t"][i] = t
        _store(t2out, i, t2.step(spec, t, st2, spectra))

    t2.finish(st2, out["t"][-1] if n_frames else 0.0)
    out["config"] = c.to_dict()
    out["t2"] = t2out
    out["t2_config"] = t2.cfg.to_dict()
    out["t2_events"] = st2.events
    return out


def run_t2_on_mag(mags, ts, t2cfg=None, det=None, cfg=None):
    """Replay T2 over a saved magnitude series - the cheap path the (N2, M2,
    tau2) sweep runs on. One expensive pass per (clip, tau2_rise) produces the
    score2 series; everything after that is arithmetic on saved numbers."""
    import operating_point as op
    det = det or D.CombDetector(cfg or op.preset_config(op.DEFAULT_PRESET)[0])
    t2 = Tier2(det, t2cfg or T2Config())
    st = t2.state()
    out = _alloc(len(ts))
    for i, (m, t) in enumerate(zip(mags, ts)):
        _store(out, i, t2.step(m, float(t), st))
    t2.finish(st, float(ts[-1]) if len(ts) else 0.0)
    return out, st.events


def track_score2(score2, f02, ts, t2cfg, det=None, cfg=None):
    """Run only the M-of-N decision logic over an already-computed (score2,
    f02) series. This is what makes the calibration sweep affordable: the floor and the
    comb depend on tau2_rise, but N2 / M2 / tau2 do not, so one cached score2
    series serves the whole (N2, M2, tau2) plane.

    It is the same code path as step()'s decision half, reached by feeding the
    cached values back in - not a second implementation of the rule.
    """
    import operating_point as op
    det = det or D.CombDetector(cfg or op.preset_config(op.DEFAULT_PRESET)[0])
    t2 = Tier2(det, t2cfg)
    st = t2.state()
    out = _alloc(len(ts))
    for i in range(len(ts)):
        rec = t2._decide(float(score2[i]), float(f02[i]), float(ts[i]), st)
        _store(out, i, rec)
    t2.finish(st, float(ts[-1]) if len(ts) else 0.0)
    return out, st.events
