"""
detector_t4.py - Tier 4, the slow comb. Reference implementation.

What it is for
--------------
v1 whitens against a per-bin adaptive floor with a 6 s rise constant. That
floor is what makes it indifferent to noise colour, and it is also what makes
a source that just sits there disappear: hold a steady comb in front of it and
within a few seconds the floor has climbed onto the teeth and the whitened
spectrum is flat again. Tier 2 slows the same mechanism down; it does not
remove it.

Tier 4 has no temporal memory at all. It whitens across frequency - each bin
against the local median of its neighbours - so a comb is measured against the
noise beside it rather than against what the same bin was doing a second ago.
A hovering drone therefore reads the same at second 60 as at second 2. That is
Tier 3's structural idea moved from the envelope band into the band where the
comb actually lives and where propagation favours range.

The price is latency: the statistic needs a couple of seconds of averaging to
be stable, so it fires in 2.5-4 s. That trade is deliberate, because the
closing case is v1's job at 0.23 s and this tier is for the loiterer.

What it is not
--------------
It is not a detector of anything drone-specific. Its near enemies are steady
tonal sources - mains hum, a generator, an HVAC fan - and it cannot tell those
from a rotor by structure alone. That is what the runtime exclusion list is
for, and why its calibration prices those families explicitly.

Geometry
--------
Rides v1's own spectra, so it costs no new transform: one periodogram per
32 ms frame, accumulated over `win_frames`, a decision every `update_frames`.
Nothing here is causal-optional - the accumulator only ever holds past frames.
"""

from dataclasses import dataclass, asdict, field

import numpy as np

FS = 16000
N_FFT = 2048
HOP = 512
N_BINS = N_FFT // 2 + 1
DF = FS / N_FFT                       # 7.8125 Hz


@dataclass
class T4Config:
    """Mirrors T2Config's style: a frozen record that goes to JSON, to the
    generated C header, and into the analysis cache key."""

    # --- the fundamental grid -------------------------------------------
    # 110 Hz floor, and it is measured rather than chosen for tidiness: wind's
    # low-frequency continuum manufactures combs out of nothing below it. The
    # null max drops from 23.2 to 15.7 when the floor moves 60 -> 110.
    f0_lo: float = 110.0
    f0_hi: float = 700.0
    f0_step: float = 1.0

    # --- the voice and struck-note veto, on Tier 4's alert, not its search -
    # Tier 4 is the slow comb tier: no temporal floor, so a source that holds
    # still does not fade. A held vowel and a sustained piano note are exactly
    # that, and it fires 1040/h and 850/h on them against v1's 420 and 840.
    # Since all four tiers are OR'd into one alarm, vetoing v1 alone would
    # leave those false alarms in place.
    #
    # The separation is the same one, and it is cleaner here: 99% of Tier 4's
    # held-vowel events and 86% of its livestock events lock below 250 Hz,
    # against 0% of hover, 0% of approach, 0% of transit and 9% of 2-blade. The
    # floor is on the latch and not on f0_lo, deliberately: the search must
    # keep seeing low combs or the tracker cannot follow one that rises into
    # band.
    veto_voice: bool = False
    veto_f0_min_hz: float = 250.0

    # --- the statistic ---------------------------------------------------
    n_harm: int = 6
    cap_db: float = 12.0           # no single line may carry a comb
    f_max_harm: float = 7800.0
    local_lo: int = 6              # local median over |db| in [local_lo,
    local_hi: int = 24             # local_hi], both sides, reflected at edges

    # --- the accumulator -------------------------------------------------
    win_frames: int = 64           # ~2.05 s at 31.25 frames/s
    # 8 frames, not 16, and it was measured both ways. Halving the interval
    # takes the stable clip's first alert from 5.73 s to 4.45 s at no cost in
    # the null (p99 27.3 either way) and a slightly wider feasible set. The
    # updates overlap more, so they carry less independent evidence each - that
    # is the thing that could have gone wrong, and on 487 s of real drone-free
    # audio it did not.
    update_frames: int = 8         # 0.256 s between decisions
    # "median"          - median over all win_frames periodograms. What the
    #                     probe used. Needs every periodogram in memory, which
    #                     is 256 KB at 1025 bins and does not fit on the board.
    # "median_of_means" - mean within each of n_sub sub-blocks, then the median
    #                     across the sub-blocks. One accumulator per sub-block,
    #                     so it is O(n_sub) memory instead of O(win_frames),
    #                     and it keeps the property that made median work: a
    #                     transient inside one sub-block cannot carry the
    #                     result. The device runs this one; the difference
    #                     between the two is measured, not assumed.
    accum: str = "median_of_means"
    n_sub: int = 4

    # --- the decision ----------------------------------------------------
    # Calibrated against 487 s of real drone-free audio: null median 21.4, p99
    # 27.3, max 28.5. Chosen mechanically as the lowest feasible threshold at
    # or above null_p99 + 3.0. The feasible set - zero events on every negative
    # and every real positive fires - is [28.0, 36.0].
    #
    # This is not a certified false-alarm rate. Zero events in 487 s bounds the
    # rate at 22/h with 95% confidence, against an allowance of 0.40/h, and
    # certifying it would take about 7.5 hours of drone-free audio. For that
    # reason the config keeps the tier off by default; whether it runs on a
    # unit is a firmware setting.
    tau4: float = 30.5
    n4: int = 8                    # ring length, 8 updates ~ 4.1 s
    m4: int = 5                    # hits to fire
    release_updates: int = 4       # consecutive misses that close an event
    warmup_s: float = 2.5          # the accumulator must be full first

    # Self-interference. The buzzer drives at 2.7 kHz and this tier's grid
    # reaches 700 Hz with six harmonics, so 450 x 6 = 2700 is a tooth of an
    # ordinary candidate. Tier-3 freezes its decision for 500 ms after an
    # output; Tier-4 must freeze its accumulator, because it integrates over
    # ~2 s and a contaminated frame would keep scoring long after the buzzer
    # stopped. A frozen frame is not accumulated and does not advance the
    # window.
    freeze_tail_s: float = 0.5

    # --- continuity, with the family rule built in -------------------------
    # v1 forgives a 2x hop and not a 3x one, and the threat is a three-blade
    # propeller whose argmax alternates between the shaft line and the third
    # harmonic. Tier 4 divides the ratio out before comparing, so a 3x hop
    # extends a track instead of killing it.
    cont_frac: float = 0.03
    cont_min_hz: float = 4.0
    ratios: tuple = (1.0, 2.0, 3.0, 0.5, 1.0 / 3.0)

    # --- persistent-source exclusion: empty by default, never self-learns -
    excluded_f0: tuple = ()

    def to_dict(self):
        d = asdict(self)
        d["excluded_f0"] = [list(b) for b in self.excluded_f0]
        d["ratios"] = list(self.ratios)
        return d

    @classmethod
    def from_dict(cls, d):
        d = dict(d)
        d["excluded_f0"] = tuple(tuple(float(v) for v in b)
                                 for b in d.get("excluded_f0", ()))
        d["ratios"] = tuple(float(v) for v in d.get("ratios", ()))
        known = {f for f in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in d.items() if k in known})


class T4State:
    """Every mutable Tier-4 quantity, in exactly one object - v1's rule. A
    second band or a second beam is a second T4State, not a second import."""

    def __init__(self, cfg: T4Config, n_bins=N_BINS):
        self.cfg = cfg
        self.n_bins = n_bins
        self.sub = np.zeros((cfg.n_sub, n_bins), np.float64)
        self.sub_n = np.zeros(cfg.n_sub, np.int64)
        self.sub_i = 0
        self.ring = []                      # only for accum == "median"
        self.n_frames = 0
        self.since = 0
        self.t = 0.0
        # tracker
        self.hits_ring = np.zeros(cfg.n4, np.uint8)
        self.ri = 0
        self.hits = 0
        self.track_f0 = None
        self.track_age = 0
        self.latched = False
        self.below = 0
        self.frozen_until = -1.0
        self.n_frozen = 0
        self.t_on = None
        self.f0_on = None
        self.ev_f0s = []
        self.ev_peak = 0.0
        self.events = []
        self.n_updates = 0

    def push_hit(self, v):
        self.hits += int(v) - int(self.hits_ring[self.ri])
        self.hits_ring[self.ri] = v
        self.ri = (self.ri + 1) % len(self.hits_ring)
        self.track_age += 1

    def clear_track(self):
        self.hits_ring[:] = 0
        self.ri = 0
        self.hits = 0
        self.track_f0 = None
        self.track_age = 0
        self.ev_f0s = []
        self.ev_peak = 0.0


class Tier4:
    """Stateless given a T4State: tables, constants, and the streaming step."""

    def __init__(self, cfg: T4Config = None, n_bins=N_BINS, df=DF):
        self.cfg = cfg or T4Config()
        self.n_bins = n_bins
        self.df = df
        c = self.cfg
        self.f0s = np.arange(c.f0_lo, c.f0_hi + 1e-9, c.f0_step)
        k = np.arange(1, c.n_harm + 1, dtype=float)
        self.kw = k ** -0.5
        f = self.f0s[:, None] * k[None, :]
        self.valid = (f <= c.f_max_harm) & (f <= (n_bins - 1) * df)
        self.bins = np.clip(np.rint(f / df).astype(np.int64), 0, n_bins - 1)
        # a candidate needs at least three usable teeth, exactly as
        # band_verdict.best_comb requires - one or two lines is not a comb
        self.enough = self.valid.sum(axis=1) >= 3
        # only these bins ever need a prominence
        need = np.unique(self.bins[self.valid])
        self.b_lo = int(max(need.min() - c.local_hi, 0))
        self.b_hi = int(min(need.max() + c.local_hi, n_bins - 1))
        self._off = np.concatenate([
            np.arange(-c.local_hi, -c.local_lo + 1),
            np.arange(c.local_lo, c.local_hi + 1)])

    # -- the statistic ----------------------------------------------------

    def prominence(self, psd):
        """dB over the local median, reflected at the edges.

        Reflection rather than truncation: a truncated window at the low edge
        would take its median from a different number of bins than everywhere
        else, and the whole point of this tier is that one number means the
        same thing at every frequency."""
        n = self.n_bins
        idx = np.arange(self.b_lo, self.b_hi + 1)[:, None] + self._off[None, :]
        # reflect: -1 -> 1, n -> n-2
        idx = np.abs(idx)
        idx = np.where(idx >= n, 2 * (n - 1) - idx, idx)
        ref = np.median(psd[idx], axis=1)
        out = np.zeros(n)
        out[self.b_lo:self.b_hi + 1] = 10.0 * np.log10(
            (psd[self.b_lo:self.b_hi + 1] + 1e-30) / (ref + 1e-30))
        return out

    def score(self, psd):
        """W4 over the whole f0 grid, plus the winner."""
        c = self.cfg
        prom = self.prominence(psd)
        tv = np.minimum(prom[self.bins], c.cap_db) * self.kw[None, :]
        W = np.where(self.valid, tv, 0.0).sum(axis=1)
        W = np.where(self.enough, W, -1e9)
        j = int(np.argmax(W))
        return {"W": W, "W4": float(W[j]), "f0": float(self.f0s[j]),
                "j": j, "prom": prom,
                "teeth_db": [float(prom[b]) for b, v
                             in zip(self.bins[j], self.valid[j]) if v]}

    # -- the accumulator --------------------------------------------------

    def _psd(self, st):
        c = self.cfg
        if c.accum == "median":
            return np.median(np.asarray(st.ring), axis=0)
        m = st.sub_n > 0
        means = st.sub[m] / st.sub_n[m][:, None]
        return np.median(means, axis=0)

    def push(self, mag, t, st: T4State, output_active=False):
        """One frame. Returns a record on update frames, else None."""
        c = self.cfg
        if output_active:
            st.frozen_until = t + c.freeze_tail_s
        if t <= st.frozen_until:
            st.n_frozen += 1
            return None
        p = np.asarray(mag, np.float64) ** 2
        if c.accum == "median":
            st.ring.append(p)
            if len(st.ring) > c.win_frames:
                st.ring.pop(0)
        else:
            per_sub = max(1, c.win_frames // c.n_sub)
            if st.sub_n[st.sub_i] >= per_sub:
                st.sub_i = (st.sub_i + 1) % c.n_sub
                st.sub[st.sub_i] = 0.0
                st.sub_n[st.sub_i] = 0
            st.sub[st.sub_i] += p
            st.sub_n[st.sub_i] += 1
        st.n_frames += 1
        st.since += 1
        st.t = t
        full = (st.n_frames >= c.win_frames)
        if not full or st.since < c.update_frames:
            return None
        st.since = 0
        return self._update(self._psd(st), t, st)

    # -- the decision -----------------------------------------------------

    def _excluded(self, f0):
        return any(abs(f0 - ctr) <= tol for ctr, tol in self.cfg.excluded_f0)

    def _continuous(self, f0, last):
        """Family-normalised continuity.

        The comparison is made after dividing out whichever of the allowed
        ratios brings the two closest together, so an argmax that jumps from
        the shaft line to the third harmonic - which is what a three-blade
        propeller does, measured - extends the track instead of ending it.
        The ratio is reported, so a track that lives on hops is visible as one
        rather than looking like a clean lock."""
        c = self.cfg
        best = None
        for r in c.ratios:
            tol = max(c.cont_frac * last * r, c.cont_min_hz)
            d = abs(f0 - last * r)
            if d <= tol and (best is None or d < best[1]):
                best = (r, d)
        return best

    def _update(self, psd, t, st: T4State):
        c = self.cfg
        sc = self.score(psd)
        st.n_updates += 1
        f0, W4 = sc["f0"], sc["W4"]
        excluded = self._excluded(f0)
        above = (W4 >= c.tau4) and (t >= c.warmup_s) and not excluded

        ratio = 1.0
        hit = False
        if above:
            if st.track_f0 is None:
                st.clear_track()
                st.track_f0 = f0
                hit = True
            else:
                m = self._continuous(f0, st.track_f0)
                if m is not None:
                    ratio = m[0]
                    hit = True
                    if ratio == 1.0:
                        st.track_f0 = f0
                else:
                    self._close(st, t)
                    st.clear_track()
                    st.track_f0 = f0
                    hit = True
        if st.track_f0 is not None:
            st.push_hit(1 if hit else 0)
            if hit:
                st.ev_f0s.append(f0)
                st.ev_peak = max(st.ev_peak, W4)
            if (not st.latched and st.hits >= c.m4
                    and (not c.veto_voice
                         or st.track_f0 >= c.veto_f0_min_hz)):
                st.latched = True
                st.t_on = t
                st.f0_on = st.track_f0
                st.below = 0
            elif st.latched:
                if hit:
                    st.below = 0
                else:
                    st.below += 1
                    if st.below >= c.release_updates:
                        self._close(st, t)
                        st.clear_track()
        return {"t": float(t), "W4": float(W4), "f0": float(f0),
                "ratio": float(ratio), "hit": bool(hit),
                "above": bool(above), "excluded": bool(excluded),
                "hits": int(st.hits), "n4": int(c.n4),
                "fired": bool(st.latched), "track_age": int(st.track_age),
                "teeth_db": sc["teeth_db"]}

    def _close(self, st, t):
        if st.latched:
            st.events.append({"t_on": st.t_on, "t_off": float(t),
                              "f0": float(np.median(st.ev_f0s))
                              if st.ev_f0s else float("nan"),
                              "peak_W4": float(st.ev_peak)})
        st.latched = False
        st.below = 0
        st.t_on = None

    def finish(self, st, t_last):
        self._close(st, t_last)
        return st.events

    def state(self):
        return T4State(self.cfg, self.n_bins)


# --------------------------------------------------------------------------
# whole-capture driver, mirroring detector_t3.analyse
# --------------------------------------------------------------------------

def analyse(X, cfg: T4Config = None, fs=FS, hop=HOP, n_fft=N_FFT):
    """Stream a whole capture through Tier-4, frame by frame, exactly as the
    device would. Channel-summed first - the same broadside sum the combiner
    makes - so the input is v1's own combined spectrum."""
    import detector as D
    import operating_point as op
    cfg = cfg or T4Config()
    X = np.atleast_2d(np.asarray(X))
    if X.dtype not in (np.float32, np.float64):
        X = np.stack([np.asarray(c, np.int16).astype(np.float32) / 32767.0
                      for c in X])
    x = X.sum(axis=0).astype(np.float32)
    det = D.CombDetector(op.preset_config()[0])
    t4 = Tier4(cfg)
    st = t4.state()
    recs = []
    n_frames = max(0, 1 + (len(x) - n_fft) // hop)
    for i in range(n_frames):
        s0 = i * hop
        spec = det.front_end(x[s0:s0 + n_fft])
        t = (s0 + n_fft) / fs
        r = t4.push(np.abs(spec), t, st)
        if r is not None:
            recs.append(r)
    t4.finish(st, recs[-1]["t"] if recs else 0.0)
    return {"records": recs, "events": st.events, "n_events": len(st.events),
            "n_updates": st.n_updates, "config": cfg.to_dict()}


def measure(X, cfg: T4Config = None):
    """A one-line summary of a capture."""
    r = analyse(X, cfg)
    W = np.array([x["W4"] for x in r["records"]], float)
    if not len(W):
        return {"W4_med": float("nan"), "W4_max": float("nan"),
                "f0_at_max": float("nan"), "n_events": 0, "n_updates": 0}
    i = int(np.argmax(W))
    return {"W4_med": float(np.median(W)), "W4_max": float(W.max()),
            "f0_at_max": float(r["records"][i]["f0"]),
            "n_events": r["n_events"], "n_updates": len(W)}
