"""
detector.py - the harmonic-comb detector, the reference the firmware is
ported from.

Everything here is causal and streaming: each frame uses only past samples,
because the real device processes audio as it arrives.

Pipeline seam
-------------
    front_end(audio_block)      -> complex spectrum, one per channel
    combiner([spectrum, ...])   -> one spectrum   (identity while N=1)
    back_end(spectrum, state)   -> per-frame decision record

Any multi-channel processing goes in the combiner and nowhere else: a per-bin
complex weighted sum across channels, with front_end and back_end untouched by
it. Several simultaneous beams are several back_end instances, each with its
own DetectorState, which is why all mutable state lives in DetectorState and
there is no module-level mutable state in this file.

Per 32 ms frame (hop=512 @ 16 kHz, window=2048):
  1. Hann window -> rFFT (front_end) -> combine (identity) -> magnitude
  2. Per-bin noise floor, `floor_mode`:
       "legacy"   asymmetric exponential smoother
       "gated"    a broadband transient temporarily shortens tau_rise
       "tonality" tau_rise is short only when the whitened rise is flat (wind)
                  and long when it is peaky (possible drone). The shipped mode.
       "minstat"  Martin-style per-bin sliding minimum
     plus optional comb-hold (see _update_hold): the bins belonging to a
     candidate comb adapt with the slow constant only.
  3. Whiten: W = mag / floor. Makes the detector indifferent to noise colour.
  4. Comb score over a candidate-f0 grid:
         score(f0) = mean_k w_k*log1p(W at k*f0) - mean_k w_k*log1p(W between)
     Teeth minus gaps, saturated at sat_log so no single bin carries a comb.
  5. Optional subharmonic re-anchoring (measured and rejected, default off).
  6. Tracker: alert after `need` continuity-consistent frames above a
     band-dependent threshold. TrackerState is a stepper so the same code runs
     inline (for chain-gated comb-hold) and offline (for sweeps).

Three octave-related mechanisms exist and they are not the same thing. Read
all three before touching any of them:
  - the search band reaches down to 70 Hz so a true fundamental is always a
    candidate (handled by construction, not by a guard);
  - an explicit "prefer f0/2 if it scores nearly as well" guard was tried and
    removed - it rewrote argmax's output and remapped drone f0s out of band;
  - octave-tolerant continuity in TrackerState only decides whether an
    already-scored frame may extend a chain, and holds last_f0 on a match.
"""

from dataclasses import dataclass, asdict

import numpy as np

# Immutable module constants only. No module-level mutable state - a second
# beam must be a second DetectorState, not a second import.
LADDER = (1, 2, 3, 4, 6, 8, 9, 12, 18, 27)


@dataclass
class Config:
    fs: int = 16000
    n_fft: int = 2048
    hop: int = 512
    f_search_lo: float = 70.0
    f_search_hi: float = 2000.0
    f_step: float = 1.0

    # Two-tier alert band. The priority band is provisional: it was set from
    # the physics of the target airframe, not measured.
    f_alert_lo: float = 200.0
    f_alert_hi: float = 2000.0
    f_prio_lo: float = 200.0
    f_prio_hi: float = 800.0
    thr_off_prio: float = 0.0
    thr_off_gen: float = 0.0

    n_harm_max: int = 12
    n_harm_min: int = 4
    f_max_harm: float = 7800.0

    floor_mode: str = "legacy"
    tau_rise_s: float = 6.0
    tau_fall_s: float = 0.8
    tau_rise_fast_s: float = 0.35
    tau_energy_s: float = 4.0
    gate_ratio: float = 1.35
    flat_hi: float = 0.55
    minstat_frames: int = 48
    minstat_sub: int = 4
    minstat_bias: float = 5.5
    sat_log: float = 2.5

    # ---- comb-hold ------------------------------------------------------
    # "off" | "chain" (tracker chain >= hold_chain_min)
    #       | "cand"  (best in-band candidate persists, sub-threshold)
    hold_mode: str = "off"
    hold_chain_min: int = 2
    hold_frac: float = 0.6
    hold_persist: int = 3
    hold_release_s: float = 0.5
    hold_w_bins: int = 1
    hold_max_bins: int = 64

    reanchor: bool = False
    reanchor_margin: float = 0.15
    reanchor_min_tooth: float = 0.70
    reanchor_depth: int = 2

    min_teeth: int = 0
    teeth_level: float = 0.50
    max_jitter: float = 1.0

    # ---- the voice and struck-note veto -----------------------------------
    # Rejects a candidate whose f0 is below the lowest measured drone f0 or
    # whose comb has too few supported teeth. Calibrated on synthetic piano
    # and held-vowel families; paired over the 486 positives it lost no
    # detections and removed the held vowel entirely.
    #
    # Off is bit-identical by construction: the branch that reads these is
    # unreachable while veto_voice is False, so the four golden vectors are
    # untouched by its presence.
    veto_voice: bool = False
    veto_f0_min_hz: float = 250.0   # a held vowel binds at 203-238 Hz
    veto_min_teeth: int = 11        # a piano is inharmonic and cannot reach 12

    # ---- the near-field broadband gate ------------------------------------
    # One flag over two changes, because each is wrong without the other.
    #
    #   * max_jitter 0.004 was fitted to a synthetic f0 that is a straight
    #     line plus 2 Hz of jitter. A real rig holds nothing that still: three
    #     real recordings measure a median |df0|/f0 of 0.0049, 0.0163 and
    #     0.3418 per frame, and 0.004 rejects all three. nf_max_jitter admits
    #     them.
    #   * Relaxing that bound lets music in, so something has to replace it,
    #     and what replaces it is physics: a propeller is a broadband source
    #     and an instrument is not. R = 10log10(E[3200:8000]/E[125:1000])
    #     reads -4.8..+0.6 dB on the real rig, -25.9..-28.7 on music, about
    #     -32 on piano and a held vowel, and -35..-46 on wind. The synthetic
    #     corpus cannot price this: its drones read -29.6, because propeller
    #     broadband was never put into the synthesiser, so the term is
    #     calibrated on real recordings only.
    #
    # R is asked only of a loud candidate. The same rig at 14 m reads -33.4:
    # air absorbs 8 kHz far faster than 500 Hz, so broadband is the first
    # thing distance removes, and a hard R gate would buy a quiet room by
    # going deaf at the range that matters. Below nf_score_hi a candidate is
    # too faint to be a near-field confuser and is passed unasked.
    nf_gate: bool = False
    nf_score_hi: float = 2.00       # ask for R at or above this score
    nf_r_min_db: float = -15.0      # midway between drone -4.8 and music -25.9
    nf_max_jitter: float = 0.05     # real rigs measure 0.005 .. 0.34
    # Band edges as BINS, not Hz, so the C and the Python cannot drift by a
    # rounding rule. At n_fft 2048 / fs 16000 a bin is 7.8125 Hz.
    r_lo_b0: int = 16     # 125 Hz
    r_lo_b1: int = 128    # 1000 Hz, exclusive
    r_hi_b0: int = 410    # 3200 Hz, to Nyquist

    t_warmup_s: float = 1.0
    track_need: int = 6
    track_miss: int = 2
    cont_frac: float = 0.02
    cont_min_hz: float = 12.0

    # ---- tracker v2: a cluster, not a tone --------------------------------
    #
    # A quadcopter is four motors at four rpms. On the device, adding motors
    # lowered the comb score at the same distance, and the 4 m rig
    # recording's argmax runs 158-190 Hz on the shaft family and 468-571 Hz on
    # the blade family - a 20% spread within each family, which is a cluster
    # rather than jitter. The sealed tracker demands one f0 within 2% frame to
    # frame, penalises every departure by 2 and needs 6 agreements: it is a
    # single-tone detector, and a held chord is a single tone while a
    # quadcopter is not.
    #
    # One flag over four changes, all saying that the target is a cluster:
    #   1. family normalisation happens before the band, veto and near-field
    #      gates, so a shaft-line frame of an in-band source is an in-band
    #      frame rather than a frame thrown away before continuity is ever
    #      consulted;
    #   2. continuity is a cluster tolerance (25%) against an exponential
    #      chain centre (tau = 1 s) rather than 2% against the last accepted
    #      f0. A chord change by a fourth (1.33) or a fifth (1.5) is outside
    #      it; a multi-motor spread of 20% is inside;
    #   3. the miss penalty is 1, which moves the chain cliff from p > 2/3
    #      to p > 1/2;
    #   4. the fire-time jitter gate is folded into continuity. The per-frame
    #      jitter is kept as a logged statistic (TrackerState.jitter) and
    #      gates nothing.
    #
    # Off is bit-identical by construction: every branch below is unreachable
    # while trk_v2 is False, and in v2 `last_f0` is never set, which is what
    # makes the sealed continuity block unreachable without rewriting it.
    trk_v2: bool = False
    # 0.25, not cont_frac, so the sealed tracker still runs on its own 0.02
    # and the analysis cache key does not move.
    trk_v2_cont_frac: float = 0.25
    trk_v2_track_miss: int = 1
    # exp(-hop/fs / tau) with tau = 1.0 s, written as a decimal literal here
    # and in detector.h so the C and the Python cannot differ by one ulp of a
    # library exp().
    trk_v2_tau_s: float = 1.0
    trk_v2_alpha: float = 0.9685065820791976
    # The cluster count: local maxima of the score curve within +-25% of the
    # argmax and above 70% of the peak. Logged, never gated - a quadcopter
    # should read 2 to 4 and a single tone 1, and that has not been measured.
    trk_v2_cluster_span: float = 0.25
    trk_v2_cluster_frac: float = 0.70

    def to_dict(self):
        return asdict(self)


def band_threshold(f0, thr, c: Config):
    """Two-tier threshold. Priority band = where the threat physically lives."""
    if c.f_prio_lo <= f0 <= c.f_prio_hi:
        return thr + c.thr_off_prio
    return thr + c.thr_off_gen


#: Tracker v2's family ladder: the same four ratios in the same order as the
#: measured family {2,3} row (src/tracker_variants.py) and detector.c's
#: FAMILY[], because the loop breaks on the first hit and a reordering would
#: be a different rule under a measured rule's name. The 3 is threat physics:
#: a three-blade rotor's shaft line and its blade-pass harmonic are a factor
#: of three apart.
#:
#: (Under the 25% tolerance the orders {2, 1/2, 3, 1/3} and {2, 3, 1/2, 1/3}
#: are the same rule: the only overlapping windows are 2 with 3 (at 2.25c to
#: 2.5c) and 1/2 with 1/3 (at 0.375c to 0.4167c), and 2 precedes 3 and 1/2
#: precedes 1/3 in both orders.)
TRK_V2_FAMILY = (2.0, 0.5, 3.0, 1.0 / 3.0)


def family_v2(centre, f0, c: Config):
    """Tracker v2: normalise a frame's argmax into the chain's family.

    Returns (f0_norm, ratio, matched). `matched` is the continuity verdict:
    once the argmax has been divided by the ratio it matched at, it is inside
    the cluster tolerance by construction, so normalisation and continuity are
    one test and not two. Mirrors family_v2() in detector.c line for line."""
    tol = max(c.trk_v2_cont_frac * centre, c.cont_min_hz)
    if abs(f0 - centre) <= tol:
        return f0, 1.0, True
    for r in TRK_V2_FAMILY:
        tgt = centre * r
        if abs(f0 - tgt) <= max(c.trk_v2_cont_frac * tgt, c.cont_min_hz):
            return f0 / r, r, True
    return f0, 1.0, False


# --------------------------------------------------------------------------
# tracker, as a stepper so inline and offline use are the same code
# --------------------------------------------------------------------------

class TrackerState:
    """
    Chain logic: a frame extends the chain if score > the band-dependent
    threshold, past warmup, f0 in the alert band, enough supported teeth, and
    f0 continuous with the last accepted frame.

    Octave-tolerant continuity: accepts f0 within tolerance of last_f0, or of
    2*last_f0, or of last_f0/2. On an octave match last_f0 is held, not
    updated, and the held value enters the event's f0 median. Under a strict
    rule an argmax slip both fails to extend the chain and overwrites last_f0,
    so the next good frame is rejected too - one slip costs two frames.

    Jitter gate (max_jitter < 1.0): at fire time the median |df0|/f0 of the raw
    argmax over the chain must be <= max_jitter. It must be the raw argmax:
    the continuity rule already refuses anything beyond cont_frac per frame, so
    chain-level jitter is capped at 2% by construction and carries no
    information. Measured medians: drone 0.0024, livestock 0.0073, dogs 0.107.
    """

    def __init__(self, cfg: Config, thr: float):
        self.c = cfg
        self.thr = thr
        self.count = 0
        self.last_f0 = None
        self.chain_f0s = []
        self.chain_raw = []
        self.fired = False
        self.t_on = None
        self.events = []
        # Tracker v2. `centre` is the exponential mean of the accepted
        # family-normalised f0 and replaces last_f0, which is never set with
        # the flag on - that is what makes the sealed continuity block below
        # unreachable without rewriting a character of it. `jitter` is the
        # per-frame normalised step, logged and never gated.
        self.centre = None
        self.jitter = 0.0

    def _jitter_ok(self):
        c = self.c
        # Tracker v2: one gate for "same source", not two. The fire-time
        # jitter gate is folded into continuity - a 25% cluster tolerance
        # against the chain centre already refuses a frame that is not the
        # same source - so nothing here may reject. Everything below this
        # line is the sealed body, unreachable with the flag on.
        if c.trk_v2:
            return True
        if (c.nf_max_jitter if c.nf_gate else c.max_jitter) >= 1.0 or len(self.chain_raw) < 4:
            return True
        a = np.asarray(self.chain_raw, float)
        d = np.abs(np.diff(a)) / np.maximum(a[:-1], 1e-9)
        # With nf_gate clear this is c.max_jitter and nothing has moved, which
        # is what makes flag-off bit-identical to the sealed tracker the four
        # golden vectors describe.
        jmax = c.nf_max_jitter if c.nf_gate else c.max_jitter
        return float(np.median(d)) <= jmax

    def step(self, t, f0, score, teeth=None, f0_raw=None, r_db=None):
        """One frame. Returns (accepted, is_octave, chain_count, latched)."""
        c = self.c
        # ---- tracker v2: normalise into the family before any gate ----------
        # The sealed chain evaluates the band floor at gate 3 and the family
        # rule at gate 6, so a shaft-line frame of an in-band source is
        # thrown away before continuity is ever consulted. The 4 m rig spends
        # 32-36% of its loud frames below 200 Hz on the shaft family; every
        # one of them is a frame the sealed tracker cannot use. Normalising
        # first makes a shaft-line frame of an in-band source an in-band
        # frame. f0n is f0 exactly while the flag is clear.
        f0n, ratio_v2, matched_v2 = f0, 1.0, False
        if c.trk_v2 and self.centre is not None:
            f0n, ratio_v2, matched_v2 = family_v2(self.centre, f0, c)
        thr_eff = band_threshold(f0, self.thr, c)
        ok = (score > thr_eff and t >= c.t_warmup_s
              and c.f_alert_lo <= f0 <= c.f_alert_hi)
        if c.trk_v2:
            # The same three comparisons on the normalised f0. Written out
            # rather than substituted into the line above so that the sealed
            # expression survives character for character and flag-off is a
            # property of the text, not of an argument about equivalence.
            ok = (score > band_threshold(f0n, self.thr, c) and t >= c.t_warmup_s
                  and c.f_alert_lo <= f0n <= c.f_alert_hi)
        if ok and c.min_teeth > 0 and teeth is not None:
            ok = teeth >= c.min_teeth
        # ---- the veto: two terms, each answering a different confuser -------
        # f0 floor: a held vowel and low speech bind at 203-238 Hz, below every
        #   measured drone event (blade2's lowest is 266 Hz). It also removes
        #   80% of livestock, which binds at 207-259 Hz on a harmonic.
        # teeth: a piano string is stiff, so its partials sit at
        #   n*f0*sqrt(1+B*n^2) and walk off the exact multiples the comb
        #   expects. Its events measure 8/11/12 teeth at p10/median/p90 where
        #   every drone class measures 12/12/12. min_teeth was rejected earlier
        #   against livestock, which does saturate at 12/12, before there was a
        #   piano to test it against.
        if ok and c.veto_voice:
            ok = (f0 >= c.veto_f0_min_hz
                  and (teeth is None or teeth >= c.veto_min_teeth))
            if c.trk_v2:
                # The 250 Hz floor asked of the normalised f0. Same reason:
                # a 158 Hz shaft line of a 474 Hz blade family is a 474 Hz
                # frame, and the floor exists to reject a held vowel binding
                # at 203-238 Hz, which has no family above it.
                ok = (f0n >= c.veto_f0_min_hz
                      and (teeth is None or teeth >= c.veto_min_teeth))
        # ---- the near-field broadband gate ----------------------------------
        # Mirrors detector.c line for line. r_db is this frame's high-band
        # ratio, handed in by the caller because the tracker cannot see the
        # spectrum - exactly as the C hands it over on detector_state_t.
        if ok and c.nf_gate and r_db is not None:
            if score >= c.nf_score_hi and r_db < c.nf_r_min_db:
                ok = False
        is_oct = False
        # The sealed continuity block. Unreachable with trk_v2 set, and not
        # because a flag guards it: in v2 `last_f0` is never assigned, so the
        # condition below is False on every frame. Nothing in it was touched.
        if ok and self.last_f0 is not None:
            lf = self.last_f0
            if abs(f0 - lf) <= max(c.cont_frac * lf, c.cont_min_hz):
                pass
            elif (abs(f0 - 2.0 * lf) <= max(c.cont_frac * 2.0 * lf,
                                            c.cont_min_hz)
                  or abs(f0 - 0.5 * lf) <= max(c.cont_frac * 0.5 * lf,
                                               c.cont_min_hz)):
                is_oct = True
            else:
                ok = False
        # ---- tracker v2 continuity: a cluster tolerance --------------------
        # 25% of the chain centre, and the centre is an exponential mean of
        # the accepted normalised f0 with tau = 1 s rather than the last
        # accepted value. `matched_v2` is already the verdict: family_v2()
        # returned True if and only if some ratio put f0 inside the window.
        if c.trk_v2 and ok and self.centre is not None and not matched_v2:
            ok = False
        if ok:
            self.count += 1
            if c.trk_v2:
                a = c.trk_v2_alpha
                self.centre = (f0n if self.centre is None
                               else a * self.centre + (1.0 - a) * f0n)
            elif not is_oct:
                self.last_f0 = f0
            if c.trk_v2:
                self.chain_f0s.append(f0n)
            else:
                self.chain_f0s.append(self.last_f0)
            raw = f0 if f0_raw is None else f0_raw
            if c.trk_v2:
                raw = raw / ratio_v2
            # The per-frame jitter, logged and never gated. O(1): the previous
            # accepted normalised argmax is the last element of chain_raw,
            # where a median over the chain would cost O(n log n) every frame.
            self.jitter = (abs(raw - self.chain_raw[-1])
                           / max(self.chain_raw[-1], 1e-9)
                           if self.chain_raw else 0.0)
            self.chain_raw.append(raw)
            if (self.count >= c.track_need and not self.fired
                    and self._jitter_ok()):
                self.fired, self.t_on = True, t
        else:
            if c.trk_v2:
                self.count = max(0, self.count - c.trk_v2_track_miss)
            else:
                self.count = max(0, self.count - c.track_miss)
            if self.count == 0:
                if self.fired:
                    self.events.append(
                        {"t_on": self.t_on, "t_off": t,
                         "f0": float(np.median(self.chain_f0s))})
                self.fired, self.last_f0 = False, None
                self.centre = None
                self.chain_f0s, self.chain_raw = [], []
        return ok, is_oct, self.count, self.fired

    def finish(self, t_last):
        if self.fired:
            self.events.append({"t_on": self.t_on, "t_off": float(t_last),
                                "f0": float(np.median(self.chain_f0s))})
        return self.events


# --------------------------------------------------------------------------
# all mutable detector state, in exactly one object
# --------------------------------------------------------------------------

class DetectorState:
    """Every mutable quantity the detector carries between frames. A second
    beam is a second DetectorState; nothing is shared, nothing is global."""

    def __init__(self, n_bins, cfg: Config, thr_ref=None):
        self.floor = None
        self.e_slow = None
        self.ms_buf = None
        self.ms_cur = None
        self.ms_i = 0
        self.ms_k = 0
        self.hold_mask = np.zeros(n_bins, bool)
        self.hold_n = 0
        self.hold_lapse = 10 ** 9
        self.cand_f0 = None
        self.cand_n = 0
        self.tracker = (TrackerState(cfg, thr_ref)
                        if (cfg.hold_mode == "chain" and thr_ref is not None)
                        else None)


class CombDetector:
    def __init__(self, cfg: Config = None):
        self.cfg = cfg or Config()
        c = self.cfg
        self.window = np.hanning(c.n_fft).astype(np.float32)
        self.bin_w = c.fs / c.n_fft
        self.n_bins = c.n_fft // 2 + 1

        self.f0s = np.arange(c.f_search_lo, c.f_search_hi + 1e-9, c.f_step)
        Km = c.n_harm_max
        harm = np.arange(1, Km + 1)[None, :]
        tooth_f = self.f0s[:, None] * harm
        gap_f = self.f0s[:, None] * (harm + 0.5)

        tooth_ok = tooth_f <= c.f_max_harm
        gap_ok = gap_f <= c.f_max_harm
        self.tooth_ok = tooth_ok
        self.valid = tooth_ok.sum(axis=1) >= c.n_harm_min

        w = 1.0 / np.sqrt(harm.astype(float))
        tw = np.where(tooth_ok, w, 0.0)
        gw = np.where(gap_ok, w, 0.0)
        self.tw = (tw / np.maximum(tw.sum(axis=1, keepdims=True), 1e-12)
                   ).astype(np.float32)
        self.gw = (gw / np.maximum(gw.sum(axis=1, keepdims=True), 1e-12)
                   ).astype(np.float32)

        def table(freqs, ok):
            pos = freqs / self.bin_w
            i0 = np.clip(pos.astype(np.int64), 0, self.n_bins - 2)
            fr = np.clip(pos - i0, 0.0, 1.0)
            i0[~ok] = 0
            fr[~ok] = 0.0
            return i0, fr.astype(np.float32)

        self.t_i0, self.t_fr = table(tooth_f, tooth_ok)
        self.g_i0, self.g_fr = table(gap_f, gap_ok)

        self.znorm = (1.0 / np.sqrt((self.tw ** 2).sum(axis=1)
                                    + (self.gw ** 2).sum(axis=1))
                      ).astype(np.float32)

        dt = c.hop / c.fs
        self.a_rise = np.exp(-dt / c.tau_rise_s)
        self.a_fall = np.exp(-dt / c.tau_fall_s)
        self.a_rise_fast = np.exp(-dt / c.tau_rise_fast_s)
        self.a_energy = np.exp(-dt / c.tau_energy_s)
        self.hold_release_frames = int(round(c.hold_release_s * c.fs / c.hop))

    # ---- pipeline stage 1 --------------------------------------------------
    def front_end(self, block):
        """One channel, one block of n_fft samples -> complex spectrum."""
        return np.fft.rfft(block * self.window)

    # ---- pipeline stage 2 --------------------------------------------------
    def combiner(self, spectra):
        """N complex spectra -> one. Identity while N=1.

        Any beamforming goes here: X_beam[b] = sum_ch W[ch, b] * X[ch, b]
        with per-bin complex weights. Nothing downstream changes."""
        return spectra[0] if len(spectra) == 1 else np.sum(spectra, axis=0)

    # ---- helpers -----------------------------------------------------------
    def _idx(self, f):
        c = self.cfg
        j = int(round((f - c.f_search_lo) / c.f_step))
        return -1 if (j < 0 or j >= len(self.f0s)) else j

    def _tooth_val(self, S, f):
        pos = f / self.bin_w
        i0 = int(pos)
        if i0 < 0 or i0 >= self.n_bins - 1:
            return 0.0
        fr = pos - i0
        return float(S[i0] * (1.0 - fr) + S[i0 + 1] * fr)

    def _hold_bins_for(self, f0):
        """Bins belonging to the K scoring teeth of f0, +- hold_w_bins.
        Capped at hold_max_bins. O(K) index writes."""
        c = self.cfg
        j = self._idx(f0)
        mask = np.zeros(self.n_bins, bool)
        if j < 0:
            return mask, 0
        n = 0
        for k in range(c.n_harm_max):
            if not self.tooth_ok[j, k]:
                continue
            i0 = int(self.t_i0[j, k])
            lo = max(0, i0 - c.hold_w_bins)
            hi = min(self.n_bins - 1, i0 + 1 + c.hold_w_bins)
            if n + (hi - lo + 1) > c.hold_max_bins:
                break
            mask[lo:hi + 1] = True
            n += hi - lo + 1
        return mask, int(mask.sum())

    def _update_hold(self, st, f0_best, score_best, thr_ref, chain_count):
        """
        Comb-hold. While a candidate comb is present, its own bins adapt with
        the slow rise constant only (the tonality fast path is disabled for
        them). They are not frozen - a genuinely stationary background line is
        still absorbed eventually, just at tau_rise_s.

        The chicken-and-egg this exists to break: a steady hovering comb is
        absorbed by its own adaptive floor before the tracker can build a
        chain, so the evidence needed to protect it is destroyed by the very
        process it would protect against. "chain" mode needs a chain first and
        so can only protect what it has already caught; "cand" mode fires
        sub-threshold and is the one that targets the deadlock.

        Measured and rejected: the hold is not selective. It raises scores on
        confusers as much as on drones, so at a fixed false-alarm budget the
        recalibrated threshold ate the whole gain. Kept, default off.
        """
        c = self.cfg
        hold_now, f0_hold = False, None
        if c.hold_mode == "chain":
            if (chain_count >= c.hold_chain_min and st.tracker is not None
                    and st.tracker.last_f0 is not None):
                hold_now, f0_hold = True, st.tracker.last_f0
        elif c.hold_mode == "cand":
            in_band = (f0_best is not None
                       and c.f_alert_lo <= f0_best <= c.f_alert_hi)
            strong = bool(in_band and score_best >= c.hold_frac * thr_ref)
            if not strong:
                st.cand_f0, st.cand_n = None, 0
            else:
                if st.cand_f0 is not None:
                    tol = max(c.cont_frac * st.cand_f0, c.cont_min_hz)
                    st.cand_n = st.cand_n + 1 if abs(f0_best - st.cand_f0) <= tol else 1
                else:
                    st.cand_n = 1
                st.cand_f0 = f0_best
                if st.cand_n >= c.hold_persist:
                    hold_now, f0_hold = True, f0_best

        if hold_now:
            st.hold_mask, st.hold_n = self._hold_bins_for(f0_hold)
            st.hold_lapse = 0
        else:
            st.hold_lapse += 1
            if st.hold_lapse > self.hold_release_frames and st.hold_n:
                # release: resume normal adaptation, no reset of the floor
                st.hold_mask = np.zeros(self.n_bins, bool)
                st.hold_n = 0

    # ---- floor -------------------------------------------------------------
    def _update_floor(self, mag, st):
        c = self.cfg
        if st.floor is None:
            st.floor = mag.copy()
        prev = st.floor
        flat = 0.0
        fast = False

        if c.floor_mode == "minstat":
            if st.ms_buf is None:
                st.ms_buf = np.tile(mag, (c.minstat_sub, 1))
                st.ms_cur = mag.copy()
                st.ms_i, st.ms_k = 0, 0
            U = max(1, c.minstat_frames // c.minstat_sub)
            np.minimum(st.ms_cur, mag, out=st.ms_cur)
            st.ms_k += 1
            if st.ms_k >= U:
                st.ms_buf[st.ms_i] = st.ms_cur
                st.ms_i = (st.ms_i + 1) % c.minstat_sub
                st.ms_cur = mag.copy()
                st.ms_k = 0
            st.floor = c.minstat_bias * np.minimum(st.ms_buf.min(axis=0),
                                                   st.ms_cur) + 1e-12
        else:
            a_up = self.a_rise
            if c.floor_mode in ("gated", "tonality"):
                e = float(mag.sum())
                if st.e_slow is None:
                    st.e_slow = e
                rising = e > c.gate_ratio * st.e_slow
                if c.floor_mode == "gated":
                    fast = rising
                else:
                    r = mag / (prev + 1e-9)
                    m1 = float(r.mean())
                    m2 = float((r * r).mean())
                    flat = (m1 * m1) / (m2 + 1e-12)
                    fast = rising and flat > c.flat_hi
                st.e_slow = (self.a_energy * st.e_slow
                             + (1.0 - self.a_energy) * e)
                if fast:
                    a_up = self.a_rise_fast
            # Comb-hold: held bins keep the slow rise constant. When nothing is
            # held, or the fast path is not active, this is a no-op and the
            # arithmetic below is bit-identical to the pre-hold detector.
            if st.hold_n and a_up is not self.a_rise:
                a_up = np.where(st.hold_mask, self.a_rise, a_up)
            up = mag > prev
            a = np.where(up, a_up, self.a_fall)
            st.floor = a * prev + (1.0 - a) * mag

        S = np.minimum(np.log1p(mag / (st.floor + 1e-9)),
                       c.sat_log).astype(np.float32)
        return S, flat, fast

    def _teeth_support(self, tv_row, b):
        return int(((tv_row >= self.cfg.teeth_level) & self.tooth_ok[b]).sum())

    def _cluster_count(self, scores, b):
        """Tracker v2, logged and never gated. Local maxima of the score curve
        within +-25% of the argmax and above 70% of the peak.

        A quadcopter is four motors at four rpms and should read 2 to 4; a
        single tone reads 1. That has not been measured yet, which is why it
        is a record field and not a term.

        Written as an explicit loop, not a vectorised one, because it is a
        parity reference: detector.c walks the same window with the same
        plateau rule (>= on the left, > on the right, so a flat top counts
        once and counts at its first sample, which is numpy's argmax rule).
        Computed only with the flag on, so the shipped image's frame budget
        cannot move by a microsecond."""
        c = self.cfg
        peak = float(scores[b])
        if peak <= 0.0:
            return 0
        f0 = float(self.f0s[b])
        lo = int(np.ceil((f0 * (1.0 - c.trk_v2_cluster_span) - c.f_search_lo)
                         / c.f_step))
        hi = int(np.floor((f0 * (1.0 + c.trk_v2_cluster_span) - c.f_search_lo)
                          / c.f_step))
        lo = max(lo, 1)
        hi = min(hi, len(scores) - 2)
        # The cut is float64 in both implementations, on purpose: 0.70f*x in
        # float32 and float32(0.70*x) in float64 differ by an ulp, and an ulp
        # at the 70% line is a cluster count that disagrees with the device.
        # The neighbour comparisons stay float32 against float32, which is
        # exact either way.
        cut = c.trk_v2_cluster_frac * peak
        n = 0
        for j in range(lo, hi + 1):
            s = scores[j]
            if float(s) >= cut and s >= scores[j - 1] and s > scores[j + 1]:
                n += 1
        return n

    def _reanchor(self, scores, S, b):
        """Subharmonic re-anchoring. Measured and rejected (no benefit);
        retained, default off. Moves only if the subharmonic both scores
        comparably and carries real energy at its own fundamental - the second
        condition is what stops a genuine 480 Hz drone being dragged to 240 Hz,
        since it has no 240 Hz component."""
        c = self.cfg
        cur, moved = b, False
        for _ in range(c.reanchor_depth):
            best_j, best_s = -1, -1e9
            f_cur = self.f0s[cur]
            for d in (2.0, 3.0):
                f_sub = f_cur / d
                if f_sub < c.f_search_lo:
                    continue
                j = self._idx(f_sub)
                if j < 0 or not self.valid[j]:
                    continue
                if (scores[j] >= scores[cur] - c.reanchor_margin
                        and self._tooth_val(S, f_sub) >= c.reanchor_min_tooth
                        and scores[j] > best_s):
                    best_j, best_s = j, scores[j]
            if best_j < 0:
                break
            cur, moved = best_j, True
        return cur, moved

    # ---- pipeline stage 3 --------------------------------------------------
    def back_end(self, spec, st, t, thr_ref=None):
        """One combined spectrum -> one per-frame decision record."""
        c = self.cfg
        mag = np.abs(spec).astype(np.float32)
        # The high-band ratio for the near-field gate, from the magnitude
        # spectrum already in hand. Computed unconditionally, exactly as the C
        # does, so a trace can be re-judged with the gate on or off without
        # re-analysing.
        _lo = float(np.dot(mag[c.r_lo_b0:c.r_lo_b1].astype(np.float64),
                           mag[c.r_lo_b0:c.r_lo_b1].astype(np.float64)))
        _hi = float(np.dot(mag[c.r_hi_b0:].astype(np.float64),
                           mag[c.r_hi_b0:].astype(np.float64)))
        r_db = 10.0 * np.log10((_hi + 1e-20) / (_lo + 1e-20))
        S, flat, fast = self._update_floor(mag, st)

        tv = S[self.t_i0] * (1.0 - self.t_fr) + S[self.t_i0 + 1] * self.t_fr
        gv = S[self.g_i0] * (1.0 - self.g_fr) + S[self.g_i0 + 1] * self.g_fr
        scores = ((tv * self.tw).sum(axis=1)
                  - (gv * self.gw).sum(axis=1)) * self.znorm
        scores[~self.valid] = -1e9

        b = int(np.argmax(scores))
        f_win = float(self.f0s[b])
        j, moved = (self._reanchor(scores, S, b) if c.reanchor else (b, False))

        rec = {"f0": float(self.f0s[j]), "f0_raw": f_win,
               "score": float(scores[b]),
               "teeth": self._teeth_support(tv[j], j),
               "reanch": moved, "flat": flat, "fast": fast,
               "n_held": st.hold_n, "r_db": r_db,
               "cluster_n": (self._cluster_count(scores, b)
                             if c.trk_v2 else 0)}

        if c.hold_mode != "off" and thr_ref is not None:
            chain = 0
            if st.tracker is not None:
                _, _, chain, _ = st.tracker.step(t, rec["f0"], rec["score"],
                                                 rec["teeth"], rec["f0_raw"])
            self._update_hold(st, rec["f0"], rec["score"], thr_ref, chain)
        return rec

    # ---- whole signal ------------------------------------------------------
    def analyze(self, x, thr_ref=None):
        """
        Streaming inside. thr_ref is the configured operating threshold and is
        used only by comb-hold (the device knows its own threshold); it does
        not gate scoring. Returns per-frame arrays.
        """
        c = self.cfg
        x = np.asarray(x, np.float32)
        st = DetectorState(self.n_bins, c, thr_ref)
        n_frames = max(0, 1 + (len(x) - c.n_fft) // c.hop)
        out = {k: np.empty(n_frames) for k in ("t", "f0", "f0_raw", "score")}
        out["teeth"] = np.zeros(n_frames, np.int16)
        out["reanch"] = np.zeros(n_frames, bool)
        out["flat"] = np.zeros(n_frames, np.float32)
        out["fast"] = np.zeros(n_frames, bool)
        out["n_held"] = np.zeros(n_frames, np.int16)
        out["r_db"] = np.zeros(n_frames)
        out["cluster_n"] = np.zeros(n_frames, np.int16)

        for i in range(n_frames):
            s0 = i * c.hop
            t = (s0 + c.n_fft) / c.fs          # frame end time: causal
            spec = self.combiner([self.front_end(x[s0:s0 + c.n_fft])])
            rec = self.back_end(spec, st, t, thr_ref)
            out["t"][i] = t
            for k in ("f0", "f0_raw", "score", "teeth", "reanch", "flat",
                      "fast", "n_held", "r_db", "cluster_n"):
                out[k][i] = rec[k]
        out["config"] = c.to_dict()
        return out


# --------------------------------------------------------------------------
# offline tracker over a recorded trace - same stepper as the inline path
# --------------------------------------------------------------------------

def track_frames(trace, thr, cfg: Config = None, collect=True):
    """Events plus the entire per-frame trajectory. The firmware port is judged
    on the trajectory, not just the verdict: float32-vs-float64 changes a
    marginal decision long before it changes a final one."""
    c = cfg or Config()
    n = len(trace["t"])
    tk = TrackerState(c, thr)
    teeth = trace.get("teeth")
    raw = trace.get("f0_raw")
    # Absent from a trace cut before the near-field gate existed, in which case
    # the gate cannot act and says so by being handed None, rather than
    # silently treating a missing measurement as a passing one.
    rdb = trace.get("r_db")
    if collect:
        above = np.zeros(n, bool)
        acc = np.zeros(n, bool)
        octv = np.zeros(n, bool)
        chain = np.zeros(n, np.int32)
        active = np.zeros(n, bool)
    for i in range(n):
        f0 = float(trace["f0"][i])
        s = float(trace["score"][i])
        ok, is_oct, cnt, fired = tk.step(
            float(trace["t"][i]), f0, s,
            None if teeth is None else teeth[i],
            None if raw is None else float(raw[i]),
            None if rdb is None else float(rdb[i]))
        if collect:
            above[i] = s > band_threshold(f0, thr, c)
            acc[i], octv[i], chain[i], active[i] = ok, is_oct, cnt, fired
    events = tk.finish(trace["t"][-1] if n else 0.0)
    if not collect:
        return events, None
    return events, {"above_thr": above, "accepted": acc, "octave": octv,
                    "chain": chain, "active": active}


def track(trace, thr, cfg: Config = None):
    """Events only."""
    return track_frames(trace, thr, cfg, collect=False)[0]


# --------------------------------------------------------------------------
# the deliberately dumb baseline every improvement must beat
# --------------------------------------------------------------------------

def baseline_hps(x, cfg: Config = None, n_harm=4):
    """Returns one scalar per clip: the best frame's HPS peak-to-median, dB."""
    c = cfg or Config()
    x = np.asarray(x, np.float32)
    win = np.hanning(c.n_fft)
    freqs = np.arange(c.n_fft // 2 + 1) * (c.fs / c.n_fft)
    f0s = np.arange(c.f_search_lo, 1600.0, 2.0)
    best = -np.inf
    n_frames = max(0, 1 + (len(x) - c.n_fft) // c.hop)
    for i in range(n_frames):
        s = i * c.hop
        mag = np.abs(np.fft.rfft(x[s:s + c.n_fft] * win))
        r = np.ones_like(f0s)
        for k in range(1, n_harm + 1):
            r *= np.interp(f0s * k, freqs, mag)
        score = 10 * np.log10((r.max() + 1e-30) / (np.median(r) + 1e-30))
        best = max(best, score)
    return best
