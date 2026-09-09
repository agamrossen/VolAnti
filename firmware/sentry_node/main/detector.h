/*
 * detector.h - the sealed v1 comb detector, ported from src/detector.py.
 *
 * The pipeline is seamed, and the seam mirrors the Python reference exactly:
 *
 *     front_end(block)            -> complex spectrum, one per channel
 *     combiner(spectra, N)        -> one spectrum   (identity while N=1)
 *     back_end(spectrum, state)   -> one per-frame decision record
 *
 * Any multi-channel processing goes in the combiner and nowhere else: a
 * per-bin complex weighted sum across channels, with front_end and back_end
 * untouched by it. Several simultaneous beams are several back_end calls,
 * each against its own detector_state_t, which is possible only because every
 * mutable quantity lives in that struct and this module has no globals except
 * the esp-dsp twiddle tables installed by detector_init().
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "generated/detector_config.h"

/* ---- the near-field broadband gate, and the field jitter bound -----------
 *
 * Two gates downstream of the score decide whether a real rotor is heard.
 *
 * 1. The jitter gate. CFG_MAX_JITTER = 0.004 was fitted to a synthetic
 *    positive whose f0 is a straight line plus 2 Hz of jitter. A real rig
 *    holds nothing still: the three real recordings this project owns measure
 *    a median |df0|/f0 of 0.0049, 0.0163 and 0.3418 per frame, and 0.004
 *    rejects all three. That is not a tuning error of a few percent, it is a
 *    gate calibrated against a model of the threat rather than the threat.
 *    CFG_MAX_JITTER_FIELD = 0.05 admits all three.
 *
 * 2. Relaxing that bound lets music in, so something has to replace it, and
 *    what replaces it is physics rather than a tighter fit: a propeller is a
 *    broadband noise source - blade-vortex shedding, turbulent boundary
 *    layers, tip noise - and a musical instrument is not. Measured as
 *
 *        R = 10*log10( E(3200..8000 Hz) / E(125..1000 Hz) )
 *
 *    a real rig reads -4.8 to +0.6 dB at the median, music -25.9 to -28.7,
 *    piano and a held vowel about -32, wind -35 to -46. Twenty-five decibels
 *    of separation on a term that costs two band sums.
 *
 *    The synthetic corpus cannot price this: its drones read -29.6, because
 *    nobody put propeller broadband into the synth. The term is calibrated on
 *    real rotor recordings and on nothing else.
 *
 * It is a near-field gate rather than a hard one. The same rig at 14 m reads
 * R = -33.4 dB, because air absorbs 8 kHz far faster than 500 Hz and the
 * broadband signature is the first thing distance takes away. A hard R gate
 * would buy a quiet room by going deaf at exactly the range that matters. So
 * R is only ever asked of a loud candidate: at or above CFG_NF_SCORE_HI a
 * comb must look like a machine, and below it the candidate is faint, cannot
 * be a near-field confuser, and is given the benefit of the doubt. That is
 * the standing miss-cost >> false-alarm-cost rule applied to one more axis.
 *
 * Known and unfixed: percussive music - cymbals, hi-hats, brushed snare - has
 * genuine 3-8 kHz energy and can carry R above this floor. Only harmonic and
 * sustained material was modelled. It is a narrower hole than the one it
 * replaces, and it is a hole. */
#define CFG_NF_SCORE_HI       2.00   /* ask for R only at or above this score */
#define CFG_NF_R_MIN_DB     (-15.00) /* midway between drone -4.8 and music -25.9 */
#define CFG_MAX_JITTER_FIELD  0.05   /* real rigs measure 0.005 .. 0.34 */
#define CFG_R_LO_B0  16    /*  125 Hz at 2048/16k  */
#define CFG_R_LO_B1  128   /* 1000 Hz, exclusive   */
#define CFG_R_HI_B0  410   /* 3200 Hz              */

/* ---- tracker v2: a cluster, not a tone -----------------------------------
 *
 * A quadcopter is four motors at four rpms. The 4 m rig recording's argmax
 * runs 158-190 Hz on the shaft family and 468-571 Hz on the blade family - a
 * 20% spread within each family, which is not jitter. The sealed tracker
 * demands one f0 within 2% frame to frame, penalises every departure by 2 and
 * needs 6 agreements: it is a single-tone detector, and a held chord is a
 * single tone while a quadcopter is not. Measured on the device, more source
 * gives less score - two motors at 3 m score higher than four.
 *
 * Four changes behind one flag, because they are one statement.
 *
 * 1. Family normalisation before every gate. The sealed chain puts the band
 *    floor before the family rule, so a shaft-line frame of an in-band source
 *    is thrown away before continuity is ever consulted, and the 4 m rig
 *    spends 32-36% of its loud frames below 200 Hz. Normalised first, a
 *    shaft-line frame of an in-band source is an in-band frame.
 *
 * 2. Continuity becomes a cluster tolerance: 25% of a chain centre that is an
 *    exponential mean of the accepted normalised f0 with tau = 1 s, rather
 *    than 2% of the last accepted f0. A chord change by a fourth (1.33) or a
 *    fifth (1.5) is outside the window; a multi-motor spread of 20% is inside
 *    it. The tolerance is what makes it a cluster tracker; the exponential
 *    centre is what stops the window walking off with the drift.
 *
 * 3. The miss penalty is one. drift = p - (1-p)*MISS, so the chain cliff
 *    moves from p > 2/3 to p > 1/2.
 *
 * 4. One gate for "same source" rather than two. The fire-time jitter gate is
 *    folded into continuity, since a 25% cluster window already refuses a
 *    frame that is not the same source. CFG_MAX_JITTER_FIELD is unreachable
 *    with this flag set, and the nf_gate flag no longer selects a jitter
 *    bound. The per-frame jitter survives as frame_rec_t::jitter, logged and
 *    never read back.
 *
 * Off is bit-identical by construction: every branch is unreachable with the
 * flag clear, and in v2 `last_f0` is never assigned, which is what makes the
 * sealed continuity block unreachable without rewriting a character of it.
 *
 * CFG_TRK_V2_ALPHA is exp(-hop/fs / tau) at tau = 1.0 s, written as a decimal
 * literal here and in src/detector.py's Config so the C and the Python cannot
 * differ by one ulp of a library exp(). The host test checks it against
 * exp(-0.032) and checks that the two files carry the same digits. */
#define CFG_TRK_V2_CONT_FRAC     0.25   /* cluster tolerance, not 2%        */
#define CFG_TRK_V2_TRACK_MISS    1      /* cliff p > 2/3 -> p > 1/2         */
#define CFG_TRK_V2_TAU_S         1.0    /* chain-centre time constant       */
#define CFG_TRK_V2_ALPHA         0.9685065820791976
#define CFG_TRK_V2_CLUSTER_SPAN  0.25   /* +-25% of the argmax   (LOGGED)   */
#define CFG_TRK_V2_CLUSTER_FRAC  0.70   /* above 70% of the peak (LOGGED)   */

/* Longest possible chain is one per frame; the longest golden vector is 916
 * frames. Overflow is reported, never silently wrapped. */
#define SENTRY_MAX_CHAIN 1024

typedef struct {
    float re, im;
} cf32_t;

/* ---- which gate rejected the frame ---------------------------------------
 *
 * Per-gate reject counters say without ambiguity which gate throws a real
 * rig's frames away, on air rather than by inference from an offline replay.
 *
 * These are observational. Every one is written after the decision it
 * describes has already been made, and nothing in the detector reads them
 * back - the sealed gate expressions survive character for character, see the
 * comment on `rej` in tracker_step - so this cannot move a decision even in
 * principle, and the parity tests re-prove that it does not.
 *
 * `p` is computed by the caller as acc / (acc + rejects after threshold), so
 * DET_REJ_THR and DET_REJ_WARMUP are excluded from the denominator: a frame
 * the score never lifted is not a frame a gate took away. */
#define DET_REJ_NONE    0u   /* accepted                                    */
#define DET_REJ_THR     1u   /* score did not clear the band threshold      */
#define DET_REJ_WARMUP  2u   /* t < CFG_T_WARMUP_S                          */
#define DET_REJ_BAND    3u   /* f0 outside [F_ALERT_LO, F_ALERT_HI]         */
#define DET_REJ_VETO    4u   /* voice / struck-note veto                    */
#define DET_REJ_NF      5u   /* near-field broadband gate                   */
#define DET_REJ_CONT    6u   /* continuity, including the family rule       */
#define DET_REJ_N       7u

/* One frame's decision record: the 14 trace fields of device_config.json,
 * plus diagnostics that are NOT trace fields. */
typedef struct {
    uint32_t frame;
    double   t_s;
    float    score;         /* float32, exactly as Python computes it */
    uint16_t f0_bin;
    double   f0_hz;
    double   f0_raw_hz;
    uint16_t teeth;
    uint8_t  floor_fast;
    uint8_t  reanch;        /* ALWAYS 0 - REJECTED feature, no code path */
    uint16_t n_held_bins;   /* ALWAYS 0 - REJECTED feature, no code path */
    uint8_t  above_thr;
    uint8_t  cont_accepted;
    int32_t  chain;
    uint8_t  fired;
    /* diagnostics - NOT trace fields */
    uint32_t us_mag;
    uint32_t us_floor;
    uint32_t us_score;
    double   flat;
    double   e;
    uint8_t  rising;
    uint8_t  is_octave;
    /* ---- observational only, see the block above ----------------------- */
    uint8_t  reject_reason;  /* DET_REJ_*                                   */
    uint8_t  jit_blocked;    /* chain reached TRACK_NEED, jitter_ok refused */
    uint16_t sat_teeth;      /* argmax teeth sitting at CFG_SAT_LOG         */
    uint16_t sat_gaps;       /* argmax gaps sitting at CFG_SAT_LOG          */
    /* ---- tracker v2, logged and never gated ----------------------------
     * `cluster_n` is the number of local maxima of the score curve within
     * +-25% of the argmax and above 70% of the peak: a quadcopter should read
     * 2 to 4 and a single tone 1, and nothing has yet measured whether it
     * does. `jitter` is this frame's normalised |df0|/f0 against the previous
     * accepted normalised argmax - the statistic the removed gate used to
     * read, kept as a record because removing a gate should not also remove
     * the number that would say whether it was right. Both are zero unless
     * trk_v2 is set. */
    uint16_t cluster_n;
    float    jitter;
} frame_rec_t;

/* Every mutable quantity the detector carries between frames. A second beam
 * is a second one of these. Nothing here is shared, nothing is global. */
typedef struct {
    /* ---- adaptive floor (float64: see PORTING_NOTES.md, "floor dtype") -- */
    bool   have_floor;
    double floor_[CFG_N_BINS];
    bool   have_e_slow;
    double e_slow;

    /* ---- TrackerState -------------------------------------------------- */
    double thr;
    int    count;
    bool   have_last_f0;
    double last_f0;
    double chain_f0s[SENTRY_MAX_CHAIN];
    double chain_raw[SENTRY_MAX_CHAIN];
    int    chain_len;
    bool   fired;
    double t_on;
    int    n_events;
    bool   overflow;        /* chain exceeded SENTRY_MAX_CHAIN - trace void */

    /* The family rule: ratio-{2, 1/2, 3, 1/3} continuity with a
     * family-normalised jitter gate. A per-state field rather than a module
     * flag for the reason the whole struct exists - a second beam is a second
     * one of these, and module-level state is what would make a second beam a
     * rewrite.
     *
     * detector_state_reset() memsets to zero, so off is the default, and off
     * is proved bit-identical to the sealed tracker. Set it after reset to
     * turn it on.
     *
     * Its justification: measured non-inferior on the sealed corpus - no
     * regressions and two gains in 486 paired positives at the same threshold
     * and the same weighted false-alarm rate, McNemar p = 0.50 - and
     * physically motivated by the measured 158-190 <-> 468-571 Hz alternation
     * of a 3-blade rotor. Shipping it as the default needs one field session
     * of paired offline replay showing no real-air regression. */
    bool   trk_family;
    /* The voice and struck-note veto, set from settings (`U c veto 1`).
     * Exactly the arrangement trk_family above uses and for the same reason:
     * while it is clear the branch in tracker_step() is unreachable, so the
     * shipped detector is bit-identical to the sealed one and all four golden
     * vectors still describe the code that runs. */
    bool   veto_voice;
    /* One flag, both halves: the relaxed jitter bound and the near-field
     * broadband gate are a pair and must never be separated. The relaxed
     * bound is what lets a real rig chain at all, and the R gate is the only
     * thing then standing between that and every sustained instrument in
     * earshot. Arranged exactly as trk_family and veto_voice above, so with
     * the flag clear both branches are unreachable and the shipped detector
     * is still bit-identical to the sealed one the golden vectors describe. */
    bool   nf_gate;
    /* Tracker v2, one flag over the whole of it, arranged exactly as
     * trk_family, veto_voice and nf_gate above and for the same reason:
     * per-state so a second beam stays a second object, and cleared by the
     * memset in detector_state_reset() so off is what a caller gets by
     * forgetting. See the block at the top of this file.
     *
     * `centre` replaces last_f0 with the flag set - it is the exponential
     * mean of the accepted family-normalised f0 - and last_f0 is then never
     * assigned, which is what makes the sealed continuity block unreachable
     * without rewriting it. Two doubles and a bool inside a struct that
     * already carries 2 x 1024 of them: no allocation, and the arming heap
     * cannot move by more than the padding. */
    bool   trk_v2;
    bool   have_centre;
    double centre;
    /* This frame's high-band ratio in dB, written by back_end() and read by
     * the tracker. Not a trace field; NOT cross-frame memory - it lives here
     * only because tracker_step() cannot see the spectrum. */
    double r_db;
} detector_state_t;

/* Per-frame scratch. No cross-frame memory lives here, so it is explicitly
 * NOT part of detector_state_t: several beams may share one work buffer. */
typedef struct {
    float  fftbuf[CFG_N_FFT] __attribute__((aligned(16)));
    cf32_t spec[CFG_N_CHANNELS][CFG_N_BINS];
    cf32_t combined[CFG_N_BINS];
    float  mag[CFG_N_BINS];
    float  S[CFG_N_BINS];
    double r[CFG_N_BINS];
    double r2[CFG_N_BINS];
    float  scores[CFG_N_F0];
    double jitter[SENTRY_MAX_CHAIN];
} detector_work_t;

/* Installs the esp-dsp twiddle tables. Call once. */
esp_err_t detector_init(void);

void detector_state_reset(detector_state_t *st, double thr);

/* ---- pipeline stage 1: one channel, one block of n_fft samples --------- */
void front_end(const float *block, cf32_t *spec_out, detector_work_t *w);

/* ---- pipeline stage 2: N spectra -> one. Identity while N=1 ----------- */
const cf32_t *combiner(const cf32_t *spectra, int n_ch, detector_work_t *w);

/* ---- pipeline stage 3: one combined spectrum -> one decision record ---- */
void back_end(const cf32_t *spec, detector_state_t *st, double t,
              uint32_t frame, detector_work_t *w, frame_rec_t *rec);

/* End of stream: latches a still-open event, exactly like TrackerState.finish */
void detector_finish(detector_state_t *st, double t_last);

/* ---- probes, for parity forensics only -------------------------------- */
typedef struct {
    uint32_t frame;
    uint32_t win_checksum;       /* FNV-1a over the windowed float32 block */
    uint16_t probe_bin[8];
    float    probe_mag[8];
    double   probe_floor[8];
    float    probe_S[8];
    double   e;
    double   e_slow;
    double   flat;
    uint8_t  rising;
    uint8_t  fast;
    uint16_t argmax_bin;
    float    argmax_score;
} probe_rec_t;

void detector_probe(const detector_work_t *w, const detector_state_t *st,
                    const frame_rec_t *rec, probe_rec_t *p);
uint32_t window_checksum(const float *block, int n);
float sentry_window_probe(int i);
#define SENTRY_WINDOW_PROBE(i) sentry_window_probe(i)
