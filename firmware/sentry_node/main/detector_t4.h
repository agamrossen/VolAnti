/*
 * detector_t4.h - ESP32-S3 port of src/detector_t4.py (Tier-4, the slow comb).
 *
 * The tier with no memory. v1 whitens each bin against an adaptive floor with
 * a 6 s rise constant, which is what makes it indifferent to noise colour and
 * also what makes a source that just sits there disappear: hold a steady comb
 * in front of it and the floor climbs onto the teeth. Tier-2 slows that down
 * but does not remove it.
 *
 * Tier-4 whitens across frequency instead - every bin against the local median
 * of its neighbours - so a comb is measured against the noise beside it rather
 * than against what the same bin was doing a second ago. A hovering source
 * reads the same at second 60 as at second 2: ratio 1.000 over 20 s, against
 * v1 absorbing the same comb to 0.81.
 *
 * It is the cheapest seam of the four. Tier-3 needs the raw block because a
 * per-frame band energy cannot see an 82 Hz modulation; Tier-4 wants the same
 * combined spectrum back_end already has, so it costs no new transform.
 *
 *   i2s block --+-- front_end -> combiner -+-- back_end      (v1)
 *               |                          +-- t2_step       (Tier-2)
 *               |                          +-- t4_push_frame (Tier-4)
 *               +-- t3_push_block                             (Tier-3)
 *
 * It ships disabled, for a number rather than a failure. The calibration
 * succeeded - tau4 = 30.5 gives zero events on every real negative the project
 * owns while both real in-band positives fire - but zero events in 487 s
 * bounds the false-alarm rate at 22/h with 95% confidence against an allowance
 * of 0.40/h. Certifying it needs about 7.5 hours of drone-free audio. See
 * data/t4_config.json.
 *
 * Cost, per 32 ms frame and amortised: one squared magnitude accumulated into
 * a sub-block, which is 563 multiply-adds. Every T4_UPDATE_FRAMES frames it
 * additionally takes a median of T4_N_SUB values per bin, a 38-element local
 * median per bin, and a 591 x 6 table-driven comb scan. The `V` line prints
 * the measured per-update cost; do not reason from an operation count.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "detector.h"
#include "generated/t4_config.h"

/* Runtime-settable Tier-4 configuration. Everything arrives as a command
 * argument or from t4_config.h; a calibration change is a typed line, never a
 * rebuild. Same precedent as `G [milli]` and the `W` arguments. */
typedef struct {
    bool   enabled;              /* default false; see the header comment  */
    double tau4;
    int    n4, m4;
    int    release_updates;
    double warmup_s;
    double cont_frac, cont_min_hz;
    double freeze_tail_s;        /* self-interference freeze tail          */
    /* The veto, on the latch and not on the search grid, so the scan still
     * sees a comb that later rises into band. Set from settings alongside the
     * v1 flag. Tier-4 is the slow comb tier and a held note is exactly what it
     * is built to find, so it fires harder on a sustained vowel than v1 does;
     * all four tiers are OR'd into one alarm, so vetoing v1 alone would leave
     * most of what an operator hears. See docs/VETO_CAL.md. */
    bool   veto_voice;
    double veto_f0_min_hz;
    int    n_excl;
    double excl_c[T4_MAX_EXCL];
    double excl_t[T4_MAX_EXCL];
} t4_cfg_t;

/* One update's worth of decision state. Mirrors the Python record field for
 * field so the host comparator can diff them without a translation layer. */
typedef struct {
    double   t;
    double   W4;
    double   f0;
    double   ratio;              /* which family hop extended the track    */
    bool     hit, above, excluded, frozen;
    int      hits, n4, track_age;
    bool     fired4;
    uint32_t us_t4;              /* measured, not estimated                */
    /* The per-stage breakdown. us_t4 alone says an update cost 21.5 ms
     * against a predicted 1.5 and cannot say where; these four can, and they
     * are required to sum to it. `us_other` is the residual, computed rather
     * than assumed, so a gap can never be hidden by arithmetic.
     *
     * Kept in the shipped build rather than behind a flag: four subtractions
     * on one frame in eight, and the only way to check this has not
     * regressed. */
    uint32_t us_psd;             /* accumulator -> one PSD (median of subs) */
    uint32_t us_prom;            /* the local-median whitening             */
    uint32_t us_scan;            /* the f0 grid scan                       */
    uint32_t us_track;           /* the decision and the tracker           */
    uint32_t us_other;           /* us_t4 minus the four above             */
} t4_rec_t;

/* Everything mutable. A second band would be a second state, not a second
 * module - the same rule detector.c, detector_t2.c and detector_t3.c follow. */
typedef struct {
    /* Accumulator: T4_N_SUB sub-block sums over the bins Tier-4 can reach.
     * Not a ring of periodograms - that would be 256 KB at 64 frames - and
     * the measured cost of the median of these sub-block means against the
     * exact median is 0.2 dB of separation out of 8.3. */
    float    sub[T4_N_SUB][T4_N_BINS_USED];
    int      sub_n[T4_N_SUB];
    int      sub_i;
    int      n_frames, since;
    double   t;
    /* tracker */
    uint8_t  ring[T4_MAX_N4];
    int      ri, hits;
    double   track_f0;           /* < 0 == no track                        */
    int      track_age;
    bool     latched;
    int      below;
    double   t_on, f0_on;
    double   ev_peak;
    int      n_events, n_updates, n_frozen;
    double   frozen_until;
    int64_t  us_total;
    t4_rec_t last;
} t4_state_t;

/* The shipped configuration. enabled = false. */
t4_cfg_t t4_default_cfg(void);

void t4_reset(t4_state_t *st);

/* How many of the six comb positions carried prominence at the last scan.
 * Reporting only - no decision reads it. It exists because a candidate Tier-4
 * teeth term would test this quantity, and that term has to be calibrated
 * against real events before it can ship. */
int t4_last_teeth(void);

/* Feed one frame's COMBINED magnitude spectrum - the same array back_end
 * consumes. Returns true and fills `out` on the frames where an update
 * completed, which is one frame in T4_UPDATE_FRAMES once the window is full. */
/* `output_active` is true while any of the device's own actuators is running.
 * See the freeze note in detector_t4.c: Tier-4 freezes its accumulator, not
 * just its decision, which is a stronger requirement than Tier-3's. */
/* `frame` is the guard's frame counter, the same one the scheduling probe
 * measures the tier phases against. Tier-4's own st->n_frames cannot be used
 * for the slice phase: it does not advance on a frozen frame, so it drifts
 * away from the counter T2 and T3 are scheduled by and the slices wander back
 * onto their frames. */
bool t4_push_frame(const cf32_t *spec, const t4_cfg_t *cfg, t4_state_t *st,
                   uint32_t frame,
                   double t, bool output_active, t4_rec_t *out);

/* Score one accumulated PSD directly. Exposed so the host parity tool can feed
 * the device the same PSD the Python reference computed and compare the
 * statistic in isolation from the accumulator. */
void t4_score(const float *psd, double *W4_out, double *f0_out);

/* The prominence, every bin from scratch: the straightforward implementation,
 * kept compiled so the fast path has something to be proved against rather
 * than argued about. Writes T4_N_BINS_USED floats. Nothing on the device calls
 * it; the host parity test does, bin for bin. */
void t4_prominence_ref(const float *psd, float *out);

/* ---- the prominence, one slice at a time --------------------------------
 *
 * begin(), then step() until it returns true. The whole-band case is the S=1
 * case of exactly this code, so a sliced walk cannot diverge from an unsliced
 * one: there is only one walk.
 *
 * `psd` must be the same frozen snapshot on every step of one update. It
 * already is: t4_psd_now() materialises it once and the accumulator writes
 * elsewhere. Re-deriving it between slices would silently change the answer
 * and nothing here could detect that. */
void t4_prominence_begin(void);
bool t4_prominence_step(const float *psd, int budget);

/* Which frame slots Tier-4 makes expensive, for the scheduling proof in
 * sentry_node.c. Updates land on frames congruent to this, modulo
 * T4_UPDATE_FRAMES, counting from the first frame of the run. */
int  t4_first_update_frame(void);
