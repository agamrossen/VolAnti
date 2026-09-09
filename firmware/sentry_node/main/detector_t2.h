/*
 * detector_t2.h - Tier-2, the slow comb, ported from src/detector_t2.py.
 *
 * Additive in every sense. This module:
 *   - contains no v1 code and calls none;
 *   - shares no mutable state with detector.c;
 *   - is reached only from the new `H` and `K` commands.
 * `G`, `Z`, `L`, `Y`, `R` never enter it, which is what makes the sealed
 * detector's golden evidence still evidence.
 *
 * The seam is the same seam. Tier-2 consumes the combiner's output - the one
 * spectrum - exactly as back_end does. It is a second consumer of one seam,
 * not a second pipeline:
 *
 *     front_end(block) per channel  ->  combiner(...)  ->  +-- back_end   (v1)
 *                                                          +-- t2_step    (T2)
 *
 * and the device alert is the OR of the two tiers.
 *
 * Why it exists. v1's floor rises with tau = 6 s, so a source that keeps
 * running is absorbed into its own noise floor and the six-frame chain never
 * closes. Tier-2 runs a second floor an order of magnitude slower over the
 * priority band only, and counts M hits in N frames instead of 6 in a row.
 * It trades latency for integration; v1 keeps the fast case.
 *
 * COST, and why the band is narrow: the T2 grid is 601 candidates (200-800 Hz)
 * against v1's 1931, and the score is the dominant per-frame stage, so T2's
 * scoring costs ~31% of v1's. The floor is cheaper than v1's because there is
 * no tonality gate: no flatness moments, no energy sum, no second pass.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "detector.h"
#include "generated/t2_config.h"

/* Runtime-settable Tier-2 configuration.
 *
 * Everything here arrives as an `H` command argument or from t2_config.h, so
 * a calibration change is a typed line and never a rebuild - the `G [milli]`
 * precedent. The smoothing COEFFICIENTS are picked from the generated header
 * rather than computed, so the device never depends on its own libm agreeing
 * with numpy's exp(). */
typedef struct {
    double tau2;            /* threshold on score2                         */
    /* The search band, settable at runtime. Row indices into the same v1
     * gather tables, never a second table. Defaults are T2_ROW_LO/T2_N_F0
     * from the generated header, i.e. 200-800 Hz.
     *
     * It is runtime because the priority band is provisional: real rotor
     * captures have put most of their energy well above it, and if a bench
     * session says the comb lives somewhere else the answer has to be a typed
     * argument. A rebuild forces a golden re-proof, which is not something to
     * do mid-session. */
    int    row_lo;          /* first row of the v1 table to score           */
    int    n_rows;          /* how many rows                                */
    int    n2;              /* ring length, in T2 STEPS                    */
    int    m2;              /* hits to fire                                */
    int    gap;             /* consecutive non-hits that kill a track      */
    int    release;         /* steps below m2/2 that close a latched event */
    int    decim;           /* 1 = full rate, 2 = half rate                */
    double a_up, a_dn;      /* floor2 coefficients FOR THIS decim          */
    double warmup_s;
    int    n_excl;
    double excl_c[T2_MAX_EXCL];
    double excl_t[T2_MAX_EXCL];
} t2_cfg_t;

/* Every mutable Tier-2 quantity, in exactly one object - v1's rule.
 * floor2 is float64 for the same reason v1's floor is: numpy's
 * np.where(up, a_up, a_dn) promotes the whole array, and the reference is
 * what the reference COMPUTES. See PORTING_NOTES.md sec 8. */
typedef struct {
    bool     have_floor;
    double   floor2[CFG_N_BINS];

    uint8_t  ring[T2_MAX_N2];
    int      ri;
    int      hits;
    int      track_age;
    int      gap;
    bool     have_track;
    double   track_f0;

    bool     latched;
    int      below;
    double   t_on;
    double   f0_on;
    float    peak2;
    int      n_events;

    uint32_t frame_i;       /* counts EVERY frame, not every T2 step */
} t2_state_t;

/* Per-frame scratch. No cross-frame memory, so deliberately not part of
 * t2_state_t - a second band would be a second state and one work buffer. */
typedef struct {
    float mag[CFG_N_BINS];
    float S2[CFG_N_BINS];
    /* Sized for the WIDEST band the runtime argument can ask for - the whole
     * v1 grid - rather than for the default. 7.7 KB against a heap that has
     * 160 KB free after the quad buffers, and it means a band change can
     * never overrun this. */
    float scores[CFG_N_F0];
} t2_work_t;

/* One frame's Tier-2 record. Streamed as a T2R trace record so the tier is as
 * auditable as v1 from its first day. */
typedef struct {
    uint32_t frame;
    double   t_s;
    float    score2;
    double   f02_hz;
    uint16_t f02_row;       /* row within the T2 slice, 0 .. T2_N_F0-1 */
    uint16_t teeth2;
    uint8_t  hit;
    uint8_t  fired2;
    uint8_t  excluded;
    int32_t  hits;
    int32_t  n2;
    int32_t  track_age;
    float    kappa;         /* 1.0 for mono - see t2_kappa() */
    uint32_t us_t2;
} t2_rec_t;

/* Fill from generated/t2_config.h. `half_rate` selects the ring/gap/release
 * counts AND the matching floor coefficients, so the integration measured in
 * SECONDS is identical either way. */
void t2_cfg_default(t2_cfg_t *c, bool half_rate);

/* Override the search band from `H` arguments, in Hz. Either may be 0 to keep
 * the compiled default. Returns false and leaves the config untouched if the
 * band is not inside the v1 grid or is inverted - a bad argument must be
 * refused, not silently clamped into something that scores the wrong rows. */
bool t2_cfg_set_band(t2_cfg_t *c, int lo_hz, int hi_hz);

/* Replace the persistent-source exclusion list at runtime (the `E` command).
 * n == 0 clears it. Returns false if n exceeds T2_MAX_EXCL. */
bool t2_cfg_set_excl(t2_cfg_t *c, int n, const double *centres,
                     const double *tols);

/* Override the threshold from an `H` argument in MILLI-units (0 = keep). */
void t2_cfg_set_thr_milli(t2_cfg_t *c, int milli);

void t2_state_reset(t2_state_t *st);

/*
 * One frame.
 *
 *   spec     the combiner's output - the SAME pointer back_end is given
 *   spectra  the per-channel spectra, or NULL. Used only for the coherence
 *            telemetry; pass NULL (or n_ch <= 1) and kappa is 1.0.
 *
 * Returns true if Tier-2 actually ran this frame. At half rate it returns
 * false on every second frame without touching `rec`, so the caller emits a
 * T2R record exactly when there is one.
 */
bool t2_step(const cf32_t *spec, const cf32_t *spectra, int n_ch,
             const t2_cfg_t *c, t2_state_t *st, double t, uint32_t frame,
             t2_work_t *w, t2_rec_t *rec);

/* End of stream: closes a still-open event, exactly as the Python finish()
 * does. Without it a detection that runs to the last sample is not an event
 * and silently scores as a miss. */
void t2_finish(t2_state_t *st, double t_last);
