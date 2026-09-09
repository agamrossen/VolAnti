/*
 * detector_t3.h - Tier-3, the wash tier, ported from src/detector_t3.py.
 *
 * The wash tier. v1 and Tier-2 both look for a harmonic comb between 200 and
 * 800 Hz. The only real propeller this project has ever recorded put +0.3 dB
 * there and +14.9 dB above 3.2 kHz, and carried its blade-pass periodicity as
 * AMPLITUDE MODULATION of that high band rather than as energy in the low one.
 * Tier-3 demodulates the high band and searches the ENVELOPE for the comb.
 *
 * A third consumer, and a different seam. Tier-2 taps the combiner's output
 * spectrum, exactly where back_end does. Tier-3 cannot: a per-frame band
 * energy is sampled at 31.25 Hz and its Nyquist is 15.6 Hz, so it cannot see
 * an 82 Hz modulation, let alone a 650 Hz one. Tier-3 therefore taps the RAW
 * BLOCK, before the window and the transform:
 *
 *   i2s block --+-- front_end -> combiner -+-- back_end   (v1)
 *               |                          +-- t2_step    (Tier-2)
 *               +-- t3_push_block                          (Tier-3)
 *
 * There is no way to fold this into the existing transform. Do not try.
 *
 * Additive in every sense, same contract as Tier-2: no v1 code, no shared
 * mutable state, reached only from the new `W` command and from `H` when
 * Tier-3 is explicitly enabled. G/Z/L/Y/R/M/T/B/P/F/U/D/A/Q/X never enter it.
 *
 * SHIPS DISABLED. T3_ENABLED_DEFAULT is 0 in the generated header and
 * t3_default_cfg() returns enabled = false. Nothing turns this on but a typed
 * argument.
 *
 * COST. Per 32 ms frame, amortised: 512 samples through 2 biquads and 3
 * one-poles per channel, then one 1024-point real FFT every 4 frames plus a
 * 513-bin local median and a 611-rate score. The `W` command prints the
 * measured cost; do not reason from an operation count.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "detector.h"
#include "generated/t3_config.h"

/* Runtime-settable Tier-3 configuration. Everything arrives as a `W` command
 * argument or from t3_config.h; a calibration change is a typed line, never a
 * rebuild. Same precedent as `G [milli]` and the `H` band arguments. */
typedef struct {
    bool   enabled;         /* DEFAULT FALSE. see the header comment.      */
    double tau3;            /* threshold on W                              */
    double fire_lo, fire_hi;/* rates allowed to FIRE (Hz)                  */
    int    n_harm;          /* harmonics summed, <= T3_MAX_HARM            */
    double weight_exp;      /* 1/k^weight_exp                              */
    double harm_cap_db;     /* per-harmonic prominence cap                 */
    int    n3;              /* ring length, in T3 UPDATES                  */
    int    m3;              /* hits to fire                                */
    int    n3_gap;
    double cont_frac;       /* rate continuity, fractional                 */
    double warmup_s;
    int    release_updates;
    double freeze_tail_s;   /* self-interference freeze tail               */
} t3_cfg_t;

/* One update's worth of decision state. Mirrors the Python record field for
 * field so the host comparator can diff them without a translation layer. */
typedef struct {
    double  t;
    double  r;              /* winning rate inside the firing band         */
    double  W;
    double  r_any;          /* winner over the WHOLE grid - telemetry      */
    double  W_any;
    bool    hit;
    int     hits;
    int     n3;
    int     track_age;
    bool    fired3;
    bool    frozen;
} t3_rec_t;

/* Everything mutable. A second band would be a second state, not a second
 * module - the same rule detector.c and detector_t2.c follow. */
typedef struct {
    /* front end ------------------------------------------------------- */
    float   sos_z[T3_MAX_CH][2][2];       /* 2 biquads x 2 delays          */
    float   lp_z[T3_MAX_CH][T3_MAX_POLES];
    float   env[T3_ENV_NFFT];             /* envelope ring, newest last    */
    int     env_n;                        /* samples held                  */
    int     env_since;                    /* since the last transform      */
    float   per[T3_WELCH_N][T3_ENV_BINS]; /* periodogram ring              */
    int     per_n, per_i;
    /* tracker --------------------------------------------------------- */
    uint8_t ring[T3_MAX_N3];
    int     ri, hits;
    double  track_r;                      /* < 0 == no track               */
    int     track_age, gap;
    bool    latched;
    int     quiet;
    double  t_on, r_on, t_off;
    int     n_events, n_latched;
    /* self-interference ------------------------------------------------ */
    double  frozen_until;
    int     n_frozen;
    /* bookkeeping ------------------------------------------------------ */
    double  t;                            /* seconds of audio consumed     */
    int64_t us_front, us_score;           /* measured, not estimated       */
    int     n_updates;
    t3_rec_t last;
} t3_state_t;

/* The shipped configuration. enabled = false. */
t3_cfg_t t3_default_cfg(void);

/* One-time init: FFT twiddles for the 1024-point transform, the Hann window,
 * and the gather tables. Safe to call more than once. */
int  t3_init(void);
void t3_reset(t3_state_t *st);

/* Feed one raw block, n_ch x T3_HOP interleaved-by-channel (ch-major, the
 * layout front_end already receives). Returns true and fills `out` on the
 * frames where an update completed - one frame in four. `output_active` is
 * true while any of the device's own actuators is running; see the FREEZE
 * note in detector_t3.c. */
bool t3_push_block(const float *const *ch, int n_ch, const t3_cfg_t *cfg,
                   t3_state_t *st, bool output_active, t3_rec_t *out);

/* Score one envelope PSD directly. Exposed so the host parity tool can feed
 * the device the same PSDs the Python reference computed and compare the
 * statistic in isolation from the front end. */
void t3_score(const float *psd, const t3_cfg_t *cfg,
              double *W_fire, double *r_fire, double *W_any, double *r_any);
