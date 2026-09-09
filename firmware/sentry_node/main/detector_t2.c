/*
 * detector_t2.c - ESP32-S3 port of src/detector_t2.py.
 *
 * Ported FROM THE CODE, under PORTING_NOTES.md discipline. Every place the
 * Python relies on a numpy implementation detail is reproduced here on
 * purpose, and every deliberate divergence is named where it happens.
 *
 * ---------------------------------------------------------------------------
 * Dtype decisions, all of them, in one place
 * ---------------------------------------------------------------------------
 * 1. floor2 is FLOAT64. Not because double is better - because
 *    `np.where(up, a2_up, a2_dn)` in the reference builds a float64 array from
 *    two numpy scalars, which promotes the floor and everything computed from
 *    it. src/detector_t2.py mirrors v1's promotion deliberately (see its
 *    "NUMERICS" section), so this port faces the identical, already-measured
 *    relationship that detector.c faces. One religion, one gate.
 *
 * 2. The WHITENING is float32 log1pf, exactly as v1's is, and for exactly the
 *    same reason: 1025 soft-float64 log1p calls cost ~21.8 ms of a 32 ms
 *    budget on a core with no double FPU. float32 log1pf is within 1 ulp
 *    (~6e-8 on S), which is orders below any decision margin. This is the ONE
 *    deliberate numerical divergence, it is the same one v1 makes, and it is
 *    re-proved by the K-mode T2 trajectory gate.
 *
 * 3. The COMB SCORE is float32 throughout - S, the gather tables, the 12-term
 *    row sums - because the reference's are. The 12-term sums go through
 *    pw_sum_f32 because numpy's sum is not left-to-right even at n = 12.
 *
 * 4. The DECISION arithmetic (thresholds, continuity, f0) is float64, as the
 *    reference's Python floats are.
 *
 * 5. `hits >= m2/2` is written as `2*hits >= m2` so an odd M2 rounds the same
 *    way on both sides. The reference does the same.
 *
 * ---------------------------------------------------------------------------
 * A duplication, deliberate and measured
 * ---------------------------------------------------------------------------
 * generated/tables.h declares its arrays `static const`, so including it here
 * gives this translation unit its OWN copy - about 189 KB of flash on top of
 * detector.c's. D3 says the T2 grid must be rows 130..730 of the EXISTING
 * tables and that there are to be no new tables, and detector.c is frozen for
 * this branch, so a shared-linkage table is not available tonight. The copy is
 * bit-identical to detector.c's by construction (same generator, same file),
 * costs flash and not bandwidth, and the one-line fix - move the tables into
 * a translation unit of their own with external linkage - is recorded in
 * PORTING_NOTES.md for the first session allowed to touch detector.c.
 */

#include "detector_t2.h"

#include <math.h>
#include <string.h>

#include "esp_timer.h"

#include "generated/tables.h"

/* Same committed table format as detector.c: one-byte index into an exact
 * LUT. Not quantisation - the LUT holds the table's own distinct float32
 * values, so the dequantised value is bit-identical to the float table. */
#ifndef SENTRY_TABLES_LUT
#define SENTRY_TABLES_LUT 1
#endif

#if SENTRY_TABLES_LUT
#define TFR(i)  SENTRY_T_FR_LUT[SENTRY_T_FR_IDX[i]]
#define GFR(i)  SENTRY_G_FR_LUT[SENTRY_G_FR_IDX[i]]
#define TWT(i)  SENTRY_TW_LUT[SENTRY_TW_IDX[i]]
#define GWT(i)  SENTRY_GW_LUT[SENTRY_GW_IDX[i]]
#else
#define TFR(i)  SENTRY_T_FR[i]
#define GFR(i)  SENTRY_G_FR[i]
#define TWT(i)  SENTRY_TW[i]
#define GWT(i)  SENTRY_GW[i]
#endif

/* ======================================================================== */
/* numpy's pairwise summation.                                              */
/*                                                                          */
/* A byte-for-byte copy of detector.c's pw_sum_f32, present here only        */
/* because detector.c's is `static` and detector.c is frozen. If the two     */
/* ever diverge the T2 gate fails, which is the point of having the gate.    */
/* ======================================================================== */
#define PW_BLOCKSIZE 128

static float t2_pw_sum_f32(const float *a, int n)
{
    if (n < 8) {
        float res = 0.0f;
        for (int i = 0; i < n; i++) {
            res += a[i];
        }
        return res;
    }
    if (n <= PW_BLOCKSIZE) {
        float r[8];
        for (int k = 0; k < 8; k++) {
            r[k] = a[k];
        }
        int i = 8;
        for (; i < n - (n % 8); i += 8) {
            for (int k = 0; k < 8; k++) {
                r[k] += a[i + k];
            }
        }
        float res = ((r[0] + r[1]) + (r[2] + r[3]))
                    + ((r[4] + r[5]) + (r[6] + r[7]));
        for (; i < n; i++) {
            res += a[i];
        }
        return res;
    }
    int n2 = n / 2;
    n2 -= n2 % 8;
    return t2_pw_sum_f32(a, n2) + t2_pw_sum_f32(a + n2, n - n2);
}

/* ======================================================================== */
/* config                                                                   */
/* ======================================================================== */

void t2_cfg_default(t2_cfg_t *c, bool half_rate)
{
    memset(c, 0, sizeof(*c));
    c->tau2 = T2_TAU2;
    c->warmup_s = T2_WARMUP_S;
    c->row_lo = T2_ROW_LO;
    c->n_rows = T2_N_F0;
    if (half_rate) {
        c->decim = 2;
        c->n2 = T2_N2_HALF;
        c->m2 = T2_M2_HALF;
        c->gap = T2_GAP_HALF;
        c->release = T2_RELEASE_HALF;
        c->a_up = T2_A_UP_HALF;
        c->a_dn = T2_A_DN_HALF;
    } else {
        c->decim = 1;
        c->n2 = T2_N2;
        c->m2 = T2_M2;
        c->gap = T2_GAP;
        c->release = T2_RELEASE;
        c->a_up = T2_A_UP;
        c->a_dn = T2_A_DN;
    }
    if (c->n2 > T2_MAX_N2) {
        c->n2 = T2_MAX_N2;
    }
#if T2_N_EXCL > 0
    {
        static const double ec[] = T2_EXCL_CENTRES;
        static const double et[] = T2_EXCL_TOLS;
        c->n_excl = T2_N_EXCL;
        for (int i = 0; i < T2_N_EXCL && i < T2_MAX_EXCL; i++) {
            c->excl_c[i] = ec[i];
            c->excl_t[i] = et[i];
        }
    }
#endif
}

void t2_cfg_set_thr_milli(t2_cfg_t *c, int milli)
{
    if (milli > 0) {
        c->tau2 = (double)milli / 1000.0;
    }
}

bool t2_cfg_set_band(t2_cfg_t *c, int lo_hz, int hi_hz)
{
    if (lo_hz <= 0 && hi_hz <= 0) {
        return true;                       /* keep the compiled default */
    }
    const double lo = (lo_hz > 0) ? (double)lo_hz : T2_F_LO;
    const double hi = (hi_hz > 0) ? (double)hi_hz : T2_F_HI;
    if (hi <= lo) {
        return false;
    }
    /* The band must land ON the v1 grid, because the rows ARE the v1 tables.
     * Anything outside is refused rather than clamped: a silently clamped
     * band would score rows the operator did not ask for and the trace would
     * not say so. */
    const int r0 = (int)((lo - CFG_F_SEARCH_LO) / CFG_F_STEP + 0.5);
    const int r1 = (int)((hi - CFG_F_SEARCH_LO) / CFG_F_STEP + 0.5);
    if (r0 < 0 || r1 >= CFG_N_F0 || r1 < r0) {
        return false;
    }
    c->row_lo = r0;
    c->n_rows = r1 - r0 + 1;
    return true;
}

bool t2_cfg_set_excl(t2_cfg_t *c, int n, const double *centres,
                     const double *tols)
{
    if (n < 0 || n > T2_MAX_EXCL) {
        return false;
    }
    c->n_excl = n;
    for (int i = 0; i < n; i++) {
        c->excl_c[i] = centres[i];
        c->excl_t[i] = tols[i];
    }
    return true;
}

void t2_state_reset(t2_state_t *st)
{
    memset(st, 0, sizeof(*st));
}

/* ======================================================================== */
/* floor2 + whitening  (Python Tier2._whiten)                               */
/*                                                                          */
/*   if floor2 is None: floor2 = mag                                        */
/*   a      = where(mag > floor2, a_up, a_dn)                               */
/*   floor2 = a*floor2 + (1-a)*mag                                          */
/*   S2     = min(log1p(mag/(floor2 + 1e-9)), sat_log)   <- the NEW floor    */
/*                                                                          */
/* NOTE the absence of everything v1 does here: no energy sum, no flatness   */
/* moments, no gate, no fast path. That is the whole point of the tier - the */
/* fast path is what erases a source that keeps running.                    */
/* ======================================================================== */

static void t2_update_floor(const float *mag, const t2_cfg_t *c,
                            t2_state_t *st, float *S2)
{
    if (!st->have_floor) {
        for (int i = 0; i < CFG_N_BINS; i++) {
            st->floor2[i] = (double)mag[i];
        }
        st->have_floor = true;
    }
    for (int i = 0; i < CFG_N_BINS; i++) {
        const double prev = st->floor2[i];
        const double m = (double)mag[i];
        const double a = (m > prev) ? c->a_up : c->a_dn;
        st->floor2[i] = a * prev + (1.0 - a) * m;
    }
    for (int i = 0; i < CFG_N_BINS; i++) {
        float v = log1pf(mag[i] / (float)(st->floor2[i] + 1e-9));
        if (v > T2_SAT_LOG_F) {
            v = T2_SAT_LOG_F;
        }
        S2[i] = v;
    }
}

/* ======================================================================== */
/* the comb score, over ROWS T2_ROW_LO .. T2_ROW_HI of the v1 tables        */
/*                                                                          */
/* Identical arithmetic to detector.c's score_all, on a slice. The validity  */
/* test is kept even though every row in 200-800 Hz has >= n_harm_min teeth  */
/* (asserted by tests/test_detector_t2.py), so the two scorers stay          */
/* structurally the same function and a future band change cannot silently   */
/* start scoring invalid rows.                                              */
/* ======================================================================== */

static int t2_score_all(const float *S2, float *scores, int row_lo, int n_rows)
{
    int argmax = 0;
    float best = -INFINITY;

    for (int r = 0; r < n_rows; r++) {
        const int j = row_lo + r;
        if (SENTRY_N_TOOTH[j] < CFG_N_HARM_MIN) {
            scores[r] = -1e9f;
            if (scores[r] > best) {
                best = scores[r];
                argmax = r;
            }
            continue;
        }
        const int base = j * TBL_N_HARM;
        float tprod[TBL_N_HARM], gprod[TBL_N_HARM];
        for (int k = 0; k < TBL_N_HARM; k++) {
            const int ti = SENTRY_T_I0[base + k];
            const float tfr = TFR(base + k);
            const float tv = S2[ti] * (1.0f - tfr) + S2[ti + 1] * tfr;
            tprod[k] = tv * TWT(base + k);

            const int gi = SENTRY_G_I0[base + k];
            const float gfr = GFR(base + k);
            const float gv = S2[gi] * (1.0f - gfr) + S2[gi + 1] * gfr;
            gprod[k] = gv * GWT(base + k);
        }
        const float sc = (t2_pw_sum_f32(tprod, TBL_N_HARM)
                          - t2_pw_sum_f32(gprod, TBL_N_HARM))
                         * SENTRY_ZNORM[j];
        scores[r] = sc;
        if (sc > best) {          /* strict >: np.argmax returns the FIRST max */
            best = sc;
            argmax = r;
        }
    }
    return argmax;
}

static int t2_teeth_support(const float *S2, int j)
{
    const int base = j * TBL_N_HARM;
    const int n = SENTRY_N_TOOTH[j];
    int cnt = 0;
    for (int k = 0; k < n; k++) {
        const int ti = SENTRY_T_I0[base + k];
        const float tfr = TFR(base + k);
        const float tv = S2[ti] * (1.0f - tfr) + S2[ti + 1] * tfr;
        if (tv >= CFG_TEETH_LEVEL) {
            cnt++;
        }
    }
    return cnt;
}

/* ======================================================================== */
/* coherence telemetry (D6). MEASURED, NEVER GATED ON.                      */
/*                                                                          */
/*   C(b) = |sum_c X_c(b)|^2 / (n_ch * sum_c |X_c(b)|^2)                    */
/*                                                                          */
/* 1.0 = coherent and time-aligned, ~1/n_ch = independent. With ONE channel  */
/* it is identically 1.0, which is why a mono golden replay carries          */
/* kappa = 1.0 rather than a missing field.                                  */
/*                                                                          */
/* The reference computes C over the whole spectrum and then takes a         */
/* tooth-weighted mean; this evaluates C at the WINNER'S TEETH ONLY - the    */
/* same <= 12 gathers the score already does - because every other bin is    */
/* multiplied by a zero weight. Same number, ~40x less work: 24 bins instead */
/* of four passes over 1025. Teeth above T2_KAPPA_F_MAX carry no weight,     */
/* because an uncompensated inter-bus start offset rotates phases enough up  */
/* there to poison the statistic.                                           */
/* ======================================================================== */

static float t2_kappa(const cf32_t *spectra, int n_ch, int j)
{
    if (n_ch <= 1) {
        return 1.0f;               /* one channel IS perfectly coherent */
    }
    if (spectra == NULL) {
        /* NOT 1.0. The caller has no per-channel spectra to give - the
         * summed-window FFT path never computes them - and 1.0 is the value
         * that means "these four microphones agree perfectly". Reporting a
         * measurement that was not made, as the most emphatic value the
         * statistic can take, is how a diagnostic becomes a lie. NaN says "not
         * measured", and run_device.py already drops NaN before it takes a
         * median. */
        return NAN;
    }
    const int base = j * TBL_N_HARM;
    const int n = SENTRY_N_TOOTH[j];
    float acc = 0.0f, wsum = 0.0f;
    for (int k = 0; k < n; k++) {
        const double f = (double)(k + 1) * (CFG_F_SEARCH_LO
                                            + CFG_F_STEP * (double)j);
        if (f > T2_KAPPA_F_MAX) {
            break;                     /* tooth_f is monotone in k */
        }
        const float w = TWT(base + k);
        if (w <= 0.0f) {
            continue;
        }
        const int i0 = SENTRY_T_I0[base + k];
        const float fr = TFR(base + k);
        float cv[2];
        for (int s = 0; s < 2; s++) {
            const int b = i0 + s;
            float sr = 0.0f, si = 0.0f, p = 0.0f;
            for (int ch = 0; ch < n_ch; ch++) {
                const cf32_t z = spectra[(size_t)ch * CFG_N_BINS + b];
                sr += z.re;
                si += z.im;
                p += z.re * z.re + z.im * z.im;
            }
            const float den = (float)n_ch * p;
            cv[s] = (den > 1e-30f) ? ((sr * sr + si * si) / den) : 0.0f;
        }
        acc += (cv[0] * (1.0f - fr) + cv[1] * fr) * w;
        wsum += w;
    }
    return (wsum > 0.0f) ? (acc / wsum) : NAN;
}

/* ======================================================================== */
/* the M-of-N decision  (Python Tier2._decide_core)                         */
/* ======================================================================== */

static bool t2_excluded(const t2_cfg_t *c, double f0)
{
    for (int i = 0; i < c->n_excl; i++) {
        if (fabs(f0 - c->excl_c[i]) <= c->excl_t[i]) {
            return true;
        }
    }
    return false;
}

/* v1's octave-tolerant continuity, verbatim. An octave match extends the
 * track but HOLDS its frequency, so one argmax slip costs one frame, not two. */
static bool t2_continuous(const t2_cfg_t *c, double f0, double lf,
                          bool *is_octave)
{
    (void)c;
    *is_octave = false;
    if (fabs(f0 - lf) <= fmax(T2_CONT_FRAC * lf, T2_CONT_MIN_HZ)) {
        return true;
    }
    if (fabs(f0 - 2.0 * lf) <= fmax(T2_CONT_FRAC * 2.0 * lf, T2_CONT_MIN_HZ)
        || fabs(f0 - 0.5 * lf) <= fmax(T2_CONT_FRAC * 0.5 * lf,
                                       T2_CONT_MIN_HZ)) {
        *is_octave = true;
        return true;
    }
    return false;
}

static void t2_close_event(t2_state_t *st)
{
    if (st->latched) {
        st->n_events++;
    }
    st->latched = false;
    st->below = 0;
    st->peak2 = 0.0f;
}

static void t2_clear_track(t2_state_t *st, const t2_cfg_t *c)
{
    memset(st->ring, 0, (size_t)c->n2);
    st->ri = 0;
    st->hits = 0;
    st->have_track = false;
    st->track_f0 = 0.0;
    st->track_age = 0;
    st->gap = 0;
}

static void t2_push(t2_state_t *st, const t2_cfg_t *c, int v)
{
    st->hits += v - (int)st->ring[st->ri];
    st->ring[st->ri] = (uint8_t)v;
    st->ri = (st->ri + 1) % c->n2;
    st->track_age++;
}

/* ======================================================================== */
/* the frame                                                                */
/* ======================================================================== */

bool t2_step(const cf32_t *spec, const cf32_t *spectra, int n_ch,
             const t2_cfg_t *c, t2_state_t *st, double t, uint32_t frame,
             t2_work_t *w, t2_rec_t *rec)
{
    /* Half rate: Python increments frame_i FIRST and skips when the remainder
     * is nonzero, so frame 0 always runs. Same order here. */
    if ((st->frame_i++ % (uint32_t)c->decim) != 0u) {
        return false;
    }
    const int64_t c0 = esp_timer_get_time();

    for (int i = 0; i < CFG_N_BINS; i++) {
        w->mag[i] = sqrtf(spec[i].re * spec[i].re + spec[i].im * spec[i].im);
    }
    t2_update_floor(w->mag, c, st, w->S2);
    const int r = t2_score_all(w->S2, w->scores, c->row_lo, c->n_rows);
    const int j = c->row_lo + r;
    const double f02 = CFG_F_SEARCH_LO + CFG_F_STEP * (double)j;
    const float s2 = w->scores[r];

    const bool excluded = t2_excluded(c, f02);
    const bool above = (double)s2 >= c->tau2 && t >= c->warmup_s && !excluded;

    bool hit = false;
    if (above) {
        if (!st->have_track) {
            t2_clear_track(st, c);
            st->have_track = true;
            st->track_f0 = f02;
            hit = true;
        } else {
            bool is_oct = false;
            if (t2_continuous(c, f02, st->track_f0, &is_oct)) {
                hit = true;
                if (!is_oct) {
                    st->track_f0 = f02;
                }
            } else {
                /* a DIFFERENT comb: the old track ends here and a new one
                 * opens at this frequency with an empty ring. */
                t2_close_event(st);
                t2_clear_track(st, c);
                st->have_track = true;
                st->track_f0 = f02;
                hit = true;
            }
        }
    }

    if (st->have_track) {
        t2_push(st, c, hit ? 1 : 0);
        st->gap = hit ? 0 : st->gap + 1;
        if (st->gap > c->gap) {
            t2_close_event(st);
            t2_clear_track(st, c);
        }
    }
    if (hit && s2 > st->peak2) {
        st->peak2 = s2;
    }

    if (st->have_track) {
        if (!st->latched && st->hits >= c->m2) {
            st->latched = true;
            st->t_on = t;
            st->f0_on = st->track_f0;
            st->below = 0;
        } else if (st->latched) {
            if (2 * st->hits >= c->m2) {
                st->below = 0;
            } else if (++st->below >= c->release) {
                t2_close_event(st);
            }
        }
    }

    rec->frame = frame;
    rec->t_s = t;
    rec->score2 = s2;
    rec->f02_hz = f02;
    rec->f02_row = (uint16_t)r;
    rec->teeth2 = (uint16_t)t2_teeth_support(w->S2, j);
    rec->hit = hit ? 1u : 0u;
    rec->fired2 = st->latched ? 1u : 0u;
    rec->excluded = excluded ? 1u : 0u;
    rec->hits = st->hits;
    rec->n2 = c->n2;
    rec->track_age = st->track_age;
    rec->kappa = t2_kappa(spectra, n_ch, j);
    rec->us_t2 = (uint32_t)(esp_timer_get_time() - c0);
    return true;
}

void t2_finish(t2_state_t *st, double t_last)
{
    (void)t_last;
    t2_close_event(st);
}
