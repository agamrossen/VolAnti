/*
 * detector.c - the sealed v1 comb detector, ported from src/detector.py.
 *
 * Ported from the code rather than from any description of it. Where the
 * Python relies on a numpy implementation detail - summation order, a dtype
 * that arises from promotion rather than from intent - that detail is
 * reproduced here deliberately and the reason is in PORTING_NOTES.md. Nothing
 * in this file is allowed to be "close enough".
 *
 * Rejected features have no code path: subharmonic re-anchoring and comb-hold
 * are absent, not disabled. The trace emits reanch = 0 and n_held_bins = 0 as
 * tripwires, and the host comparator fails if either is ever nonzero.
 */

#include "detector.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_timer.h"
#include "dsps_fft2r.h"
#include "dsps_fft4r.h"

/* Gather-table format. 1 is a one-byte index into an exact lookup table, and
 * is not quantisation: the table holds the gather table's own distinct float32
 * values, so the value read back is bit-identical to the float table. See
 * PORTING_NOTES.md sec 6. Build with SENTRY_TABLES_LUT=0 for the float tables. */
#ifndef SENTRY_TABLES_LUT
#define SENTRY_TABLES_LUT 1
#endif

/* Whitening precision. 1 selects float32 log1pf. See update_floor() below and
 * PORTING_NOTES.md sec 4 for why this is a deliberate, measured divergence
 * from numpy's float64 rather than an accident. Build with
 * SENTRY_WHITEN_F32=0 to get the float64 reference path back. */
#ifndef SENTRY_WHITEN_F32
#define SENTRY_WHITEN_F32 1
#endif

#include "generated/tables.h"
#include "generated/window.h"

/* ======================================================================== */
/* numpy's pairwise summation, reproduced exactly.                          */
/*                                                                          */
/* np.sum / np.mean on a contiguous axis do not sum left to right: numpy     */
/* uses an 8-accumulator unrolled block up to PW_BLOCKSIZE = 128 and recurses*/
/* above it. Left-to-right accumulation over the 1025-bin spectrum differs   */
/* from numpy in the 6th significant figure, which is the same order as the  */
/* tightest decision margin in the golden set. So we reproduce the order.    */
/* Validated bit-for-bit against numpy 2.4.6 on the host before porting.     */
/* ======================================================================== */
#define PW_BLOCKSIZE 128

static float pw_sum_f32(const float *a, int n)
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
    return pw_sum_f32(a, n2) + pw_sum_f32(a + n2, n - n2);
}

static double pw_sum_f64(const double *a, int n)
{
    if (n < 8) {
        double res = 0.0;
        for (int i = 0; i < n; i++) {
            res += a[i];
        }
        return res;
    }
    if (n <= PW_BLOCKSIZE) {
        double r[8];
        for (int k = 0; k < 8; k++) {
            r[k] = a[k];
        }
        int i = 8;
        for (; i < n - (n % 8); i += 8) {
            for (int k = 0; k < 8; k++) {
                r[k] += a[i + k];
            }
        }
        double res = ((r[0] + r[1]) + (r[2] + r[3]))
                     + ((r[4] + r[5]) + (r[6] + r[7]));
        for (; i < n; i++) {
            res += a[i];
        }
        return res;
    }
    int n2 = n / 2;
    n2 -= n2 % 8;
    return pw_sum_f64(a, n2) + pw_sum_f64(a + n2, n - n2);
}

/* ======================================================================== */
/* init / reset                                                             */
/* ======================================================================== */

esp_err_t detector_init(void)
{
    /* fft2r: the radix-2 complex transform itself.
     * fft4r: dsps_cplx2real_fc32 reads the radix-4 twiddle table, so its
     *        init is required even though we never call a radix-4 FFT. */
    esp_err_t e = dsps_fft2r_init_fc32(NULL, CFG_N_FFT >> 1);
    if (e != ESP_OK) {
        return e;
    }
    return dsps_fft4r_init_fc32(NULL, CFG_N_FFT >> 1);
}

void detector_state_reset(detector_state_t *st, double thr)
{
    memset(st, 0, sizeof(*st));
    st->thr = thr;
}

/* ======================================================================== */
/* pipeline stage 1 - front_end                                             */
/*                                                                          */
/* Python:  np.fft.rfft(block * self.window)                                */
/*                                                                          */
/* esp-dsp's real FFT differs from numpy's rfft in TWO ways that matter, both*/
/* reconciled here (see PORTING_NOTES.md "FFT"):                            */
/*   ordering  the N real samples are consumed as N/2 COMPLEX values, and    */
/*             dsps_cplx2real_fc32 leaves bins 0..N/2-1 in place with the    */
/*             REAL-VALUED Nyquist bin packed into the imaginary slot of     */
/*             bin 0. numpy returns N/2+1 separate bins. We unpack to the    */
/*             numpy layout so nothing downstream knows the difference.      */
/*   scaling   none. Both are the unnormalised DFT; there is no 1/N.         */
/* Sign convention is irrelevant: only |X| is ever used, and conjugation     */
/* leaves the magnitude unchanged.                                           */
/* ======================================================================== */

float sentry_window_probe(int i)
{
    return (i >= 0 && i < CFG_N_FFT) ? SENTRY_WINDOW[i] : -1.0f;
}

void front_end(const float *block, cf32_t *spec_out, detector_work_t *w)
{
    for (int i = 0; i < CFG_N_FFT; i++) {
        w->fftbuf[i] = block[i] * SENTRY_WINDOW[i];
    }
    dsps_fft2r_fc32(w->fftbuf, CFG_N_FFT >> 1);
    dsps_bit_rev2r_fc32(w->fftbuf, CFG_N_FFT >> 1);
    dsps_cplx2real_fc32(w->fftbuf, CFG_N_FFT >> 1);

    spec_out[0].re = w->fftbuf[0];          /* DC, real */
    spec_out[0].im = 0.0f;
    for (int k = 1; k < CFG_N_BINS - 1; k++) {
        spec_out[k].re = w->fftbuf[2 * k];
        spec_out[k].im = w->fftbuf[2 * k + 1];
    }
    spec_out[CFG_N_BINS - 1].re = w->fftbuf[1];   /* Nyquist, real, packed */
    spec_out[CFG_N_BINS - 1].im = 0.0f;
}

/* ======================================================================== */
/* pipeline stage 2 - combiner. Identity while N_CHANNELS == 1.             */
/*                                                                          */
/* This is where any multi-channel weighting goes:                          */
/*     X_beam[b] = sum_ch W[ch][b] * X[ch][b]     (per-bin complex weights)  */
/* Nothing downstream changes. back_end never learns how many microphones    */
/* there were.                                                              */
/* ======================================================================== */

const cf32_t *combiner(const cf32_t *spectra, int n_ch, detector_work_t *w)
{
    if (n_ch == 1) {
        return spectra;                      /* identity: no copy, no cost */
    }
    for (int b = 0; b < CFG_N_BINS; b++) {
        float sr = 0.0f, si = 0.0f;
        for (int c = 0; c < n_ch; c++) {
            sr += spectra[c * CFG_N_BINS + b].re;
            si += spectra[c * CFG_N_BINS + b].im;
        }
        w->combined[b].re = sr;
        w->combined[b].im = si;
    }
    return w->combined;
}

/* ======================================================================== */
/* the tonality-gated adaptive floor  (Config.floor_mode == "tonality")      */
/*                                                                          */
/* Python _update_floor, "tonality" branch, in order:                        */
/*     prev   = floor                    (the OLD array; Python rebinds)     */
/*     e      = float(mag.sum())         float32 pairwise sum                */
/*     rising = e > gate_ratio * e_slow  BEFORE e_slow is updated            */
/*     r      = mag / (prev + 1e-9)                                          */
/*     flat   = mean(r)^2 / (mean(r*r) + 1e-12)                              */
/*     fast   = rising and flat > flat_hi                                    */
/*     e_slow = a_energy*e_slow + (1-a_energy)*e                             */
/*     a_up   = a_rise_fast if fast else a_rise                              */
/*     a      = where(mag > prev, a_up, a_fall)                              */
/*     floor  = a*prev + (1-a)*mag                                           */
/*     S      = min(log1p(mag/(floor + 1e-9)), sat_log)   <- the NEW floor   */
/*                                                                          */
/* The last line is easy to get wrong: S whitens against the floor AFTER     */
/* this frame's update, while the tonality gate looks at the one BEFORE.     */
/* ======================================================================== */

static void update_floor(const float *mag, detector_state_t *st,
                         detector_work_t *w, float *S,
                         double *flat_out, bool *fast_out, bool *rising_out,
                         double *e_out)
{
    if (!st->have_floor) {
        for (int i = 0; i < CFG_N_BINS; i++) {
            st->floor_[i] = (double)mag[i];
        }
        st->have_floor = true;
    }

    /* mag.sum() is a float32 pairwise sum, then widened by float(). */
    double e = (double)pw_sum_f32(mag, CFG_N_BINS);
    if (!st->have_e_slow) {
        st->e_slow = e;
        st->have_e_slow = true;
    }
    bool rising = e > CFG_GATE_RATIO * st->e_slow;

    for (int i = 0; i < CFG_N_BINS; i++) {
        double ri = (double)mag[i] / (st->floor_[i] + 1e-9);
        w->r[i] = ri;
        w->r2[i] = ri * ri;
    }
    double m1 = pw_sum_f64(w->r, CFG_N_BINS) / CFG_N_BINS;
    double m2 = pw_sum_f64(w->r2, CFG_N_BINS) / CFG_N_BINS;
    double flat = (m1 * m1) / (m2 + 1e-12);
    bool fast = rising && (flat > CFG_FLAT_HI);

    st->e_slow = CFG_A_ENERGY * st->e_slow + (1.0 - CFG_A_ENERGY) * e;

    const double a_up = fast ? CFG_A_RISE_FAST : CFG_A_RISE;
    for (int i = 0; i < CFG_N_BINS; i++) {
        double prev = st->floor_[i];
        double m = (double)mag[i];
        double a = (m > prev) ? a_up : CFG_A_FALL;
        st->floor_[i] = a * prev + (1.0 - a) * m;
    }

    /* Whitening precision - the one deliberate numerical divergence.
     *
     * numpy computes this log1p in float64 (the floor is float64 by promotion,
     * see PORTING_NOTES.md sec 0). Reproducing that on a core with no double
     * FPU costs 21.8 ms/frame of a 32 ms budget - 1025 soft-float64 log1p
     * calls at ~5100 cycles each - which is the difference between a detector
     * that cannot run in real time and one that can.
     *
     * float32 log1pf is within 1 ulp of float32 on S (~6e-8), which is ~150x
     * below the 1e-5 decision tolerance measured for this golden set and
     * ~3000x below the tightest margin in it (1.78e-4 at marginal_pos f74).
     * Measured rather than argued: this path was run against all the golden
     * vectors with zero decision mismatches and identical chain trajectories
     * before it was committed. Cost: 21.8 -> 7.5 ms/frame. */
#if SENTRY_WHITEN_F32
    for (int i = 0; i < CFG_N_BINS; i++) {
        float v = log1pf(mag[i] / (float)(st->floor_[i] + 1e-9));
        if (v > CFG_SAT_LOG_F) {
            v = CFG_SAT_LOG_F;
        }
        S[i] = v;
    }
#else
    for (int i = 0; i < CFG_N_BINS; i++) {
        double v = log1p((double)mag[i] / (st->floor_[i] + 1e-9));
        if (v > CFG_SAT_LOG) {
            v = CFG_SAT_LOG;
        }
        S[i] = (float)v;
    }
#endif

    *flat_out = flat;
    *fast_out = fast;
    *rising_out = rising;
    *e_out = e;
}

/* ======================================================================== */
/* comb score                                                               */
/*                                                                          */
/* Python:                                                                  */
/*   tv = S[t_i0]*(1-t_fr) + S[t_i0+1]*t_fr        (float32)                 */
/*   gv = S[g_i0]*(1-g_fr) + S[g_i0+1]*g_fr                                  */
/*   scores = ((tv*tw).sum(1) - (gv*gw).sum(1)) * znorm                      */
/*   scores[~valid] = -1e9                                                   */
/*                                                                          */
/* The 12-term row sums go through pw_sum_f32 for the same reason as the     */
/* spectrum sums: numpy's 8-accumulator block is not left-to-right even at   */
/* n = 12 (it is r0..r7 tree-combined, then a8..a11 appended in order).      */
/* ======================================================================== */

#if SENTRY_TABLES_LUT
/* The four LUTs total 402 floats (1.6 KB) and are touched on every one of the
 * 46,344 gathers per frame, so they stay resident in cache. The index arrays
 * are what streams from flash, at one byte per entry instead of four. */
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

static int score_all(const float *S, float *scores)
{
    int argmax = 0;
    float best = -INFINITY;

    for (int j = 0; j < CFG_N_F0; j++) {
        /* valid == (n_tooth >= n_harm_min), asserted by the generator */
        if (SENTRY_N_TOOTH[j] < CFG_N_HARM_MIN) {
            scores[j] = -1e9f;
            if (scores[j] > best) {
                best = scores[j];
                argmax = j;
            }
            continue;
        }
        const int base = j * TBL_N_HARM;
        float tprod[TBL_N_HARM], gprod[TBL_N_HARM];
        for (int k = 0; k < TBL_N_HARM; k++) {
            const int ti = SENTRY_T_I0[base + k];
            const float tfr = TFR(base + k);
            const float tv = S[ti] * (1.0f - tfr) + S[ti + 1] * tfr;
            tprod[k] = tv * TWT(base + k);

            const int gi = SENTRY_G_I0[base + k];
            const float gfr = GFR(base + k);
            const float gv = S[gi] * (1.0f - gfr) + S[gi + 1] * gfr;
            gprod[k] = gv * GWT(base + k);
        }
        const float sc = (pw_sum_f32(tprod, TBL_N_HARM)
                          - pw_sum_f32(gprod, TBL_N_HARM)) * SENTRY_ZNORM[j];
        scores[j] = sc;
        if (sc > best) {          /* strict >: np.argmax returns the FIRST max */
            best = sc;
            argmax = j;
        }
    }
    return argmax;
}

/* _teeth_support: interpolated tooth values at or above teeth_level, counted
 * only over the teeth that are actually below f_max_harm. tooth_ok is a prefix
 * mask in k (tooth_f = k*f0 is monotone), so n_tooth reproduces it exactly. */
static int teeth_support(const float *S, int j)
{
    const int base = j * TBL_N_HARM;
    const int n = SENTRY_N_TOOTH[j];
    int cnt = 0;
    for (int k = 0; k < n; k++) {
        const int ti = SENTRY_T_I0[base + k];
        const float tfr = TFR(base + k);
        const float tv = S[ti] * (1.0f - tfr) + S[ti + 1] * tfr;
        if (tv >= CFG_TEETH_LEVEL) {
            cnt++;
        }
    }
    return cnt;
}

/* ---- how much of the comb is pinned at the ceiling -----------------------
 *
 * S is clipped at CFG_SAT_LOG (2.5), reached at about 11.2x the local floor.
 * At high level teeth and gaps both saturate and the score, which is teeth
 * minus gaps, collapses. Measured on one bed as P(d) falling from 12/14 at
 * 0 dB to 8/14 at +24 dB, which is why getting closer is not guaranteed to
 * help - the closest real recording fires at 1.50 and is silent at 1.70.
 *
 * Counting both sides is the point: teeth alone rising is signal, teeth and
 * gaps rising together is the collapse. Saturation itself is unchanged; this
 * measures whether a close source actually reaches it, so the fix can be
 * priced rather than guessed at.
 *
 * Observational. Reads S and the same gather tables score_all() just used,
 * writes two record fields, and is read by nothing in the detector. */
static void sat_count(const float *S, int j, uint16_t *t_sat, uint16_t *g_sat)
{
    const int base = j * TBL_N_HARM;
    const int n = SENTRY_N_TOOTH[j];
    int ts = 0, gs = 0;
    for (int k = 0; k < n; k++) {
        const int ti = SENTRY_T_I0[base + k];
        const float tfr = TFR(base + k);
        const float tv = S[ti] * (1.0f - tfr) + S[ti + 1] * tfr;
        if (tv >= CFG_SAT_LOG_F) { ts++; }

        const int gi = SENTRY_G_I0[base + k];
        const float gfr = GFR(base + k);
        const float gv = S[gi] * (1.0f - gfr) + S[gi + 1] * gfr;
        if (gv >= CFG_SAT_LOG_F) { gs++; }
    }
    *t_sat = (uint16_t)ts;
    *g_sat = (uint16_t)gs;
}

/* ---- the cluster count: logged, never gated ------------------------------
 *
 * Local maxima of the score curve within +-25% of the argmax and above 70% of
 * the peak. A quadcopter should read 2 to 4 and a single tone 1, and nothing
 * has yet measured whether it does - which is why it is a record field and
 * not a term in a decision.
 *
 * Observational: reads the score curve score_all() just wrote, writes one
 * record field, and is read by nothing in the detector.
 *
 * Computed only with the flag set, so the shipped image's frame budget cannot
 * move on account of a number nobody is using yet. The window is 0.5*f0
 * candidates wide - 200 at f0 = 400 Hz, 1000 at the top of the grid - against
 * the 1931 the argmax scan already walks. That is a predicted small cost, not
 * a measured one.
 *
 * The cut is float64 in both implementations on purpose: 0.70f*x in float32
 * and float32(0.70*x) in float64 differ by an ulp, and an ulp at the 70% line
 * is a cluster count that disagrees with the reference. The neighbour
 * comparisons stay float32 against float32, which is exact either way. The
 * plateau rule is >= on the left and > on the right, so a flat top counts
 * once and counts at its first sample - numpy's argmax rule. */
static uint16_t cluster_count(const float *scores, int b)
{
    const float peak = scores[b];
    if (peak <= 0.0f) {
        return 0;
    }
    const double f0 = CFG_F_SEARCH_LO + (double)b * CFG_F_STEP;
    int lo = (int)ceil((f0 * (1.0 - CFG_TRK_V2_CLUSTER_SPAN)
                        - CFG_F_SEARCH_LO) / CFG_F_STEP);
    int hi = (int)floor((f0 * (1.0 + CFG_TRK_V2_CLUSTER_SPAN)
                         - CFG_F_SEARCH_LO) / CFG_F_STEP);
    if (lo < 1) { lo = 1; }
    if (hi > CFG_N_F0 - 2) { hi = CFG_N_F0 - 2; }
    const double cut = CFG_TRK_V2_CLUSTER_FRAC * (double)peak;
    int n = 0;
    for (int j = lo; j <= hi; j++) {
        const float s = scores[j];
        if ((double)s >= cut && s >= scores[j - 1] && s > scores[j + 1]) {
            n++;
        }
    }
    return (uint16_t)n;
}

/* ======================================================================== */
/* tracker - a direct port of TrackerState.step                             */
/* ======================================================================== */

static double band_threshold(double f0, double thr)
{
    if (CFG_F_PRIO_LO <= f0 && f0 <= CFG_F_PRIO_HI) {
        return thr + CFG_THR_OFF_PRIO;
    }
    return thr + CFG_THR_OFF_GEN;
}

static int cmp_double(const void *a, const void *b)
{
    const double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

/* np.median: odd n -> middle element; even n -> mean of the two middle. */
static double median_inplace(double *a, int n)
{
    qsort(a, n, sizeof(double), cmp_double);
    if (n & 1) {
        return a[n / 2];
    }
    return (a[n / 2 - 1] + a[n / 2]) / 2.0;
}

/* TrackerState._jitter_ok, on the raw argmax. The accepted chain is capped at
 * cont_frac per frame by construction and carries no information, which is why
 * this must be the raw series. */
static bool jitter_ok(const detector_state_t *st, double *scratch)
{
    /* Tracker v2: one gate for "same source", not two. The fire-time jitter
     * gate is folded into continuity - a 25% cluster window against the chain
     * centre already refuses a frame that is not the same source - so nothing
     * here may reject. Everything below is the sealed body, character for
     * character, unreachable with the flag set. */
    if (st->trk_v2) {
        return true;
    }
    /* CFG_MAX_JITTER = 0.004 was fitted to a synthetic f0 that is a straight
     * line plus 2 Hz of jitter. The three real rig recordings this project
     * owns measure a median |df0|/f0 of 0.0049, 0.0163 and 0.3418 per frame,
     * so the sealed bound rejects every one of them. With nf_gate clear this
     * expression is CFG_MAX_JITTER and nothing has moved. */
    const double jmax = st->nf_gate ? CFG_MAX_JITTER_FIELD : CFG_MAX_JITTER;
    if (jmax >= 1.0 || st->chain_len < 4) {
        return true;
    }
    const int n = st->chain_len - 1;
    for (int i = 0; i < n; i++) {
        double denom = st->chain_raw[i];
        if (denom < 1e-9) {
            denom = 1e-9;                     /* np.maximum(a[:-1], 1e-9) */
        }
        scratch[i] = fabs(st->chain_raw[i + 1] - st->chain_raw[i]) / denom;
    }
    return median_inplace(scratch, n) <= jmax;
}

/* ---- normalise a frame's argmax into the chain's family ------------------
 *
 * Returns true if some ratio put f0 inside the cluster window, and writes the
 * normalised argmax and the ratio it matched at. Once f0 has been divided by
 * that ratio it is inside the tolerance by construction, so normalisation and
 * continuity are one test and not two - the same "one gate for same source"
 * rule applied to the frequency axis as well as to the jitter one.
 *
 * The ratio order is the measured one, {2, 1/2, 3, 1/3}, the same table and
 * the same first-hit-wins break as the family rule below, because a
 * reordering would be a different rule wearing a measured rule's name. Under
 * the 25% window it is equivalent to the order {2, 3, 1/2, 1/3}: the only
 * overlapping windows are 2 with 3 (2.25c to 2.5c) and 1/2 with 1/3 (0.375c
 * to 0.4167c), and 2 precedes 3 and 1/2 precedes 1/3 in both orders. The host
 * test proves that by exhaustion rather than leaving it as this paragraph.
 *
 * Mirrors family_v2() in src/detector.py line for line. */
static bool family_v2(double centre, double f0, double *f0n, double *ratio)
{
    static const double FAMILY_V2[] = {2.0, 0.5, 3.0, 1.0 / 3.0};
    const double tol = fmax(CFG_TRK_V2_CONT_FRAC * centre, CFG_CONT_MIN_HZ);
    if (fabs(f0 - centre) <= tol) {
        *f0n = f0;
        *ratio = 1.0;
        return true;
    }
    for (unsigned r = 0; r < sizeof(FAMILY_V2) / sizeof(FAMILY_V2[0]); r++) {
        const double tgt = centre * FAMILY_V2[r];
        if (fabs(f0 - tgt) <= fmax(CFG_TRK_V2_CONT_FRAC * tgt,
                                   CFG_CONT_MIN_HZ)) {
            *f0n = f0 / FAMILY_V2[r];
            *ratio = FAMILY_V2[r];
            return true;
        }
    }
    *f0n = f0;
    *ratio = 1.0;
    return false;
}

static void tracker_step(detector_state_t *st, double t, double f0,
                         float score, int teeth, double f0_raw,
                         double *jscratch, frame_rec_t *rec)
{
    /* `teeth` is read by the veto below and, when CFG_MIN_TEETH > 0, by the
     * harmonic-extent gate. It is no longer unused. */

    /* ---- normalise into the family before any gate ----------------------
     * The sealed chain evaluates the band floor before the family rule, so a
     * shaft-line frame of an in-band source is thrown away before continuity
     * is ever consulted. The 4 m rig spends 32-36% of its loud frames below
     * 200 Hz on the shaft family, and every one of them is a frame the sealed
     * tracker cannot use. `f0n` is `f0` exactly while the flag is clear, and
     * every use of it below is inside a flag branch. */
    double f0n = f0, famr = 1.0;
    bool matched_v2 = false;
    if (st->trk_v2 && st->have_centre) {
        matched_v2 = family_v2(st->centre, f0, &f0n, &famr);
    }

    const double thr_eff = band_threshold(f0, st->thr);
    bool ok = ((double)score > thr_eff
               && t >= CFG_T_WARMUP_S
               && CFG_F_ALERT_LO <= f0 && f0 <= CFG_F_ALERT_HI);
    if (st->trk_v2) {
        /* The same three comparisons on the normalised f0, written out
         * rather than substituted into the sealed line above so that the
         * sealed expression survives character for character and flag-off is
         * a property of the text rather than of an argument about
         * equivalence. `thr_eff` below is band_threshold(f0) and this is
         * band_threshold(f0n); they are equal while both band offsets are
         * 0.0, which they are, and the attribution that reads thr_eff is
         * observational either way. */
        ok = ((double)score > band_threshold(f0n, st->thr)
              && t >= CFG_T_WARMUP_S
              && CFG_F_ALERT_LO <= f0n && f0n <= CFG_F_ALERT_HI);
    }

    /* ---- attribute the rejection, do not change it ----------------------
     * `rej` is derived from the same comparisons the sealed expression above
     * already made, re-evaluated read-only. Splitting that expression into
     * sequential ifs would have been equivalent too, but the standard for a
     * sealed expression is that it survives character for character, so that
     * bit-identity is a property of the text rather than of an argument about
     * short-circuit evaluation. Nothing below assigns to `ok`. */
    uint8_t rej = DET_REJ_NONE;
    if (!ok) {
        rej = ((double)score <= thr_eff) ? DET_REJ_THR
            : (t < CFG_T_WARMUP_S)       ? DET_REJ_WARMUP
            :                              DET_REJ_BAND;
    }
#if CFG_MIN_TEETH > 0
    if (ok) {
        ok = teeth >= CFG_MIN_TEETH;
    }
#endif
    /* ---- the voice and struck-note veto ---------------------------------
     * Two terms, each answering a different confuser.
     *
     * The f0 floor answers the voice. A held vowel binds at 203-238 Hz and
     * low speech below that, while the lowest drone event measured anywhere
     * in the sealed corpus is a 2-blade at 266 Hz. It also removes 80% of
     * livestock, which binds on a harmonic at 207-259 Hz and pays 86% of the
     * weighted false-alarm budget.
     *
     * The teeth floor answers the piano. A piano string is stiff, so its
     * partials sit at n*f0*sqrt(1+B*n^2) with B around 2e-4 to 8e-4 and walk
     * off the exact multiples this comb expects. Piano events measure 8/11/12
     * teeth at p10/median/p90, where every drone class measures 12/12/12. The
     * same gate was measured earlier and rejected as showing no separation -
     * but that was against livestock, which saturates at 12/12 just as drones
     * do. There was no piano in the corpus to test it against.
     *
     * Recalibrated and paired over 486 positives: 0 lost, 92 gained, McNemar
     * p < 0.0001. A strict improvement rather than a trade, because the false
     * alarms it removes buy threshold headroom back.
     *
     * Three terms were built and rejected first, and the reasons are worth
     * keeping: a high-band ratio (does not separate on synthetic drones,
     * which carry no broadband propeller noise), pitch jitter (hover measures
     * a median 41.5% against the piano's 0.00), and a gap/decay level rule
     * (piano events fire during the attack, while the level is still rising).
     * See docs/VETO_CAL.md. */
    if (ok && st->veto_voice) {
        ok = (f0 >= CFG_VETO_F0_MIN_HZ && teeth >= CFG_VETO_MIN_TEETH);
        if (st->trk_v2) {
            /* The 250 Hz floor asked of the normalised f0. Same reason as
             * the band floor - a 158 Hz shaft line of a 474 Hz blade family
             * is a 474 Hz frame - and it costs the veto nothing, because a
             * held vowel binding at 203-238 Hz has no family above it to
             * normalise into. */
            ok = (f0n >= CFG_VETO_F0_MIN_HZ && teeth >= CFG_VETO_MIN_TEETH);
        }
        if (!ok) { rej = DET_REJ_VETO; }        /* observational, read-only */
    }

    /* ---- the near-field broadband gate ----------------------------------
     * A propeller is a broadband noise source and a musical instrument is
     * not: measured as R = 10log10(E[3.2-8k]/E[125-1000]), a real rig reads
     * -4.8 to +0.6 dB and music -25.9 to -28.7. Asked only of a loud
     * candidate, because at 14 m the same rig reads -33.4 - air takes 8 kHz
     * long before it takes 500 Hz, so a hard R gate would trade the whole
     * standoff range for a quiet room. Below CFG_NF_SCORE_HI the candidate is
     * too faint to be a near-field confuser and is let through unasked. See
     * the block in detector.h for the full measurement. */
    if (ok && st->nf_gate && (double)score >= CFG_NF_SCORE_HI
        && st->r_db < CFG_NF_R_MIN_DB) {
        ok = false;
        rej = DET_REJ_NF;                       /* observational, read-only */
    }

    bool is_oct = false;
    /* The ratio the match was made at: 1.0 unless the family rule is on and
     * it matched on a ratio, which is exactly when the jitter gate below
     * needs to divide by it. */
    double ratio = 1.0;
    /* The sealed continuity block, including the family rule. Unreachable
     * with trk_v2 set, and not because a flag guards it: in v2 `last_f0` is
     * never assigned, so `have_last_f0` is false on every frame. Nothing in
     * it was touched. */
    if (ok && st->have_last_f0) {
        const double lf = st->last_f0;
        const double tol1 = fmax(CFG_CONT_FRAC * lf, CFG_CONT_MIN_HZ);
        if (fabs(f0 - lf) <= tol1) {
            /* in tolerance of last_f0 */
        } else if (st->trk_family) {
            /* ---- the family rule ----------------------------------------
             * {2, 1/2, 3, 1/3}, scanned in that order, first hit wins. The
             * order is load-bearing because the loop breaks, and it is the
             * order the offline variant study measured; a reordering here
             * would be a different variant wearing the measured one's name.
             *
             * The 3 and the 1/3 are the threat's physics rather than a fitted
             * parameter: a three-blade rotor's shaft line and its blade-pass
             * harmonic are a factor of three apart, and the 4 m rig
             * measurement shows the argmax alternating between 158-190 Hz and
             * 468-571 Hz, a ratio of almost exactly 3.
             *
             * This branch is unreachable with the flag off, which is what
             * makes flag-off bit-identical by construction rather than by an
             * argument about floating point: the `else if` below is the
             * original expression, character for character. */
            static const double FAMILY[] = {2.0, 0.5, 3.0, 1.0 / 3.0};
            for (unsigned r = 0; r < sizeof(FAMILY) / sizeof(FAMILY[0]); r++) {
                const double tgt = lf * FAMILY[r];
                if (fabs(f0 - tgt) <= fmax(CFG_CONT_FRAC * tgt,
                                           CFG_CONT_MIN_HZ)) {
                    is_oct = true;
                    ratio = FAMILY[r];
                    break;
                }
            }
            if (!is_oct) {
                ok = false;
                rej = DET_REJ_CONT;             /* observational, read-only */
            }
        } else if (fabs(f0 - 2.0 * lf)
                   <= fmax(CFG_CONT_FRAC * 2.0 * lf, CFG_CONT_MIN_HZ)
                   || fabs(f0 - 0.5 * lf)
                   <= fmax(CFG_CONT_FRAC * 0.5 * lf, CFG_CONT_MIN_HZ)) {
            is_oct = true;      /* octave match: last_f0 is HELD, not moved */
        } else {
            ok = false;
            rej = DET_REJ_CONT;                 /* observational, read-only */
        }
    }

    /* ---- tracker v2 continuity: a cluster tolerance ---------------------
     * 25% of the chain centre, which is an exponential mean of the accepted
     * normalised f0 with tau = 1 s rather than the last accepted value. A
     * chord change by a fourth (1.33) or a fifth (1.5) is outside that
     * window; a four-motor spread of 20% is inside it. `matched_v2` is
     * already the verdict - family_v2() returned true if and only if some
     * ratio put f0 inside the window - so this is one test, not two. */
    if (st->trk_v2 && ok && st->have_centre && !matched_v2) {
        ok = false;
        rej = DET_REJ_CONT;                     /* observational, read-only */
    }

    if (ok) {
        st->count += 1;
        if (st->trk_v2) {
            /* The chain centre: seeded by the first accepted frame, then an
             * exponential mean at tau = 1 s. It replaces last_f0, which is
             * deliberately left unset so the sealed block above is
             * unreachable rather than merely unentered. */
            st->centre = st->have_centre
                ? (CFG_TRK_V2_ALPHA * st->centre
                   + (1.0 - CFG_TRK_V2_ALPHA) * f0n)
                : f0n;
            st->have_centre = true;
        } else if (!is_oct) {
            st->last_f0 = f0;
            st->have_last_f0 = true;
        }
        if (st->chain_len >= SENTRY_MAX_CHAIN) {
            st->overflow = true;
        } else {
            /* v2 stores the normalised f0, because last_f0 is never assigned
             * with the flag set and the family value is the one the event's
             * median is about. */
            st->chain_f0s[st->chain_len] = st->trk_v2 ? f0n : st->last_f0;
            /* The per-frame jitter, logged and never gated. O(1) against the
             * previous accepted normalised argmax; a median over the chain
             * would be O(n log n) every frame. Removing a gate should not also
             * remove the number that would say whether removing it was
             * right. */
            if (st->trk_v2) {
                if (st->chain_len > 0) {
                    double prev = st->chain_raw[st->chain_len - 1];
                    if (prev < 1e-9) { prev = 1e-9; }
                    rec->jitter =
                        (float)(fabs(f0_raw / famr - prev) / prev);
                } else {
                    rec->jitter = 0.0f;   /* first frame of a chain */
                }
            }
            /* The jitter gate sees the family-normalised argmax, and only
             * when the family rule is on.
             *
             * The sealed tracker appends the raw argmax unnormalised, and
             * that is the baseline rather than an oversight to fix quietly.
             * On a ratio hop the raw value triples, the median relative step
             * goes to ~100%, and the jitter gate refuses a chain the
             * continuity rule had just accepted. So normalising is not an
             * improvement that could ship on its own - it is the other half
             * of the family rule and never ships alone. Relaxing the jitter
             * gate by itself, recalibrated, takes hover 0.585 -> 0.230, 125
             * losses against 1 gain, p < 0.0001.
             *
             * With the flag off `ratio` is 1.0 and this stores f0_raw, the
             * same double the sealed tracker stored. */
            st->chain_raw[st->chain_len] = st->trk_v2
                ? (f0_raw / famr)
                : ((st->trk_family && ratio != 1.0) ? (f0_raw / ratio) : f0_raw);
            st->chain_len++;
        }
        /* Same condition as the sealed `&&` chain: jitter_ok() is still
         * called if and only if count >= NEED and !fired, so the short-circuit
         * behaviour and the call count are unchanged. The only addition is
         * recording that it refused. */
        if (st->count >= CFG_TRACK_NEED && !st->fired) {
            if (jitter_ok(st, jscratch)) {
                st->fired = true;
                st->t_on = t;
            } else {
                rec->jit_blocked = 1;           /* observational, read-only */
            }
        }
    } else {
        if (st->trk_v2) {
            /* The penalty is one. drift/frame = p - (1-p)*MISS, so the cliff
             * moves from p > 2/3 to p > 1/2. At the p = 0.68 the field
             * measured, MISS = 2 needs 150 frames (4.8 s) to reach 6 and
             * MISS = 1 needs 17 (0.54 s). */
            st->count -= CFG_TRK_V2_TRACK_MISS;
        } else {
            st->count -= CFG_TRACK_MISS;
        }
        if (st->count < 0) {
            st->count = 0;
        }
        if (st->count == 0) {
            if (st->fired) {
                st->n_events++;
            }
            st->fired = false;
            st->have_last_f0 = false;
            st->have_centre = false;
            st->chain_len = 0;
        }
    }

    rec->reject_reason = ok ? DET_REJ_NONE : rej;
    rec->cont_accepted = ok ? 1 : 0;
    rec->is_octave = is_oct ? 1 : 0;
    rec->chain = st->count;
    rec->fired = st->fired ? 1 : 0;
    /* track_frames computes above_thr independently of warmup/band/continuity */
    rec->above_thr = ((double)score > band_threshold(f0, st->thr)) ? 1 : 0;
}

void detector_finish(detector_state_t *st, double t_last)
{
    (void)t_last;
    if (st->fired) {
        st->n_events++;
    }
}

/* ======================================================================== */
/* pipeline stage 3 - back_end                                              */
/* ======================================================================== */

void back_end(const cf32_t *spec, detector_state_t *st, double t,
              uint32_t frame, detector_work_t *w, frame_rec_t *rec)
{
    /* np.abs(spec).astype(np.float32) */
    const int64_t z0 = esp_timer_get_time();
    for (int i = 0; i < CFG_N_BINS; i++) {
        w->mag[i] = sqrtf(spec[i].re * spec[i].re + spec[i].im * spec[i].im);
    }

    double flat, e;
    bool fast, rising;
    const int64_t z1 = esp_timer_get_time();
    update_floor(w->mag, st, w, w->S, &flat, &fast, &rising, &e);

    const int64_t z2 = esp_timer_get_time();
    const int b = score_all(w->S, w->scores);
    const int64_t z3 = esp_timer_get_time();

    /* re-anchoring is rejected: j == b unconditionally, there is no other path */
    const int j = b;
    const double f0 = CFG_F_SEARCH_LO + (double)j * CFG_F_STEP;

    rec->frame = frame;
    rec->t_s = t;
    rec->score = w->scores[b];
    rec->f0_bin = (uint16_t)j;
    rec->f0_hz = f0;
    rec->f0_raw_hz = f0;            /* argmax before re-anchoring == argmax */
    rec->teeth = (uint16_t)teeth_support(w->S, j);
    rec->floor_fast = fast ? 1 : 0;
    rec->reanch = 0;                /* tripwire */
    rec->n_held_bins = 0;           /* tripwire */
    /* The caller's frame_rec_t is an uninitialised stack struct, so every one
     * of these is written unconditionally every frame; jit_blocked in
     * particular is only ever set later, never cleared. */
    rec->jit_blocked = 0;
    rec->reject_reason = DET_REJ_NONE;
    /* Observational. Written unconditionally so a stale stack value can never
     * be read as a cluster; the scan itself is flag-gated. */
    rec->jitter = 0.0f;
    rec->cluster_n = st->trk_v2 ? cluster_count(w->scores, b) : 0;
    sat_count(w->S, j, &rec->sat_teeth, &rec->sat_gaps);
    rec->us_mag = (uint32_t)(z1 - z0);
    rec->us_floor = (uint32_t)(z2 - z1);
    rec->us_score = (uint32_t)(z3 - z2);
    rec->flat = flat;
    rec->e = e;
    rec->rising = rising ? 1 : 0;

    /* The high-band ratio, from the magnitude spectrum already in hand. Two
     * band sums, ~730 multiply-adds, computed unconditionally so that the test
     * page can report R whether or not the gate is armed - a number an
     * operator cannot see is a number nobody can check in the field. */
    {
        /* float32, and the divergence from the reference is deliberate.
         * src/detector.py accumulates these two sums in float64; doing the
         * same here costs 790 us of a 32 ms hop, measured, because the
         * ESP32-S3 has no double-precision unit and every one of these ~730
         * multiply-adds is software-emulated. In float32 they are hardware.
         *
         * It is affordable here and would not be in the score, because this
         * is a ratio of two large sums compared against a threshold 10 dB
         * away from anything the field produces: a real rotor reads -5 to +13
         * and music -26 to -46, while float32 carries ~7 significant digits
         * over a 700-term sum of squares. The tracker parity test feeds both
         * implementations the same r_db, so the decision path is unaffected;
         * what diverges is the last digits of a number whose gate is a cliff
         * at -15. Same rule as the float32 whitening: say so, measure it, and
         * re-prove the gate. */
        float lo = 0.0f, hi = 0.0f;
        for (int i = CFG_R_LO_B0; i < CFG_R_LO_B1; i++) {
            lo += w->mag[i] * w->mag[i];
        }
        for (int i = CFG_R_HI_B0; i < CFG_N_BINS; i++) {
            hi += w->mag[i] * w->mag[i];
        }
        st->r_db = 10.0 * log10(((double)hi + 1e-20) / ((double)lo + 1e-20));
    }

    tracker_step(st, t, f0, rec->score, rec->teeth, rec->f0_raw_hz,
                 w->jitter, rec);
}

/* ======================================================================== */
/* forensic probes                                                          */
/* ======================================================================== */

uint32_t window_checksum(const float *block, int n)
{
    uint32_t h = 2166136261u;
    const uint8_t *p = (const uint8_t *)block;
    for (int i = 0; i < n * (int)sizeof(float); i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

void detector_probe(const detector_work_t *w, const detector_state_t *st,
                    const frame_rec_t *rec, probe_rec_t *p)
{
    /* Eight bins spread across the priority band and its harmonics, plus DC
     * and Nyquist, so a scaling or ordering error in the FFT shows up as a
     * pattern rather than a single number. */
    static const uint16_t PROBE[8] = {0, 26, 51, 64, 128, 256, 512, 1024};
    p->frame = rec->frame;
    for (int i = 0; i < 8; i++) {
        const int b = PROBE[i];
        p->probe_bin[i] = (uint16_t)b;
        p->probe_mag[i] = w->mag[b];
        p->probe_floor[i] = st->floor_[b];
        p->probe_S[i] = w->S[b];
    }
    p->e = rec->e;
    p->rising = rec->rising;
    p->e_slow = st->e_slow;
    p->flat = rec->flat;
    p->fast = rec->floor_fast;
    p->argmax_bin = rec->f0_bin;
    p->argmax_score = rec->score;
}
