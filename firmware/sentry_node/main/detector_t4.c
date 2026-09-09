/*
 * detector_t4.c - Tier-4, the slow comb. Port of src/detector_t4.py.
 *
 * PORTING NOTES, the ones that could have gone wrong:
 *
 * 1. THE TOOTH BINS ARE NOT COMPUTED HERE. T4_TOOTH is generated, rounded on
 *    the host in float64. A device that rounded 6*127/7.8125 differently from
 *    numpy would score a different comb and nothing would crash.
 *
 * 2. THE MEDIAN IS numpy's. np.median of an EVEN-length window is the MEAN of
 *    the two middle elements, and T4_MED_N is 38. Getting this wrong shifts
 *    every prominence by a fraction of a dB in the same direction, which is
 *    exactly the kind of bias a threshold absorbs and a parity gate does not.
 *    Same note as detector_t3.c; same fix.
 *
 * 3. THE ACCUMULATOR IS A DELIBERATE APPROXIMATION and the only one. The
 *    reference offers an exact median over all 64 periodograms; that is 256 KB
 *    and cannot exist on this device. Both are implemented in the reference and
 *    measured against each other - 8.1 dB of separation against 8.3 - so this
 *    is a priced trade, not a shortcut. The device runs `median_of_means` and
 *    the reference must be run in that mode to compare.
 *
 * 4. THE EDGES REFLECT. A truncated local-median window would take its median
 *    from a different number of bins near DC than everywhere else, and the
 *    whole claim of this tier is that one number means the same thing at every
 *    frequency.
 */
#include "detector_t4.h"

#include "ui_config.h"   /* T4_SLICE_PHASE, T4_SLICE_PHASE_MOD */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_timer.h"

/* ---------------------------------------------------------------- config */

t4_cfg_t t4_default_cfg(void)
{
    t4_cfg_t c;
    memset(&c, 0, sizeof(c));
    c.enabled          = (T4_ENABLED_DEFAULT != 0);
    c.tau4             = T4_TAU4;
    c.n4               = T4_N4;
    c.m4               = T4_M4;
    c.release_updates  = T4_RELEASE_UPDATES;
    c.warmup_s         = T4_WARMUP_S;
    c.cont_frac        = T4_CONT_FRAC;
    c.cont_min_hz      = T4_CONT_MIN_HZ;
    c.freeze_tail_s    = T4_FREEZE_TAIL_S;
    c.veto_voice       = false;          /* set from settings, as v1's is */
    c.veto_f0_min_hz   = CFG_VETO_F0_MIN_HZ;
    c.n_excl           = 0;
    return c;
}

void t4_reset(t4_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->track_f0 = -1.0;
    st->frozen_until = -1.0;
}

/* ------------------------------------------------------------- the median
 *
 * The measurement that forced this section to be rewritten:
 *
 *     us=21813  psd=1383  prom=19988  scan=379  track=16  other=47
 *
 * against a predicted "under 1.5 ms". The local-median whitening was 91.6% of
 * it - 563 bins, each building a 38-element window and calling qsort with a
 * function-pointer comparator, about 112,000 indirect calls per update. The
 * f0 grid scan, which had been the suspect, was 379 us.
 *
 * Everything below is an exact-equivalence transform. Same statistic, same
 * constants, same grid, same window contents. The argument is always the
 * same one: a median is a function of a MULTISET, so any two routines that
 * see the same multiset sort it to the same sequence of values and take the
 * same two middle elements. Not "close" - bit-identical.
 *
 * t4_prominence_ref() below is the ORIGINAL, kept and still compiled, and
 * tests/test_t4_parity.py runs the two against each other bin for bin. That
 * is the precedent Tier-3's envelope-ring fix set: an optimisation whose
 * equivalence is argued in a comment is an optimisation nobody can check.
 */

/* np.median: the MEAN of the two middle elements when n is even.
 *
 * Insertion sort, not qsort. For n<=38 it is faster outright, but the reason
 * it is here is the indirect call: qsort cannot inline a comparator through a
 * function pointer and this ran 563 times per update. Exactly equivalent -
 * both produce the sorted sequence of values. */
static float median_of(float *v, int n)
{
    for (int i = 1; i < n; i++) {
        const float x = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > x) {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = x;
    }
    if (n & 1) {
        return v[n / 2];
    }
    return 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

/* The accumulator's median is over AT MOST T4_N_SUB = 4 values and runs once
 * per bin - 563 qsort calls per update for a four-element array, which is
 * almost all call overhead. Sorting networks instead. Same multiset, same
 * sorted order, same answer. */
#define T4_CEX(a, b) do {                    \
        if (v[a] > v[b]) {                   \
            const float t_ = v[a];           \
            v[a] = v[b];                     \
            v[b] = t_;                       \
        }                                    \
    } while (0)

static inline float median_small(float *v, int n)
{
    switch (n) {
    case 0:  return 0.0f;
    case 1:  return v[0];
    case 2:  return 0.5f * (v[0] + v[1]);
    case 3:  T4_CEX(0, 1); T4_CEX(1, 2); T4_CEX(0, 1);
             return v[1];
    default: T4_CEX(0, 1); T4_CEX(2, 3); T4_CEX(0, 2);
             T4_CEX(1, 3); T4_CEX(1, 2);
             return 0.5f * (v[1] + v[2]);
    }
}

/* ---- THE SLIDING WINDOW ------------------------------------------------
 * The window for bin b is psd[b-local_hi .. b-local_lo] U
 * psd[b+local_lo .. b+local_hi] - two blocks with a gap, not one run. Moving
 * b -> b+1 shifts both blocks by one, so it is exactly TWO OUT AND TWO IN:
 *
 *     out: psd[b-1-local_hi]   psd[b-1+local_lo]
 *     in:  psd[b-local_lo]     psd[b+local_hi]
 *
 * Keeping the 38 values in a SORTED array turns each step into two binary
 * searches and two short SHIFTS instead of a fresh sort. The multiset after
 * the four operations is exactly the multiset the from-scratch build would
 * have produced, so the median is bit-identical. The shifts are open-coded
 * rather than memmove() calls, which cost most of the stage - see win_del.
 *
 * Only in the interior. Below T4_INC_LO the window reflects at DC, and above
 * T4_INC_HI the C substitutes 0.0f where the reference reads real PSD; in
 * neither region does b -> b+1 shift the window uniformly, so those bins are
 * still built from scratch. That is 2*local_hi = 48 bins of 563 - the fast
 * path covers 91.5% and the edges keep the exact code they always had. */
#define T4_INC_LO  (T4_B_LO + T4_LOCAL_HI)
#define T4_INC_HI  (T4_B_HI - T4_LOCAL_HI)

static float s_win[T4_MED_N];

/* lower_bound. `v` is present when win_del is called, so a[i] == v there. */
static inline int win_lb(const float *a, int n, float v)
{
    int lo = 0, hi = n;
    while (lo < hi) {
        const int mid = (lo + hi) >> 1;
        if (a[mid] < v) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

/* ---------------------------------------------------------------------------
 * The shift, and the fourth time this project has paid for a library call.
 *
 * These two used memmove(). It looked free - "two out and two in" is four
 * short moves of a 38-element array - and it was not: MEASURED at 9762 us for
 * 563 bins, about 4160 cycles per bin, which no operation count explains.
 *
 * The reason is that the linked memmove is the ROM's newlib variant and it
 * moves ONE BYTE AT A TIME in both directions, and win_ins always takes the
 * backward (destructive-overlap) path because it copies dst=&a[i+1] from
 * src=&a[i]. Per update that is 2056 calls moving about 150 KB, byte by byte,
 * through a CALL8 each time. The same failure mode as Tier-3's envelope ring
 * at 262 KB of memcpy a frame, one abstraction layer further down.
 *
 * A word loop instead. s_win is a 4-byte-aligned float array, so a word copy
 * is always legal and the byte loop was pure waste.
 *
 * Exact by construction, and that is why it may ship. There is no arithmetic
 * here at all: it is the identical permutation of the identical float bit
 * patterns into the identical slots, so s_prom is bit-identical and the capped
 * dB sum W4 cannot move. tests/test_t4_parity.py proves it rather than
 * assuming it.
 *
 * The suspects going in were log10f per bin and implicit double promotion.
 * There is no double promotion anywhere in this path - log10f and logf are
 * genuine single precision - and the log tail is only 10-17% of the stage.
 * Measure before choosing what to fix: an operation count is not a cost.
 * ------------------------------------------------------------------------ */
static inline void win_del(float *a, int n, float v)
{
    int i = win_lb(a, n, v);
    for (; i < n - 1; i++) {
        a[i] = a[i + 1];
    }
}

static inline void win_ins(float *a, int n, float v)
{
    const int i = win_lb(a, n, v);
    for (int k = n; k > i; k--) {
        a[k] = a[k - 1];
    }
    a[i] = v;
}

/* ---------------------------------------------------------- the statistic */

static float s_psd[T4_N_BINS_USED];
static float s_prom[T4_N_BINS_USED];
static bool  s_prom_ready;

/* Reflect an index into [0, CFG_N_BINS): -1 -> 1, n -> n-2. numpy's
 * np.abs / mirror pair in the reference, and the same two lines. */
static inline int reflect(int i)
{
    if (i < 0) {
        i = -i;
    }
    if (i >= CFG_N_BINS) {
        i = 2 * (CFG_N_BINS - 1) - i;
    }
    return i;
}

/* ONE BIN, FROM SCRATCH. The original body, unchanged except for being
 * lifted into a function so the edges and the reference implementation can
 * share it. This is the definition of the window; everything faster below is
 * measured against what this produces. */
static inline float prom_ref_bin(const float *psd, int b, float *w)
{
    int n = 0;
    for (int d = -T4_LOCAL_HI; d <= -T4_LOCAL_LO; d++) {
        int i = reflect(b + d);
        w[n++] = (i <= T4_B_HI) ? psd[i - T4_B_LO] : 0.0f;
    }
    for (int d = T4_LOCAL_LO; d <= T4_LOCAL_HI; d++) {
        int i = reflect(b + d);
        w[n++] = (i <= T4_B_HI) ? psd[i - T4_B_LO] : 0.0f;
    }
    const float ref = median_of(w, n);
    return 10.0f * log10f((psd[b - T4_B_LO] + 1e-30f) / (ref + 1e-30f));
}

/* The reference: every bin from scratch, and what the fast path is proved
 * against bin for bin by the host parity test. Kept compiled rather than
 * deleted or commented out, because an equivalence claim with nothing to
 * compare against is an assertion. */
void t4_prominence_ref(const float *psd, float *out)
{
    float w[T4_MED_N];
    for (int b = T4_B_LO; b <= T4_B_HI; b++) {
        out[b - T4_B_LO] = prom_ref_bin(psd, b, w);
    }
}

static void t4_prominence(const float *psd)
{
    t4_prominence_begin();
    (void)t4_prominence_step(psd, T4_N_BINS_USED);
}

/* ---- the same walk, resumable -------------------------------------------
 *
 * Tier-4 costs about 11 ms per update and lands whole on one frame, which is
 * why it shipped disarmed: 11 ms on top of the other three tiers does not fit
 * a 32 ms hop, and it updates one frame in eight so nothing amortises.
 * Spreading the walk over several frames is the fix, and the requirement on
 * it is absolute - it must be a reordering and not a different Tier-4.
 *
 * So the whole-band path is a call to the sliced one. Identity is not
 * argued, and not even really tested: there is one implementation, and
 * t4_prominence() is the S=1 case of it. tests/test_t4_parity.py then proves
 * the S>1 cases against the from-scratch reference anyway, because a claim
 * with nothing to compare against is an assertion.
 *
 * The snapshot is already frozen and this costs no memory. Section 6.2 asks
 * for a double buffer; s_psd is materialised from the accumulator ONCE per
 * update by t4_psd_now(), and the per-frame accumulation writes st->sub[][],
 * which s_psd no longer depends on. The caller must simply not re-derive it
 * between slices.
 *
 * The interior window carries across slices, which is the only subtle part:
 * s_win is maintained bin to bin, so a slice boundary inside the interior is
 * safe precisely because s_win is file scope and nothing else touches it. */
static int  s_slice_b;          /* next bin to whiten                       */
static bool s_slice_primed;     /* the interior window has been built       */

void t4_prominence_begin(void)
{
    s_slice_b = T4_B_LO;
    s_slice_primed = false;
    s_prom_ready = false;
}

bool t4_prominence_step(const float *psd, int budget)
{
    float w[T4_MED_N];

    while (budget > 0 && s_slice_b <= T4_B_HI) {
        const int b = s_slice_b;

        if (b < T4_INC_LO) {
            /* low edge: the window reflects at DC, so build each one */
            s_prom[b - T4_B_LO] = prom_ref_bin(psd, b, w);
        } else if (b <= T4_INC_HI) {
            /* interior: one window, carried */
            if (!s_slice_primed) {
                int n = 0;
                for (int d = -T4_LOCAL_HI; d <= -T4_LOCAL_LO; d++) {
                    s_win[n++] = psd[b + d - T4_B_LO];
                }
                for (int d = T4_LOCAL_LO; d <= T4_LOCAL_HI; d++) {
                    s_win[n++] = psd[b + d - T4_B_LO];
                }
                /* Sorted once, then maintained. median_of sorts in place and
                 * its return is discarded here - the sort is the point. */
                (void)median_of(s_win, n);
                s_slice_primed = true;
            } else {
                /* two out, two in - see the comment on T4_INC_LO */
                win_del(s_win, T4_MED_N,
                        psd[b - 1 - T4_LOCAL_HI - T4_B_LO]);
                win_del(s_win, T4_MED_N - 1,
                        psd[b - 1 + T4_LOCAL_LO - T4_B_LO]);
                win_ins(s_win, T4_MED_N - 2,
                        psd[b - T4_LOCAL_LO - T4_B_LO]);
                win_ins(s_win, T4_MED_N - 1,
                        psd[b + T4_LOCAL_HI - T4_B_LO]);
            }
            /* T4_MED_N is even by construction (two blocks of the same
             * length), so this is np.median's mean-of-the-middle-two. */
            const float ref = 0.5f * (s_win[T4_MED_N / 2 - 1] +
                                      s_win[T4_MED_N / 2]);
            s_prom[b - T4_B_LO] = 10.0f *
                log10f((psd[b - T4_B_LO] + 1e-30f) / (ref + 1e-30f));
        } else {
            /* top edge: the C zero-pads above T4_B_HI where the reference
             * reads real PSD, so the window is not a shift of its neighbour */
            s_prom[b - T4_B_LO] = prom_ref_bin(psd, b, w);
        }

        s_slice_b++;
        budget--;
    }

    if (s_slice_b > T4_B_HI) {
        s_prom_ready = true;
        return true;
    }
    return false;
}

_Static_assert(T4_MED_N % 2 == 0,
               "the interior fast path takes the mean of the two middle "
               "elements; an odd window needs the other branch");
_Static_assert(T4_MED_N == 2 * (T4_LOCAL_HI - T4_LOCAL_LO + 1),
               "T4_MED_N is not the two blocks' total length - the sliding "
               "window would carry the wrong number of samples");

/* The stage timers. File-scope because t4_prominence() and the scan sit
 * inside t4_score(), which the reference's shape puts one call below where the
 * record is filled in. Written on every update, read once. */
static uint32_t s_us_prom, s_us_scan, s_us_psd;

/* An update in progress: the PSD is frozen and the walk is part done. */
static bool s_t4_slicing;

/* Bins per frame. The right value is the largest budget whose worst
 * per-frame Tier-4 contribution measures under 2.5 ms on the board; 563 bins
 * over eight frames would be 71. */
#ifndef T4_SLICE_BINS
/* 563 bins over the four free frames of a 16-frame update. */
#define T4_SLICE_BINS 141
#endif

/* The grid scan alone, on whatever is already in s_prom. Split out so a
 * sliced walk can be scanned after its last slice without re-whitening, and
 * so t4_score() below stays exactly what the parity harness measures. */
/* Prominence, in dB, above which a comb position counts as a tooth for the
 * REPORTED count. Reporting only; nothing branches on it. */
#define T4_TOOTH_COUNT_DB 3.0f
static int s_teeth;

int t4_last_teeth(void) { return s_teeth; }

static void t4_scan(double *W4_out, double *f0_out)
{
    const int64_t p1 = esp_timer_get_time();
    float best = -1e30f;
    int   bj = 0;
    for (int j = 0; j < T4_N_F0; j++) {
        const uint16_t *tb = &T4_TOOTH[(size_t)j * T4_N_HARM];
        float acc = 0.0f;
        for (int k = 0; k < T4_N_HARM; k++) {
            const int b = (int)tb[k];
            float p = (b >= T4_B_LO && b <= T4_B_HI)
                      ? s_prom[b - T4_B_LO] : 0.0f;
            if (p > T4_CAP_DB) {
                p = T4_CAP_DB;
            }
            acc += p * T4_KW[k];
        }
        if (acc > best) {
            best = acc;
            bj = j;
        }
    }
    /* ---- the teeth count, for the winner only ---------------------------
     * How many of the six comb positions actually carried prominence. It
     * gates nothing - no decision reads it - and it is here because a
     * candidate Tier-4 teeth term would test exactly this quantity, and that
     * term has to be calibrated against real events before it ships.
     * Recording it now is what makes those events usable as evidence later.
     *
     * Counted for the winning grid point alone: six iterations once every
     * eight frames, against the 3030 us the update already costs. Counting it
     * inside the scan would have been T4_N_F0 times that, for a number only
     * the winner needs.
     *
     * T4_TOOTH_COUNT_DB is a REPORTING floor, chosen not calibrated. If the
     * teeth term ever ships, its threshold is a separate constant arrived at
     * through evaluate.analyze_corpus, not this one. */
    {
        const uint16_t *tb = &T4_TOOTH[(size_t)bj * T4_N_HARM];
        int teeth = 0;
        for (int k = 0; k < T4_N_HARM; k++) {
            const int b = (int)tb[k];
            const float p = (b >= T4_B_LO && b <= T4_B_HI)
                            ? s_prom[b - T4_B_LO] : 0.0f;
            if (p > T4_TOOTH_COUNT_DB) {
                teeth++;
            }
        }
        s_teeth = teeth;
    }
    *W4_out = (double)best;
    *f0_out = T4_F0_LO + T4_F0_STEP * (double)bj;
    s_us_scan = (uint32_t)(esp_timer_get_time() - p1);
}

void t4_score(const float *psd, double *W4_out, double *f0_out)
{
    const int64_t p0 = esp_timer_get_time();
    t4_prominence(psd);
    s_us_prom = (uint32_t)(esp_timer_get_time() - p0);
    t4_scan(W4_out, f0_out);
}

/* --------------------------------------------------------- the accumulator */

static void t4_psd_now(const t4_state_t *st, float *out)
{
    float v[T4_N_SUB];
    for (int b = 0; b < T4_N_BINS_USED; b++) {
        int n = 0;
        for (int s = 0; s < T4_N_SUB; s++) {
            if (st->sub_n[s] > 0) {
                v[n++] = st->sub[s][b] / (float)st->sub_n[s];
            }
        }
        out[b] = median_small(v, n);
    }
}

/* ----------------------------------------------------------- the decision */

static bool t4_excluded(const t4_cfg_t *c, double f0)
{
    for (int i = 0; i < c->n_excl; i++) {
        if (fabs(f0 - c->excl_c[i]) <= c->excl_t[i]) {
            return true;
        }
    }
    return false;
}

/* Family-normalised continuity. The comparison is made after dividing out
 * whichever allowed ratio brings the two closest together, so an argmax that
 * jumps from the shaft line to the third harmonic - measured, on a three-blade
 * propeller - extends the track instead of ending it. */
static bool t4_continuous(const t4_cfg_t *c, double f0, double last,
                          double *ratio_out)
{
    bool found = false;
    double bd = 0.0, br = 1.0;
    for (int i = 0; i < T4_N_RATIO; i++) {
        const double r = T4_RATIO[i];
        const double tgt = last * r;
        double tol = c->cont_frac * tgt;
        if (tol < c->cont_min_hz) {
            tol = c->cont_min_hz;
        }
        const double d = fabs(f0 - tgt);
        if (d <= tol && (!found || d < bd)) {
            found = true;
            bd = d;
            br = r;
        }
    }
    if (found) {
        *ratio_out = br;
    }
    return found;
}

static void t4_push_hit(t4_state_t *st, const t4_cfg_t *c, int v)
{
    st->hits += v - (int)st->ring[st->ri];
    st->ring[st->ri] = (uint8_t)v;
    st->ri = (st->ri + 1) % c->n4;
    st->track_age++;
}

static void t4_clear_track(t4_state_t *st, const t4_cfg_t *c)
{
    memset(st->ring, 0, sizeof(st->ring));
    (void)c;
    st->ri = 0;
    st->hits = 0;
    st->track_f0 = -1.0;
    st->track_age = 0;
    st->ev_peak = 0.0;
}

static void t4_close(t4_state_t *st, double t)
{
    (void)t;
    if (st->latched) {
        st->n_events++;
    }
    st->latched = false;
    st->below = 0;
}

static void t4_update(const float *psd, const t4_cfg_t *c, t4_state_t *st,
                      double t, t4_rec_t *out, bool prom_done)
{
    double W4, f0;
    if (prom_done) {
        t4_scan(&W4, &f0);          /* the slices already whitened s_prom */
    } else {
        t4_score(psd, &W4, &f0);
    }
    const int64_t k0 = esp_timer_get_time();
    st->n_updates++;

    const bool excluded = t4_excluded(c, f0);
    const bool above = (W4 >= c->tau4) && (t >= c->warmup_s) && !excluded;

    double ratio = 1.0;
    bool hit = false;
    if (above) {
        if (st->track_f0 < 0.0) {
            t4_clear_track(st, c);
            st->track_f0 = f0;
            hit = true;
        } else if (t4_continuous(c, f0, st->track_f0, &ratio)) {
            hit = true;
            if (ratio == 1.0) {
                st->track_f0 = f0;
            }
        } else {
            t4_close(st, t);
            t4_clear_track(st, c);
            st->track_f0 = f0;
            hit = true;
        }
    }
    if (st->track_f0 >= 0.0) {
        t4_push_hit(st, c, hit ? 1 : 0);
        if (hit && W4 > st->ev_peak) {
            st->ev_peak = W4;
        }
        if (!st->latched && st->hits >= c->m4
            && (!c->veto_voice || st->track_f0 >= c->veto_f0_min_hz)) {
            st->latched = true;
            st->t_on = t;
            st->f0_on = st->track_f0;
            st->below = 0;
        } else if (st->latched) {
            if (hit) {
                st->below = 0;
            } else if (++st->below >= c->release_updates) {
                t4_close(st, t);
                t4_clear_track(st, c);
            }
        }
    }

    out->t = t;
    out->W4 = W4;
    out->f0 = f0;
    out->ratio = ratio;
    out->hit = hit;
    out->above = above;
    out->excluded = excluded;
    out->frozen = false;
    out->hits = st->hits;
    out->n4 = c->n4;
    out->track_age = st->track_age;
    out->fired4 = st->latched;
    out->us_prom = s_us_prom;
    out->us_scan = s_us_scan;
    out->us_track = (uint32_t)(esp_timer_get_time() - k0);
}

/* ------------------------------------------------------------- the stream */

/* ---- THE FREEZE, AND WHY TIER-4's IS STRONGER THAN TIER-3's -------------
 *
 * The buzzer is IN BAND for this tier. It drives at 2.7 kHz by default, and
 * Tier-4's grid reaches 700 Hz with six harmonics - so 450 x 6 = 2700 is a
 * tooth of a perfectly ordinary candidate. A device that alarms, hears its own
 * buzzer as a comb tooth and re-alarms is a device that never stops.
 *
 * Tier-3 freezes its DECISION for 500 ms after any output. That is not enough
 * here: Tier-4 integrates over T4_WIN_FRAMES = 64 frames, about two seconds,
 * so a contaminated frame keeps scoring for two seconds after the buzzer
 * stops. **Tier-4 therefore freezes the ACCUMULATOR** - a frozen frame is not
 * added to any sub-block and does not advance the window - so the statistic
 * simply pauses and resumes on clean air with its history intact.
 *
 * Freezing rather than inhibiting is deliberate, and the reason is Tier-3's:
 * a frozen update counts as neither a hit nor a miss, so an established track
 * survives an alert instead of being broken by it. */
bool t4_push_frame(const cf32_t *spec, const t4_cfg_t *cfg, t4_state_t *st,
                   uint32_t frame, double t, bool output_active,
                   t4_rec_t *out)
{
    const int64_t c0 = esp_timer_get_time();

    if (output_active) {
        st->frozen_until = t + cfg->freeze_tail_s;
    }
    if (t <= st->frozen_until) {
        st->n_frozen++;
        *out = st->last;
        out->t = t;
        out->frozen = true;
        out->hit = false;
        out->above = false;
        out->fired4 = st->latched;
        st->last = *out;
        st->us_total += esp_timer_get_time() - c0;
        return false;
    }

    if (st->sub_n[st->sub_i] >= T4_SUB_FRAMES) {
        st->sub_i = (st->sub_i + 1) % T4_N_SUB;
        memset(st->sub[st->sub_i], 0, sizeof(st->sub[0]));
        st->sub_n[st->sub_i] = 0;
    }
    float *acc = st->sub[st->sub_i];
    for (int b = T4_B_LO; b <= T4_B_HI; b++) {
        const cf32_t z = spec[b];
        acc[b - T4_B_LO] += z.re * z.re + z.im * z.im;
    }
    st->sub_n[st->sub_i]++;
    st->n_frames++;
    st->since++;
    st->t = t;

    /* ---- the sliced update ---------------------------------------------
     *
     * Doing the whole update on one frame puts about 11 ms of whitening, scan
     * and tracker inside a 32 ms hop that already has three other tiers in it.
     * Measured that way the guard runs p99 43.1 ms with 234 frames over the
     * hop, and because the tier updates one frame in eight, nothing
     * amortises.
     *
     * So the walk is spent T4_SLICE_BINS bins per frame and the decision
     * happens on the frame after the last slice. The PSD is frozen at the
     * start by t4_psd_now(), and the accumulation that continues underneath
     * writes st->sub[][], which the frozen copy no longer depends on. The
     * result is the same to the bit; tests/test_t4_parity.py proves that
     * against the from-scratch reference at eight different budgets. */
    if (s_t4_slicing) {
        /* ---- slices keep out of the other tiers' frames ------------------
         * The tier phases are T2 on even frames, T3 on 3 (mod 4) and T4's
         * decision on 1 (mod 8), which are disjoint. Spreading the slices
         * across consecutive frames ignores that and lands them on Tier-3's,
         * which the scheduling probe reports as a collision. The free frames
         * are 1 (mod 4); slices run only there. */
        if ((frame % T4_SLICE_PHASE_MOD) != T4_SLICE_PHASE) {
            st->us_total += esp_timer_get_time() - c0;
            return false;
        }
        const int64_t sl0 = esp_timer_get_time();
        const bool done = t4_prominence_step(s_psd, T4_SLICE_BINS);
        s_us_prom += (uint32_t)(esp_timer_get_time() - sl0);
        if (!done) {
            st->us_total += esp_timer_get_time() - c0;
            return false;               /* no decision on a slice frame */
        }
        s_t4_slicing = false;
        t4_update(s_psd, cfg, st, t, out, true);
        out->us_psd = s_us_psd;
        out->us_t4 = (uint32_t)(esp_timer_get_time() - c0);
        {
            const uint32_t named = out->us_psd + out->us_prom + out->us_scan +
                                   out->us_track;
            out->us_other = (out->us_t4 > named) ? (out->us_t4 - named) : 0u;
        }
        st->us_total += out->us_t4;
        st->last = *out;
        return true;
    }

    if (st->n_frames < T4_WIN_FRAMES || st->since < T4_UPDATE_FRAMES) {
        st->us_total += esp_timer_get_time() - c0;
        return false;
    }
    st->since = 0;
    const int64_t d0 = esp_timer_get_time();
    t4_psd_now(st, s_psd);              /* FREEZE: read once, sliced after */
    s_us_psd = (uint32_t)(esp_timer_get_time() - d0);
    s_us_prom = 0;
    t4_prominence_begin();

    /* THE FIRST SLICE HAPPENS NOW, on the trigger frame. That is what makes a
     * budget covering the whole band EXACTLY the old behaviour, decision on
     * the same frame and all: the parity harness compiles with such a budget
     * and still measures decision-for-decision identity against the Python,
     * while the firmware ships a small budget and spreads the cost. The only
     * thing a small budget changes is WHEN the decision lands, by at most
     * T4_N_BINS_USED / T4_SLICE_BINS frames. */
    /* The trigger frame takes a slice only if it is itself a free frame; the
     * decision phase (1 mod 8) is a subset of the slice phase (1 mod 4), so
     * it always is. */
    const int64_t sl0 = esp_timer_get_time();
    const bool done_now = t4_prominence_step(s_psd, T4_SLICE_BINS);
    s_us_prom += (uint32_t)(esp_timer_get_time() - sl0);
    if (!done_now) {
        s_t4_slicing = true;
        st->us_total += esp_timer_get_time() - c0;
        return false;
    }
    t4_update(s_psd, cfg, st, t, out, true);
    out->us_psd = s_us_psd;
    out->us_t4 = (uint32_t)(esp_timer_get_time() - c0);
    {
        const uint32_t named = out->us_psd + out->us_prom + out->us_scan +
                               out->us_track;
        out->us_other = (out->us_t4 > named) ? (out->us_t4 - named) : 0u;
    }
    st->us_total += out->us_t4;
    st->last = *out;
    return true;


}

/* The window has to FILL before the first update - T4_WIN_FRAMES pushes - so
 * the first update is on the (T4_WIN_FRAMES-1)-th frame Tier-4 sees, not the
 * first. Exactly the arithmetic that a comment in sentry_node.c got backwards
 * for Tier-3, which is why it is a function here and asserted there rather
 * than described in prose. */
int t4_first_update_frame(void)
{
    return T4_WIN_FRAMES - 1;
}
