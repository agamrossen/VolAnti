/*
 * detector_t3.c - ESP32-S3 port of src/detector_t3.py.
 *
 * Ported FROM THE CODE, under PORTING_NOTES.md discipline.
 *
 * ---------------------------------------------------------------------------
 * Dtype decisions, all of them, in one place
 * ---------------------------------------------------------------------------
 * 1. The FRONT END is float32 - biquads, squaring, the one-pole cascade, the
 *    decimation. The reference computes it in float64 (scipy's sosfilt and
 *    lfilter promote), so this is a DELIBERATE DIVERGENCE of the same kind and
 *    for the same reason as v1's float32 log1pf whitening: a soft-float64
 *    biquad over 512 samples x 4 channels x 2 sections costs more than the
 *    whole frame budget on a core with no double FPU.
 *
 *    It is bounded rather than hoped about. The quantity that survives to a
 *    decision is a per-bin RATIO to a local median of neighbouring bins - the
 *    prominence - and both sides of that ratio carry the same rounding, so
 *    first-order error cancels. The parity tool measures what is left, and the
 *    number it measures is the one to quote; nothing here asserts a bound.
 *
 * 2. The ENVELOPE TRANSFORM is float32, as v1's is. numpy computes rfft in
 *    float64 always. Same divergence, same gate.
 *
 * 3. The LOCAL MEDIAN is computed over float32 and returns the MEAN OF THE TWO
 *    MIDDLE ELEMENTS of an even-length window, because np.median does and the
 *    window is 14 wide. Taking the lower middle - the obvious C shortcut -
 *    changes the reference in a way that shows up at the fourth significant
 *    figure of every prominence.
 *
 * 4. The PROMINENCE and the SCORE are float32; the DECISION arithmetic
 *    (thresholds, continuity, rates, times) is float64, as the reference's
 *    Python floats are.
 *
 * 5. `2*hits >= m3` rather than `hits >= m3/2`, so an odd M3 rounds the same
 *    way on both sides. Same as Tier-2.
 *
 * ---------------------------------------------------------------------------
 * Why the freeze is a freeze
 * ---------------------------------------------------------------------------
 * The vibration motor is an ERM running at roughly 100-200 Hz, bolted to the
 * same structure as the microphone cones, radiating broadband noise modulated
 * at exactly the rate this tier searches for. The buzzer drives at 2-4 kHz,
 * inside the high band. Left alone, the failure mode is a self-latching loop:
 * the tier fires, the motor runs, the tier sees a textbook envelope comb, and
 * the alert never clears.
 *
 * The tracker is therefore FROZEN while any output is active and for a tail
 * afterwards - a frozen update counts as neither a hit nor a miss. Inhibiting
 * instead would count misses and break an established chain, losing a real
 * detection during the alert burst. The burst is 3 s on / 5 s off, so even
 * during an active alert the tier still accumulates five seconds in eight.
 *
 * This is closed by construction rather than by measurement because it is a
 * device-breaking field failure and there was no hardware available on the
 * night it was written.
 */

#include "detector_t3.h"

#include <math.h>
#include <string.h>

#include "dsps_fft2r.h"
#include "esp_timer.h"

/* ------------------------------------------------------------------ */
/* tables, built once                                                  */
/* ------------------------------------------------------------------ */
static float  s_win[T3_ENV_NFFT];               /* np.hanning(1024)     */
static int16_t s_pidx[T3_ENV_BINS][T3_MED_N];   /* local-median gather  */
static int16_t s_sidx[T3_N_RATES][T3_MAX_HARM]; /* rate x harmonic bin  */
static uint8_t s_svalid[T3_N_RATES][T3_MAX_HARM];
static bool   s_ready;

static float s_fftbuf[2 * T3_ENV_NFFT];

t3_cfg_t t3_default_cfg(void)
{
    t3_cfg_t c = {
        .enabled          = (bool)T3_ENABLED_DEFAULT,   /* false */
        .tau3             = T3_TAU3,
        .fire_lo          = T3_FIRE_LO,
        .fire_hi          = T3_FIRE_HI,
        .n_harm           = T3_N_HARM,
        .weight_exp       = T3_WEIGHT_EXP,
        .harm_cap_db      = T3_HARM_CAP_DB,
        .n3               = T3_N3,
        .m3               = T3_M3,
        .n3_gap           = T3_N3_GAP,
        .cont_frac        = T3_CONT_FRAC,
        .warmup_s         = T3_WARMUP_S,
        .release_updates  = T3_RELEASE_UPDATES,
        .freeze_tail_s    = T3_FREEZE_TAIL_S,
    };
    return c;
}

int t3_init(void)
{
    if (s_ready) return 0;
    esp_err_t e = dsps_fft2r_init_fc32(NULL, T3_ENV_NFFT);
    if (e != ESP_OK) return (int)e;

    /* np.hanning(N) is SYMMETRIC: denominator N-1, endpoints exactly 0. */
    for (int i = 0; i < T3_ENV_NFFT; i++)
        s_win[i] = (float)(0.5 - 0.5 * cos(2.0 * M_PI * i /
                                           (double)(T3_ENV_NFFT - 1)));

    /* Local-median gather. Edge bins REFLECT rather than truncate: a
     * truncated window is a smaller sample, so its median has a higher
     * variance, and that happens exactly at the low bins where the fan and
     * the vortex shedding live. */
    int p = 0;
    int8_t off[T3_MED_N];
    for (int d = -T3_MED_HALF; d <= T3_MED_HALF; d++)
        if (d < -T3_MED_SKIP || d > T3_MED_SKIP) off[p++] = (int8_t)d;
    for (int i = 0; i < T3_ENV_BINS; i++)
        for (int j = 0; j < T3_MED_N; j++) {
            int k = i + off[j];
            if (k < 0) k = -k;
            if (k > T3_ENV_BINS - 1) k = 2 * (T3_ENV_BINS - 1) - k;
            s_pidx[i][j] = (int16_t)k;
        }

    /* rate x harmonic bin table. A harmonic past the usable envelope band
     * STOPS the sum: everything after the first invalid k is invalid too,
     * never only the invalid ones. */
    const double df = (double)T3_FS_ENV / (double)T3_ENV_NFFT;
    for (int j = 0; j < T3_N_RATES; j++) {
        double r = T3_GRID_LO + j * T3_RATE_STEP;
        int ok = 1;
        for (int k = 1; k <= T3_MAX_HARM; k++) {
            double f = k * r;
            int i = (int)floor(f / df + 0.5);
            if (f > T3_HARM_F_MAX || i >= T3_ENV_BINS) ok = 0;
            s_svalid[j][k - 1] = (uint8_t)ok;
            s_sidx[j][k - 1] = (int16_t)(ok ? i : 0);
        }
    }
    s_ready = true;
    return 0;
}

void t3_reset(t3_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->track_r = -1.0;
    st->frozen_until = -1.0;
}

/* ------------------------------------------------------------------ */
/* front end                                                           */
/* ------------------------------------------------------------------ */

/* 4th-order Butterworth high-pass as two biquads, transposed direct form II -
 * the form scipy's sosfilt uses, so the state variables mean the same thing on
 * both sides and a mid-stream comparison is meaningful. */
static inline float biquad2(const float b[3], const float a[3], float z[2],
                            float x)
{
    float y = b[0] * x + z[0];
    z[0] = b[1] * x - a[1] * y + z[1];
    z[1] = b[2] * x - a[2] * y;
    return y;
}

static void t3_envelope(const float *const *ch, int n_ch, t3_state_t *st,
                        float *out, int *n_out)
{
    const float a = T3_LP_A;
    float acc[T3_HOP];
    memset(acc, 0, sizeof(float) * T3_HOP);

    for (int c = 0; c < n_ch; c++) {
        for (int i = 0; i < T3_HOP; i++) {
            float y = ch[c][i];
            for (int s = 0; s < 2; s++)
                y = biquad2(T3_HP_SOS_B[s], T3_HP_SOS_A[s],
                            st->sos_z[c][s], y);
            float e = y * y;
            for (int q = 0; q < T3_LP_POLES; q++) {
                e = (1.0f - a) * e + a * st->lp_z[c][q];
                st->lp_z[c][q] = e;
            }
            acc[i] += e;
        }
    }
    /* ENVELOPES are averaged, never the high-band waveforms. Envelopes are
     * non-negative and add incoherently, which is what makes this tier
     * geometry-free: nothing here depends on the array spacing, so it
     * transfers from the 40.64/60.96 mm breadboard to the 56 mm PCB unchanged.
     * A coherent sum could not: at 5 kHz the wavelength is 68.6 mm against a
     * 60.96 mm baseline, so it would have direction-dependent nulls. */
    int n = 0;
    for (int i = 0; i < T3_HOP; i += T3_DECIM) {
        float v = acc[i] / (float)n_ch;
        out[n++] = sqrtf(v > 0.0f ? v : 0.0f);
    }
    *n_out = n;
}

/* ------------------------------------------------------------------ */
/* the statistic                                                       */
/* ------------------------------------------------------------------ */

static int cmp_f(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

/* np.median of an EVEN-length window is the MEAN of the two middle elements.
 * Returning the lower middle - the obvious C shortcut - moves every prominence
 * in the fourth significant figure. */
static float med_n(float *v, int n)
{
    qsort(v, n, sizeof(float), cmp_f);
    return 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

static void t3_prominence(const float *psd, float *prom)
{
    float w[T3_MED_N];
    for (int i = 0; i < T3_ENV_BINS; i++) {
        for (int j = 0; j < T3_MED_N; j++) w[j] = psd[s_pidx[i][j]];
        float ref = med_n(w, T3_MED_N);
        prom[i] = 10.0f * log10f((psd[i] + 1e-30f) / (ref + 1e-30f));
    }
}

void t3_score(const float *psd, const t3_cfg_t *cfg,
              double *W_fire, double *r_fire, double *W_any, double *r_any)
{
    static float prom[T3_ENV_BINS];
    t3_prominence(psd, prom);

    float kw[T3_MAX_HARM];
    for (int k = 1; k <= cfg->n_harm; k++)
        kw[k - 1] = (float)(1.0 / pow((double)k, cfg->weight_exp));

    double bw = -1e30, br = 0.0, aw = -1e30, ar = 0.0;
    const float cap = (float)cfg->harm_cap_db;
    for (int j = 0; j < T3_N_RATES; j++) {
        double r = T3_GRID_LO + j * T3_RATE_STEP;
        float s = 0.0f;
        for (int k = 0; k < cfg->n_harm; k++) {
            if (!s_svalid[j][k]) break;
            float v = prom[s_sidx[j][k]];
            s += kw[k] * (v < cap ? v : cap);
        }
        if (s > aw) { aw = s; ar = r; }
        if (r >= cfg->fire_lo && r <= cfg->fire_hi && s > bw) {
            bw = s; br = r;
        }
    }
    if (bw <= -1e29) { bw = aw; br = ar; }      /* empty firing band */
    *W_fire = bw; *r_fire = br; *W_any = aw; *r_any = ar;
}

/* ------------------------------------------------------------------ */
/* the tracker                                                         */
/* ------------------------------------------------------------------ */

static void ring_push(t3_state_t *st, int v, int n3)
{
    st->hits += v - (int)st->ring[st->ri];
    st->ring[st->ri] = (uint8_t)v;
    st->ri = (st->ri + 1) % n3;
    st->track_age++;
}

static void clear_track(t3_state_t *st, int n3)
{
    memset(st->ring, 0, (size_t)n3);
    st->ri = 0; st->hits = 0; st->track_r = -1.0;
    st->track_age = 0; st->gap = 0;
}

/* End a latched event. Called ONLY on release or at end of signal - never on a
 * chain break, because a chain break is a rate jump and a rate jump is still
 * the same source. Identifying "event" with "unbroken chain" reported ONE
 * continuously running rotor as twelve alerts on a real capture, where the
 * argmax hopped between 82 Hz and its octave 106 times in 914 updates. */
static void close_event(t3_state_t *st, double t)
{
    if (st->latched) { st->n_events++; st->t_off = t; }
    st->latched = false;
    st->quiet = 0;
}

static void t3_update(const float *psd, double t, const t3_cfg_t *cfg,
                      t3_state_t *st, bool output_active, t3_rec_t *out)
{
    if (output_active) st->frozen_until = t + cfg->freeze_tail_s;

    double Wf, rf, Wa, ra;
    int64_t t0 = esp_timer_get_time();
    t3_score(psd, cfg, &Wf, &rf, &Wa, &ra);
    st->us_score += esp_timer_get_time() - t0;
    st->n_updates++;

    if (t <= st->frozen_until) {
        st->n_frozen++;
        *out = st->last;
        out->t = t; out->frozen = true; out->hit = false;
        out->fired3 = st->latched;
        out->r = rf; out->W = Wf; out->r_any = ra; out->W_any = Wa;
        st->last = *out;
        return;
    }

    bool above = (Wf >= cfg->tau3) && (t >= cfg->warmup_s) &&
                 (rf >= cfg->fire_lo) && (rf <= cfg->fire_hi);
    bool hit = false;
    if (above) {
        if (st->track_r < 0.0) {
            clear_track(st, cfg->n3); st->track_r = rf; hit = true;
        } else if (fabs(rf - st->track_r) <= cfg->cont_frac * st->track_r) {
            hit = true; st->track_r = rf;
        } else {
            clear_track(st, cfg->n3); st->track_r = rf; hit = true;
        }
    }
    if (st->track_r >= 0.0) {
        ring_push(st, hit ? 1 : 0, cfg->n3);
        st->gap = hit ? 0 : st->gap + 1;
        if (st->gap > cfg->n3_gap) clear_track(st, cfg->n3);
    }
    if (!st->latched) {
        if (st->track_r >= 0.0 && st->hits >= cfg->m3) {
            st->latched = true; st->t_on = t; st->r_on = st->track_r;
            st->quiet = 0;
        }
    } else {
        /* Release is driven by ABSENCE, not by track identity. An event is a
         * latched INTERVAL; it ends when the tier stops seeing anything, not
         * when the thing it sees changes rate. */
        st->quiet = above ? 0 : st->quiet + 1;
        if (st->quiet >= cfg->release_updates) close_event(st, t);
    }
    st->n_latched += st->latched ? 1 : 0;

    out->t = t; out->r = rf; out->W = Wf; out->r_any = ra; out->W_any = Wa;
    out->hit = hit; out->hits = st->hits; out->n3 = cfg->n3;
    out->track_age = st->track_age; out->fired3 = st->latched;
    out->frozen = false;
    st->last = *out;
}

/* ------------------------------------------------------------------ */
/* the block interface                                                 */
/* ------------------------------------------------------------------ */

bool t3_push_block(const float *const *ch, int n_ch, const t3_cfg_t *cfg,
                   t3_state_t *st, bool output_active, t3_rec_t *out)
{
    if (!s_ready) return false;
    if (n_ch > T3_MAX_CH) n_ch = T3_MAX_CH;

    int64_t t0 = esp_timer_get_time();
    float e[T3_HOP / T3_DECIM];
    int ne = 0;
    t3_envelope(ch, n_ch, st, e, &ne);
    st->us_front += esp_timer_get_time() - t0;
    st->t += (double)T3_HOP / (double)CFG_FS;

    /* The envelope ring: newest last, shifted once per block.
     *
     * Shifting the whole 1024-float ring left by one for every sample, with
     * 64 new envelope samples per frame, is 64 memmoves of 4092 bytes -
     * 262 KB of copying per 32 ms frame to make room for 256 bytes of new
     * data. Measured, that cost about 8 ms per frame and took the quad build
     * from 28 ms to a median of 36 ms against a 32 ms hop, breaking real time
     * on its own, against an estimate of under 1 ms for the whole front end.
     * The estimate was an operation count and the cost was memory traffic.
     *
     * Shifting by `rest` once and appending all `rest` samples is exactly
     * what shifting by one `rest` times does - the ring holds the same values
     * in the same order - so this is a bandwidth fix, not an algorithm
     * change. It requires ne <= T3_ENV_NFFT, which is 64 <= 1024. */
    if (ne > 0) {
        const int space = T3_ENV_NFFT - st->env_n;
        const int take = ne < space ? ne : space;
        if (take > 0) {
            memcpy(st->env + st->env_n, e, sizeof(float) * (size_t)take);
            st->env_n += take;
        }
        const int rest = ne - take;
        if (rest > 0) {
            memmove(st->env, st->env + rest,
                    sizeof(float) * (size_t)(T3_ENV_NFFT - rest));
            memcpy(st->env + T3_ENV_NFFT - rest, e + take,
                   sizeof(float) * (size_t)rest);
        }
        st->env_since += ne;
    }
    if (st->env_n < T3_ENV_NFFT || st->env_since < T3_ENV_HOP) return false;
    st->env_since = 0;

    /* mean-remove, window, transform */
    double mu = 0.0;
    for (int i = 0; i < T3_ENV_NFFT; i++) mu += st->env[i];
    mu /= (double)T3_ENV_NFFT;
    for (int i = 0; i < T3_ENV_NFFT; i++) {
        s_fftbuf[2 * i]     = (float)(st->env[i] - mu) * s_win[i];
        s_fftbuf[2 * i + 1] = 0.0f;
    }
    dsps_fft2r_fc32(s_fftbuf, T3_ENV_NFFT);
    dsps_bit_rev2r_fc32(s_fftbuf, T3_ENV_NFFT);

    float *p = st->per[st->per_i];
    for (int k = 0; k < T3_ENV_BINS; k++) {
        float re = s_fftbuf[2 * k], im = s_fftbuf[2 * k + 1];
        p[k] = re * re + im * im;
    }
    st->per_i = (st->per_i + 1) % T3_WELCH_N;
    if (st->per_n < T3_WELCH_N) st->per_n++;
    if (st->per_n < T3_WELCH_N) return false;

    static float psd[T3_ENV_BINS];
    for (int k = 0; k < T3_ENV_BINS; k++) {
        float s = 0.0f;
        for (int w = 0; w < T3_WELCH_N; w++) s += st->per[w][k];
        psd[k] = s / (float)T3_WELCH_N;
    }
    t3_update(psd, st->t, cfg, st, output_active, out);
    return true;
}
