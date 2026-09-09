/*
 * combiners_cx.c - port of src/combiners.py.
 *
 * Two identities are guaranteed BY CONSTRUCTION here, not by luck of
 * floating-point rounding, because the device leans on them:
 *
 *   cx 'a'                -> detector.c's own combiner(), called, not copied.
 *   cx 'd' with busoff 0  -> falls through to 'a'.
 *
 * So `H` with its default arguments is provably the same arithmetic as `G`,
 * and the only way to change a bit of the shipped path is to type a different
 * letter.
 */

#include "combiners_cx.h"

#include <math.h>

#include "generated/detector_config.h"

bool combiner_cx_valid(char cx)
{
    return cx == 'a' || cx == 'b' || cx == 'c' || cx == 'd';
}

const cf32_t *combiner_cx(char cx, float f_split_hz, float busoff,
                          const cf32_t *spectra, int n_ch,
                          detector_work_t *w, cx_work_t *cw)
{
    if (cx == 'd' && busoff == 0.0f) {
        cx = 'a';           /* the device default: identical, not equivalent */
    }
    if (cx == 'a' || n_ch <= 1) {
        return combiner(spectra, n_ch, w);
    }

    if (cx == 'b') {
        for (int b = 0; b < CFG_N_BINS; b++) {
            float p = 0.0f;
            for (int c = 0; c < n_ch; c++) {
                const cf32_t z = spectra[(size_t)c * CFG_N_BINS + b];
                p += z.re * z.re + z.im * z.im;
            }
            cw->out[b].re = sqrtf(p);
            cw->out[b].im = 0.0f;
        }
        return cw->out;
    }

    if (cx == 'c') {
        /* complex below the split, power above. The split bin is INCLUSIVE on
         * the low side, matching src/combiners.py's `f <= f_split`. */
        const int b_split = (int)(f_split_hz / (float)CFG_BIN_WIDTH_HZ);
        for (int b = 0; b < CFG_N_BINS; b++) {
            if (b <= b_split) {
                float sr = 0.0f, si = 0.0f;
                for (int c = 0; c < n_ch; c++) {
                    sr += spectra[(size_t)c * CFG_N_BINS + b].re;
                    si += spectra[(size_t)c * CFG_N_BINS + b].im;
                }
                cw->out[b].re = sr;
                cw->out[b].im = si;
            } else {
                float p = 0.0f;
                for (int c = 0; c < n_ch; c++) {
                    const cf32_t z = spectra[(size_t)c * CFG_N_BINS + b];
                    p += z.re * z.re + z.im * z.im;
                }
                cw->out[b].re = sqrtf(p);
                cw->out[b].im = 0.0f;
            }
        }
        return cw->out;
    }

    /* cx == 'd': X = X_M1 + X_M2 + exp(+j*theta(b)) * (X_M3 + X_M4)
     *
     * SIGN, stated once so it cannot be got backwards: `busoff` is how far bus
     * B LAGS bus A. A stream delayed by d samples has spectrum
     * X(b)*exp(-2j*pi*b*d/N), so undoing it multiplies by exp(+2j*pi*b*d/N).
     * The host tests prove this two ways - exact recovery of an injected lag,
     * and a broadband clap whose compensated peak must SHARPEN. */
    const float k = 2.0f * (float)M_PI * busoff / (float)CFG_N_FFT;
    for (int b = 0; b < CFG_N_BINS; b++) {
        float ar = 0.0f, ai = 0.0f, br = 0.0f, bi = 0.0f;
        for (int c = 0; c < n_ch; c++) {
            const cf32_t z = spectra[(size_t)c * CFG_N_BINS + b];
            if (c >= 2) {
                br += z.re;
                bi += z.im;
            } else {
                ar += z.re;
                ai += z.im;
            }
        }
        const float th = k * (float)b;
        const float cth = cosf(th), sth = sinf(th);
        cw->out[b].re = ar + (br * cth - bi * sth);
        cw->out[b].im = ai + (br * sth + bi * cth);
    }
    return cw->out;
}
