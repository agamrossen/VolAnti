#include "mic_cal.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "detector.h"

/* 500 Hz - 4 kHz, in bins. Computed from the shipped constants rather than
 * written down, so a change of FFT size or sample rate moves the band with
 * it instead of silently measuring somewhere else. */
#define MIC_CAL_F_LO 500.0
#define MIC_CAL_F_HI 4000.0
#define MIC_CAL_DF   ((double)CFG_FS / (double)CFG_N_FFT)

int mic_cal_bin_lo(void) { return (int)(MIC_CAL_F_LO / MIC_CAL_DF + 0.5); }
int mic_cal_bin_hi(void)
{
    const int b = (int)(MIC_CAL_F_HI / MIC_CAL_DF + 0.5);
    return (b < CFG_N_BINS - 1) ? b : CFG_N_BINS - 1;
}

void mic_cal_begin(mic_cal_acc_t *a)
{
    memset(a, 0, sizeof(*a));
}

void mic_cal_add(mic_cal_acc_t *a, const void *spec_all, int n_ch)
{
    const cf32_t *sp = (const cf32_t *)spec_all;
    const int lo = mic_cal_bin_lo(), hi = mic_cal_bin_hi();
    if (n_ch > MIC_CAL_N) {
        n_ch = MIC_CAL_N;
    }
    for (int c = 0; c < n_ch; c++) {
        const cf32_t *x = &sp[(size_t)c * CFG_N_BINS];
        double s = 0.0;
        for (int b = lo; b <= hi; b++) {
            s += (double)x[b].re * (double)x[b].re +
                 (double)x[b].im * (double)x[b].im;
        }
        a->acc[c] += s;
    }
    a->n_frames++;
}

bool mic_cal_gains(const mic_cal_acc_t *a, int n_ch, int16_t *gain_milli,
                   char *why, int why_cap)
{
    if (n_ch > MIC_CAL_N) {
        n_ch = MIC_CAL_N;
    }
    if (a->n_frames <= 0 || n_ch <= 0) {
        snprintf(why, why_cap, "no frames measured");
        return false;
    }
    double rms[MIC_CAL_N];
    double mean = 0.0;
    for (int c = 0; c < n_ch; c++) {
        rms[c] = sqrt(a->acc[c] / (double)a->n_frames);
        if (!(rms[c] > 0.0)) {
            /* A SILENT CHANNEL IS NOT A CALIBRATION PROBLEM. A gain cannot
             * rescue a microphone that delivered nothing, and storing an
             * enormous one would hide a dead capsule behind a correction. */
            snprintf(why, why_cap,
                     "channel %d measured no energy in %d-%d Hz - that is a "
                     "dead or unplugged microphone, not a gain error",
                     c + 1, (int)MIC_CAL_F_LO, (int)MIC_CAL_F_HI);
            return false;
        }
        mean += rms[c];
    }
    mean /= (double)n_ch;

    /* Gains relative to the four-channel MEAN, so the calibration is a
     * redistribution and not a level change: a device that quietly got
     * louder or quieter after `Lc` would move its own operating point. */
    for (int c = 0; c < n_ch; c++) {
        const double g = mean / rms[c];
        const long m = lround(g * (double)MIC_CAL_UNITY);
        if (m < MIC_CAL_MIN || m > MIC_CAL_MAX) {
            /* Integers and a hand-placed decimal point: PicoLibC's printf has
             * no float support, which is the same reason the whole trace is
             * binary. Centi-dB, signed. */
            const long cdb = lround(2000.0 * log10(g));
            snprintf(why, why_cap,
                     "channel %d wants %ld.%02ld dB, outside the +-3.00 dB "
                     "this will store - the capsule is faulty or obstructed, "
                     "and a gain would hide that",
                     c + 1, cdb / 100, (cdb < 0 ? -cdb : cdb) % 100);
            return false;
        }
        gain_milli[c] = (int16_t)m;
    }
    for (int c = n_ch; c < MIC_CAL_N; c++) {
        gain_milli[c] = MIC_CAL_UNITY;
    }
    snprintf(why, why_cap, "ok");
    return true;
}
