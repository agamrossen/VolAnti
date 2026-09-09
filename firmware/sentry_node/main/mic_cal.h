/*
 * mic_cal.h - relative gains between the four microphones. Operator-run,
 * persisted, and NOT self-learning.
 *
 * What it is for. The four capsules are summed before the transform, and a
 * coherent sum is only worth sqrt(N) if the four channels agree about level.
 * The ICS-43434's sensitivity tolerance is +-1 dB, so two capsules can differ
 * by 2 dB out of the box - which costs the sum and, worse, makes the MIC
 * health line in the footer mean something different on each channel.
 *
 * What it deliberately is not. Nothing here adapts, learns, or runs by
 * itself. `Lc` is typed by a human, with the array unobstructed, in ambient
 * noise; the result is PRINTED and has to be CONFIRMED before it is stored.
 * A device that quietly re-calibrated its own microphones would be a device
 * that could talk itself into deafness one quiet night at a time - which is
 * the same rule the exclusion list and the adaptive floor already live under.
 *
 * THE MEASUREMENT. 500 Hz - 4 kHz band-limited RMS per channel, over a long
 * ambient capture, then gains relative to the FOUR-CHANNEL MEAN. The band is
 * chosen to sit above the wind corner and below the point where capsule
 * directivity starts to differ, so what is measured is sensitivity and not
 * where the operator happened to be standing.
 *
 * Status: the arithmetic and the store are here and host-tested; the gains
 * are not yet applied to the sum path. Applying them has to be flag-off
 * bit-identical and proven by the golden and quad parity gates, both of which
 * run on a device, so it is bring-up work rather than something to land
 * unproven.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MIC_CAL_N        4
/* Unity in milli-units. Integers because PicoLibC's printf has no float
 * support - the whole trace is binary for that reason - and because a gain
 * is a number an operator reads off a screen and writes in a log. */
#define MIC_CAL_UNITY    1000

/* THE ACCEPTANCE BAND, and it is a REFUSAL not a clamp. A capsule needing
 * more than this is not badly calibrated, it is broken or obstructed, and
 * storing a large gain would hide that behind a correction. +-3 dB is
 * comfortably outside the part's +-1 dB tolerance and comfortably inside
 * "something is wrong with this microphone". */
#define MIC_CAL_MIN      708    /* -3.0 dB */
#define MIC_CAL_MAX     1413    /* +3.0 dB */

typedef struct {
    /* Sum of squares per channel over the measured band, and the frame count
     * they were taken over. Kept as doubles: this accumulates for a minute
     * and a float32 sum of 500 Hz-4 kHz energy over 1875 frames loses its
     * low bits long before the end. */
    double  acc[MIC_CAL_N];
    int     n_frames;
} mic_cal_acc_t;

void mic_cal_begin(mic_cal_acc_t *a);

/* One frame's per-channel spectra, already computed. Adds |X|^2 over the
 * calibration band to each channel's accumulator. */
void mic_cal_add(mic_cal_acc_t *a, const void *spec_all, int n_ch);

/* Turn the accumulator into gains, relative to the four-channel mean RMS.
 *
 * Returns false and writes nothing if the measurement cannot support a
 * calibration - no frames, a silent channel, or any channel outside the
 * acceptance band. `why` gets a one-line reason for the operator.
 *
 * PURE ARITHMETIC, no ESP-IDF: tests/test_mic_cal.py compiles this file for
 * the host and drives it, so what is tested is what is flashed. */
bool mic_cal_gains(const mic_cal_acc_t *a, int n_ch, int16_t *gain_milli,
                   char *why, int why_cap);

/* The band, in bins, for a given FFT size and sample rate. Exposed so the
 * test can assert the band is what the brief says rather than what the
 * implementation happens to use. */
int mic_cal_bin_lo(void);
int mic_cal_bin_hi(void);
