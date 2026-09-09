/*
 * nf_probe.h - single-channel near-field probe: the broadband ratio R1 and
 * the level term L.
 *
 * detector.c computes R = 10*log10(E[3200..8000] / E[125..1000]) from the
 * magnitude spectrum it already has, but that spectrum is the FFT of the sum
 * of four microphone channels, while the near-field gate's constant was
 * calibrated on single-microphone recordings. Those are not the same
 * measurement. Measured inter-channel coherence on this array:
 *
 *     band              coherence        four channels add as   power gain
 *     200-800 Hz        0.22 .. 0.96     substantially coherent  up to +12 dB
 *     3.2-7.8 kHz       0.01 .. 0.12     essentially incoherent  about  +6 dB
 *
 * The denominator of R therefore gains up to 12 dB while the numerator gains
 * about 6, so the ratio reads up to 6 dB low on the summed path. The size of
 * that error moves with the low-band coherence frame by frame, so it is not a
 * fixed offset that could be folded into the constant. The 3200-8000 Hz
 * numerator also sits above this array's 3063 Hz spatial-aliasing limit and
 * above its 2166 Hz half-wavelength, so summing there is not merely
 * incoherent but comb-filtered by geometry as a function of arrival angle.
 *
 * R is a spectral shape statistic and gains nothing from four capsules. One
 * channel removes the coherence dependence entirely. The cost is that
 * self-noise and wind are about 6 dB worse than the sum's, which does not
 * matter because R is only ever consulted when the source is loud.
 *
 * Biquads rather than a second FFT, for three reasons in order: the transform
 * that exists is of the sum and this needs one channel; another 2048-point
 * transform costs about 0.84 ms of a 32 ms hop on a board whose worst
 * four-tier frame already measures 33.0 ms; and the statistic is two band
 * energies, which is what a filter is for.
 *
 * The filters are higher order than two sections each, because leakage here
 * is a systematic bias on the number being measured rather than a rounding
 * error. A single 2nd-order band-pass spanning 125 to 1000 Hz has 6 dB/octave
 * skirts and passes 4 kHz at roughly -15 dB, which would compress R1 toward
 * zero on exactly the broadband sources it has to separate. Host-verified
 * response:
 *
 *     freq       HIGH chain     LOW chain
 *      125 Hz     -117.7 dB       -3.0 dB
 *      500 Hz      -69.4 dB       -0.0 dB
 *     1000 Hz      -45.0 dB       -3.0 dB
 *     3200 Hz       -3.0 dB      -45.0 dB
 *     4000 Hz       -0.3 dB      -56.1 dB
 *     7800 Hz       -0.0 dB     -168.6 dB
 *
 * 4 kHz leaks into the low band at -56 dB and 500 Hz into the high band at
 * -69 dB, both far below any separation R1 is asked to resolve.
 *
 * L is the low-band energy in dB above its own tracked reference: "how much
 * louder than the recent background". Its one limitation is written down
 * below with the reference itself.
 *
 * Nothing in this file gates anything. R1 and L are computed, printed and
 * logged; no decision reads them.
 */
#pragma once

#include <stdbool.h>

/* ---- the reference smoother ------------------------------------------- */
/* The reference is a minimum tracker, not a mean.
 *
 * A mean sits above the noise floor and is dragged further up by every loud
 * event, so after a rig run or a piece of music the reference is high, the
 * room goes quiet, and L reads tens of dB negative. That is a true statement
 * about the last thirty seconds and useless as "is a source present now".
 *
 * A minimum tracker answers the second question: the reference falls
 * instantly to any new quietest frame and leaks up slowly, so it sits on the
 * noise floor rather than above it. L is then about 0 at rest by construction
 * and rises with a source, whatever happened a minute ago. This is minimum
 * statistics, the standard noise-floor estimator, without the bias correction
 * a spectral subtractor would need: an estimator that sits slightly low makes
 * L slightly high, which asks for R more often, which is the safe direction
 * for a gate whose job is to be conditioned on level.
 *
 * The leak is 60 s upward - long enough that a hovering source cannot drag
 * the floor up under itself within a measurement step, short enough that a
 * genuinely risen background (wind getting up) is followed within a couple of
 * minutes. */
#define NFP_REF_LEAK     1.000533476f   /* exp(+0.032 / 60.0), the UP leak */

/* Energy floor, so a dead channel gives a finite number rather than -inf.
 * Well below anything a live INMP441 produces: quiet-room rms is about 605
 * counts and q_to_f() scales by 1/32768, so a quiet frame's low-band energy
 * is of order 1e-1, not 1e-12. */
#define NFP_E_EPS        1e-12f

/* ---- biquad coefficients, RBJ cookbook, fs = 16000 --------------------
 * HIGH: 4th-order Butterworth high-pass at 3200 Hz (sections Q 0.5412, 1.3066)
 * LOW : 2nd-order Butterworth high-pass at 125 Hz, then 4th-order low-pass
 *       at 1000 Hz. Generated and response-checked on the host; the table in
 *       the header comment above is that check. */
#define NFP_N_HI 2
#define NFP_N_LO 3

typedef struct {
    float b0, b1, b2, a1, a2;
} nfp_biquad_t;

typedef struct {
    /* Direct Form II transposed state, two per section. */
    float  hi_z[NFP_N_HI][2];
    float  lo_z[NFP_N_LO][2];
    /* Last frame's band energies and the derived numbers. */
    float  e_hi;
    float  e_lo;
    float  r_db;          /* 10*log10(e_hi / e_lo)                        */
    float  l_db;          /* 10*log10(e_lo / ref_lo), 0 dB at rest        */
    double ref_lo;        /* the tracked quiet floor of the low band       */
    bool   have_ref;
    unsigned warm;        /* frames seen (diagnostic only)                 */
    unsigned us_last;     /* time of the last frame, for the frame budget  */
} nf_probe_t;

void  nf_probe_reset(nf_probe_t *p);

/* Feed exactly the hop's new samples of one channel (channel 0). Updates
 * e_hi, e_lo, r_db, l_db and the reference. Reads nothing else and writes
 * nothing outside `p`. */
void  nf_probe_frame(nf_probe_t *p, const float *x, int n);
