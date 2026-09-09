/*
 * combiners_cx.h - the combiner variants, selectable at runtime.
 *
 * Port of src/combiners.py. Additive and opt-in: `G`, `Z`, `L`, `Y`, `R` never
 * call anything here, and CX-A is not reimplemented - it CALLS the sealed
 * detector.c combiner(), so "cx a" and `G` are the same arithmetic by
 * construction rather than by inspection.
 *
 * Why a choice exists at all. The shipped combiner is an unweighted complex
 * sum, which is a broadside beam and exactly right while the array is small
 * against the wavelength. Half a wavelength at the 60.96 mm baseline is
 * 2813 Hz, and the comb score reaches 7800 Hz, so the top third of the scored
 * teeth are already past the point where an off-axis source can arrive at two
 * capsules in antiphase - where a complex sum CANCELS the tooth it meant to
 * add. Measured on the real quad captures: inter-microphone coherence 0.97
 * below 800 Hz, 0.23 above 3.2 kHz.
 *
 * The shipped default is CX-A and stays CX-A. These exist so the rig day can
 * give a variant device time with a typed argument instead of a rebuild. The
 * decision to change the default belongs to planning, with rig evidence.
 *
 * NOT BEAMFORMING. Nothing here knows where a microphone is. CX-D's `busoff`
 * is a measured ELECTRICAL constant - the start offset between two I2S RX
 * engines - identical for every direction of arrival.
 */
#pragma once

#include "detector.h"

/* Scratch for the variants that cannot write into detector_work_t (CX-A must
 * be free to return the sealed combiner's own buffer). */
typedef struct {
    cf32_t out[CFG_N_BINS];
} cx_work_t;

/* cx: 'a' complex sum (DEFAULT) | 'b' incoherent RMS | 'c' hybrid | 'd'
 *     offset-compensated complex sum.
 * f_split_hz: CX-C only. busoff: CX-D only, in samples, bus B = channels 2,3.
 *
 * Returns a pointer to CFG_N_BINS spectra. For 'a' that is detector.c's own
 * result; for the others it is cw->out.
 *
 * A note on CX-B's output. It has no phase, so the imaginary part is zero and
 * the real part is sqrt(sum |X_c|^2). That is legal at this seam because
 * back_end and t2_step only ever take the magnitude - and |x + 0i| = |x| for
 * a non-negative x, which sqrt always is.
 */
const cf32_t *combiner_cx(char cx, float f_split_hz, float busoff,
                          const cf32_t *spectra, int n_ch,
                          detector_work_t *w, cx_work_t *cw);

/* True if `cx` names a variant. Used to reject a bad `H` argument with
 * BAD_ARG rather than silently running the default. */
bool combiner_cx_valid(char cx);
