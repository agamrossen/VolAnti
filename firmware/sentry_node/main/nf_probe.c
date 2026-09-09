/*
 * nf_probe.c - one-channel R1 and L. See nf_probe.h for why this exists and
 * why the filters are the order they are.
 */

#include "nf_probe.h"

#include <math.h>
#include <string.h>

#include "esp_timer.h"

/* 4th-order Butterworth high-pass, 3200 Hz, fs 16000. */
static const nfp_biquad_t HI[NFP_N_HI] = {
    { +0.348390833f, -0.696781666f, +0.348390833f, -0.328975677f, +0.064587655f },
    { +0.479861272f, -0.959722545f, +0.479861272f, -0.453119520f, +0.466325569f },
};

/* 2nd-order Butterworth high-pass at 125 Hz, then 4th-order low-pass at
 * 1000 Hz. Together: the 125..1000 Hz band. */
static const nfp_biquad_t LO[NFP_N_LO] = {
    { +0.965885290f, -1.931770579f, +0.965885290f, -1.930606427f, +0.932934732f },
    { +0.028118753f, +0.056237506f, +0.028118753f, -1.365117237f, +0.477592250f },
    { +0.033198435f, +0.066396871f, +0.033198435f, -1.611727096f, +0.744520837f },
};

void nf_probe_reset(nf_probe_t *p)
{
    memset(p, 0, sizeof(*p));
}

/* Direct Form II transposed. Chosen over DF-I because it needs two states per
 * section rather than four and is the numerically better-behaved of the two
 * in float32 at these corner frequencies (the 125 Hz section's poles sit at
 * radius 0.966, where DF-I's coefficient cancellation is worst). */
static inline float run_chain(const nfp_biquad_t *s, float z[][2], int n,
                              float x)
{
    for (int i = 0; i < n; i++) {
        const float y = s[i].b0 * x + z[i][0];
        z[i][0] = s[i].b1 * x - s[i].a1 * y + z[i][1];
        z[i][1] = s[i].b2 * x - s[i].a2 * y;
        x = y;
    }
    return x;
}

void nf_probe_frame(nf_probe_t *p, const float *x, int n)
{
    const int64_t t0 = esp_timer_get_time();

    float ehi = 0.0f, elo = 0.0f;
    for (int i = 0; i < n; i++) {
        const float xi = x[i];
        const float h = run_chain(HI, p->hi_z, NFP_N_HI, xi);
        const float l = run_chain(LO, p->lo_z, NFP_N_LO, xi);
        ehi += h * h;
        elo += l * l;
    }

    p->e_hi = ehi;
    p->e_lo = elo;
    p->r_db = 10.0f * log10f((ehi + NFP_E_EPS) / (elo + NFP_E_EPS));

    /* THE FLOOR TRACKER. Falls instantly to a new minimum, leaks up slowly.
     * See nf_probe.h: a MEAN reference made L negative at rest and more
     * negative after a loud session, which is the opposite of a level gate. */
    if (!p->have_ref || (double)elo < p->ref_lo) {
        p->ref_lo = (double)elo;
        p->have_ref = true;
    } else {
        p->ref_lo *= (double)NFP_REF_LEAK;
    }
    p->l_db = 10.0f * log10f(((double)elo + NFP_E_EPS) /
                             (p->ref_lo + NFP_E_EPS));
    if (p->warm < 0xFFFFFFFFu) {
        p->warm++;
    }

    p->us_last = (unsigned)(esp_timer_get_time() - t0);
}
