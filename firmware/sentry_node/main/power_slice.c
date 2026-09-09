#include "power_slice.h"

int power_slice_plain(const power_slice_cfg_t *c, int mv)
{
    int s = 0;
    while (s < POWER_SLICE_N && mv >= c->mv[s]) {
        s++;
    }
    return s;
}

int power_slice_step(const power_slice_cfg_t *c, int cur, int mv)
{
    if (cur < 0) {
        /* Nothing on the panel yet, so there is nothing to be sticky about.
         * The first reading takes the plain mapping - applying hysteresis to
         * a slice that was never displayed would just bias the first draw. */
        return power_slice_plain(c, mv);
    }
    if (cur > POWER_SLICE_N) {
        cur = POWER_SLICE_N;
    }
    int s = cur;
    /* Rise only when the voltage clears the NEXT threshold by the full
     * hysteresis, and fall only when it drops below THIS one by the same.
     * The two loops cannot both run: the smallest gap between thresholds is
     * 140 mV against a 2 x 40 mV band. */
    while (s < POWER_SLICE_N && mv >= c->mv[s] + c->hyst_mv) {
        s++;
    }
    while (s > 0 && mv < c->mv[s - 1] - c->hyst_mv) {
        s--;
    }
    return s;
}

void power_sustain_reset(power_sustain_t *s)
{
    s->since_ms = 0;
    s->below = false;
}

bool power_sustain_step(power_sustain_t *s, int mv, int limit_mv,
                        uint32_t hold_ms, uint32_t now_ms)
{
    if (mv >= limit_mv) {
        s->below = false;
        s->since_ms = 0;
        return false;
    }
    if (!s->below) {
        s->below = true;
        s->since_ms = now_ms;
        /* A zero hold would otherwise need a second call to fire, which is a
         * surprise nobody wants from a function whose contract is "for at
         * least N ms". Zero milliseconds have elapsed, so zero is satisfied. */
        return hold_ms == 0u;
    }
    return (uint32_t)(now_ms - s->since_ms) >= hold_ms;
}

int power_median(int *a, int n)
{
    if (n <= 0) {
        return 0;
    }
    if (n > POWER_MEDIAN_MAX) {
        n = POWER_MEDIAN_MAX;
    }
    /* Insertion sort. n is 15; anything cleverer would be slower and would
     * need a comparator, and this runs once every ten seconds. */
    for (int i = 1; i < n; i++) {
        const int v = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > v) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = v;
    }
    return a[n / 2];
}
