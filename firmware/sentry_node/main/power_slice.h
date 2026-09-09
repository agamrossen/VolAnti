/*
 * power_slice.h - millivolts to a four-cell battery bar, and the two timers
 * that decide when to say LOW and when to stop.
 *
 * Why this is its own file, with no ESP-IDF in it anywhere.
 *
 * Everything here is arithmetic over integers: a threshold table, a
 * hysteresis rule and two "has this been true continuously for N seconds"
 * timers. None of it needs an ADC, a GPIO, a task or a clock - the caller
 * passes the millivolts and the milliseconds in. That makes it compilable by
 * a host compiler, which makes it TESTABLE BY THE HOST TEST rather than
 * modelled by one.
 *
 * That distinction is the whole point. tests/test_power_battery.py builds
 * THIS FILE with clang and calls these functions through ctypes, so what the
 * test walks is the code that will be flashed. A Python re-implementation
 * would test the Python.
 *
 * It is board-independent by construction: every constant arrives in a
 * power_slice_cfg_t from the board header. There is no PCB-only code here,
 * which is why it sits outside the BOARD_HAS_VBAT guard - power_mon.c, which
 * owns the ADC and the panel, is entirely inside it.
 *
 * Nothing here self-learns. The thresholds are numbers a human wrote in a
 * board header and will re-anchor against a meter at bring-up.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Four cells, so four rising thresholds, lowest first. mv[0] is the boundary
 * between LOW (no cells) and one cell. */
#define POWER_SLICE_N 4

typedef struct {
    int mv[POWER_SLICE_N];   /* rising thresholds, ascending                 */
    int hyst_mv;             /* applied to EVERY boundary, both directions   */
} power_slice_cfg_t;

/* The plain mapping, with no memory: how many cells this voltage alone earns.
 * Used for the FIRST reading, where there is no previous slice to be sticky
 * about, and by the tests as the thing hysteresis is measured against. */
int power_slice_plain(const power_slice_cfg_t *c, int mv);

/* The mapping WITH memory. `cur` is the slice currently displayed, or -1 for
 * "nothing displayed yet".
 *
 * Why hysteresis is not cosmetic here. This panel is e-paper: every change of
 * the bar is a full ~2 s refresh, and a battery resting exactly on a boundary
 * would otherwise oscillate for hours, which is both a visible flicker and a
 * standing 6% duty cycle on the slowest peripheral on the board. With 40 mV
 * either way, a slice change is a few redraws a day by construction. */
int power_slice_step(const power_slice_cfg_t *c, int cur, int mv);

/* ---- "continuously below X for N ms" ------------------------------------
 * A single dip below a limit means nothing: the buzzer and the vibration
 * motor pull a hundred milliamps between them, and a 1S cell sags while they
 * do. Only a sustained reading is evidence about the CELL rather than about
 * the load, which is why both battery decisions are timed and neither is
 * taken on one sample.
 *
 * Deliberately NOT latching. A device on a charger must be able to climb back
 * out of LOW without a reboot, and a latch that cleared itself would be a
 * latch in name only. Whoever wants a one-way decision - the shutdown does -
 * makes it one by acting on the first true and never asking again. */
typedef struct {
    uint32_t since_ms;   /* when the run of below-limit readings began */
    bool     below;      /* is a run in progress                       */
} power_sustain_t;

void power_sustain_reset(power_sustain_t *s);

/* Feed one reading. Returns true once the voltage has been below `limit_mv`
 * on EVERY call for at least `hold_ms`. One reading at or above the limit
 * ends the run and the timer starts again from zero. */
bool power_sustain_step(power_sustain_t *s, int mv, int limit_mv,
                        uint32_t hold_ms, uint32_t now_ms);

/* ---- median of a small odd sample --------------------------------------
 * The ADC on this part is noisy enough that a single conversion is not a
 * measurement. A median rejects the occasional wild sample outright, where a
 * mean would let one spike move the answer by tens of millivolts - and a
 * spike at a boundary is precisely a spurious 2 s panel refresh.
 *
 * Sorts `a` IN PLACE. n must be odd and at most POWER_MEDIAN_MAX. */
#define POWER_MEDIAN_MAX 31
int power_median(int *a, int n);
