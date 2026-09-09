/*
 * power_mon.h - how much battery is left, and what the device does about it.
 *
 * PCB-ONLY. Every function below compiles to nothing when BOARD_HAS_VBAT is
 * 0, and the DevKit build calls them anyway - they are inert, and the call
 * sites stay readable instead of being wrapped in #if at four places in the
 * frame loop.
 *
 * The hardware is VBAT through a 100k/100k divider into GPIO1 / ADC1_CH0 with
 * 100 nF across the lower leg, a BQ24074-class power path whose STAT line is
 * open-drain on GPIO2, and a protected 1S 2500 mAh cell.
 *
 * The one thing that will look like a fault and is not. There is a buck-boost
 * on the rail, so the 3V3 supply is held up across the whole discharge curve.
 * VBAT BELOW 3.3 V IS NORMAL OPERATION on this board, not a brown-out. A
 * future session reading 3.1 V and reaching for the reset button should read
 * this paragraph first.
 *
 * Where it runs. Never on the frame loop. The measurement is taken by the
 * standalone loop's own housekeeping tick at most once every ten seconds, and
 * a measurement is fifteen ADC conversions - about 150 us. It is skipped
 * outright while a bounded capture or a parity stream is running, because the
 * one thing worse than not knowing the battery level is a host capture with a
 * hole in it.
 *
 * Nothing here writes flash. Not on the hot path and not off it: the slice is
 * RAM, the thresholds are compiled, and the event ring is RAM. A battery
 * monitor that logged to NVS would wear the part out on exactly the device
 * that is left running for weeks.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board_pins.h"

/* What the monitor is telling the rest of the device. */
#define POWER_OK      0u   /* nothing to say                                */
#define POWER_LOW     1u   /* < BOARD_VBAT_MV_1, sustained: annunciate      */
#define POWER_EMPTY   2u   /* < BOARD_VBAT_MV_EMPTY, sustained: stop        */

/* How long each condition must hold before it is believed. Both are here
 * rather than in the board header because they are properties of the DEVICE's
 * load - the buzzer and motor sag a 1S cell for three seconds at a time - and
 * not of the cell or the board. */
#define POWER_LOW_HOLD_MS    60000u
#define POWER_EMPTY_HOLD_MS  30000u

/* At most one measurement this often. */
#define POWER_PERIOD_MS      10000u

/* Configure the ADC and the STAT input, and start the sampling task. Lazy,
 * idempotent, and never fatal: a device that cannot read its battery must
 * still guard.
 *
 * The task is pinned to core 1 - the e-paper's core, not the detector's - at
 * a priority below everything that matters, and it wakes once a second to ask
 * whether ten have passed. That is the "non-audio core" the brief asks for,
 * literally rather than approximately.
 *
 * It costs RAM, and on this device that is never free: about 2.5 KB of stack
 * and TCB. The prediction of ~16 KB free at arming is a DEVKIT number and the
 * DevKit does not build this file's contents at all, so that prediction is
 * unaffected. The PCB's own heap headroom with this task, the LoRa task and
 * three tiers resident has never been measured and is owed at bring-up. */
void power_mon_begin(void);

/* THE OPERATOR'S STATEMENT that no cell is fitted. Disables the
 * empty-battery park and the bar outright, from the first tick, because a
 * floating BAT node cannot be told from a cell by measurement alone. */
void power_mon_set_no_cell(bool none);

/* Stop the task and release the ADC. */
void power_mon_end(void);

/* THE MEASUREMENT. Called by the sampling task and by nothing else on the
 * device; it is in the header because the host test calls it directly. Does
 * nothing until POWER_PERIOD_MS has elapsed since the last measurement.
 *
 * `quiet` suppresses the measurement entirely.
 *
 * Returns POWER_OK / POWER_LOW / POWER_EMPTY. The caller acts on EMPTY; this
 * module never stops anything by itself, because stopping means stopping the
 * detector and the I2S buses and those belong to whoever owns them. */
uint8_t power_mon_tick(uint32_t now_ms, bool quiet);

/* SUPPRESS SAMPLING ENTIRELY while a bounded capture or a parity stream is in
 * flight. Fifteen ADC conversions is about 150 us on a core the frame loop
 * does not use, once every ten seconds - which SHOULD be invisible and has
 * never been measured. Until it has been, a run whose whole purpose is
 * decision-for-decision parity does not take the risk. */
void power_mon_quiet(bool quiet);

/* The last state the sampling task computed, without taking a measurement.
 * This is what the frame loop reads - a load of one byte, once a second - so
 * the ADC never runs on the audio path. */
uint8_t power_mon_state(void);

/* The last measurement. -1 / 0 before the first one. */
int      power_mon_mv(void);

/* ONE reading, with the ADC brought up if it is not already, and no task and
 * no state machine behind it. The wake path needs a number before the guard
 * exists. Returns mV, or -1 if it could not be read at all. */
int      power_mon_probe(void);

/* R1's trust window, asked rather than duplicated: the wake path and the
 * monitoring tick must never disagree about what "trusted" means. */
bool     power_mon_trusted_mv(int mv);

/* CHG_STAT right now, or -1 on a board without one. */
int      power_mon_chg_level(void);
int      power_mon_slice(void);      /* 0..4 cells, -1 = never measured */
bool     power_mon_charging(void);

/* True on the tick the displayed slice or the charge state CHANGED, and false
 * on every other tick. This is what gates the redraw: on e-paper a redraw is
 * a two-second full refresh, so the bar is drawn when it changes and at no
 * other time. */
bool power_mon_display_changed(void);

/* One line for `I` and for the standalone banner. */
void power_mon_describe(char *dst, int cap);

/* `U vbat`: the last five medians with their timestamps, the raw ADC count,
 * the attenuation, the calibration scheme and the charger line. The point is
 * that the reading can be watched directly, rather than inferred from what
 * the device did about it. */
void power_mon_describe_raw(char *dst, int cap);
