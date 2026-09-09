/*
 * settings.h - the handful of things a field device must remember across a
 * power cycle, and nothing else.
 *
 * Every tunable on this device is a runtime argument rather than a compiled
 * constant, because a rebuild forces a fresh golden re-proof. That works when
 * a laptop is attached to type the arguments; a device running off a phone
 * charger on a hillside has no laptop, so a runtime-only tunable resets to its
 * default every time the cable is pulled. NVS closes that gap without turning
 * any of them back into constants.
 *
 * It deliberately holds nothing the detector computes: no learned floors, no
 * adaptive thresholds, no self-populating exclusion list. Every value here is
 * one a human typed or one that shipped as a default.
 * NOTHING IN THIS FILE SELF-LEARNS.
 *
 * The store is versioned. An older or unreadable blob is discarded and the
 * shipped defaults used, because a field device that comes up in an unknown
 * configuration is worse than one that comes up in the documented one.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Version history. Two mechanisms, and which one applies is decided by
 * whether the layout moved:
 *
 *   DISCARD - a field changed meaning, or new fields came out of `reserved`
 *             where a stale value would be read as a real setting. The blob
 *             is discarded and the shipped defaults used. Costs the operator
 *             one `U c` line; cannot put an unknown configuration in the
 *             field.
 *   MIGRATE - append-only, so every earlier byte still means what it meant
 *             and the stored blob is a valid prefix. Used where discarding
 *             would cost a measured value that is expensive to recover, such
 *             as panel rotation or microphone calibration.
 *
 * A bump is also the only way to force a value onto boards already in the
 * field: a blob matching SETTINGS_VERSION is taken by the exact-version
 * branch and never reaches the migration, so a change of default that must
 * reach a deployed unit needs the bump to do it.
 *
 * VERSION 2: epd_rotation changed MEANING - an involution index became a
 * clockwise quadrant - and epd_mirror was added. A v1 blob is DISCARDED
 * rather than reinterpreted, because a stored 1 used to mean "mirror-X" and
 * now means "90 degrees", and carrying it forward would put a field device in
 * an orientation nobody chose.
 *
 * VERSION 3: t4_enabled and thr4_milli join the record. DISCARD, because the
 * two new bytes come out of `reserved` and a device that read a v2 blob as a
 * v3 one would take whatever those bytes held as its Tier-4 configuration.
 *
 * VERSION 4: lora_enabled, lora_hz and trk_family, in one bump rather than
 * three, so the operator re-types once. DISCARD.
 *
 * VERSION 5: mic_cal_enabled and mic_gain_milli. DISCARD, and this is the
 * bump where it matters most: a stale gain read out of padding does not
 * announce itself, it quietly unbalances the coherent sum.
 *
 * VERSION 6: veto_voice added, append-only, so a v5 blob is a valid prefix.
 * The first bump that MIGRATES rather than discards, because throwing it away
 * would cost the operator their rotation and their measured microphone
 * calibration.
 *
 * VERSION 7: no bytes moved. The bump exists to force t2, t3, t4 and
 * trk_family on, because a stored 0 in any of them is an old default rather
 * than a decision. MIGRATE.
 *
 * VERSION 8: no bytes moved, and the same mechanism a second time, for
 * veto_voice. MIGRATE.
 *
 * VERSION 9: appends test_mode, and forces nothing - test mode is an
 * operator's decision about a bench, never one the firmware asserts. The
 * migration sets it to 0 explicitly rather than relying on the copy, because
 * an appended byte can land in padding the compiler was leaving anyway, and a
 * unit could otherwise come up in test mode with nothing on the console to
 * say why. MIGRATE.
 *
 * VERSION 10: appends nf_gate and forces thr1_milli to 1500. The force
 * overrides an operator value deliberately: a stored 1700 cannot be told
 * apart from the default it shipped as, and that default detects nothing at
 * close range. `U c thr1 1700` restores it. MIGRATE.
 *
 * VERSION 11: no bytes moved. Forces alert_max_ms to the compiled default,
 * which went from five seconds to seven: a stored 5000 cannot be told apart
 * from the default that produced it, and without the bump every deployed unit
 * would come up still alarming for five seconds with nothing to say why.
 * MIGRATE.
 */
#define SETTINGS_VERSION 11u

typedef struct {
    uint32_t version;

    /* Detection. Milli-units, matching every command that takes a threshold.
     * 0 means "the compiled deployment default", so a stored zero can never
     * silently deafen the device. */
    uint32_t thr1_milli;      /* v1        (0 -> DEFAULT_THRESHOLD, 1.700)  */
    uint32_t thr2_milli;      /* Tier-2    (0 -> T2_TAU2, 1.150)            */
    uint32_t thr3_milli;      /* Tier-3    (0 -> T3_TAU3, 20.000)           */
    uint32_t thr4_milli;      /* Tier-4    (0 -> T4_TAU4, 30.500)           */
    uint8_t  t2_enabled;      /* Tier-2 in the standalone loop              */
    uint8_t  t3_enabled;      /* Tier-3 in the standalone loop              */
    uint8_t  t4_enabled;      /* Tier-4 in the standalone loop. Default 0,  */
                              /* for two reasons: its false-alarm rate is   */
                              /* bounded only at 16/h against a 0.40        */
                              /* allowance, and it costs 21.5 ms per update */
                              /* against a 32 ms hop. Armed, the guard runs */
                              /* p99 43.1 ms with 234 frames over the hop;  */
                              /* without it, 29.3 ms and none. It updates   */
                              /* one frame in eight, so nothing amortises.  */
    uint8_t  t2_rate;         /* 0 compiled default, 1 full, 2 half         */
    uint8_t  cx;              /* combiner variant, 'a'..'d'                 */

    /* Alert surface. */
    uint8_t  buzz_drive;      /* BUZZ_DRIVE_DC | BUZZ_DRIVE_PWM             */
    uint8_t  epd_rotation;    /* EPD_ROT_* quadrant, clockwise (default 270) */
    uint16_t buzz_hz;         /* PWM drive frequency                        */
    uint32_t snooze_ms;       /* the button's silence window                */
    uint32_t alert_max_ms;    /* one alert's output burst                   */

    /* Standalone. */
    uint8_t  autostart;       /* run the guard on power-up with no host     */
    uint8_t  epd_mirror;      /* mirror the artwork (a different fault to   */
                              /* rotation, and it needs its own fix)        */

    /* Detection and local alerting only: the only thing that ever leaves this
     * device is the eighteen-byte peer alert beacon in lora_proto.h. There is
     * nothing else on the radio and there never will be. */
    uint8_t  lora_enabled;    /* the peer alert beacon. 0 on a board with no */
                              /* radio, and not settable there               */
    uint32_t lora_hz;         /* 0 -> BOARD_LORA_HZ_DEFAULT. A deployment    */
                              /* frequency is set per local regulations, not */
                              /* by a code change, which is why it persists  */

    /* Ratio-{2,3,1/2,1/3} tracker continuity with a family-normalised jitter
     * gate. Off is bit-identical to the sealed tracker. Measured non-inferior
     * on the offline corpus - no regressions and two gains in 486 paired
     * positives at the same threshold and false-alarm rate - and it stays off
     * by default until a field session of paired offline replay shows no
     * real-air regression. */
    uint8_t  trk_family;

    /* Relative microphone gains in milli-units, 1000 = unity, one per
     * channel, measured by `Lc` and confirmed by a human before they are
     * stored. Nothing here self-learns: a device that re-calibrated its own
     * microphones could talk itself into deafness one quiet night at a time.
     *
     * mic_cal_enabled 0 means the gains are not applied AT ALL - not applied
     * as unity - so flag-off is bit-identical by construction rather than by
     * 1.0f being an exact multiplier. */
    uint8_t  mic_cal_enabled;
    int16_t  mic_gain_milli[4];

    /* Is a cell fitted? An operator statement, not a measurement.
     *
     * power_mon latches "no cell" on a reading under BOARD_VBAT_MV_ABSENT,
     * which catches the 0.000 V case but not all of them: with no cell the
     * BQ24074's BAT node drifts, and has been measured passing through 2.8,
     * 3.9 and 4.1 V on one board. A run that boots while the node happens to
     * sit in the 2.0-3.3 V band accumulates thirty seconds of "sustained
     * empty" and parks the device before the latch has ever seen a low
     * reading. Inference cannot be made reliable against a floating ADC
     * input, so the human says. */
    uint8_t  batt_absent;

    /* The voice and struck-note veto: rejects a candidate whose f0 is below
     * 250 Hz or that shows fewer than 11 teeth. Close sustained speech and
     * piano both clear the score threshold and are stopped here. */
    uint8_t  veto_voice;

    /* Test mode changes the glass, the ring, the radio payload and the
     * console, and nothing about detection: no threshold, no timing and no
     * output moves with it. It is persisted so a bench survives a reset, and
     * it is printed on the boot line and tagged on the home screen so a unit
     * cannot reach a deployment still in it. */
    uint8_t  test_mode;

    /* One flag for two changes that must never be separated: the jitter bound
     * relaxes from 0.004 to 0.05, which is what lets a real rig hold a chain
     * at all, and the near-field broadband gate is then the only thing
     * standing between that and every sustained instrument in earshot.
     * Turning half of this off is worse than turning all of it off, which is
     * why there is one byte and not two. */
    uint8_t  nf_gate;
} settings_t;

/* Reads NVS once and caches. Safe to call repeatedly and safe to call before
 * anything else; on any failure it returns the shipped defaults rather than
 * refusing to run. Never fatal: a broken settings partition must not stop a
 * detector from detecting. */
const settings_t *settings_get(void);

/* Mutable pointer to the cache. Change fields, then call settings_save(). */
settings_t *settings_mut(void);

/* Writes the cache to NVS. Returns false if the store could not be written -
 * the running configuration is unaffected either way. */
bool settings_save(void);

/* Back to the shipped defaults, in RAM and in NVS. */
bool settings_reset(void);

/* One line of text for the `I` banner and the `U s` path. */
void settings_describe(char *dst, int cap);

/* The same line for the shipped defaults rather than the running values.
 *
 * The defaults are the validated field configuration, so an erased board
 * needs no console at all - a claim only worth anything if they can be read
 * back and checked, and read back from the code that produces them rather
 * than from a table maintained beside it. The `I` banner prints this
 * underneath the running configuration so the two can be compared by eye. */
void settings_describe_defaults(char *dst, int cap);
