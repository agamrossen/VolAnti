/*
 * boot_rec.h - what survives a reset.
 *
 * Two facts drove this module into existence.
 *
 * One. Without a reset reason and a scrap of RTC memory, a unit that "did
 * nothing" leaves no evidence behind, and a fault that is only described is
 * indistinguishable from a dead board. An episode of that kind has to be
 * explainable from this ring afterwards.
 *
 * TWO. Opening a serial port on this board RESETS the chip. That is measured,
 * repeatedly, on this hardware: every host open produces
 * rst:0x15 (USB_UART_CHIP_RESET) and a fresh boot banner. So the morning's
 * first act destroys the RAM event ring that the overnight guard spent the
 * night filling. A ring that cannot survive being read is not a record.
 *
 * WHY RTC_NOINIT_ATTR AND NOT RTC_DATA_ATTR. The .rtc.data section is
 * re-initialised by the startup code on every reset that is not a deep-sleep
 * wake. A RTC_DATA_ATTR ring would therefore be wiped by exactly the USB chip
 * reset this module exists to survive. .rtc_noinit is left alone by every
 * reset path, which is also why it needs its own magic word and checksum: it
 * is uninitialised garbage on a cold power-on and there is nothing else to
 * tell the two cases apart.
 *
 * The cost is 64 mirror entries at sizeof(evlog_rec_t) = 20 bytes, so 1280
 * bytes plus the boot ring and the counters, out of 8 KB of RTC slow memory.
 * The record is mirrored whole rather than packed down: a lossy mirror of the
 * only surviving record of a field night is a false economy and the space is
 * there.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "event_log.h"

/* ---- the alert timeline's phases ----------------------------------------
 * Written at every step of the alert, so a reset can be attributed to the
 * load that was switching on when it happened. They live in their own RTC
 * word rather than in the blob, because a brownout tears the blob's seal -
 * see boot_rec.c. */
#define BOOTPHASE_UNKNOWN        0u
#define BOOTPHASE_IDLE           1u
#define BOOTPHASE_TX1            2u
#define BOOTPHASE_TX2            3u
#define BOOTPHASE_TX3            4u
#define BOOTPHASE_OUTPUTS_RAMP   5u
#define BOOTPHASE_OUTPUTS        6u
#define BOOTPHASE_SCREEN         7u
#define BOOTPHASE_HOLD           8u
#define BOOTPHASE_RETURN         9u

/* Set by the UI task at each step of the alert timeline. One aligned 32-bit
 * store, so there is no torn state for a power collapse to leave behind. */
void bootrec_set_alert_phase(uint8_t phase);

/* The phase in force when THIS boot's reset happened, or BOOTPHASE_UNKNOWN
 * on a cold power-on. */
uint8_t bootrec_phase_at_reset(void);
const char *bootrec_phase_name(uint8_t p);

#define BOOTREC_N        4      /* boot records kept                        */
#define BOOTMIRROR_N    64      /* event entries mirrored, matches EVLOG_N  */

/* What the boot decided to do. Recorded so a silent morning is readable. */
#define BOOTDEC_UNKNOWN          0u
#define BOOTDEC_LISTENING        1u
#define BOOTDEC_EMPTY            2u
#define BOOTDEC_OFF_DEEP_RESUME  3u   /* EXT1 hold completed, went guarding  */
#define BOOTDEC_OFF_DEEP_BACK    4u   /* woke, hold not completed, slept on  */

typedef struct {
    uint32_t prev_uptime_s;   /* uptime at the previous shutdown, 0 unknown  */
    int32_t  vbat_mv;         /* first reading this boot, -1 = none yet      */
    uint16_t prev_alerts;     /* counters carried off the previous session   */
    uint16_t prev_vetoed;
    uint16_t prev_drops;
    uint8_t  reset_reason;    /* esp_reset_reason()                          */
    uint8_t  wake_cause;      /* esp_sleep_get_wakeup_cause()                */
    uint8_t  decision;        /* BOOTDEC_*                                   */
    uint8_t  chg;             /* CHG_STAT at boot: 0 no, 1 yes, 2 unknown    */
    uint8_t  btn_level;       /* GPIO21 at boot: 0 pressed, 1 up, 2 unknown  */
} bootrec_t;

/* Called ONCE, as early as possible, before the guard starts. Validates the
 * RTC blob, initialises it when the magic or the checksum is wrong (a cold
 * power-on, or corruption), carries the previous session's counters into the
 * new record, and zeroes the live ones. Returns true when the blob survived,
 * false when it had to be initialised. */
bool bootrec_begin(void);

/* The reset reason and wake cause this boot actually had, cached by
 * bootrec_begin so every later caller reads the same two values. */
uint8_t bootrec_reset_reason(void);
uint8_t bootrec_wake_cause(void);
bool    bootrec_was_deepsleep_wake(void);

/* Fill in the newest record as the boot proceeds. */
void bootrec_set_vbat(int mv, int chg_level);
void bootrec_set_decision(uint8_t decision);
void bootrec_set_btn(int level);

/* ---- the wake-loop fail-safe --------------------------------------------
 *
 * A wake that cannot complete its own reason - EXT1 with the pin not held,
 * a timer wake whose reading still refuses to resume - is recorded here with
 * an RTC timestamp that survives the sleep. More than three inside ten
 * seconds and the device STOPS SLEEPING and goes back to guarding.
 *
 * The asymmetry is deliberate and is written in C.2: "the unit turned itself
 * on" is an acceptable failure, "the unit will not wake" is not. A box that
 * cannot be woken on a hillside is indistinguishable from a dead one. */
bool bootrec_note_failed_wake(void);
void bootrec_clear_failed_wakes(void);
uint8_t bootrec_failed_wakes(void);

/* The newest record, for the INFO page. Never NULL. */
const bootrec_t *bootrec_newest(void);

/* Human-readable names, for INFO and for `U boot`. */
const char *bootrec_reason_name(uint8_t reason);
const char *bootrec_wake_name(uint8_t cause);
const char *bootrec_decision_name(uint8_t decision);

/* `U boot`: print the ring, newest first. */
void bootrec_dump(void);

/* ---- the live counters, mirrored so they survive the morning's reset ---- */
void     bootrec_count_alert(uint8_t tier, float hz, uint32_t uptime_ms);
void     bootrec_count_veto(void);
void     bootrec_count_drop(void);
void     bootrec_note_uptime(uint32_t uptime_ms);   /* call every 10 s */

/* Boots since the last cold power-on. A morning that finds this at 30 has a
 * device that has been resetting all night, which is a finding. */
uint16_t bootrec_boots(void);
uint16_t bootrec_alerts(void);
uint16_t bootrec_vetoed(void);
uint16_t bootrec_drops(void);
bool     bootrec_last_alert(uint8_t *tier, float *hz, uint32_t *uptime_ms);

/* ---- the event ring mirror ---------------------------------------------
 * Pushed alongside every evlog write. `V` prints the RAM ring first and then
 * this, whenever the reset reason says the RAM ring is not the whole story. */
void     bootrec_mirror_push(const evlog_rec_t *r);
uint16_t bootrec_mirror_count(void);
int      bootrec_mirror_recent(evlog_rec_t *dst, int max);
void     bootrec_mirror_dump(void);
