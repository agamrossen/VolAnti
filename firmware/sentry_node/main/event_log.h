/*
 * event_log.h - the last 64 alerts, in RAM, so a field day is reconstructible.
 *
 * In standalone the device has no host. Every alert record it emits goes into
 * a USB link that nothing is draining, and drop mode throws them away -
 * correctly, because blocking the detector to preserve telemetry would be the
 * wrong trade. Without this ring, a day of guarding on a power bank leaves one
 * recoverable fact: that the buzzer went off some number of times.
 *
 * That is not enough to decide anything. Whether a tier latched more than
 * twice, whether any latch ran longer than 30 s, whether a confuser class
 * repeated - none of it survives in an operator's memory across four hours in
 * a field. So: a fixed RAM ring, written on the alert edges, read by one
 * command.
 *
 * It is not persistent. There are no flash writes here, on the hot path or off
 * it: a pulled battery loses the ring, which is why the paper log exists
 * beside it. It is not a trace - it holds decisions, not frames. And it does
 * not self-learn: nothing reads it back into the detector.
 *
 * Entries are stamped in uptime milliseconds, because that is the only clock
 * the device has. Photographing the status screen next to a phone clock at
 * power-on converts the whole ring to civil time afterwards.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define EVLOG_N 64

/* Why an alert ended. The distinction that matters in the field is between
 * "the source stopped" and "the operator silenced it", because a self-latch
 * looks like the first and is actually neither. */
#define EVLOG_REL_OPEN     0u   /* still open                                */
#define EVLOG_REL_SOURCE   1u   /* every tier released - the source went     */
#define EVLOG_REL_SNOOZE   2u   /* the operator pressed the button           */
#define EVLOG_REL_STOP     3u   /* the run ended with the alert still open   */
/* ---- not alerts: the two battery transitions ---------------------------
 * They are in the alert ring because the ring is the only thing that survives
 * a field day, and because they are the entries that explain the gaps in it.
 * "The device stopped" and "the device ran out of battery" look identical in
 * a capture and are completely different facts. */
#define EVLOG_REL_LOWBATT  4u   /* crossed into LOW, sustained               */
#define EVLOG_REL_BATTERY  5u   /* stopped: the cell is empty                */
/* The park that did not happen: the empty rule would have fired and charging
 * inhibited it. Recorded because a rule that silently does not fire is
 * indistinguishable from a rule that is not there, and this one guards against
 * a sagging cell on a charger parking the device. */
#define EVLOG_REL_EMPTY_INHIB 6u

/* ---- fail-safe to guard ------------------------------------------------
 * Three ways a battery verdict can be refused, each recorded, because a rule
 * that silently declines to fire is indistinguishable from a rule that is not
 * there. The device guards through all three. */
#define EVLOG_REL_VBAT_BAD   7u   /* R1: unreadable or out of trust window  */
#define EVLOG_REL_VBAT_JUMP  8u   /* R2: fell faster than a cell can        */
#define EVLOG_REL_WAKE_LOOP  9u   /* R3: woke and could not finish, thrice  */

/* A tier id OUTSIDE the ALERT_TIER_* space, for entries that are not alerts.
 * Deliberately far away from it: a battery note must never be counted as a
 * detection by anything that tallies tiers. */
#define EVLOG_TIER_POWER 200u

/* ---- the output audit --------------------------------------------------
 * An alert that is cut short - a beep and a screen change, or a beep with no
 * page at all - should be readable off the ring rather than reconstructed
 * from a timeline afterwards.
 *
 * BUZZ rows name who commanded the buzzer, so a third caller would be visible
 * rather than inferred. `hz` carries the commanded duration in ms and `score`
 * the measured one; the two differing is the short alert. */
#define EVLOG_TIER_BUZZ  201u
#define EVLOG_BUZZ_PRESS         0u
#define EVLOG_BUZZ_ALERT_LOCAL   1u
#define EVLOG_BUZZ_ALERT_REMOTE  2u
#define EVLOG_BUZZ_OTHER         3u

/* ALERT_BEGIN carries the tier; ALERT_END carries why it stopped, so a burst
 * that ended early is distinguishable from one that was silenced. */
/* A detection that arrived while the box was snoozed: heard, counted, ringed
 * and transmitted, and deliberately not sounded. It is not an alert - nothing
 * alerted - so evlog_is_alert() excludes it, and the test page shows it as
 * QUIET beside the alert count. Snooze is the operator's own silence and is
 * the only silence there is; no page or mode suppresses an alert. */
#define EVLOG_TIER_QUIET  204u
void evlog_note_quiet(uint8_t tier, float score, float thr, uint32_t now_ms);

#define EVLOG_TIER_ABEGIN 202u
#define EVLOG_TIER_AEND   203u
#define EVLOG_AEND_COMPLETE   0u
#define EVLOG_AEND_SILENCED   1u
#define EVLOG_AEND_PREEMPTED  2u
#define EVLOG_AEND_ABORTED    3u

void evlog_note_buzz(uint8_t src, uint32_t cmd_ms, uint32_t meas_ms,
                     uint32_t now_ms);
void evlog_note_alert_begin(uint8_t tier, float score, float thr,
                            uint32_t now_ms);
void evlog_note_alert_end(uint8_t why, uint32_t dur_ms, uint32_t now_ms);

typedef struct {
    uint32_t uptime_ms;     /* when it fired                                 */
    uint32_t dur_ms;        /* 0 while open                                  */
    float    hz;            /* v1/Tier-2: f0. Tier-3: the shaft RATE         */
    int16_t  score_x100;    /* v1/Tier-2: score. Tier-3: W. Both x100        */
    uint8_t  tier;          /* ALERT_TIER_*                                  */
    uint8_t  release;       /* EVLOG_REL_*                                   */
    uint16_t seq;           /* 1-based, survives ring wrap                   */
    /* Every tier that fired. `tier` above is the first of a priority list
     * (v1, then T2, T3, T4), so an alert where T3 latched at its natural 2 s
     * and v1 crossed in the same frame is recorded as v1 and the T3 latch
     * vanishes from the record - which makes "v1 never locked and a slow tier
     * fired on time" indistinguishable from "v1 fired late". This carries the
     * whole set.
     *
     * Taken out of the existing `pad`, so sizeof(evlog_rec_t) does not move
     * and neither does the ring's footprint. */
    uint8_t  fired_set;     /* EVLOG_FS_* bitmask, 0 on an older entry     */
    uint8_t  pad;
} evlog_rec_t;

#define EVLOG_FS_V1  0x01u
#define EVLOG_FS_T2  0x02u
#define EVLOG_FS_T3  0x04u
#define EVLOG_FS_T4  0x08u

/* Record which tiers were up on the deciding frame. Written onto the entry
 * evlog_open() has just opened; a no-op if none is open. Separate from
 * evlog_open() so the four existing call sites keep their signature. */
void evlog_set_fired(uint8_t fired_set);

/* Open an entry. `hz` and `score` are the values on the firing frame - the
 * evidence, not a later maximum, because a maximum invites the question "over
 * what window" and the firing frame does not. */
void evlog_open(uint8_t tier, float hz, float score, uint32_t now_ms);

/* Close the open entry, if there is one. Safe to call when there is not. */
void evlog_close(uint32_t now_ms, uint8_t release);

/* The operator pressed the button while an alert was open. Recorded on the
 * open entry so a self-latch can be told from a real source afterwards. */
void evlog_note_snooze(uint32_t now_ms);

/* One closed entry for a battery transition. Not open/close, because a
 * battery state has no duration to measure - it is a moment, and pretending
 * otherwise would put a meaningless dur_ms next to the meaningful ones. The
 * millivolts ride in the score field, which is what a reader of the dump
 * wants beside "LOW BATT" anyway. */
void evlog_note_power(bool empty, int mv, uint32_t now_ms);

/* The empty rule tripped but was inhibited (charging, or inside the boot
 * grace window). Written once per inhibited episode, not once per tick. */
void evlog_note_power_inhibited(int mv, uint32_t now_ms);

/* The three refusals above. `mv` is the reading that caused it, or the raw
 * ADC count when there was no reading to speak of. */
void evlog_note_vbat(uint8_t release, int mv, uint32_t now_ms);

void evlog_reset(void);

/* Total alerts since boot, which is what the status screen shows. Not the ring
 * occupancy: after 64 the ring wraps and the count keeps counting. */
uint16_t evlog_count(void);
uint16_t evlog_count_tier(uint8_t tier);

/* ---- what the operator display reads ------------------------------------
 *
 * Both counts are monotonic, which is the point of having two functions
 * rather than reusing evlog_count_tier(). The main page shows "alerts N,
 * remote M" side by side, and a monotonic total beside a ring-scoped one
 * would start disagreeing on the 65th alert of a night with nothing on the
 * screen to hint that it had. Battery notes are excluded from both. */
uint16_t evlog_count_local(void);
uint16_t evlog_count_remote(void);

/* Uptime of the most recent alert, for MAIN's "last alert" age. False when
 * nothing has fired yet - which is a different thing from "0 ms ago" and the
 * page says so ("NONE"). */
bool evlog_last_alert_ms(uint32_t *out);

/* The newest `max` alerts, NEWEST FIRST, for the RECENT page. Battery notes
 * are skipped: that page answers "what did it hear". Returns how many were
 * written. */
int evlog_recent(evlog_rec_t *dst, int max);

/* The tier as at most two characters, for the RECENT page's 28 columns. */
const char *evlog_tier_short(uint8_t tier);

/* Text, oldest first, over the trace link. One line per entry. */
void evlog_dump(void);
