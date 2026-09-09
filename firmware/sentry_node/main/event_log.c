#include "event_log.h"
#include "boot_rec.h"

#include <stdio.h>
#include <string.h>

#include "alert_ui.h"
#include "trace.h"

static evlog_rec_t s_ring[EVLOG_N];
static uint16_t    s_n;          /* entries written, saturating at 0xFFFF   */
static uint16_t    s_seq;        /* 1-based sequence, never reused          */
static int         s_open = -1;  /* index of the open entry, or -1          */
/* MONOTONIC PER-KIND COUNTS, for the operator display.
 *
 * evlog_count() is monotonic but untyped, and evlog_count_tier() is typed but
 * only sees the last 64 entries. MAIN says "alerts N / remote M" and both
 * halves must mean the same thing - a monotonic total beside a ring-scoped
 * one would silently start disagreeing on the 65th alert of a night, and the
 * screen would give no hint that it had. */
static uint16_t    s_n_local;    /* this box heard it                       */
static uint16_t    s_n_remote;   /* a peer heard it and said so             */
static uint32_t    s_last_ms;    /* uptime of the most recent alert         */
static bool        s_have_last;

void evlog_reset(void)
{
    memset(s_ring, 0, sizeof(s_ring));
    s_n = 0;
    s_seq = 0;
    s_n_local = 0;
    s_n_remote = 0;
    s_last_ms = 0;
    s_have_last = false;
    s_open = -1;
}

/* ---- is this row an alert? ----------------------------------------------
 * The ring carries two different things: detections, whose tier is in the
 * ALERT_TIER_* space, and an audit of what the box then did about them -
 * ABEGIN, AEND, BUZZ, QUIET - alongside battery notes. Everything that
 * tallies or lists "alerts" wants the first kind and only the first kind.
 *
 * A predicate of `tier != EVLOG_TIER_POWER` is right only while POWER is the
 * only non-alert tier there is. Four more were added
 * beside it at 201-204 and this predicate was never generalised, so:
 *
 *   ONE alarm moved ALERTS by FOUR - the V1 row, then ABEGIN, BUZZ and AEND.
 *   ONE alarm filled the whole RECENT page, which calls itself "the last
 *     five alerts" and was showing one.
 *   The TEST page's LAST line read the newest row, which after every alarm
 *     is AEND - whose `hz` field carries a DURATION - so it reported the
 *     alert's length in milliseconds as a frequency.
 *
 * The visible symptom is an alert counter that jumps by four for a single
 * audible alarm, which reads as the box having detected passes it stayed
 * silent for. A number that cannot be reconciled with what the operator just
 * heard is worse than no number.
 *
 * The rule is the one stated above EVLOG_TIER_POWER: a tier id outside the
 * ALERT_TIER_* space is not an alert. The predicate has to be that rule and
 * not a list, which is this
 * project's most-repeated defect and now its most-repeated lesson. */
static bool evlog_is_alert(uint8_t tier)
{
    return tier >= ALERT_TIER_V1 && tier <= ALERT_TIER_REMOTE;
}

void evlog_open(uint8_t tier, float hz, float score, uint32_t now_ms)
{
    /* An already-open entry means the previous alert never closed. Close it
     * against this moment rather than losing it - a dropped entry is worse
     * than an approximate duration. */
    if (s_open >= 0) {
        evlog_close(now_ms, EVLOG_REL_SOURCE);
    }
    const int i = (s_n < 0xFFFFu ? s_n : 0) % EVLOG_N;
    evlog_rec_t *r = &s_ring[i];
    r->uptime_ms = now_ms;
    r->dur_ms = 0;
    r->hz = hz;
    /* x100 into an int16: v1 scores run to ~4 and W to ~50, so the range is
     * ample and the resolution (0.01) is finer than any decision. Clamped
     * rather than wrapped, because a wrapped score reads as a real number. */
    float sx = score * 100.0f;
    if (sx > 32767.0f)  { sx = 32767.0f; }
    if (sx < -32768.0f) { sx = -32768.0f; }
    r->score_x100 = (int16_t)sx;
    r->tier = tier;
    r->release = EVLOG_REL_OPEN;
    r->seq = ++s_seq;
    /* Only a DETECTION counts as an alert. The audit rows this same ring
     * carries - ABEGIN, BUZZ, AEND, QUIET - and battery notes do not. */
    if (evlog_is_alert(tier)) {
        uint16_t *c = (tier == ALERT_TIER_REMOTE) ? &s_n_remote : &s_n_local;
        if (*c < 0xFFFFu) {
            (*c)++;
        }
        s_last_ms = now_ms;
        s_have_last = true;
    }
    if (s_n < 0xFFFFu) {
        s_n++;
    }
    s_open = i;
}

void evlog_close(uint32_t now_ms, uint8_t release)
{
    if (s_open < 0) {
        return;
    }
    evlog_rec_t *r = &s_ring[s_open];
    r->dur_ms = now_ms - r->uptime_ms;
    /* A snooze already recorded on this entry is the more informative reason;
     * do not overwrite it with the generic one. */
    if (r->release == EVLOG_REL_OPEN) {
        r->release = release;
    }
    /* MIRROR IT WHERE A RESET CANNOT REACH. The RAM ring dies with the
     * morning's first port open; this copy does not. Pushed on CLOSE rather
     * than on open so the mirrored entry carries its duration and its real
     * release reason rather than a half-written one. */
    bootrec_mirror_push(r);
    s_open = -1;
}

void evlog_note_power(bool empty, int mv, uint32_t now_ms)
{
    /* Written through evlog_open so an alert that happened to be open is
     * closed properly first, then closed immediately with its own release
     * code: a battery transition is a moment, not an interval. */
    evlog_open(EVLOG_TIER_POWER, 0.0f, (float)mv / 1000.0f, now_ms);
    evlog_close(now_ms, empty ? EVLOG_REL_BATTERY : EVLOG_REL_LOWBATT);
}

void evlog_note_power_inhibited(int mv, uint32_t now_ms)
{
    evlog_open(EVLOG_TIER_POWER, 0.0f, (float)mv / 1000.0f, now_ms);
    evlog_close(now_ms, EVLOG_REL_EMPTY_INHIB);
}

void evlog_note_vbat(uint8_t release, int mv, uint32_t now_ms)
{
    evlog_open(EVLOG_TIER_POWER, 0.0f, (float)mv / 1000.0f, now_ms);
    evlog_close(now_ms, release);
}

void evlog_set_fired(uint8_t fired_set)
{
    /* Same guard as evlog_note_snooze(): s_open is the index of the entry
     * currently open, or negative when there is none. */
    if (s_open >= 0) {
        s_ring[s_open].fired_set = fired_set;
    }
}

void evlog_note_snooze(uint32_t now_ms)
{
    (void)now_ms;
    if (s_open >= 0) {
        s_ring[s_open].release = EVLOG_REL_SNOOZE;
    }
}

/* THE TIER, IN AT MOST TWO CHARACTERS, for the RECENT page - which has 28
 * columns for five events and cannot spend six of them on a word. The long
 * names stay in evlog_dump(), where there is room and where the reader has a
 * laptop. */
/* ---- the output audit ------------------------------------------------- */
void evlog_note_buzz(uint8_t src, uint32_t cmd_ms, uint32_t meas_ms,
                     uint32_t now_ms)
{
    evlog_open(EVLOG_TIER_BUZZ, (float)cmd_ms, (float)meas_ms, now_ms);
    evlog_close(now_ms, src);
}

void evlog_note_quiet(uint8_t tier, float score, float thr, uint32_t now_ms)
{
    evlog_open(EVLOG_TIER_QUIET, thr, score, now_ms);
    evlog_close(now_ms, tier);
}

void evlog_note_alert_begin(uint8_t tier, float score, float thr,
                            uint32_t now_ms)
{
    evlog_open(EVLOG_TIER_ABEGIN, thr, score, now_ms);
    evlog_close(now_ms, tier);
}

void evlog_note_alert_end(uint8_t why, uint32_t dur_ms, uint32_t now_ms)
{
    evlog_open(EVLOG_TIER_AEND, (float)dur_ms, 0.0f, now_ms);
    evlog_close(now_ms, why);
}

const char *evlog_tier_short(uint8_t t)
{
    switch (t) {
    case ALERT_TIER_V1:     return "V1";
    case ALERT_TIER_T2:     return "T2";
    case ALERT_TIER_T3:     return "T3";
    case ALERT_TIER_T4:     return "T4";
    case ALERT_TIER_REMOTE: return "RM";
    case EVLOG_TIER_BUZZ:   return "BZ";
    case EVLOG_TIER_ABEGIN: return "AB";
    case EVLOG_TIER_AEND:   return "AE";
    case EVLOG_TIER_QUIET:  return "QT";
    case EVLOG_TIER_POWER:  return "PW";
    default:                return "??";
    }
}

uint16_t evlog_count(void) { return s_seq; }
uint16_t evlog_count_local(void)  { return s_n_local; }
uint16_t evlog_count_remote(void) { return s_n_remote; }

bool evlog_last_alert_ms(uint32_t *out)
{
    if (!s_have_last) {
        return false;
    }
    *out = s_last_ms;
    return true;
}

int evlog_recent(evlog_rec_t *dst, int max)
{
    if (!dst || max <= 0) {
        return 0;
    }
    const int n = (s_n < EVLOG_N) ? (int)s_n : EVLOG_N;
    /* THE WRAP, worked out once in evlog_dump() and reused rather than
     * re-derived: `start` is the oldest live entry. */
    const int start = (s_n <= EVLOG_N) ? 0 : (int)(s_n % EVLOG_N);
    int out = 0;
    for (int k = n - 1; k >= 0 && out < max; k--) {   /* newest first */
        const evlog_rec_t *r = &s_ring[(start + k) % EVLOG_N];
        if (!evlog_is_alert(r->tier)) {
            continue;               /* the RECENT page lists ALERTS */
        }
        dst[out++] = *r;
    }
    return out;
}

uint16_t evlog_count_tier(uint8_t tier)
{
    const int n = (s_n < EVLOG_N) ? (int)s_n : EVLOG_N;
    uint16_t c = 0;
    for (int i = 0; i < n; i++) {
        if (s_ring[i].tier == tier) {
            c++;
        }
    }
    return c;
}

static const char *tier_name(uint8_t t)
{
    switch (t) {
    case ALERT_TIER_V1: return "v1";
    case ALERT_TIER_T2: return "T2";
    case ALERT_TIER_T3: return "T3";
    /* Tier-4 reaches evlog_open() by exactly the path the other three do, so
     * omitting it here makes a Tier-4 event unattributable in the one record
     * that survives a field day. */
    case ALERT_TIER_T4: return "T4";
    /* Not a tier: a peer heard it and said so over the radio. The Hz column
     * holds the originating device id and the score column its tier, which
     * is where the eighteen-byte packet's two facts survive a field day. */
    case ALERT_TIER_REMOTE: return "RM";
    case EVLOG_TIER_POWER: return "batt";
    default:            return "??";
    }
}

static const char *rel_name(uint8_t r)
{
    switch (r) {
    case EVLOG_REL_SOURCE:  return "source-gone";
    case EVLOG_REL_SNOOZE:  return "SNOOZED";
    case EVLOG_REL_STOP:    return "run-stopped";
    case EVLOG_REL_LOWBATT: return "LOW BATTERY";
    case EVLOG_REL_BATTERY: return "BATTERY EMPTY - stopped";
    case EVLOG_REL_EMPTY_INHIB: return "EMPTY-INHIBITED (charging/grace)";
    case EVLOG_REL_VBAT_BAD:  return "VBAT-BAD (untrusted, kept guarding)";
    case EVLOG_REL_VBAT_JUMP: return "VBAT-JUMP (fell too fast, refused)";
    case EVLOG_REL_WAKE_LOOP: return "WAKE-LOOP (stopped sleeping)";
    default:                return "OPEN";
    }
}

void evlog_dump(void)
{
    char b[320];        /* the banner is four lines now; the rows need ~90 */
    const int n = (s_n < EVLOG_N) ? (int)s_n : EVLOG_N;
    snprintf(b, sizeof(b),
             "\nEVENTS %u total (v1 %u, T2 %u, T3 %u, T4 %u), ring holds %d\n"
             "  seq   uptime      dur   tier   Hz     score/W  release\n"
             "  (tier 'batt' rows are not detections: score is VOLTS;\n"
             "   tier 'RM' rows came from a PEER: Hz is its id, score its "
             "tier)\n",
             (unsigned)s_seq, (unsigned)evlog_count_tier(ALERT_TIER_V1),
             (unsigned)evlog_count_tier(ALERT_TIER_T2),
             (unsigned)evlog_count_tier(ALERT_TIER_T3),
             (unsigned)evlog_count_tier(ALERT_TIER_T4), n);
    trace_text(b);
    if (n == 0) {
        trace_text("  (none)\n");
        return;
    }
    /* Oldest first. After a wrap the oldest entry is the one after the newest,
     * so walk from there rather than from index 0. */
    const int start = (s_n <= EVLOG_N) ? 0 : (int)(s_n % EVLOG_N);
    for (int k = 0; k < n; k++) {
        const evlog_rec_t *r = &s_ring[(start + k) % EVLOG_N];
        if (r->seq == 0) {
            continue;
        }
        /* PicoLibC's printf has no float support - this is why the whole trace
         * is binary. Integers and a hand-placed decimal point. */
        const unsigned up_s = r->uptime_ms / 1000u;
        const int hz10 = (int)(r->hz * 10.0f + (r->hz >= 0 ? 0.5f : -0.5f));
        snprintf(b, sizeof(b),
                 "  %4u  %3u:%02u.%u  %5u.%us  %-4s %5d.%1d  %4d.%02d  %s\n",
                 (unsigned)r->seq,
                 (unsigned)(up_s / 60u), (unsigned)(up_s % 60u),
                 (unsigned)((r->uptime_ms % 1000u) / 100u),
                 (unsigned)(r->dur_ms / 1000u),
                 (unsigned)((r->dur_ms % 1000u) / 100u),
                 tier_name(r->tier),
                 hz10 / 10, (hz10 < 0 ? -hz10 : hz10) % 10,
                 r->score_x100 / 100,
                 (r->score_x100 < 0 ? -r->score_x100 : r->score_x100) % 100,
                 rel_name(r->release));
        trace_text(b);
    }
}
