/* boot_rec.c - see boot_rec.h for why this module exists at all. */

#include "boot_rec.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_rtc_time.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "trace.h"

/* "SNB3": bumped whenever the blob layout changes, because a stale layout
 * read back through a new struct is worse than no record at all. */
#define BOOTREC_MAGIC 0x534E4233u

typedef struct {
    uint32_t magic;
    uint32_t sum;                 /* FNV-1a over everything after THIS BLOCK */

    /* ---- outside the checksum, deliberately -----------------------------
     * blob_sum() starts at offsetof(rtcblob_t, rec), so everything above
     * that line is excluded. `drops` lives here because it is incremented
     * from the frame loop on every overrun and bootrec_count_drop() does NOT
     * re-seal - a 1.4 KB hash on the audio path is exactly the cost this
     * project has mispredicted four times, and that reasoning is sound.
     *
     * What was NOT sound was leaving the counter inside the checksummed
     * region while not re-sealing it. Every drop after the last seal left the
     * stored sum describing a blob that no longer existed, so the next boot
     * discarded the whole ring. It read `magic` intact with
     * `stored=84bf5d98 calc=eef9124e`, IDENTICALLY across runs - which is what
     * says deterministic bookkeeping rather than a race, and is why adding a
     * spinlock changed nothing.
     *
     * A diagnostic counter is the right thing to leave unprotected: the magic
     * still guards it, and a wrong drop count costs a wrong number on INFO
     * where a discarded ring costs a whole field night. */
    uint16_t  drops;
    uint16_t  pad_hot;

    bootrec_t rec[BOOTREC_N];
    uint8_t   rec_head;           /* index of the NEWEST record              */
    uint8_t   rec_count;
    uint8_t   pad0;
    uint8_t   pad1;

    uint16_t  alerts;
    uint16_t  vetoed;
    uint16_t  pad2;
    uint32_t  uptime_ms;          /* written every 10 s, so a power cut is    */
                                  /* bounded rather than unknown              */
    uint32_t  last_alert_ms;
    float     last_alert_hz;
    uint8_t   last_alert_tier;
    uint8_t   pad3;
    uint16_t  pad4;

    evlog_rec_t mir[BOOTMIRROR_N];
    uint16_t  mir_head;           /* next slot to write                       */
    uint16_t  mir_count;
    uint16_t  boots;   /* since the last cold power-on */

    /* R3. The RTC clock is used rather than esp_timer because it is the only
     * one that keeps counting through a deep sleep; esp_timer restarts at
     * zero on every wake, which would make every loop look instantaneous. */
    uint64_t  wake_fail_us[4];
    uint8_t   wake_fail_n;
    uint8_t   pad5;
    uint16_t  pad6;
} rtcblob_t;

/* THE ONE OBJECT THAT OUTLIVES A RESET. RTC_NOINIT_ATTR, never
 * RTC_DATA_ATTR: see the header. */
static RTC_NOINIT_ATTR rtcblob_t s_blob;

/* ONE CAUSE OUT OF THE BITMASK. IDF v6 deprecates
 * esp_sleep_get_wakeup_cause() in favour of the plural form, which returns
 * BIT(ESP_SLEEP_WAKEUP_x) per active source (verified against
 * components/esp_hw_support/sleep_modes.c, not assumed). Only one source is
 * ever armed at a time on this device, but the order below is the order that
 * matters if that ever stops being true: a button press outranks a timer,
 * because the operator is standing there. */
static uint8_t wake_cause_one(void)
{
    const uint32_t m = esp_sleep_get_wakeup_causes();
    static const uint8_t pref[] = {
        (uint8_t)ESP_SLEEP_WAKEUP_EXT1,
        (uint8_t)ESP_SLEEP_WAKEUP_EXT0,
        (uint8_t)ESP_SLEEP_WAKEUP_GPIO,
        (uint8_t)ESP_SLEEP_WAKEUP_TIMER,
        (uint8_t)ESP_SLEEP_WAKEUP_UART,
    };
    for (unsigned i = 0; i < sizeof(pref) / sizeof(pref[0]); i++) {
        if (m & (1u << pref[i])) {
            return pref[i];
        }
    }
    return (uint8_t)ESP_SLEEP_WAKEUP_UNDEFINED;
}

static uint8_t s_reset_reason;
static uint8_t s_wake_cause;
static bool    s_survived;

/* ---- the alert phase, in one word, outside the blob ---------------------
 * The alert phase has to survive a reset so a brownout can be attributed to
 * the load that was switching on. It cannot live in the RTC blob, and the
 * measurement says why.
 *
 * The blob is validated by a checksum over 1448 bytes. A clean reset
 * preserves it - measured, `rtc=survived`, stored == calc. A BROWNOUT does
 * not: the magic survives and the sum does not, because the collapse lands
 * between a mutation and the re-seal that follows it. That is not a rare
 * race. The brownout happens AT AN ALERT, and the alert path itself writes
 * the blob, so the tear is correlated with the write rather than random -
 * which is why three resets in a row came up INITIALISED.
 *
 * So the one event the phase exists to attribute is exactly the event that
 * would destroy it. It lives in its own word instead. A single aligned
 * 32-bit store either lands or it does not; there is no torn state. The tag
 * and the complement are what stop uninitialised RTC memory from reading as
 * a valid phase, since there is no checksum here to do it. */
static RTC_NOINIT_ATTR uint32_t s_phase_word;
#define PHASE_TAG   0x5041u              /* 'PA' */
#define PHASE_ENC(p) (((uint32_t)PHASE_TAG << 16) | \
                      ((uint32_t)(uint8_t)(p) << 8) | \
                      (uint8_t)(0xFFu - (uint8_t)(p)))
static uint8_t s_phase_at_reset = BOOTPHASE_UNKNOWN;

void bootrec_set_alert_phase(uint8_t phase)
{
    s_phase_word = PHASE_ENC(phase);
}

uint8_t bootrec_phase_at_reset(void) { return s_phase_at_reset; }

const char *bootrec_phase_name(uint8_t p)
{
    switch (p) {
    case BOOTPHASE_IDLE:         return "IDLE";
    case BOOTPHASE_TX1:          return "TX1";
    case BOOTPHASE_TX2:          return "TX2";
    case BOOTPHASE_TX3:          return "TX3";
    case BOOTPHASE_OUTPUTS_RAMP: return "OUTPUTS_RAMP";
    case BOOTPHASE_OUTPUTS:      return "OUTPUTS";
    case BOOTPHASE_SCREEN:       return "SCREEN";
    case BOOTPHASE_HOLD:         return "HOLD";
    case BOOTPHASE_RETURN:       return "RETURN";
    default:                     return "unknown";
    }
}

/* ---- WHY THE BLOB DID NOT SURVIVE (diagnostic) ------------------------
 * The header asserts that .rtc_noinit is left alone by every reset path.
 * That was reasoned, never measured, and the board reports "INITIALISED"
 * after a plain USB reset as well as after a brownout. These three numbers,
 * read BEFORE anything in bootrec_begin() touches the blob, separate the
 * only three explanations there are:
 *   magic 0, stored 0        -> the memory came back CLEARED
 *   magic SNB2, stored!=calc -> the memory survived, the SEAL went stale
 *   magic anything else      -> true garbage, i.e. a cold power-on
 * A record that cannot say which of these happened cannot choose a remedy. */
static uint32_t s_raw_magic;
static uint32_t s_raw_stored;
static uint32_t s_raw_calc;

/* ---- integrity ---------------------------------------------------------
 * On a cold power-on this memory is whatever the silicon woke up holding, so
 * a magic word alone is not enough: a checksum is what separates "a previous
 * session left this" from "these bytes happen to start with our magic". */
static uint32_t blob_sum(void)
{
    const uint8_t *p = (const uint8_t *)&s_blob + offsetof(rtcblob_t, rec);
    const size_t   n = sizeof(rtcblob_t) - offsetof(rtcblob_t, rec);
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

/* ---- mutate and seal are one operation ----------------------------------
 * s_blob is written from four tasks - alert_ui, event_log, power_mon and the
 * guard - and every writer does a read-modify-write followed by a full
 * re-seal. Interleave two of those and the sum stored last describes neither
 * state, so the next boot reads magic intact and the checksum wrong and
 * throws the whole ring away.
 *
 * This stays latent at a few writes a second, where the torn window is too
 * small to hit. Add three ring rows per alert and a 20-alert soak comes back
 * initialised, with the seal stale and the memory perfectly intact.
 *
 * A spinlock is the right tool for this tear and the wrong one for the other
 * tear in this same blob: that one is a brownout, and no critical section can
 * mask a power collapse. This one is a preemption, which is exactly what a
 * critical
 * section is for. The two faults look identical at the next boot and have
 * nothing else in common.
 *
 * The cost is the FNV over 1448 bytes with interrupts off on this core -
 * about 4.3k cycles, ~18 us at 240 MHz. The audio path is DMA-buffered and
 * runs on the other core. Every caller is task context; none is an ISR. */
static portMUX_TYPE s_blob_mux = portMUX_INITIALIZER_UNLOCKED;

#define BLOB_LOCK()   taskENTER_CRITICAL(&s_blob_mux)
#define BLOB_UNLOCK() taskEXIT_CRITICAL(&s_blob_mux)

static void blob_seal(void)
{
    s_blob.magic = BOOTREC_MAGIC;
    s_blob.sum   = blob_sum();
}

static bool blob_valid(void)
{
    return s_blob.magic == BOOTREC_MAGIC && s_blob.sum == blob_sum();
}

static bootrec_t *newest_mut(void)
{
    return &s_blob.rec[s_blob.rec_head % BOOTREC_N];
}

bool bootrec_begin(void)
{
    /* FIRST, before a single byte is written: what did the memory hold?
     * The phase word is read here too, for the same reason and before
     * anything can overwrite it. */
    {
        const uint32_t w = s_phase_word;
        const uint8_t  p = (uint8_t)((w >> 8) & 0xFFu);
        s_phase_at_reset = ((w >> 16) == PHASE_TAG &&
                            (uint8_t)(w & 0xFFu) == (uint8_t)(0xFFu - p))
                               ? p : (uint8_t)BOOTPHASE_UNKNOWN;
    }
    s_raw_magic  = s_blob.magic;
    s_raw_stored = s_blob.sum;
    s_raw_calc   = blob_sum();

    s_reset_reason = (uint8_t)esp_reset_reason();
    s_wake_cause   = wake_cause_one();

    const bool ok = blob_valid();
    s_survived = ok;

    uint32_t prev_uptime_s = 0;
    uint16_t prev_alerts = 0, prev_vetoed = 0, prev_drops = 0;

    if (ok) {
        prev_uptime_s = s_blob.uptime_ms / 1000u;
        prev_alerts   = s_blob.alerts;
        prev_vetoed   = s_blob.vetoed;
        prev_drops    = s_blob.drops;
        s_blob.rec_head = (uint8_t)((s_blob.rec_head + 1u) % BOOTREC_N);
        if (s_blob.rec_count < BOOTREC_N) {
            s_blob.rec_count++;
        }
    } else {
        /* Cold power-on or corruption. Everything starts here, and the first
         * record carries the reset reason so the two cases are told apart
         * afterwards rather than guessed at. */
        memset(&s_blob, 0, sizeof(s_blob));
        s_blob.rec_count = 1;
        s_blob.rec_head  = 0;
        s_blob.boots     = 0;
        s_blob.wake_fail_n = 0;
    }

    if (s_blob.boots < 0xFFFFu) {
        s_blob.boots++;
    }

    bootrec_t *r = newest_mut();
    memset(r, 0, sizeof(*r));
    r->reset_reason  = s_reset_reason;
    r->wake_cause    = s_wake_cause;
    r->decision      = BOOTDEC_UNKNOWN;
    r->chg           = 2u;                 /* unknown until power_mon reads it */
    r->vbat_mv       = -1;
    r->prev_uptime_s = prev_uptime_s;
    r->prev_alerts   = prev_alerts;
    r->prev_vetoed   = prev_vetoed;
    r->prev_drops    = prev_drops;

    /* The live counters belong to THIS session. The previous session's are
     * preserved in the record above before they are cleared. */
    s_blob.alerts = 0;
    s_blob.vetoed = 0;
    s_blob.drops  = 0;
    s_blob.uptime_ms = 0;
    s_blob.last_alert_ms = 0;
    s_blob.last_alert_hz = 0.0f;
    s_blob.last_alert_tier = 0;

    blob_seal();
    return ok;
}

uint8_t bootrec_reset_reason(void) { return s_reset_reason; }
uint8_t bootrec_wake_cause(void)   { return s_wake_cause; }

bool bootrec_was_deepsleep_wake(void)
{
    return s_reset_reason == (uint8_t)ESP_RST_DEEPSLEEP;
}

void bootrec_set_vbat(int mv, int chg_level)
{
    BLOB_LOCK();
    bootrec_t *r = newest_mut();
    if (r->vbat_mv < 0) {                  /* the FIRST reading, not the last */
        r->vbat_mv = mv;
        r->chg = (uint8_t)(chg_level < 0 ? 2 : (chg_level ? 1 : 0));
        blob_seal();
    }
    BLOB_UNLOCK();
}

void bootrec_set_btn(int level)
{
    BLOB_LOCK();
    newest_mut()->btn_level = (uint8_t)(level < 0 ? 2 : (level ? 1 : 0));
    blob_seal();
    BLOB_UNLOCK();
}

#define WAKE_LOOP_WINDOW_US 10000000ULL   /* ten seconds */
#define WAKE_LOOP_MAX       3             /* more than this and we stop */

bool bootrec_note_failed_wake(void)
{
    BLOB_LOCK();
    const uint64_t now = esp_rtc_get_time_us();

    /* Drop anything older than the window, keeping the array packed. */
    uint8_t k = 0;
    for (uint8_t i = 0; i < s_blob.wake_fail_n && i < 4; i++) {
        if (now - s_blob.wake_fail_us[i] < WAKE_LOOP_WINDOW_US) {
            s_blob.wake_fail_us[k++] = s_blob.wake_fail_us[i];
        }
    }
    if (k < 4) {
        s_blob.wake_fail_us[k++] = now;
    }
    s_blob.wake_fail_n = k;
    blob_seal();
    /* The unlock must precede the return. An earlier mechanical wrap put it
     * after, where it is unreachable, and the spinlock would have been held
     * for ever - the next blob write on any task would have deadlocked. */
    BLOB_UNLOCK();
    return k > WAKE_LOOP_MAX;
}

void bootrec_clear_failed_wakes(void)
{
    BLOB_LOCK();
    s_blob.wake_fail_n = 0;
    blob_seal();
    BLOB_UNLOCK();
}

uint8_t bootrec_failed_wakes(void) { return s_blob.wake_fail_n; }

void bootrec_set_decision(uint8_t decision)
{
    BLOB_LOCK();
    newest_mut()->decision = decision;
    blob_seal();
    BLOB_UNLOCK();
}

const bootrec_t *bootrec_newest(void)
{
    return &s_blob.rec[s_blob.rec_head % BOOTREC_N];
}

const char *bootrec_reason_name(uint8_t reason)
{
    switch ((esp_reset_reason_t)reason) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    case ESP_RST_USB:       return "USB";
    case ESP_RST_JTAG:      return "JTAG";
    default:                return "UNKNOWN";
    }
}

const char *bootrec_wake_name(uint8_t cause)
{
    switch ((esp_sleep_source_t)cause) {
    case ESP_SLEEP_WAKEUP_UNDEFINED: return "none";
    case ESP_SLEEP_WAKEUP_EXT0:      return "EXT0";
    case ESP_SLEEP_WAKEUP_EXT1:      return "EXT1";
    case ESP_SLEEP_WAKEUP_TIMER:     return "TIMER";
    case ESP_SLEEP_WAKEUP_GPIO:      return "GPIO";
    case ESP_SLEEP_WAKEUP_UART:      return "UART";
    default:                         return "other";
    }
}

const char *bootrec_decision_name(uint8_t decision)
{
    switch (decision) {
    case BOOTDEC_LISTENING:       return "LISTENING";
    case BOOTDEC_EMPTY:           return "EMPTY";
    case BOOTDEC_OFF_DEEP_RESUME: return "OFF-DEEP-RESUME";
    case BOOTDEC_OFF_DEEP_BACK:   return "OFF-DEEP-BACK";
    default:                      return "unknown";
    }
}

void bootrec_dump(void)
{
    char b[224];
    snprintf(b, sizeof(b),
             "\n==== BOOT RECORDS (newest first, %u kept, blob %s) ====\n",
             (unsigned)s_blob.rec_count, s_survived ? "survived" : "INITIALISED");
    trace_text(b);
    for (int i = 0; i < s_blob.rec_count && i < BOOTREC_N; i++) {
        const int idx = ((int)s_blob.rec_head - i + 2 * BOOTREC_N) % BOOTREC_N;
        const bootrec_t *r = &s_blob.rec[idx];
        /* No float printf on this libc: millivolts are integers already. */
        snprintf(b, sizeof(b),
                 "  [%d] reset=%-9s wake=%-5s vbat=%ld mV chg=%s btn=%s "
                 "decision=%s\n",
                 i, bootrec_reason_name(r->reset_reason),
                 bootrec_wake_name(r->wake_cause),
                 (long)r->vbat_mv,
                 r->chg == 2u ? "?" : (r->chg ? "yes" : "no"),
                 r->btn_level == 2u ? "?" : (r->btn_level ? "up" : "DOWN"),
                 bootrec_decision_name(r->decision));
        trace_text(b);
        snprintf(b, sizeof(b),
                 "        prev session: uptime %lu s  alerts %u  vetoed %u  "
                 "drops %u\n",
                 (unsigned long)r->prev_uptime_s, (unsigned)r->prev_alerts,
                 (unsigned)r->prev_vetoed, (unsigned)r->prev_drops);
        trace_text(b);
    }
    snprintf(b, sizeof(b),
             "  this session: alerts %u  vetoed %u  drops %u  uptime %lu s  "
             "mirror %u\n",
             (unsigned)s_blob.alerts, (unsigned)s_blob.vetoed,
             (unsigned)s_blob.drops, (unsigned long)(s_blob.uptime_ms / 1000u),
             (unsigned)s_blob.mir_count);
    trace_text(b);
    snprintf(b, sizeof(b),
             "  blob as found: magic=%08lx stored=%08lx calc=%08lx  (want "
             "magic=%08lx, stored==calc)\n",
             (unsigned long)s_raw_magic, (unsigned long)s_raw_stored,
             (unsigned long)s_raw_calc, (unsigned long)BOOTREC_MAGIC);
    trace_text(b);
}

/* ---- counters ---------------------------------------------------------- */

void bootrec_count_alert(uint8_t tier, float hz, uint32_t uptime_ms)
{
    BLOB_LOCK();
    if (s_blob.alerts < 0xFFFFu) {
        s_blob.alerts++;
    }
    s_blob.last_alert_tier = tier;
    s_blob.last_alert_hz   = hz;
    s_blob.last_alert_ms   = uptime_ms;
    blob_seal();
    BLOB_UNLOCK();
}

void bootrec_count_veto(void)
{
    BLOB_LOCK();
    if (s_blob.vetoed < 0xFFFFu) {
        s_blob.vetoed++;
    }
    blob_seal();
    BLOB_UNLOCK();
}

void bootrec_count_drop(void)
{
    BLOB_LOCK();
    if (s_blob.drops < 0xFFFFu) {
        s_blob.drops++;
    }
    /* NOT sealed here. A drop is counted from the frame loop's own accounting
     * and a 1.4 KB checksum on that path is exactly the kind of cost this
     * project has mispredicted four times. The next seal picks it up, and the
     * 10 s uptime write guarantees one within ten seconds. */
    BLOB_UNLOCK();
}

void bootrec_note_uptime(uint32_t uptime_ms)
{
    BLOB_LOCK();
    s_blob.uptime_ms = uptime_ms;
    blob_seal();
    BLOB_UNLOCK();
}

uint16_t bootrec_boots(void)  { return s_blob.boots; }
uint16_t bootrec_alerts(void) { return s_blob.alerts; }
uint16_t bootrec_vetoed(void) { return s_blob.vetoed; }
uint16_t bootrec_drops(void)  { return s_blob.drops; }

bool bootrec_last_alert(uint8_t *tier, float *hz, uint32_t *uptime_ms)
{
    if (s_blob.last_alert_ms == 0u && s_blob.last_alert_tier == 0u) {
        return false;
    }
    if (tier)      { *tier = s_blob.last_alert_tier; }
    if (hz)        { *hz = s_blob.last_alert_hz; }
    if (uptime_ms) { *uptime_ms = s_blob.last_alert_ms; }
    return true;
}

/* ---- the mirror -------------------------------------------------------- */

void bootrec_mirror_push(const evlog_rec_t *r)
{
    /* The null check is OUTSIDE the lock. Taking it and then returning would
     * hold the spinlock for ever, which is the same defect the mechanical
     * wrap left in bootrec_note_failed_wake(). */
    if (!r) {
        return;
    }
    BLOB_LOCK();
    s_blob.mir[s_blob.mir_head % BOOTMIRROR_N] = *r;
    s_blob.mir_head = (uint16_t)((s_blob.mir_head + 1u) % BOOTMIRROR_N);
    if (s_blob.mir_count < BOOTMIRROR_N) {
        s_blob.mir_count++;
    }
    blob_seal();
    BLOB_UNLOCK();
}

uint16_t bootrec_mirror_count(void) { return s_blob.mir_count; }

int bootrec_mirror_recent(evlog_rec_t *dst, int max)
{
    if (!dst || max <= 0) {
        return 0;
    }
    int n = s_blob.mir_count;
    if (n > max) {
        n = max;
    }
    for (int i = 0; i < n; i++) {
        const int idx = ((int)s_blob.mir_head - 1 - i + 2 * BOOTMIRROR_N)
                        % BOOTMIRROR_N;
        dst[i] = s_blob.mir[idx];
    }
    return n;
}

void bootrec_mirror_dump(void)
{
    char b[192];
    snprintf(b, sizeof(b),
             "\n==== RTC EVENT MIRROR (%u entries, survived reset %s) ====\n",
             (unsigned)s_blob.mir_count, bootrec_reason_name(s_reset_reason));
    trace_text(b);
    if (s_blob.mir_count == 0u) {
        trace_text("  (empty)\n");
        return;
    }
    for (int i = 0; i < s_blob.mir_count && i < BOOTMIRROR_N; i++) {
        const int idx = ((int)s_blob.mir_head - 1 - i + 2 * BOOTMIRROR_N)
                        % BOOTMIRROR_N;
        const evlog_rec_t *e = &s_blob.mir[idx];
        const int hz10 = (int)(e->hz * 10.0f + (e->hz >= 0.0f ? 0.5f : -0.5f));
        snprintf(b, sizeof(b),
                 "  seq %-5u t=%lu.%03lu s  tier %-3s  f0 %d.%d Hz  "
                 "score %d.%02d  dur %lu ms\n",
                 (unsigned)e->seq,
                 (unsigned long)(e->uptime_ms / 1000u),
                 (unsigned long)(e->uptime_ms % 1000u),
                 evlog_tier_short(e->tier),
                 hz10 / 10, (hz10 < 0 ? -hz10 : hz10) % 10,
                 e->score_x100 / 100,
                 (e->score_x100 < 0 ? -e->score_x100 : e->score_x100) % 100,
                 (unsigned long)e->dur_ms);
        trace_text(b);
    }
}
