/* ===========================================================================
 * Fail-safe to guard, which is the rule this whole file answers to:
 *
 *   When the battery reading cannot be trusted, the unit guards. It never
 *   sleeps on an unverified number. The cell's own protection circuit is the
 *   last line; the firmware's EMPTY cutoff is a COURTESY.
 *
 * The asymmetry is the point. A device that keeps guarding on a bad reading
 * costs a cell that could have been protected. A device that parks on a bad
 * reading is a device that is not listening, on a hillside, and cannot be
 * told from a dead one until somebody walks to it. On a detector whose
 * standing rule is that a miss can kill someone, those are not comparable
 * failures, and every refusal below is written to the ring so that "it kept
 * guarding" is never confused with "nothing happened".
 * ========================================================================= */
#include "power_mon.h"
#include "ui_config.h"   /* BATT_VERDICT_GRACE_MS */
#include "boot_rec.h"

#include <stdio.h>
#include <string.h>

#include "power_slice.h"

#if BOARD_HAS_VBAT

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "event_log.h"

/* GPIO1 is ADC1_CH0 on the S3. The channel is derived from the pin rather
 * than written down beside it, so a board that moves the sense pin moves the
 * channel with it and cannot end up reading a different one. */
#define VBAT_ADC_UNIT     ADC_UNIT_1
#define VBAT_ADC_CHANNEL  ((adc_channel_t)(PIN_VBAT_SENSE - 1))
_Static_assert(PIN_VBAT_SENSE >= 1 && PIN_VBAT_SENSE <= 10,
               "VBAT sense must be on ADC1 (GPIO1..GPIO10 on the ESP32-S3)");

/* Fifteen conversions, median. Odd so the median is an actual sample. */
#define VBAT_N_READS 15
_Static_assert(VBAT_N_READS % 2 == 1, "the median wants an odd sample");
_Static_assert(VBAT_N_READS <= POWER_MEDIAN_MAX, "sample larger than the sort");

/* ---- the bar's thresholds -----------------------------------------------
 *
 * The symptom these fix is a battery that never shows full when it is.
 *
 * BOARD_VBAT_MV_4 = 3920 mV is a threshold a loaded full cell cannot reach: a
 * 1S pack rests at 4.10 to 4.18 V off the
 * charger and sags 30 to 50 mV under this device's roughly 100 mA class load,
 * but the SLICE is read from the same 10 s median as everything else, taken
 * while the guard is running. The bar was therefore reporting the load, not
 * the charge. 4/4 now begins at 4.00 V.
 *
 * These live in ui_config.h with the rest of the operator-facing numbers
 * rather than in the board header, because they are a property of the cell
 * and the load, which is something a deployment retunes rather than something
 * the PCB fixes. BOARD_VBAT_MV_1 stays where it is and keeps its own job: the
 * sustained-low annunciation, which is a different question from how many
 * bars to draw. */
static const power_slice_cfg_t SLICES = {
    .mv = {BATT_SLICE1_MV, BATT_SLICE2_MV, BATT_SLICE3_MV, BATT_SLICE4_MV},
    .hyst_mv = BATT_SLICE_HYST_MV,
};

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cali;
static bool                      s_ready;
static bool                      s_cali_ok;

/* WRITTEN ON THE POWER TASK (core 1), READ BY THE FRAME LOOP (core 0), so
 * every one of them is volatile: without it the compiler is entitled to hoist
 * the frame loop's read out of the loop and the bar would never change. Each
 * is a naturally-aligned word or byte, which is atomic on this part, so
 * volatile is sufficient as well as necessary - there is no value here whose
 * halves could be seen from two different measurements. */
static volatile int      s_mv = -1;
static volatile int      s_slice = -1;
static volatile bool     s_charging;
static volatile bool     s_changed;
static uint32_t s_next_ms;
static bool     s_first;

static power_sustain_t s_low, s_empty;
static bool            s_low_said, s_empty_said;
static bool            s_bad_said, s_jump_said;
static bool            s_inhib_said;
static bool            s_absent;      /* no cell on the sense line */
static bool            s_declared;    /* ...and the operator said so */

static TaskHandle_t s_task;
static volatile bool s_task_stop;
static volatile uint8_t s_state;
static volatile bool s_quiet;

/* 2048 words is the smallest stack that survives an ESP_LOG-free path with an
 * ADC driver call in it, taken from the e-paper task's own figure rather
 * than measured. Read uxTaskGetStackHighWaterMark once and bring this down to
 * what it actually needs. */
#define POWER_TASK_STACK 2048
#define POWER_TASK_PRIO  1
#define POWER_TASK_CORE  1

static void power_task(void *arg)
{
    (void)arg;
    while (!s_task_stop) {
        s_state = power_mon_tick((uint32_t)(esp_timer_get_time() / 1000),
                                 s_quiet);
        /* Wake once a second and ask whether ten have passed. A one-second
         * tick rather than a ten-second sleep so `quiet` and the stop flag
         * are honoured promptly; the measurement itself still happens at
         * POWER_PERIOD_MS and not more often. */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    s_task = NULL;
    vTaskDelete(NULL);
}

uint8_t power_mon_state(void) { return s_state; }

void power_mon_quiet(bool quiet) { s_quiet = quiet; }

void power_mon_begin(void)
{
    if (s_ready) {
        return;
    }
    adc_oneshot_unit_init_cfg_t u = {.unit_id = VBAT_ADC_UNIT};
    if (adc_oneshot_new_unit(&u, &s_adc) != ESP_OK) {
        return;                 /* never fatal: a blind device still guards */
    }
    /* 12 dB gives roughly 0-3.1 V at the pin, which covers VBAT/2 for a 1S
     * cell (2.1 V at 4.2 V) with room over. */
    adc_oneshot_chan_cfg_t c = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(s_adc, VBAT_ADC_CHANNEL, &c) != ESP_OK) {
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
        return;
    }
    /* THE eFUSE CURVE, NOT A DIVISION. The S3's raw counts are not linear in
     * volts and the per-part correction is burned into eFuse at the factory.
     * Reading it is the difference between a bar that is right and a bar that
     * is plausible; without it, uncalibrated error is tens of millivolts,
     * which is most of a slice. If the scheme is unavailable the monitor says
     * so in `I` rather than quietly reporting a worse number as a good one. */
    adc_cali_curve_fitting_config_t cc = {
        .unit_id = VBAT_ADC_UNIT,
        .chan = VBAT_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    s_cali_ok = (adc_cali_create_scheme_curve_fitting(&cc, &s_cali) == ESP_OK);

#if BOARD_HAS_CHG_STAT
    /* Open drain with a 100k pull-up on the board; the internal pull-up is
     * enabled as well so an unpopulated resistor reads "not charging"
     * rather than floating. */
    gpio_config_t g = {
        .pin_bit_mask = 1ULL << PIN_CHG_STAT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&g);
#endif

    power_sustain_reset(&s_low);
    power_sustain_reset(&s_empty);
    /* The annunciation latches are per-run. Left to outlive a run, a device
     * that had once said EMPTY would park the next run on its first tick,
     * before taking a single measurement. s_absent is deliberately not here:
     * it is a fact about the board, not about the run. */
    s_low_said = false;
    s_empty_said = false;
    s_first = true;
    s_next_ms = 0;
    s_ready = true;

    s_task_stop = false;
    if (!s_task) {
        (void)xTaskCreatePinnedToCore(power_task, "power", POWER_TASK_STACK,
                                      NULL, POWER_TASK_PRIO, &s_task,
                                      POWER_TASK_CORE);
    }
}

void power_mon_end(void)
{
    if (!s_ready) {
        return;
    }
    /* Ask the task to leave and give it a tick to do so, rather than deleting
     * it from underneath a driver call it may be inside. */
    s_task_stop = true;
    for (int i = 0; i < 30 && s_task; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (s_cali_ok) {
        adc_cali_delete_scheme_curve_fitting(s_cali);
        s_cali_ok = false;
    }
    if (s_adc) {
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
    }
    s_ready = false;
}

/* ---- R1's TRUST WINDOW --------------------------------------------------
 * A 1S cell that is connected at all sits between these, and the pack's own
 * protection disconnects around 2.5 V. A median outside them is not a flat
 * battery, it is a measurement fault: a floating divider, a failed
 * conversion, or a calibration that did not load. */
#define VBAT_TRUST_LO_MV  2500
#define VBAT_TRUST_HI_MV  4500

/* ---- R2's JUMP CEILING --------------------------------------------------
 * Ten seconds apart. A 1S cell under this device's roughly 100 mA class load
 * does not fall 300 mV in ten seconds; a reading that does is the instrument
 * moving, not the cell. */
#define VBAT_JUMP_MAX_MV   300

#define VBAT_HIST_N 5
static int      s_hist_mv[VBAT_HIST_N];
static uint32_t s_hist_ms[VBAT_HIST_N];
static uint8_t  s_hist_n, s_hist_head;
static int      s_last_raw = -1;      /* the raw ADC median, for `U vbat`   */
static int      s_last_err;           /* esp_err_t of the last failed read  */

static void hist_push(int mv, uint32_t now_ms)
{
    s_hist_mv[s_hist_head] = mv;
    s_hist_ms[s_hist_head] = now_ms;
    s_hist_head = (uint8_t)((s_hist_head + 1u) % VBAT_HIST_N);
    if (s_hist_n < VBAT_HIST_N) {
        s_hist_n++;
    }
}

/* The newest `want` readings, newest first. Returns how many exist. */
static int hist_recent(int *mv, uint32_t *ms, int want)
{
    int n = (s_hist_n < want) ? s_hist_n : want;
    for (int i = 0; i < n; i++) {
        const int idx = ((int)s_hist_head - 1 - i + 2 * VBAT_HIST_N)
                        % VBAT_HIST_N;
        if (mv) { mv[i] = s_hist_mv[idx]; }
        if (ms) { ms[i] = s_hist_ms[idx]; }
    }
    return n;
}

/* One measurement: VBAT_N_READS conversions, median, then the divider. */
static int measure_mv(void)
{
    int raw[VBAT_N_READS];
    int n = 0;
    esp_err_t last = ESP_OK;
    for (int i = 0; i < VBAT_N_READS; i++) {
        int v = 0;
        const esp_err_t e = adc_oneshot_read(s_adc, VBAT_ADC_CHANNEL, &v);
        if (e == ESP_OK) {
            raw[n++] = v;
        } else {
            last = e;
        }
    }
    s_last_err = (int)last;
    if (n == 0) {
        s_last_raw = -1;
        return -1;
    }
    if ((n & 1) == 0) {
        n--;                            /* keep the median an actual sample */
    }
    const int mid = power_median(raw, n);
    s_last_raw = mid;
    int pin_mv = mid;
    if (s_cali_ok) {
        int cal = 0;
        if (adc_cali_raw_to_voltage(s_cali, mid, &cal) == ESP_OK) {
            pin_mv = cal;
        }
    }
    return pin_mv * BOARD_VBAT_DIV_NUM / BOARD_VBAT_DIV_DEN;
}

uint8_t power_mon_tick(uint32_t now_ms, bool quiet)
{
    if (!s_ready || quiet) {
        return POWER_OK;
    }
    if (!s_first && (int32_t)(now_ms - s_next_ms) < 0) {
        return (s_empty_said ? POWER_EMPTY
                             : (s_low_said ? POWER_LOW : POWER_OK));
    }
    s_first = false;
    s_next_ms = now_ms + POWER_PERIOD_MS;

    const int mv = measure_mv();
    hist_push(mv, now_ms);

    /* ---- R1: AN UNTRUSTED READING NEVER COUNTS -------------------------
     * It does not merely fail to advance the empty window, it RESETS it, so
     * a run of good readings interrupted by a fault starts again rather than
     * carrying a part-accumulated verdict across the gap. And the device
     * keeps guarding, which is the whole principle at the top of this file. */
    if (mv < 0) {
        power_sustain_reset(&s_empty);
        power_sustain_reset(&s_low);
        s_low_said = false;
        if (!s_bad_said) {
            s_bad_said = true;
            evlog_note_vbat(EVLOG_REL_VBAT_BAD, s_last_raw, now_ms);
        }
        return POWER_OK;
    }
    if (mv < VBAT_TRUST_LO_MV || mv > VBAT_TRUST_HI_MV) {
        /* A reading here is a measurement fault, not a flat cell: the pack's
         * own protection disconnects around 2.5 V, so nothing connected can
         * present less. It is still SHOWN, because a number an operator can
         * see is how a floating divider gets diagnosed. */
        s_mv = mv;
        if (mv < BOARD_VBAT_MV_ABSENT && !s_absent) {
            s_absent = true;
            s_changed = true;           /* the bar goes away, exactly once */
        }
        power_sustain_reset(&s_empty);
        power_sustain_reset(&s_low);
        s_low_said = false;
        if (!s_bad_said) {
            s_bad_said = true;
            evlog_note_vbat(EVLOG_REL_VBAT_BAD, mv, now_ms);
        }
        return POWER_OK;
    }
    s_bad_said = false;
    s_mv = mv;

    /* ---- R2a: A JUMP DISCARDS THE RUN, THE MOMENT IT IS SEEN -----------
     *
     * This has to happen here and not at verdict time, and the first version
     * of it got that wrong. Checking the last three readings only when the
     * thirty-second window completes means the jump has already AGED OUT of
     * those three: 4100 then 3200 then thirty seconds of 3200 ends with three
     * identical readings, no pair differing at all, and the veto waves through
     * exactly the verdict it exists to refuse. tests/test_power_failsafe.py
     * caught that, on the shipped C, which is the whole reason it compiles it.
     *
     * So the run is discarded where C.2 says it is: at the jump. */
    bool jump_now = false;
    {
        int h[2];
        if (hist_recent(h, NULL, 2) == 2 && h[0] >= 0 && h[1] >= 0) {
            int d = h[0] - h[1];
            if (d < 0) { d = -d; }
            if (d > VBAT_JUMP_MAX_MV) {
                jump_now = true;
                power_sustain_reset(&s_empty);
                if (!s_jump_said) {
                    s_jump_said = true;
                    evlog_note_vbat(EVLOG_REL_VBAT_JUMP, mv, now_ms);
                }
            } else {
                s_jump_said = false;
            }
        }
    }

    /* THE ABSENT CHECK COMES FIRST, and it has to. Below it the slice and the
     * charger line are read and any change sets s_changed, which asks the
     * panel for a FULL 2 s REFRESH. With no cell the BQ24074's STAT line
     * toggles as it tries to charge nothing, so the panel was redrawing over
     * and over, which presents as a display that keeps updating and also
     * starves the ALERT and SNOOZE screens, because each has to wait out
     * EPD_MIN_DWELL_MS behind the last redraw. */
    if (s_declared || mv < BOARD_VBAT_MV_ABSENT) {
        if (!s_absent) {
            s_absent = true;
            s_changed = true;              /* the bar goes away, exactly once */
        }
        power_sustain_reset(&s_empty);
        power_sustain_reset(&s_low);
        s_low_said = false;
        return POWER_OK;
    }

    const int slice = power_slice_step(&SLICES, s_slice, mv);
#if BOARD_HAS_CHG_STAT
    const bool chg = (gpio_get_level(PIN_CHG_STAT) == 0) ==
                     (BOARD_CHG_ACTIVE_LOW != 0);
#else
    const bool chg = false;
#endif
    /* THE REDRAW GATE. Nothing else in this module asks the panel for
     * anything: it reports that the display's two facts changed, and the
     * caller decides whether the panel is in a state where a redraw is
     * appropriate. On e-paper a redraw is a two-second full refresh. */
    if (slice != s_slice || chg != s_charging) {
        s_changed = true;
    }
    s_slice = slice;
    s_charging = chg;

    /* THE FIRST READING OF THE BOOT, into the record that survives a reset.
     * Ignored after the first, because what matters afterwards is what the
     * device DECIDED, and that is recorded separately. */
    bootrec_set_vbat(mv, chg ? 1 : 0);
    /* AND THE UPTIME, every measurement period. A power cut leaves no chance
     * to write anything, so the last surviving uptime bounds how long the
     * device ran to within POWER_PERIOD_MS instead of leaving it unknown. */
    bootrec_note_uptime(now_ms);

    /* NO CELL FITTED. Checked BEFORE the two sustained decisions, because an
     * absent battery is not a discharged one and must not reach the park.
     * Both timers are reset rather than merely skipped: a board that had a
     * cell removed mid-run must not resume a part-accumulated countdown when
     * one is fitted again. See BOARD_VBAT_MV_ABSENT for the measurement that
     * put this here. */

    /* THE LATCH IS ONE-WAY, AND THAT IS THE WHOLE POINT. An earlier version of
     * this only skipped the park while the reading was under the threshold,
     * and the board parked anyway within two minutes: with no cell the BAT
     * node does not sit at zero, it DRIFTS - 0.000 V, 3.896 V, 3.940 V,
     * 4.106 V were all measured on one board in one session - so it spends
     * time in the 2.0-3.3 V band, and thirty seconds anywhere in there is a
     * sustained-empty verdict.
     *
     * A reading below the threshold is therefore evidence about the BOARD, not
     * about this instant: it says there is no cell on the sense line, and one
     * cannot appear without a power cycle, because fitting a battery means
     * opening the case and the slider hard-cuts the regulator. So the fact is
     * latched for the run and no later reading revokes it. A real cell never
     * sets it in the first place - the pack's own protection disconnects
     * around 2.5 V, well above this. */

    /* The two sustained decisions. Both are timed, because the buzzer and the
     * motor pull about a hundred milliamps between them and a 1S cell sags
     * while they run - a single dip is evidence about the LOAD, not the cell. */
    /* THE JUMP READING ITSELF DOES NOT COUNT. Resetting the window and then
     * stepping it in the same tick would re-arm it on this very reading,
     * which is no discard at all: the run would start exactly where it would
     * have started anyway. Skipping the step is what makes "discards the run"
     * mean something, and it is what the host test measures. */
    const bool empty_raw = jump_now ? false
        : power_sustain_step(&s_empty, mv, BOARD_VBAT_MV_EMPTY,
                             POWER_EMPTY_HOLD_MS, now_ms);
    const bool low = power_sustain_step(&s_low, mv, BOARD_VBAT_MV_1,
                                        POWER_LOW_HOLD_MS, now_ms);

    /* ---- the verdict is patient, and it defers to the charger -----------
     *
     * Two inhibitions.
     *
     * Never empty while charging. With the empty decision consulting the
     * voltage alone, a cell sagging under the buzzer and the motor while
     * sitting on a charger can park a device that is at that moment being
     * actively refilled.
     *
     * And not in the first BATT_VERDICT_GRACE_MS. The 30 s sustain is the
     * real protection and is kept; the grace is additional cover against one
     * early bad reading taken before the ADC and the divider have settled,
     * which is exactly the class of reading that parked a board twice during
     * PCB bring-up. */
    const bool in_grace = (now_ms < (uint32_t)BATT_VERDICT_GRACE_MS);
    if (empty_raw && (chg || in_grace)) {
        if (!s_inhib_said) {
            s_inhib_said = true;
            evlog_note_power_inhibited(mv, now_ms);
        }
        return low ? POWER_LOW : POWER_OK;
    }
    if (!empty_raw) {
        s_inhib_said = false;   /* a fresh episode may be recorded later */
    }
    const bool empty = empty_raw;
    /* ---- R2: THE JUMP VETO ---------------------------------------------
     * The sustain window says "below for thirty seconds". This asks whether
     * the three medians behind that verdict look like a CELL. A 1S pack under
     * this device's load does not fall 300 mV in ten seconds, so a pair that
     * does is the instrument moving, and a verdict built on it would park a
     * charged device. Fewer than three readings is also a refusal: there is
     * not enough to check, and the principle at the top of this file says what
     * to do when the reading cannot be trusted. */
    /* R2b: and at verdict time, the three readings behind it must all be
     * below. R2a has already handled the jump itself. */
    bool r2_ok = true, r2_jump = false;
    {
        int h[3];
        const int nh = hist_recent(h, NULL, 3);
        if (nh < 3) {
            r2_ok = false;
        } else {
            for (int i = 0; i < 3; i++) {
                if (h[i] < 0 || h[i] >= BOARD_VBAT_MV_EMPTY) {
                    r2_ok = false;
                }
            }
            for (int i = 0; i + 1 < 3; i++) {
                int d = h[i] - h[i + 1];
                if (d < 0) { d = -d; }
                if (d > VBAT_JUMP_MAX_MV) {
                    r2_ok = false;
                    r2_jump = true;
                }
            }
        }
    }
    if (empty && !r2_ok) {
        power_sustain_reset(&s_empty);
        if (r2_jump && !s_jump_said) {
            s_jump_said = true;
            evlog_note_vbat(EVLOG_REL_VBAT_JUMP, mv, now_ms);
        }
        return low ? POWER_LOW : POWER_OK;
    }
    if (!empty) {
        s_jump_said = false;
    }

    if (empty) {
        if (!s_empty_said) {
            s_empty_said = true;
            /* The ring is how a gap in the negative-hours accounting is
             * explained afterwards: "it stopped" and "it ran out" look
             * identical in a capture and are entirely different facts. */
            evlog_note_power(true, mv, now_ms);
        }
        return POWER_EMPTY;
    }
    if (low) {
        if (!s_low_said) {
            s_low_said = true;
            s_changed = true;           /* LOW BATT joins the footer */
            evlog_note_power(false, mv, now_ms);
        }
        return POWER_LOW;
    }
    if (s_low_said) {
        /* Back on a charger. The annunciation clears, and it is allowed to be
         * said again later - a latch that never cleared would need a reboot
         * to stop lying, on the one device that is left running for weeks. */
        s_low_said = false;
        s_changed = true;
    }
    return POWER_OK;
}

int  power_mon_mv(void)        { return s_mv; }

bool power_mon_trusted_mv(int mv)
{
    return mv >= VBAT_TRUST_LO_MV && mv <= VBAT_TRUST_HI_MV;
}

int power_mon_chg_level(void)
{
#if BOARD_HAS_CHG_STAT
    return ((gpio_get_level(PIN_CHG_STAT) == 0) ==
            (BOARD_CHG_ACTIVE_LOW != 0)) ? 1 : 0;
#else
    return -1;
#endif
}

int power_mon_probe(void)
{
    if (!s_ready) {
        /* QUIET FIRST. begin() starts the monitoring task, and a task that
         * began issuing verdicts while the wake path was still deciding what
         * to do would be two things deciding the same question. The guard
         * clears quiet when it arms. */
        power_mon_quiet(true);
        power_mon_begin();
    }
    if (!s_ready || !s_adc) {
        return -1;
    }
    return measure_mv();
}

/* `U vbat`: the reading itself, so it can be watched rather than inferred. */
void power_mon_describe_raw(char *dst, int cap)
{
    int mv[VBAT_HIST_N];
    uint32_t ms[VBAT_HIST_N];
    const int n = hist_recent(mv, ms, VBAT_HIST_N);
    int off = snprintf(dst, cap,
                       "vbat: raw=%d err=0x%x atten=12dB width=default cal=%s "
                       "chg=%s trust=%d..%d mV\n",
                       s_last_raw, (unsigned)s_last_err,
                       s_cali_ok ? "efuse-curve" : "NONE",
#if BOARD_HAS_CHG_STAT
                       ((gpio_get_level(PIN_CHG_STAT) == 0) ==
                        (BOARD_CHG_ACTIVE_LOW != 0)) ? "yes" : "no",
#else
                       "n/a",
#endif
                       VBAT_TRUST_LO_MV, VBAT_TRUST_HI_MV);
    for (int i = 0; i < n && off > 0 && off < cap; i++) {
        off += snprintf(dst + off, cap - off,
                        "  [%d] %5d mV at t=%lu.%03lu s%s\n", i, mv[i],
                        (unsigned long)(ms[i] / 1000u),
                        (unsigned long)(ms[i] % 1000u),
                        (mv[i] < 0) ? "  READ FAILED"
                        : ((mv[i] < VBAT_TRUST_LO_MV ||
                            mv[i] > VBAT_TRUST_HI_MV) ? "  UNTRUSTED" : ""));
    }
    if (n == 0 && off > 0 && off < cap) {
        snprintf(dst + off, cap - off, "  (no reading yet)\n");
    }
}
/* -1 is epaper_set_battery()'s "draw no bar": an absent cell must not
 * render as an empty one, which is the same picture as a dead device. */
int  power_mon_slice(void)     { return s_absent ? -1 : s_slice; }

/* The operator's declaration. Latches s_absent directly, so the park is off
 * from the FIRST tick rather than from the first low reading. */
void power_mon_set_no_cell(bool none)
{
    s_declared = none;
    if (none) { s_absent = true; }
}
bool power_mon_charging(void)  { return s_charging; }
bool power_mon_display_changed(void)
{
    /* READ-AND-CLEAR, and it has to be. The flag is set on the power task and
     * read by the frame loop's once-a-second housekeeping; both run at about
     * 1 Hz, so a flag that the WRITER cleared would be missed roughly half
     * the time and a slice change would simply never reach the panel. The
     * reader consuming it is the same latch discipline as the button's
     * power-off request.
     *
     * The cross-core read-modify-write can lose a change that lands in the
     * same microsecond as the clear. That costs one missed redraw of a status
     * bar and self-corrects on the next slice change; a lock on the audio
     * core's housekeeping path to protect a battery icon would not. */
    const bool c = s_changed;
    s_changed = false;
    return c;
}

void power_mon_describe(char *dst, int cap)
{
    if (s_mv < 0) {
        snprintf(dst, cap, "battery: not measured yet (%s)\n",
                 s_ready ? "adc ready" : "ADC UNAVAILABLE");
        return;
    }
    if (s_absent) {
        snprintf(dst, cap,
                 "battery: NO CELL FITTED (%d mV, %s) - external power, the "
                 "empty-battery park is disabled\n",
                 s_mv, s_declared ? "declared by `U c batt 0`" : "measured");
        return;
    }
    snprintf(dst, cap,
             "battery: %d.%03d V  %d/4 cells%s%s  cal=%s\n",
             s_mv / 1000, s_mv % 1000, s_slice,
             s_charging ? "  CHARGING" : "",
             s_low_said ? "  LOW" : "",
             s_cali_ok ? "efuse-curve" : "RAW (uncalibrated)");
}

#else  /* !BOARD_HAS_VBAT */

/* THE BREADBOARD HAS NO DIVIDER, so there is nothing to read and no bar to
 * draw. These are the inert forms rather than #if at every call site: the
 * standalone loop says `power_mon_tick(now, quiet)` on both boards and the
 * linker deletes the call here. A reader of the frame loop sees one behaviour
 * described once, which is the whole reason the board seam uses capability
 * flags instead of conditional call sites. */

void power_mon_begin(void) {}
void power_mon_end(void) {}
void power_mon_quiet(bool quiet) { (void)quiet; }
void power_mon_set_no_cell(bool none) { (void)none; }
uint8_t power_mon_state(void) { return POWER_OK; }
uint8_t power_mon_tick(uint32_t now_ms, bool quiet)
{
    (void)now_ms;
    (void)quiet;
    return POWER_OK;
}
int  power_mon_mv(void)        { return -1; }
int  power_mon_slice(void)     { return -1; }
bool power_mon_charging(void)  { return false; }
bool power_mon_display_changed(void) { return false; }

void power_mon_describe_raw(char *dst, int cap)
{
    snprintf(dst, cap, "vbat: no divider on this board (%s)\n", BOARD_NAME);
}

int power_mon_probe(void) { return -1; }
bool power_mon_trusted_mv(int mv) { (void)mv; return false; }
int power_mon_chg_level(void) { return -1; }

void power_mon_describe(char *dst, int cap)
{
    snprintf(dst, cap, "battery: this board has no VBAT sense (%s)\n",
             BOARD_NAME);
}

#endif /* BOARD_HAS_VBAT */
