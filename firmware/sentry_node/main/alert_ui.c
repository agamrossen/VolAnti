#include "alert_ui.h"

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "driver/rtc_io.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "actuators.h"
#include "board_pins.h"
#include "event_log.h"
#include "boot_rec.h"
#include "epaper.h"
#include "epaper_draw.h"
#include "led_ws2812.h"
#include "lora_link.h"
#include "trace.h"

/* ---------------------------------------------------------------------------
 * The LED, and the one fact that makes colour names possible.
 *
 * WS2812 parts ship in both GRB and RGB wire orderings, so naming a colour is
 * only safe once the fitted module's ordering has been measured. It has been,
 * from what was actually seen on the bench and recorded in the calibration
 * file: this module is wired G, R, B. If a different module is ever fitted,
 * `U l 255 0 0` says what it does in one command and only these macros
 * change.
 *
 * Brightness: the listening flash is deliberately not full scale. It is an
 * "the box is alive" indication a metre away in daylight, not a beacon, and
 * on a hillside at night a full-brightness green flash carries a very long
 * way. The alert stays full: at that moment being seen is the point.
 * ------------------------------------------------------------------------ */
#define LED_RGB(r, g, b)   (uint8_t)(g), (uint8_t)(r), (uint8_t)(b)

/* The colours and the cadences live in ui_config.h with the rest of the state
 * language, and the pattern engine in led_ws2812.c is the only thing that
 * reads them. This file picks a state; it does not know what colour that is. */

/* The heartbeat has to be seen to be a heartbeat. A 3.5% duty cycle reads as
 * a dead LED to somebody looking at a board that is flashing exactly as
 * designed, and "is it even on?" is the one question this indicator exists to
 * answer. Brightness is deliberately not full scale for a separate reason
 * that still holds: on a hillside at night a full-brightness green flash
 * carries a very long way. On-time is the variable, not amplitude.
 *
 * The cadence constants live in ui_config.h and only there. A duplicate here
 * is not a second opinion, it is a second place to be wrong that nothing
 * compares against the first - and a test that reads the duplicate through
 * the harness is a green check on a dead value. */

/* The alarm flash: fast red, and deliberately faster than anything else the
 * LED does - 100 ms on, 150 ms off, a 250 ms period against the heartbeat's
 * 2000. Local and remote alerts share it, because the buzzer cadence and the
 * panel carry that distinction and a colour an operator has to compare
 * against a remembered shade across a dark field is not a distinction at all.
 *
 * Red belongs to the alarm and to nothing else. The no-audio fault has its
 * own colour, used nowhere else, because two meanings for one signal would
 * make a dead microphone bus indistinguishable from an alert. See
 * alert_ui_fault_led(). The live values are LED_ALERT_ON_MS and
 * LED_ALERT_OFF_MS in ui_config.h. */

/* No-cell alert cadence. See outputs_cadence(). */
#define NOCELL_SLOT_MS      300u

/* E-paper pacing. A full refresh is ~2 s and the panel is the slowest thing
 * on the board by three orders of magnitude, while alerts can arrive far
 * faster than that. So screen changes are coalesced: the state machine records
 * what it wants shown, and a change is enqueued only when the last one has had
 * time to be seen. Without this the panel spends a field session permanently
 * mid-refresh and shows neither state. */
#define EPD_MIN_DWELL_MS   4000u

/* ---------------------------------------------------------------------------
 * The three pages, and the one button that reaches them.
 *
 * MAIN is the resting screen. STATUS and RECENT are reached by tapping, and
 * the device returns to MAIN by itself after PAGE_RETURN_MS - because a box
 * found the next morning showing page 3 is a box whose resting screen is
 * whatever the last person left it on, and the whole value of a resting
 * screen is that it says the same thing every time you walk past.
 *
 * Thirty seconds is chosen against the panel rather than against attention:
 * the return costs one full refresh and there is no partial-refresh path, so
 * a shorter timeout would spend more of the day refreshing than displaying.
 *
 * A tap is only a page change while guarding. During an alert every press is
 * a snooze and nothing else - see alert_ui_tick(). The INFO page has its own
 * timeout, INFO_TIMEOUT_MS, with the other operator-facing constants. */
#define PAGE_RETURN_MS   INFO_TIMEOUT_MS

/* ---- the loads must not overlap, and a comment cannot enforce that -------
 * The buzzer, motor, transmit burst and panel refresh are sequenced because
 * starting them together resets the board. Lengthening the transmit burst
 * without moving the outputs brings the collision straight back, as a reset
 * on every alert that the operator hears as one faint beep and no alarm.
 *
 * Prose describing the arrangement does not fail a build. This does. */
_Static_assert(ALERT_OUTPUTS_START_MS >= ALERT_TX_BURST_MS,
               "the buzzer starts while the radio is still transmitting - "
               "raise ALERT_OUTPUTS_START_MS above ALERT_TX_BURST_MS");
_Static_assert(ALERT_SCREEN_START_MS >= ALERT_TX_BURST_MS,
               "the panel refresh starts while the radio is still "
               "transmitting - raise ALERT_SCREEN_START_MS");

static bool     s_btn_ready;
static char     s_mode;
static uint8_t  s_state = TRACE_ALERT_IDLE;
static uint8_t  s_tier;             /* which tier raised the current alert */
static uint32_t s_alert_started_ms;
static bool     s_no_cell;          /* running without a battery */
static uint32_t s_snooze_until_ms;
static uint32_t s_cooldown_until_ms;
static uint16_t s_press_count;
static uint32_t s_last_stat_ms;

static uint32_t s_alert_max_ms = ALERT_MAX_MS;
static uint32_t s_snooze_ms    = SNOOZE_MS;

/* e-paper coalescing */
static int      s_epd_want = -1;    /* what the state machine wants shown  */
static int      s_epd_shown = -1;   /* what was last enqueued              */
static uint32_t s_epd_shown_gen;    /* MAIN's content generation, as drawn */
/* When the panel was last asked to draw, which is a different instant from
 * when the state machine last changed its mind. EPD_MIN_DWELL_MS exists to
 * let the previous screen be seen, so it is measured from the last enqueue.
 * Measured from the want change instead, every page tap waits the full dwell
 * even when the panel has been idle for hours, which reads as a broken
 * button. */
static uint32_t s_epd_drawn_ms;
static uint32_t s_epd_changed_ms;

/* The LED is written only when the colour actually changes. An RMT
 * transaction per frame would be 31 pointless transactions a second. */

/* ---------------------------------------------------------------------------
 * Button: an ISR for the edge, a poll for the hold.
 *
 * The switch is a plain two-pin push button, one leg to GPIO21 and the other
 * to GND, with the chip's internal pull-up doing the rest. Released reads
 * high; pressing shorts the pin to ground, giving a falling edge.
 *
 * The ISR exists because a tap far shorter than the 32 ms frame tick would
 * otherwise be missed entirely. The poll exists because an edge cannot measure
 * a hold, and the long press is how this device is turned off. They cooperate:
 * the ISR counts presses, the poll times how long the pin stays low, and a
 * press that turns out to be long has its edge swallowed so a power-off does
 * not also register as a snooze on the way past.
 *
 * Debounce is a time window in the ISR: edges arriving within BTN_DEBOUNCE_MS
 * of the last accepted one are contact bounce and are dropped.
 *
 * For diagnosis: an interrupt cannot manufacture an edge. If the bench shows a
 * pin that never leaves high while the button is held, the switch is not
 * reaching that pin and ground, and no amount of ISR will change it. Use
 * `U gs` to find which pin the button is actually on.
 * ------------------------------------------------------------------------ */
/* BTN_DEBOUNCE_MS lives in ui_config.h with the rest of the button timing, so
 * the gesture windows are read together. */


static bool     s_held;                    /* pin is LOW right now          */
static uint32_t s_held_since_ms;
static bool     s_long_fired;              /* this hold already fired long  */
static bool     s_power_req;               /* latched power-off request     */
static bool     s_rot_req;                 /* rotation to be persisted      */

/* The page lives here rather than in epaper.c because the button does.
 * epaper.c draws what it is told to draw; deciding which page an operator is
 * looking at is a piece of the same state machine that decides whether a
 * press is a snooze. */
/* The page the operator is on, and when they last changed it. */
static uint8_t  s_page;
static uint32_t s_page_ms;
/* While non-zero, the ALERT screen is being held on the panel and must not be
 * replaced by LISTENING. This is what reads ALERT_SCREEN_HOLD_MS. */
static uint32_t s_alert_screen_until_ms;
/* When the self-interference freeze may lift - see alert_ui_outputs_active(). */
static uint32_t s_freeze_until_ms;
/* When the current snooze began - the SNOOZE screen counts down from it. */
static uint32_t s_snooze_started_ms;

/* ---------------------------------------------------------------------------
 * The button, as an event source.
 *
 * The state machine below consumes TAP and HOLD and knows nothing about
 * GPIOs. Two things produce them: the real button, and `U btn <tap|hold>`
 * from the console. That is not a convenience - it is what makes the grammar
 * provable on a bench, and it means a wiring fault can never be mistaken for a
 * firmware one, or the reverse.
 *
 * Gesture recognition is split across an ISR and a poll because a tap shorter
 * than one 32 ms frame tick is invisible to a poll and a hold cannot be
 * measured by an edge. The ISR timestamps presses and the poll times how long
 * the pin stays low.
 *
 * Snooze fires on release: a press-down snooze would buzz on the way to every
 * power-off. A tap release is about 100 ms, so it is still immediate, and a
 * hold never fires a tap because no short release ever happens.
 * ------------------------------------------------------------------------ */
static bool     s_press_open;      /* a press is down and not yet resolved   */
static volatile uint8_t s_injected;/* from `U btn`, consumed once            */

/* ---- the virtual button -------------------------------------------------
 * `U btn down` / `U btn up` drive this instead of the pad, so a hold of an
 * exact length can be produced on a bench without a thumb. -1 means "read the
 * real pin", which is the state everything returns to after an `up`: an
 * override left latched would mask a real press, and a device that ignores its
 * own button is the worst thing this file could ship. */
static volatile int8_t s_inject_level = -1;

/* ---- the injected alert -------------------------------------------------
 *
 * `U alert v1 433` puts an event into the alert path at exactly the point a
 * real detection enters it, so the ALERT screen, the outputs and the return to
 * LISTENING can be timed without a drone. A bug that cannot be reproduced on a
 * bench cannot be proved fixed on one.
 *
 * s_auto_alert_s is the same hook on a repeat: the frame-budget gate needs an
 * alert every 30 s during a live guard, and a console line cannot provide one
 * because reading it stops the guard. Test hook, off by default, never
 * persisted. */
static volatile bool     s_inject_alert;
static volatile uint8_t  s_inject_alert_tier;
static volatile float    s_inject_alert_hz;
static volatile uint32_t s_auto_alert_s;
static uint32_t          s_auto_alert_last_ms;
/* How many the current drill has fired. See AUTO_ALERT_MAX_REPEATS. */
static uint32_t          s_auto_alert_n;

/* Every read of the button goes through here, so no call site can read the pad
 * directly and diverge from the debounced view. */
static volatile bool     s_btn_armed;
static volatile bool     s_btn_down_deb;
static volatile uint16_t s_btn_edges;
static uint16_t          s_btn_edges_seen;
static volatile uint32_t s_btn_up_ms;
static volatile uint16_t s_btn_phantoms;

/* Repaints of the resting screen since power-on. */
static volatile uint32_t s_home_redraws;

/* Presses that registered inside an alert window. */
static volatile uint16_t s_btn_alert_presses;

/* Where the alert timeline has got to. Both cleared at onset. */
static bool s_alert_outputs_on;
static bool s_alert_screen_sent;
/* True while the frozen alert page is showing. */
static bool s_frozen;
/* Detections that arrived while the operator had the box snoozed. Silenced at
 * the outputs, never at the detector, and the count is on the test page so a
 * window of deliberate silence can be accounted for afterwards. */
static volatile uint16_t s_quiet_n;
/* True while one silenced detection is still asserting `trigger`, so it is
 * counted once rather than once per frame. */
static bool s_quiet_latched;

/* ---- the output audit ---------------------------------------------------
 * s_buzz_on_ms is when the buzzer was last commanded high, s_buzz_ms the total
 * it has actually been high in this alert. A burst that ends with s_buzz_ms
 * far short of its commanded length is the short alert, recorded rather than
 * reconstructed. */
static uint32_t s_buzz_on_ms;
static uint32_t s_buzz_ms;
/* The tail pulse window. 0 when no tail is owed or it has finished. */
static uint32_t s_tail_until_ms;
static uint32_t s_tail_from_ms;
static float s_decide_score, s_decide_thr;

void alert_ui_set_decision(float score, float thr)
{
    s_decide_score = score;
    s_decide_thr = thr;
}

/* ---- test mode, pushed in rather than read ------------------------------
 * Set from the settings by the caller, so this module keeps no dependency on
 * NVS and the host harness can drive both postures with a fake clock. It
 * changes two navigation rules and nothing else: the page does not time out,
 * and an alert returns to the page instead of to MAIN. */
static bool s_test_mode;
static volatile uint16_t s_btn_glitches;
static esp_timer_handle_t s_btn_timer;

/* The debounced level, never the pad. Everything above the sampler sees a
 * button that has already been agreed on for BTN_PRESS_SAMPLES samples. */
static bool btn_down_now(void)
{
    return s_btn_ready && s_btn_down_deb;
}

/* ---- press feedback -----------------------------------------------------
 *
 * Every press-down gets a motor pulse and a short tick, and a press that is
 * held gets a tick train for exactly as long as it is held, so a tap and a
 * hold sound different without anybody counting.
 *
 * None of it blocks. A 12 ms tick driven with vTaskDelay would be 12 ms of a
 * 32 ms hop, the one cost this device cannot pay, so outputs are started here
 * and turned off by fb_pump() on a later frame. Ownership is tracked because
 * the alert cadence drives the same two pins: whoever turned an output on is
 * the only thing allowed to turn it off. */
static uint32_t s_fb_buzz_until;
static uint32_t s_fb_motor_until;
static uint32_t s_fb_next_tick;    /* 0 = no train running                   */
static bool     s_fb_owns_buzz;
static bool     s_fb_owns_motor;
static bool     s_fb_was_down;
static bool     s_fb_want_down;
static bool     s_fb_want_confirm;

static void fb_tick_start(uint32_t now)
{
#if BTN_BEEP_MODE == 1
    (void)actuators_buzzer_tone(BTN_BEEP_PWM_HZ, BTN_BEEP_DUTY_PCT, true);
#else
    actuators_buzzer(true);
#endif
    s_fb_owns_buzz = true;
    s_fb_buzz_until = now + BTN_TICK_MS;
}

static void fb_motor_pulse(uint32_t now, uint32_t ms)
{
    actuators_motor(true);
    s_fb_owns_motor = true;
    s_fb_motor_until = now + ms;
}

/* with_tick is false for the one press that must not make a noise: the tap
 * that silences a running alert. Answering "make it stop" with another buzz
 * 12 ms after the alarm stops is not feedback. The motor pulse still fires, so
 * the press is confirmed in the hand; only the sound is dropped, and only for
 * that one transition. */
static void fb_press_down(uint32_t now, bool with_tick)
{
    fb_motor_pulse(now, BTN_MOTOR_PULSE_MS);
    if (!with_tick) {
        s_fb_next_tick = 0;
        return;
    }
    fb_tick_start(now);
    s_fb_next_tick = now + BTN_HOLD_TICK_PERIOD_MS;
    if (s_fb_next_tick == 0u) { s_fb_next_tick = 1u; }  /* 0 means "no train" */
}

static void fb_confirm(uint32_t now)
{
    s_fb_next_tick = 0;                /* the ticks stop at the confirmation */
    fb_motor_pulse(now, BTN_CONFIRM_MOTOR_MS);
}

static void fb_pump(uint32_t now, bool held, bool alerting)
{
    if (alerting) {
        /* The alert cadence owns both pins for the whole ALERTING state.
         * Stand down without touching them, or a tick would punch a hole in
         * the alarm. */
        s_fb_owns_buzz = false;
        s_fb_owns_motor = false;
        s_fb_next_tick = 0;
        return;
    }
    if (s_fb_owns_buzz && (int32_t)(now - s_fb_buzz_until) >= 0) {
#if BTN_BEEP_MODE == 1
        (void)actuators_buzzer_tone(BTN_BEEP_PWM_HZ, 0, false);
#else
        actuators_buzzer(false);
#endif
        s_fb_owns_buzz = false;
    }
    if (s_fb_owns_motor && (int32_t)(now - s_fb_motor_until) >= 0) {
        actuators_motor(false);
        s_fb_owns_motor = false;
    }
    if (s_fb_next_tick != 0u && held && (int32_t)(now - s_fb_next_tick) >= 0) {
        fb_tick_start(now);
        s_fb_next_tick = now + BTN_HOLD_TICK_PERIOD_MS;
        if (s_fb_next_tick == 0u) { s_fb_next_tick = 1u; }
    }
    if (!held) {
        s_fb_next_tick = 0;            /* released: the train ends with it */
    }
}
/* A press seen while guarding, waiting for the button to come back up. See
 * alert_ui_tick(): a hold starts with an edge too, and that edge must not be
 * spent on a page change. */
static bool     s_tap_banked;

/* ---- the polled sampler -------------------------------------------------
 *
 * An edge interrupt on GPIO21 counts noise. That pin has no external pull-up -
 * the optional 100 nF pads are empty - only the chip's internal one, on a
 * board carrying a brushed ERM, a buzzer coil and a transmitter. The symptom
 * is random beeps with nobody touching the unit and screens changing by
 * themselves, every one of them a counted edge.
 *
 * So the pin is sampled every BTN_POLL_MS and debounced in both directions. A
 * press needs BTN_PRESS_SAMPLES consecutive lows; a release needs
 * BTN_RELEASE_SAMPLES consecutive highs. A single spike can never register,
 * and is counted instead, because that count is the instrument that says
 * whether the capacitor is worth fitting.
 *
 * And nothing registers until the pin has been seen released. That is what
 * makes boot silent: a pad sitting low at reset cannot produce a press-down
 * tick, so the unit cannot beep at somebody who is not there. */

static bool btn_raw_low(void)
{
    if (s_inject_level >= 0) {
        return s_inject_level != 0;
    }
    return gpio_get_level(PIN_SNOOZE_BTN) == 0;
}

/* The debounce runs at file scope rather than buried in the callback. A static
 * inside the function would be invisible to anything that needs to reset the
 * sampler, which makes a test suite order-dependent and a reset incomplete. */
static uint8_t s_btn_low_run, s_btn_high_run;

static void btn_sample_cb(void *arg)
{
    (void)arg;
    uint8_t low_run = s_btn_low_run, high_run = s_btn_high_run;
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

    if (btn_raw_low()) {
        high_run = 0;
        if (low_run < 250u) {
            low_run++;
        }
        /* A longer run is required during an alert.
         * `>=` rather than `==`: the threshold changes with the state, and an
         * equality test can be stepped straight over if the state changes
         * while the pad is already low. !s_btn_down_deb is what keeps this
         * from re-firing on every later sample. */
        const uint8_t need = (s_state == TRACE_ALERT_ALERTING)
                                 ? (uint8_t)BTN_ALERT_PRESS_SAMPLES
                                 : (uint8_t)BTN_PRESS_SAMPLES;
        if (low_run >= need && !s_btn_down_deb) {
            if (!s_btn_armed) {
                /* Boot, or a pad never yet seen released. Not a press. */
            } else if (s_btn_up_ms != 0u &&
                       (uint32_t)(now - s_btn_up_ms) < BTN_MIN_GAP_MS) {
                if (s_btn_phantoms < 0xFFFFu) { s_btn_phantoms++; }
            } else {
                s_btn_down_deb = true;
                if (s_btn_edges < 0xFFFFu) { s_btn_edges++; }
                /* Counted separately: a press inside an alert window is the
                 * one that turns five seconds into two beeps, so "how many
                 * were there" is a question the ring has to be able to
                 * answer after a field day. */
                if (s_state == TRACE_ALERT_ALERTING &&
                    s_btn_alert_presses < 0xFFFFu) {
                    s_btn_alert_presses++;
                }
            }
        }
    } else {
        if (low_run > 0u && low_run < BTN_PRESS_SAMPLES) {
            if (s_btn_glitches < 0xFFFFu) { s_btn_glitches++; }
        }
        low_run = 0;
        if (high_run < 250u) {
            high_run++;
        }
        if (high_run >= BTN_RELEASE_SAMPLES) {
            s_btn_armed = true;
            if (s_btn_down_deb) {
                s_btn_down_deb = false;
                s_btn_up_ms = now;
            }
        }
    }
    s_btn_low_run = low_run;
    s_btn_high_run = high_run;
}

static void button_init(void)
{
    if (s_btn_ready) {
        return;
    }
    /* The sleep path may have left the pad owned by the RTC mux. */
    rtc_gpio_deinit(PIN_SNOOZE_BTN);
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_SNOOZE_BTN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,     /* POLLED. See above. */
    };
    gpio_config(&cfg);

    const esp_timer_create_args_t ta = {
        .callback = btn_sample_cb,
        .name = "btn",
        .dispatch_method = ESP_TIMER_TASK,
    };
    if (esp_timer_create(&ta, &s_btn_timer) != ESP_OK) {
        return;
    }
    if (esp_timer_start_periodic(s_btn_timer, BTN_POLL_MS * 1000ULL) != ESP_OK) {
        return;
    }
    s_btn_ready = true;
}

/* One debounced press per tick, so a burst cannot advance the machine twice
 * in one frame. */
static bool button_pressed_edge(void)
{
    if (!s_btn_ready) {
        return false;
    }
    if ((uint16_t)(s_btn_edges - s_btn_edges_seen) == 0u) {
        return false;
    }
    s_btn_edges_seen++;
    s_press_count++;
    return true;
}

uint32_t alert_ui_home_redraws(void) { return s_home_redraws; }

/* Which tier raised the alert that is running, ALERT_TIER_NONE when none is.
 * The ALERT screen's line is built from this. */
uint8_t alert_ui_tier(void)
{
    return (s_state == TRACE_ALERT_ALERTING) ? s_tier : ALERT_TIER_NONE;
}

void alert_ui_set_test_mode(bool on) { s_test_mode = on; }

/* ---- the link test's feedback -------------------------------------------
 * Driven from the tick so it never blocks, and deliberately unlike an alert:
 * a short beep for sent or received, two for a closed round trip, plus a
 * short pulse. It uses the same outputs and none of the alert machinery, so it
 * cannot start an alert window or reach the ALERT screen. */
static uint32_t s_link_fb_until_ms;
static uint32_t s_link_fb_from_ms;
static uint8_t  s_link_fb_beeps;

void alert_ui_link_feedback(uint8_t beeps)
{
    s_link_fb_beeps = beeps ? beeps : 1u;
    s_link_fb_from_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_link_fb_until_ms = s_link_fb_from_ms +
        (uint32_t)s_link_fb_beeps * (LINK_BEEP_MS + LINK_GAP_MS);
}

static void link_feedback_tick(uint32_t now_ms)
{
    if (!s_link_fb_until_ms) {
        return;
    }
    const uint32_t e = now_ms - s_link_fb_from_ms;
    if ((int32_t)(now_ms - s_link_fb_until_ms) >= 0) {
        s_link_fb_until_ms = 0u;
        actuators_buzzer(false);
        actuators_motor(false);
        return;
    }
    const uint32_t period = LINK_BEEP_MS + LINK_GAP_MS;
    actuators_buzzer((e % period) < LINK_BEEP_MS);
    if (e < LINK_MOTOR_MS) {
        actuators_motor_duty(LINK_MOTOR_DUTY_PCT);
    } else {
        actuators_motor(false);
    }
}

bool alert_ui_frozen(void) { return s_frozen; }

uint16_t alert_ui_quiet_count(void) { return s_quiet_n; }

uint16_t alert_ui_btn_alert_presses(void) { return s_btn_alert_presses; }

void alert_ui_btn_stats(uint16_t *presses, uint16_t *phantoms,
                        uint16_t *glitches)
{
    if (presses)  { *presses  = s_btn_edges; }
    if (phantoms) { *phantoms = s_btn_phantoms; }
    if (glitches) { *glitches = s_btn_glitches; }
}

/* Times the hold and latches the long press. Returns true on the tick the
 * long press is recognised, while the button is still down: the operator gets
 * the feedback - the panel starts drawing OFF - at the hold threshold rather
 * than whenever they happen to let go. */
static bool button_long_press(uint32_t now_ms)
{
    if (!s_btn_ready) {
        return false;
    }
    const bool down = btn_down_now();
    if (down && !s_held) {
        s_held = true;
        s_held_since_ms = now_ms;
        s_long_fired = false;
    } else if (!down && s_held) {
        s_held = false;
    }
    if (down && !s_long_fired && (now_ms - s_held_since_ms) >= LONG_PRESS_MS) {
        s_long_fired = true;
        /* Swallow every edge this hold produced: a power-off must not also
         * register as a snooze. */
        s_btn_edges_seen = s_btn_edges;
        return true;
    }
    return false;
}

/* ---- outputs ----------------------------------------------------------- */


/* The heartbeat: a short flash once every LISTEN_PERIOD_MS. Phase comes from
 * the monotonic clock, so it neither drifts nor needs its own timer. */
/* The UI picks a state; the pattern engine picks the light. Computing the
 * blink here and writing RMT from the detector task stops the heartbeat
 * whenever the frame loop is busy, which is precisely when somebody most
 * wants to see that the box is alive. */
static void led_listening(uint32_t now_ms)
{
    (void)now_ms;
    led_pattern_set(LED_PAT_GUARD);
}

static void led_for_state(uint32_t now_ms)
{
    /* The off ritual owns the LED the moment it is asked for. The panel is
     * drawing OFF and the operator is about to throw a switch; a heartbeat
     * still pulsing underneath would say the opposite of what the screen
     * says, and the screen is the one telling the truth. */
    if (s_power_req) {
        led_pattern_set(LED_PAT_OFF);
        return;
    }
    switch (s_state) {
    case TRACE_ALERT_ALERTING:
        /* Fast red, local and remote alike. Carrying local-versus-remote in
         * a hue does not work: across a dark field, at this brightness and
         * against no reference, amber and red are the same light. The
         * distinction that survives is the one an operator can hear and read
         * - the buzzer's double pulse and the panel's REMOTE N-XXXX line. */
        led_pattern_set(LED_PAT_ALERT);
        break;
    case TRACE_ALERT_SNOOZED:
        /* Solid for the whole snooze. The operator silenced the device, and
         * the one thing they must be able to check at a glance is that it is
         * still snoozed. A pattern that matched "listening" would say the
         * opposite of the truth. */
        led_pattern_set(LED_PAT_SNOOZE);
        break;
    default:                                 /* IDLE and COOLDOWN both listen */
        led_listening(now_ms);
        break;
    }
}

/* Which screen a page index means. MAIN is EPAPER_SCREEN_READY because that
 * enum member has meant "the resting screen" since the field build and the
 * host tools speak its number. */
static int page_screen(uint8_t page)
{
    switch (page) {
    case EPAPER_PAGE_STATUS: return EPAPER_SCREEN_STATUS;
    case EPAPER_PAGE_RECENT: return EPAPER_SCREEN_RECENT;
    default:                 return EPAPER_SCREEN_READY;
    }
}

static void epd_want(int screen, uint32_t now_ms)
{
    if (s_epd_want == screen) {
        return;
    }
    s_epd_want = screen;
    s_epd_changed_ms = now_ms;
}

/* The refresh discipline, in one place.
 *
 * A screen is enqueued when, and only when, one of these is true:
 *   - the wanted SCREEN differs from the one shown (a state change, or the
 *     operator tapped to another page);
 *   - the wanted screen is MAIN and its CONTENT has changed - a new alert
 *     count, a new battery slice, a warning appearing, the hourly heartbeat.
 *
 * The second clause is the one that did not exist before and had to. Without
 * it a MAIN page whose counts changed would never redraw, because the screen
 * id had not moved; with a naive version of it, a producer that publishes a
 * snapshot once a second would redraw the panel once a second. The middle
 * course is that epaper_set_main() bumps the generation ONLY when the
 * snapshot would draw differently, so this comparison is free when nothing
 * has happened - and EPD_MIN_DWELL_MS remains the rate limiter on top.
 *
 * A quiet guarding day therefore costs about one refresh an hour. */
static void epd_pump(uint32_t now_ms)
{
    if (s_epd_want < 0) {
        return;
    }
    const uint32_t gen = epaper_main_gen();
    const bool content_moved = (s_epd_want == EPAPER_SCREEN_READY) &&
                               (gen != s_epd_shown_gen);
    if (s_epd_want == s_epd_shown && !content_moved) {
        return;
    }
    /* With no cell, the panel waits for the buzzer. A full refresh draws tens
     * of milliamps for two seconds, and on a USB-only board that lands on top
     * of the alert's own load and browns the rail out - measured both with
     * the buzzer and motor interleaved and with the buzzer alone. The buzzer
     * alone is safe in isolation; buzzer plus a refresh is not.
     *
     * Deferring costs nothing that matters: the screen is enqueued the moment
     * the outputs stop, about two seconds later, and an ALERT screen arriving
     * as the noise ends is still the record of what happened. On a board with
     * a cell this branch never runs. */
    if (s_no_cell && (actuators_buzzer_state() || actuators_motor_state())) {
        return;
    }
    /* An alert is the one thing that does not queue. EPD_MIN_DWELL_MS exists
     * so a chatty MAIN page cannot repaint the panel every few seconds;
     * applying it to an ALERT screen would drop the alarm's own record
     * because the battery slice had moved recently. The alert is the event
     * the panel exists to show. */
    const bool is_alert = (s_epd_want == (int)EPAPER_SCREEN_ALERT ||
                           s_epd_want == (int)EPAPER_SCREEN_ALERT_WASH);
    if (!is_alert && s_epd_shown >= 0 &&
        (now_ms - s_epd_drawn_ms) < EPD_MIN_DWELL_MS) {
        return;                     /* let the previous screen be seen */
    }
    if (epaper_request((epaper_screen_t)s_epd_want)) {
        /* ---- home redraws, counted at the only place one happens --------
         * How often the resting screen repaints has to be counted where the
         * panel is actually driven, not where a redraw is requested: a
         * request dropped by the dwell or by the no-cell deferral above is
         * not a redraw, and counting requests would report work the panel
         * never did. */
        if (s_epd_want == (int)EPAPER_SCREEN_READY &&
            s_home_redraws < 0xFFFFFFFFu) {
            s_home_redraws++;
        }
        s_epd_shown = s_epd_want;
        s_epd_shown_gen = gen;
        s_epd_changed_ms = now_ms;
        s_epd_drawn_ms = now_ms;
    }
}

static void outputs_on(void)
{
    actuators_buzzer(true);
    actuators_motor(true);
}

/* ---- every buzzer activation, attributed and measured -------------------
 * Wrapping the buzzer here rather than in actuators.c is deliberate: the
 * press-feedback path and the alert timeline are the two legitimate owners
 * and both live in this file, so a call that does not come through here is
 * visible as a BUZZ row with source OTHER. */
static void buzz_track(bool on, uint32_t now_ms)
{
    if (on && s_buzz_on_ms == 0u) {
        s_buzz_on_ms = now_ms ? now_ms : 1u;
    } else if (!on && s_buzz_on_ms != 0u) {
        s_buzz_ms += (uint32_t)(now_ms - s_buzz_on_ms);
        s_buzz_on_ms = 0u;
    }
}

static void outputs_off(void)
{
    actuators_buzzer(false);
    actuators_motor(false);
}

/* The cadence, driven once per tick while ALERTING.
 *
 * A local alert is continuous: every frame writes true to two pins that are
 * already true.
 *
 * A remote alert is a repeating double pulse, because "something is near you"
 * and "something is near somebody else" ask for different actions and an
 * operator must be able to tell them apart with the box in a pocket. */
static void outputs_cadence(uint8_t tier, uint32_t since_ms)
{
    if (tier != ALERT_TIER_REMOTE) {
        /* Both outputs at once is the design, and it needs the battery.
         *
         * Measured with no cell fitted: the buzzer alone is fine, the motor
         * alone is fine, and the two together brown the rail out every time -
         * rst 0x3 with the saved PC inside rtc_brownout_isr_handler. The 1S
         * cell is what supplies that peak; the USB path through the BQ24074
         * does not, so every alert resets the device and the alarm becomes a
         * reboot.
         *
         * With no cell the two are interleaved instead - one 300 ms slot
         * each, alternating - so the peak is one load and never two. It is
         * audible and tactile and it survives. This is a degraded mode and it
         * says so; `U c batt 1` with a cell fitted restores the simultaneous
         * burst, which is louder and is what ships. */
        if (s_no_cell) {
            /* The buzzer alone. Interleaving the two at 300 ms was tried
             * first and still browned out, with both-on samples at zero and a
             * reset anyway. Two things defeat it: an ERM's inrush is several
             * times its running current, so alternating restarts it three
             * times a burst instead of once, and the ALERT screen's 2 s full
             * refresh lands inside the alert as well.
             *
             * The buzzer alone measures safe, and of the two outputs it is
             * the one that carries - the point of the alert is to be heard
             * from another room. The motor is what is lost with no cell, and
             * it comes back with `U c batt 1`. */
            actuators_buzzer(true);
            actuators_motor(false);
            return;
        }
        /* Pulsed, not solid. Seconds of continuous buzzer at close range are
         * unusable, and a pulse train is both more audible and more obviously
         * artificial than a steady note. The motor runs continuously across
         * the window - it is felt, not heard, and pulsing it would only add
         * ERM inrush transients.
         *
         * ALERT_BUZZER_PULSED 0 restores the solid tone in one constant. */
        if (ALERT_BUZZER_PULSED) {
            const uint32_t period = ALERT_BUZZER_ON_MS + ALERT_BUZZER_OFF_MS;
            actuators_buzzer((since_ms % period) < ALERT_BUZZER_ON_MS);
        } else {
            actuators_buzzer(true);
        }
        /* The motor is driven at its cap and nowhere else. actuators_motor
         * (true) would mean full duty under PWM and would undo the ramp on
         * the very next tick, so the burst's motor lives here, in one place,
         * for the whole window. */
        if (ALERT_MOTOR_IN_BURST) {
            actuators_motor_duty(ALERT_MOTOR_DUTY_PCT);
        } else {
            actuators_motor(false);
        }
        return;
    }
    const uint32_t ph = since_ms % REMOTE_PERIOD_MS;
    const bool on = (ph < REMOTE_PULSE_MS) ||
                    (ph >= REMOTE_PULSE_MS + REMOTE_GAP_MS &&
                     ph <  2u * REMOTE_PULSE_MS + REMOTE_GAP_MS);
    actuators_buzzer(on);
    /* Capped for the same reason as the local burst. A remote alert is a
     * double pulse, so this restarts the motor twice a second and is if
     * anything the harder case for the rail. */
    if (on) {
        actuators_motor_duty(ALERT_MOTOR_DUTY_PCT);
    } else {
        actuators_motor(false);
    }
}

/* How long this alert's outputs run. The local burst is a runtime setting
 * because an operator on a bench wants it shorter; the remote latch is fixed,
 * because it is a protocol-level promise about what a peer's alert looks like
 * here and a device whose peers each held it for a different length would be
 * unreadable. */
static uint32_t alert_len(uint8_t tier)
{
    /* The same length as a local alert. A remote alert has no source in
     * front of the operator, but "not yours" is already carried by the REMOTE
     * word on the panel and by the double pulse, and holding the outputs on
     * for twice as long reads as an alarm that will not stop. */
    (void)tier;
    return s_alert_max_ms;
}

bool alert_ui_outputs_active(void)
{
    /* "Is anything making noise", not "is a pin high this microsecond", and
     * the difference is the whole self-interference story.
     *
     * Tier-3 and Tier-4 freeze while the device's own outputs are audible,
     * because a buzzer 30 mm from four microphones is a rotor as far as an
     * envelope statistic is concerned. The remote cadence is a double pulse,
     * so for 550 ms of every second both pins are low while the alert is
     * still very much running - and a predicate that only sampled the pins
     * would unfreeze the wash tiers in the gaps and feed them the pulse
     * train.
     *
     * So ALERTING counts as active in its own right. For a local alert that
     * is exactly the pin predicate, because outputs_on() has been called and
     * both pins are high for the whole state. The two pin reads stay because
     * `U b1` from the console drives the buzzer with no alert at all, which
     * is what the acoustic interference test does. */
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    const bool live = actuators_buzzer_state() || actuators_motor_state() ||
                      s_state == TRACE_ALERT_ALERTING
#if SNOOZE_MOTOR_CONTINUOUS
                      /* The snooze motor runs continuously and weakly for
                       * the whole snooze window. It is an ERM at 100-200 Hz
                       * sitting inside Tier-3's 100-320 Hz firing band, so it
                       * must freeze the wash tiers exactly as an alert does.
                       *
                       * The clause is gated on that motor actually running.
                       * With it disabled, a blanket freeze for the whole
                       * SNOOZED state would keep every millisecond of the
                       * 10.5 s of deafness that removing the motor was meant
                       * to buy back - silent and deaf for ten seconds instead
                       * of merely silent. The freeze follows the noise, not
                       * the state name. */
                      || s_state == TRACE_ALERT_SNOOZED
#endif
                      ;
    if (live) {
        s_freeze_until_ms = now + FREEZE_TAIL_MS;
        return true;
    }
    /* The tail. A buzzer's decay and an ERM's spin-down are still a rotor to
     * an envelope statistic, so the freeze outlives the pins by
     * FREEZE_TAIL_MS. */
    return (int32_t)(s_freeze_until_ms - now) > 0;
}

void alert_ui_set_no_cell(bool none)
{
    s_no_cell = none;
}

void alert_ui_set_timing(uint32_t alert_max_ms, uint32_t snooze_ms)
{
    if (alert_max_ms) {
        s_alert_max_ms = alert_max_ms;
    }
    if (snooze_ms) {
        s_snooze_ms = snooze_ms;
    }
}

void alert_ui_begin(char mode_letter)
{
    button_init();
    s_mode = mode_letter;
    s_state = TRACE_ALERT_IDLE;
    s_tier = ALERT_TIER_NONE;
    s_alert_started_ms = 0;
    s_snooze_until_ms = 0;
    s_cooldown_until_ms = 0;
    s_last_stat_ms = 0;
    s_epd_want = -1;
    s_epd_shown = -1;
    s_epd_shown_gen = 0;
    s_epd_changed_ms = 0;
    s_epd_drawn_ms = 0;
    /* A long press that landed while the console had the device must not shut
     * down the run that starts next: the latch belongs to one armed session.
     * The page resets with it, so every armed run starts on MAIN. */
    s_power_req = false;
    s_page = EPAPER_PAGE_MAIN;
    s_page_ms = 0;
    s_tap_banked = false;
    actuators_safe_all_off();
}

void alert_ui_end(void)
{
    outputs_off();
    actuators_safe_all_off();
    s_state = TRACE_ALERT_IDLE;
    s_mode = 0;
}

/* Turn edges and pin levels into one gesture. Returns ALERT_BTN_*, or
 * ALERT_BTN_NONE. */
static uint8_t button_gesture(uint32_t now_ms)
{
    /* An injected event outranks the pin: `U btn` must be able to drive the
     * machine on a board whose button is not wired. */
    if (s_injected) {
        const uint8_t ev = s_injected;
        s_injected = 0;
        return ev;
    }
    const bool held_now = btn_down_now();

    /* Two gestures and no timing window.
     *
     * A tap is a press released before BTN_HOLD_MS. A hold is a press still
     * down at BTN_HOLD_MS, and it fires at that instant rather than at
     * release, so the operator gets the confirmation while their thumb is
     * still on the button.
     *
     * There is no double press. It would need a timing window, and a window
     * is a way for a gesture to be missed by being too slow or swallowed by
     * being too quick; worse, a tap-count ritual collides with paging, since
     * three taps is simply how an operator reads three pages. Nothing here
     * counts presses or measures the gap between them. */

    /* Hold first: it swallows the edge that started it, so a power-off never
     * also registers as a snooze on the way past. */
    if (button_long_press(now_ms)) {
        s_press_open = false;
        return ALERT_BTN_HOLD;
    }

    if (button_pressed_edge()) {
        if (!held_now) {
            /* Already released - a tap shorter than one frame tick, seen only
             * by the ISR. Resolve it now. */
            return ALERT_BTN_TAP;
        }
        s_press_open = true;
        return ALERT_BTN_NONE;
    }

    /* A press that was still down last tick and is up now: that is the
     * release the tap fires on. */
    if (s_press_open && !held_now) {
        s_press_open = false;
        return ALERT_BTN_TAP;
    }
    return ALERT_BTN_NONE;
}

void alert_ui_inject_button(uint8_t ev) { s_injected = ev; }

void alert_ui_screens(int *want, int *shown)
{
    if (want)  { *want  = s_epd_want; }
    if (shown) { *shown = s_epd_shown; }
}

void alert_ui_inject_alert(uint8_t tier, float hz)
{
    s_inject_alert_tier = tier;
    s_inject_alert_hz = hz;
    s_inject_alert = true;
}

void alert_ui_set_auto_alert(uint32_t period_s)
{
    s_auto_alert_s = period_s;
    s_auto_alert_last_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_auto_alert_n = 0u;
}

uint32_t alert_ui_auto_alert(void) { return s_auto_alert_s; }

void alert_ui_inject_level(int level)
{
    if (level > 0) {
        s_inject_level = 1;
        /* Synthesise the edge the ISR would have counted, so a simulated press
         * reaches button_pressed_edge() exactly as a real one does. */
        s_btn_edges++;
    } else {
        /* Release the override rather than latch "up": see s_inject_level. */
        s_inject_level = -1;
    }
}

bool alert_ui_tick(bool trigger, uint8_t tier, uint32_t now_ms)
{
    /* Captured before the state machine moves, because the feedback fires
     * after it and would otherwise never see that this press was the one that
     * silenced an alarm. */
    const bool was_alerting = (s_state == TRACE_ALERT_ALERTING);

    /* Press-down is its own event, independent of which gesture it becomes:
     * the operator must feel the press land before the device knows whether
     * they are tapping or holding. Sampled before the gesture is resolved. */
    {
        const bool down_now = btn_down_now();
        if (down_now && !s_fb_was_down) {
            s_fb_want_down = true;
        }
        s_fb_was_down = down_now;
    }

    /* The injected alert joins the real one here, before any state is
     * examined, so everything downstream cannot tell them apart. */
    if (s_auto_alert_s != 0u &&
        (uint32_t)(now_ms - s_auto_alert_last_ms) >= s_auto_alert_s * 1000u) {
        s_auto_alert_last_ms = now_ms;
        /* The drill disarms itself. A host that armed this and then went
         * away - or a USB bus that dropped underneath it - must not leave a
         * unit sounding on a cell with nothing able to reach it. */
        if (s_auto_alert_n >= (uint32_t)AUTO_ALERT_MAX_REPEATS) {
            s_auto_alert_s = 0u;
        } else {
            s_auto_alert_n++;
            s_inject_alert = true;
            s_inject_alert_tier = ALERT_TIER_V1;
            s_inject_alert_hz = 433.0f;
        }
    }
    if (s_inject_alert) {
        s_inject_alert = false;
        if (s_state == TRACE_ALERT_IDLE) {
            /* Written to the ring exactly as a real detection would be, so
             * the alert counter and the RECENT page move too. */
            evlog_open(s_inject_alert_tier, s_inject_alert_hz, 0.0f, now_ms);
            bootrec_count_alert(s_inject_alert_tier, s_inject_alert_hz, now_ms);
            trigger = true;
            tier = s_inject_alert_tier;
        }
    }

    const uint8_t gesture = button_gesture(now_ms);
    const bool longp = (gesture == ALERT_BTN_HOLD);
    const bool press = (gesture == ALERT_BTN_TAP);

    /* An injected gesture has no pad edge behind it, so it is given the
     * feedback a real press would have had. That is what makes `U btn tap`
     * audible proof rather than a silent state change. */
    if (gesture != ALERT_BTN_NONE && !s_fb_want_down && !s_fb_was_down) {
        s_fb_want_down = true;
    }
    if (longp) {
        s_fb_want_confirm = true;
    }

    if (longp) {
        /* ---- the hold means two things, and the page says which ---------
         *
         * Everywhere except INFO a hold is the off ritual. On the INFO page it
         * rotates the screen a quarter turn, because that is the one thing an
         * operator cannot otherwise do without a laptop and a serial cable,
         * and a device carried to a hilltop in an enclosure is exactly where
         * you discover the panel is upside down.
         *
         * The hold, because every other gesture is taken: a tap snoozes from
         * MAIN and from an alert and dismisses INFO, and the grammar has no
         * double press.
         *
         * The cost is real - the hold was the one gesture whose meaning never
         * changed. It is paid for by the INFO page carrying "HOLD ROTATES" in
         * words, so the operator is told what the hold will do while they are
         * looking at the screen that does it, and by the off ritual still
         * being one tap and one hold away from anywhere.
         *
         * The rotation takes effect on the panel immediately and is banked
         * for the main loop to persist: settings live outside this module on
         * purpose, and pulling them in would put NVS behind every host test
         * of the button. */
        if (s_page == EPAPER_PAGE_STATUS && !s_frozen) {
            /* ---- on the STATUS page the hold is the link test -----------
             * One gesture must do one thing, and this branch is the only
             * place that decides which. Two arrangements to avoid: a second
             * `if (longp)` block further down, where both would run and the
             * panel would rotate as well as sending a request; and gating the
             * link test on test mode, which ships off, so the capability
             * would be reachable only in a posture nobody runs. A capability
             * gated on a mode that ships off is a capability that does not
             * ship.
             *
             * Rotation keeps a laptop-free home on the RECENT page's hold
             * rather than being deleted - it is the one thing an operator
             * cannot otherwise do without a serial cable. `U c rot N` still
             * works from a console. The off ritual stays on MAIN, which from
             * anywhere is one tap and one hold away. */
            if (lora_link_send_linktest()) {
                /* Two beeps, not one: a single short chirp beside a panel
                 * refresh is easy to miss outdoors. The three link signals
                 * have to stay tellable apart, which is the whole point of
                 * the test - 2 = I sent, 2 = the far unit heard a request,
                 * 3 = the reply came back and the link is proven. */
                alert_ui_link_feedback(2u);
            }
            s_page_ms = now_ms;          /* the operator is still reading it */
        } else if (s_page == EPAPER_PAGE_RECENT
                   || s_page == EPAPER_PAGE_STATUS) {
            /* RECENT rotates. STATUS reaches here only when FROZEN, where a
             * hold must not put a packet on the air: the frozen page is a
             * record of what fired and the radio would be changing the thing
             * being read. */
            epaper_set_rotation((uint8_t)((epaper_get_rotation() + 1u) & 3u));
            s_rot_req = true;
            s_page_ms = now_ms;          /* the operator is still reading it */
            epd_want(EPAPER_SCREEN_STATUS, now_ms);
        } else {
            s_power_req = true;
        }
        s_tap_banked = false;       /* the hold consumed it */
    }

    /* ---- the tap table --------------------------------------------------
     *
     *   LISTENING  tap -> SNOOZE            hold -> USER OFF
     *   SNOOZE     tap -> INFO              hold -> USER OFF
     *   INFO       tap -> the base state    hold -> USER OFF
     *   ALERT      tap -> silence, SNOOZE   hold -> USER OFF
     *
     * A tap does not mean one thing from every state. It means "the next
     * thing", and what that is depends only on what is in front of the
     * operator: silence it, then show me the numbers, then put it away. The
     * hold is the only gesture whose meaning never changes, which is why it
     * is the one that turns the device off.
     *
     * During an alert a tap cancels the outputs immediately, and while
     * already snoozed it restarts the window, which is what somebody pressing
     * again is asking for. Silencing is what an operator does with a buzzer
     * going and gloves on, so it depends on nothing being counted. */
    /* A tap on the frozen page returns, and specifically does not snooze:
     * the operator has finished reading a record of a decision, and silencing
     * a device that is already quiet would be a gesture with no meaning that
     * cost ten seconds of detection. A hold still reaches OFF. */
    if (press) {
        s_alert_screen_until_ms = 0u;   /* the operator has taken the panel */
    }
    if (press && s_frozen) {
        s_frozen = false;
        s_page = EPAPER_PAGE_MAIN;
        epd_want(EPAPER_SCREEN_READY, now_ms);
        alert_ui_emit_stat();
        return false;
    }
    if (press && s_page != EPAPER_PAGE_MAIN) {
        /* ---- a page is up: the tap advances, it does not dismiss --------
         * Jumping straight back to MAIN would leave EPAPER_PAGE_RECENT
         * rendered by the frame loop and reachable by no gesture at all - a
         * page nobody could open. The grammar is "tap = next page" over the
         * three pages, which also gives rotation somewhere real to live now
         * that the STATUS hold tests the radio.
         *
         * Snooze keeps running underneath the whole cycle: an operator can
         * read every page mid-snooze without cancelling the silence they just
         * asked for. */
        s_page = (uint8_t)(s_page + 1u);
        if (s_page >= EPAPER_PAGE_COUNT) {
            s_page = EPAPER_PAGE_MAIN;
            epd_want(s_state == TRACE_ALERT_SNOOZED ? EPAPER_SCREEN_SNOOZE
                                                    : EPAPER_SCREEN_READY,
                     now_ms);
        } else {
            s_page_ms = now_ms;
            epd_want(EPAPER_SCREEN_STATUS, now_ms);
        }
    } else if (press && s_state == TRACE_ALERT_SNOOZED) {
        /* SNOOZE -> INFO. The snooze window, its LED and its timer all keep
         * running underneath, so an operator can read the numbers mid-snooze
         * without cancelling the silence they just asked for. */
        s_page = EPAPER_PAGE_STATUS;
        s_page_ms = now_ms;
        epd_want(EPAPER_SCREEN_STATUS, now_ms);
    } else if (press) {
        /* Recorded on the open entry, so a self-latch can be told from a real
         * source afterwards: a source that stops is "source-gone", one the
         * operator had to silence is "snoozed". */
        evlog_note_snooze(now_ms);
        /* Snooze is local only. It drops this device's remote latch so the
         * outputs stop, but it transmits nothing, tells the peer nothing and
         * does not touch the dedupe ring, so the next alert from any peer is
         * heard in full. A device that went deaf to its neighbours because
         * somebody quieted a buzzer would be the worst surprise this product
         * could spring. Inert on a board with no radio. */
        lora_link_snooze();
        s_state = TRACE_ALERT_SNOOZED;
        s_snooze_until_ms = now_ms + s_snooze_ms;
        s_snooze_started_ms = now_ms;
        outputs_off();
        /* The snooze's own outputs: the solid LED, and the weak continuous
         * motor only if it is turned back on.
         *
         * SNOOZE_MOTOR_CONTINUOUS ships 0. The ERM is a rotor at 100 to
         * 200 Hz with harmonics up the band, so running it for the whole
         * window forces the self-interference freeze to cover all ten
         * seconds - a snooze costing 10.5 s of deafness - and feeds a steady
         * in-band comb to v1 and Tier-2 that the floor absorbs and then
         * releases at the tail, which is a self-alert waiting to happen. The
         * press feedback already tells the operator the press landed. */
        if (s_state == TRACE_ALERT_ALERTING) {
            buzz_track(false, now_ms);
            evlog_note_buzz(EVLOG_BUZZ_ALERT_LOCAL, alert_len(s_tier),
                            s_buzz_ms, now_ms);
            evlog_note_alert_end(EVLOG_AEND_SILENCED,
                                 now_ms - s_alert_started_ms, now_ms);
        }
#if SNOOZE_MOTOR_CONTINUOUS
        actuators_motor_snooze(true);
#endif
        epd_want(EPAPER_SCREEN_SNOOZE, now_ms);
        alert_ui_emit_stat();
    } else {

        /* ---- what the box heard while it was silenced -------------------
         * Snooze is the one deliberate silence there is, and an operator is
         * entitled to know what went past during the window they asked for.
         *
         * No page and no mode silences the outputs. Silencing them while a
         * live page is showing inverts in the field: the operator is twenty
         * metres away flying a rig, that page is the only place the live
         * scores exist, and so the page they must be on to watch a test would
         * be the page that stopped the test making a sound. Measured on this
         * file, ten passes with nobody touching the button: ten alarms in
         * product mode and one in test mode, the other nine counted and
         * silent. A test mode must not change what is being tested.
         *
         * The page half of that behaviour is the useful half and stays: an
         * alert does not take the panel away from the numbers, because
         * s_frozen owns the STATUS slot and holds it.
         *
         * Counted on the onset edge, never per frame. `trigger` is asserted
         * every frame a tier is latched; it is not an edge. Counting it per
         * frame writes thousands of ring rows in minutes - a mirror push and
         * a 1448-byte re-seal inside a critical section on the audio path,
         * thirty-one times a second - costs frames over the 32 ms hop, and
         * destroys the record it exists to keep, because a 64-entry ring
         * churned that fast holds nothing of a night. */
        if (trigger && s_state == TRACE_ALERT_SNOOZED) {
            if (!s_quiet_latched) {
                s_quiet_latched = true;
                s_quiet_n++;
                if (s_quiet_n == 0u) { s_quiet_n = 0xFFFFu; }
                evlog_note_quiet(tier, s_decide_score, s_decide_thr, now_ms);
                alert_ui_emit_stat();
            }
        } else if (!trigger) {
            s_quiet_latched = false;
        }

        switch (s_state) {
        case TRACE_ALERT_IDLE:
            if (trigger) {
                s_state = TRACE_ALERT_ALERTING;
                s_tier = tier;
                s_alert_started_ms = now_ms;
                /* Whatever page the operator was reading, the alert takes
                 * the panel - and when it ends the panel returns to MAIN, not
                 * to the page somebody left open twenty minutes ago.
                 *
                 * Except in test mode, where the page is what the operator is
                 * running the rig against: dropping them back to MAIN after
                 * every run would make the one screen they need the one they
                 * keep having to re-open. s_page is left alone, and the
                 * return path below re-shows it because it draws
                 * page_screen(s_page) once the alert goes idle. */
                if (!(s_test_mode && TEST_STICKY_AFTER_ALERT)) {
                    s_page = EPAPER_PAGE_MAIN;
                }
                /* ---- the loads are sequenced ----------------------------
                 * Starting buzzer, motor, a transmit burst and a 2 s panel
                 * refresh on the same millisecond browns the board out: an
                 * alert followed immediately by reset=BROWNOUT on a 4.164 V
                 * cell, on USB.
                 *
                 * So the onset starts nothing. The radio's burst is spread by
                 * lora_link's own scheduler; the outputs wait until
                 * ALERT_OUTPUTS_START_MS and the panel until
                 * ALERT_SCREEN_START_MS, both driven from the tick below, and
                 * the static assertions in ui_config.h keep them there. */
                s_alert_outputs_on = false;
                s_alert_screen_sent = false;
                s_buzz_on_ms = 0u;
                s_buzz_ms = 0u;
                evlog_note_alert_begin(tier, s_decide_score, s_decide_thr,
                                       now_ms);
                bootrec_set_alert_phase(BOOTPHASE_TX1);
                alert_ui_emit_stat();
            }
            break;

        case TRACE_ALERT_ALERTING: {
            const uint32_t since = now_ms - s_alert_started_ms;

            /* The phase, stamped at every step. It is what attributes a
             * brownout to the load that was switching on, and it is written
             * to its own RTC word because the blob's seal does not survive
             * the event being recorded. */
            if (!s_alert_outputs_on) {
                bootrec_set_alert_phase(
                    since < ALERT_TX_SPACING_MS        ? BOOTPHASE_TX1 :
                    since < 2u * ALERT_TX_SPACING_MS   ? BOOTPHASE_TX2 :
                    since < ALERT_OUTPUTS_START_MS     ? BOOTPHASE_TX3
                                                       : BOOTPHASE_OUTPUTS_RAMP);
            }

            /* The outputs wait. Until they start, both pins stay low and the
             * radio has the rail to itself for its burst. */
            if (since < ALERT_OUTPUTS_START_MS) {
                outputs_off();
                break;
            }
            if (!s_alert_outputs_on) {
                s_alert_outputs_on = true;
                bootrec_set_alert_phase(BOOTPHASE_OUTPUTS);
            }

            /* The panel waits longer still and is enqueued once. The e-paper
             * task on core 1 does the work; the frame loop must never block
             * on a 2 s refresh. */
            if (!s_alert_screen_sent && since >= ALERT_SCREEN_START_MS) {
                s_alert_screen_sent = true;
                bootrec_set_alert_phase(BOOTPHASE_SCREEN);
                /* ---- the panel waits for the buzzer, on every board ------
                 * Enqueued mid-burst, an ALERT screen has a narrow and often
                 * empty window. Two things close it: EPD_MIN_DWELL_MS drops
                 * the request outright if the panel drew anything in the
                 * previous few seconds, and a full refresh takes ~2 s of
                 * whatever window remains - after which the end of the burst
                 * tears it down again. The visible symptom is a panel that
                 * refreshes from LISTENING to LISTENING with no alert screen
                 * in between.
                 *
                 * It also puts a 2 s full refresh on top of the buzzer, which
                 * is the load pair measured as a brownout on a board with no
                 * cell.
                 *
                 * So the screen is drawn when the outputs stop and is held
                 * for ALERT_SCREEN_HOLD_MS. The buzzer is the immediate
                 * signal; the panel is the record of what fired, and a record
                 * that arrives as the noise ends and then stays is worth far
                 * more than one that flickers past mid-alarm or never draws
                 * at all. */
                if (s_test_mode) {
                    /* The frozen page occupies the STATUS slot, and s_frozen
                     * keeps it there: no refresh, no timeout and no return to
                     * LISTENING until a tap. */
                    s_frozen = true;
                    s_page = EPAPER_PAGE_STATUS;
                    /* Stamp the clock the FROZEN_RETURN_MS timeout measures
                     * from. Without this it would run from whenever a page
                     * was last turned, which could be hours earlier, and the
                     * page would clear itself the instant it appeared. */
                    s_page_ms = now_ms;
                    epd_want(EPAPER_SCREEN_STATUS, now_ms);
                } else {
                    epd_want(s_tier == ALERT_TIER_T3
                                 ? EPAPER_SCREEN_ALERT_WASH
                                 : EPAPER_SCREEN_ALERT, now_ms);
                }
                bootrec_set_alert_phase(BOOTPHASE_OUTPUTS);
            }

            /* ---- the inrush ramp -------------------------------------
             * The first ALERT_MOTOR_RAMP_MS of the window brings the motor up
             * through PWM instead of stepping it on. Every brownout in a
             * 20-alert soak was stamped OUTPUTS, and the buzzer alone
             * measures safe, so the ERM's inrush is what is left in that
             * phase.
             *
             * The buzzer runs its normal cadence throughout: it is the part
             * of the alert that carries, and holding it back would buy
             * nothing here. */
            {
                const uint32_t out_ms = since - ALERT_OUTPUTS_START_MS;
                if (out_ms < ALERT_MOTOR_START_MS + ALERT_MOTOR_RAMP_MS) {
                    bootrec_set_alert_phase(BOOTPHASE_OUTPUTS_RAMP);
                    /* The buzzer alone first. It measures safe on its own,
                     * and it is the half of the alert that carries to another
                     * room. */
                    actuators_buzzer(true);
                    buzz_track(actuators_buzzer_state(), now_ms);
                    if (!ALERT_MOTOR_IN_BURST ||
                        out_ms < ALERT_MOTOR_START_MS) {
                        actuators_motor(false);
                    } else {
                        const uint32_t r = out_ms - ALERT_MOTOR_START_MS;
                        actuators_motor_duty(
                            (int)((r * (uint32_t)ALERT_MOTOR_DUTY_PCT) /
                                  ALERT_MOTOR_RAMP_MS));
                    }
                    break;
                }
            }
            /* Keep driving the pattern, from the OUTPUTS' start rather than
             * the detection, so the cadence an operator hears is unchanged. */
            outputs_cadence(s_tier, since - ALERT_OUTPUTS_START_MS);
            /* Measured from the pin, not from what was commanded: the point
             * of the audit is to catch the two disagreeing. */
            buzz_track(actuators_buzzer_state(), now_ms);
            /* A burst, not a siren: the outputs run for s_alert_max_ms and
             * then stop whether or not the trigger is still active, because a
             * continuous alarm at close range makes the rig unworkable. A
             * remote alert runs for exactly as long - see alert_len().
             *
             * The window runs from the outputs' start rather than from the
             * detection, so the operator gets the full ALERT_OUTPUT_MS rather
             * than that minus the sequencing delay. */
            if (since >= ALERT_OUTPUTS_START_MS + alert_len(s_tier)) {
                s_state = TRACE_ALERT_COOLDOWN;
                s_cooldown_until_ms = now_ms + COOLDOWN_MS;
                outputs_off();
                s_tail_until_ms = ALERT_MOTOR_TAIL_MS
                                      ? now_ms + ALERT_MOTOR_TAIL_MS : 0u;
                s_tail_from_ms = now_ms;
                buzz_track(false, now_ms);
                evlog_note_buzz(s_tier == ALERT_TIER_REMOTE
                                    ? EVLOG_BUZZ_ALERT_REMOTE
                                    : EVLOG_BUZZ_ALERT_LOCAL,
                                alert_len(s_tier), s_buzz_ms, now_ms);
                evlog_note_alert_end(EVLOG_AEND_COMPLETE,
                                     now_ms - s_alert_started_ms, now_ms);
                bootrec_set_alert_phase(BOOTPHASE_RETURN);
                if (s_frozen) {
                    /* The sound stops; the page does not. It holds so the
                     * operator can walk over when they are ready. */
                    s_state = TRACE_ALERT_COOLDOWN;
                    s_cooldown_until_ms = now_ms + COOLDOWN_MS;
                    alert_ui_emit_stat();
                    break;
                }
                /* The alert screen, now that the noise has stopped, so the
                 * panel has the rail to itself. It is held for
                 * ALERT_SCREEN_HOLD_MS so the operator can walk to the box
                 * and read which tier fired, and the hold expires and hands
                 * back to LISTENING so the panel never ends up claiming an
                 * alert that has ended. */
                epd_want(s_tier == ALERT_TIER_T3 ? EPAPER_SCREEN_ALERT_WASH
                                                 : EPAPER_SCREEN_ALERT,
                         now_ms);
                s_alert_screen_until_ms = now_ms + ALERT_SCREEN_HOLD_MS;
                if (s_alert_screen_until_ms == 0u) {
                    s_alert_screen_until_ms = 1u;
                }
                alert_ui_emit_stat();
            }
            break;
        }

        case TRACE_ALERT_COOLDOWN:
            /* ---- the tail pulse --------------------------------------
             * The one moment in the alert when nothing else is switching.
             * Ramped for the same reason the burst motor is: an ERM's inrush
             * is several times its running current, and a step is what the
             * phase stamp catches. */
            if (s_tail_until_ms && (int32_t)(now_ms - s_tail_until_ms) < 0) {
                const uint32_t e = now_ms - s_tail_from_ms;
                const int d = (e < ALERT_MOTOR_TAIL_RAMP_MS)
                    ? (int)((e * (uint32_t)ALERT_MOTOR_TAIL_DUTY_PCT)
                            / ALERT_MOTOR_TAIL_RAMP_MS)
                    : ALERT_MOTOR_TAIL_DUTY_PCT;
                actuators_motor_duty(d);
            } else if (s_tail_until_ms) {
                s_tail_until_ms = 0u;
                actuators_motor(false);
            }
            /* Detection continues; only the outputs are held off. */
            if (s_alert_screen_until_ms &&
                (int32_t)(now_ms - s_alert_screen_until_ms) >= 0) {
                s_alert_screen_until_ms = 0u;
                epd_want(EPAPER_SCREEN_READY, now_ms);
            }
            if ((int32_t)(now_ms - s_cooldown_until_ms) >= 0) {
                s_state = TRACE_ALERT_IDLE;
                alert_ui_emit_stat();
            }
            break;

        case TRACE_ALERT_SNOOZED:
            /* Triggers during snooze are deliberately invisible at the outputs
             * and fully visible in the records. */
            if ((int32_t)(now_ms - s_snooze_until_ms) >= 0) {
                /* The weak run stops with the window, and PIN_MOTOR goes
                 * back to a plain GPIO driven low. Called unconditionally: it
                 * is the safe-off path, and it must run even if the constant
                 * above was flipped mid-run. */
                actuators_motor_snooze(false);
                s_state = TRACE_ALERT_IDLE;
                if (!(s_test_mode && TEST_STICKY_AFTER_ALERT)) {
                    s_page = EPAPER_PAGE_MAIN;
                    epd_want(EPAPER_SCREEN_READY, now_ms);
                } else {
                    epd_want(page_screen(s_page), now_ms);
                }
                alert_ui_emit_stat();
            }
            break;

        default:
            s_state = TRACE_ALERT_IDLE;
            break;
        }
    }

    /* The link test's feedback, which owns the outputs only while no alert
     * does - an alert always wins the buzzer. */
    if (s_state != TRACE_ALERT_ALERTING) {
        link_feedback_tick(now_ms);
    }

    /* The page returns to MAIN by itself. A box found in the morning showing
     * page 3 is a box whose resting screen is whatever the last person left
     * it on.
     *
     * TEST_PAGE_TIMEOUT_MS is 0, meaning never. In product mode the return is
     * the safeguard; during a test the operator is twenty metres away with a
     * rig running, and the page going dark on its own is the failure rather
     * than the safeguard. */
    if (!s_frozen &&
        !(s_test_mode && TEST_PAGE_TIMEOUT_MS == 0) &&
        s_page != EPAPER_PAGE_MAIN &&
        (now_ms - s_page_ms) >= PAGE_RETURN_MS) {
        s_page = EPAPER_PAGE_MAIN;
        epd_want(EPAPER_SCREEN_READY, now_ms);
    }
    /* And the frozen page comes back too, eventually. See FROZEN_RETURN_MS:
     * holding a decision on the glass is right for the minutes somebody needs
     * to walk over and read it, and wrong for the rest of the night. Left
     * latched on the first alert, the box records every later one without
     * ever showing it. */
    if (s_frozen && (now_ms - s_page_ms) >= FROZEN_RETURN_MS) {
        s_frozen = false;
        s_page = EPAPER_PAGE_MAIN;
        epd_want(EPAPER_SCREEN_READY, now_ms);
        alert_ui_emit_stat();
    }
    if (s_state == TRACE_ALERT_IDLE && s_epd_want < 0) {
        epd_want(page_screen(s_page), now_ms);
    }
    /* Fired here, not at the top. The state handlers above call
     * outputs_off() on their way through, so feedback started before them
     * would be silently cancelled by the very transition it is meant to
     * confirm. */
    if (s_fb_want_down) {
        s_fb_want_down = false;
        if (s_state != TRACE_ALERT_ALERTING) {
            fb_press_down(now_ms, !was_alerting);
        }
    }
    if (s_fb_want_confirm) {
        s_fb_want_confirm = false;
        fb_confirm(now_ms);
    }
    fb_pump(now_ms, btn_down_now(), s_state == TRACE_ALERT_ALERTING);

    led_for_state(now_ms);
    epd_pump(now_ms);

    /* Heartbeat STAT once per second while a mode is running. */
    if (s_mode && (now_ms - s_last_stat_ms) >= 1000u) {
        s_last_stat_ms = now_ms;
        alert_ui_emit_stat();
    }

    return s_state == TRACE_ALERT_ALERTING;
}

bool alert_ui_take_rotation(void)
{
    /* Take, not peek: the main loop persists one rotation per hold. A peek
     * would write NVS on every pass of the loop for as long as the operator
     * stood there. */
    const bool req = s_rot_req;
    s_rot_req = false;
    return req;
}

void alert_ui_idle_led(bool listening, uint32_t now_ms)
{
    if (listening) {
        led_listening(now_ms);
    } else {
        led_pattern_set(LED_PAT_OFF);
    }
}

/* Armed but receiving no audio: a microphone bus has stopped, everything else
 * looks healthy, and this is the indication that saves a wasted afternoon.
 *
 * Magenta - red and blue together, a colour used nowhere else on this device.
 * It cannot share red with alerting, because one signal with two meanings
 * would make a dead microphone bus indistinguishable from a drone overhead.
 * It is not green (guarding), not blue (snoozed) and not red (alerting), so
 * it cannot be read as any of them, and its cadence is distinct from the
 * alarm's as well. */
void alert_ui_fault_led(uint32_t now_ms)
{
    (void)now_ms;
    led_pattern_set(LED_PAT_FAULT);
}

bool alert_ui_standby_press(uint32_t now_ms)
{
    button_init();
    /* An injected hold wakes it too, so `U btn hold` can drive the off round
     * trip on a bench with nobody's thumb on the board. A tap is deliberately
     * not honoured: OFF has exactly one exit. */
    if (s_injected == ALERT_BTN_HOLD) {
        s_injected = 0;
        return true;
    }
    if (s_injected == ALERT_BTN_TAP) {
        s_injected = 0;                 /* consumed, and deliberately inert */
        return false;
    }
    if (button_long_press(now_ms)) {
        /* A hold wakes it too. Holding a button to turn a device on is what
         * every other device in the operator's pocket does. The hold that
         * turned it off cannot wake it by accident: that one has already
         * fired and cannot fire again until the pin goes high. */
        return true;
    }
    if (s_held) {
        return false;                   /* the power-off press, not yet let go */
    }
    /* Only a hold wakes it. Returning on any press edge lets a device in a
     * bag be woken by a knock against the button, and then guard - and alarm
     * - from inside the bag. A hold is the one gesture that cannot happen by
     * accident, which is why it is the only way in and the only way out. */
    (void)button_pressed_edge();        /* consume the edge, do not act on it */
    return false;
}

bool alert_ui_power_off_requested(void) { return s_power_req; }
void alert_ui_clear_power_request(void) { s_power_req = false; }

void alert_ui_power_down(void)
{
    outputs_off();
    actuators_safe_all_off();
    /* Stop the re-send before blanking, not after: otherwise the loop's next
     * turn re-lights the colour just cleared, and a device showing OFF sits
     * there with a lit LED. */
    led_refresh_stop();
    led_pattern_set(LED_PAT_OFF);
    s_state = TRACE_ALERT_IDLE;
    /* Draw it and wait. This is the one place in the firmware that waits on
     * the panel, and it is correct here: the detector is already stopped and
     * the entire purpose of the screen is to be right when the power goes. */
    s_epd_want = EPAPER_SCREEN_OFF;
    s_epd_shown = EPAPER_SCREEN_OFF;
    if (epaper_request(EPAPER_SCREEN_OFF)) {
        (void)epaper_wait_idle(6000u);
    }
}

static void fill_stat(trace_stat_t *s, char mode)
{
    uint8_t c0 = 0, c1 = 0, c2 = 0;
    led_get_raw(&c0, &c1, &c2);
    s->magic = TRACE_MAGIC_STA;
    s->uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s->mode = (uint8_t)mode;
    s->buzzer = actuators_buzzer_state() ? 1 : 0;
    s->motor = actuators_motor_state() ? 1 : 0;
    s->led_cmd = led_last_written_nonzero() ? 1 : 0;
    s->led_c0 = c0;
    s->led_c1 = c1;
    s->led_c2 = c2;
    s->alert_state = s_state;
    s->snooze_remaining_ms = alert_ui_snooze_remaining_ms();
    s->epaper_state = epaper_state();
    /* 1 = released (the internal pull-up holds it high), 0 = pressed to GND.
     * If the pin somehow is not configured yet, report 1 - but every caller of
     * this now configures it first, precisely so that "1" always means
     * "measured and released" rather than "never looked". */
    s->button_level = s_btn_ready ? (uint8_t)gpio_get_level(PIN_SNOOZE_BTN) : 1;
    s->press_count = s_press_count;
}

void alert_ui_emit_stat(void)
{
    trace_stat_t s;
    fill_stat(&s, s_mode);
    trace_send_stat(&s);
}

void alert_ui_emit_stat_idle(void)
{
    /* `U s` outside any mode, and the button must be configured here.
     *
     * The pin is configured lazily and fill_stat() reports "released" whenever
     * it has not been configured yet, so without this the one command whose
     * job is to read the button is the one command that never turns it on -
     * and it reports a plausible value instead of admitting it does not
     * know. */
    button_init();
    trace_stat_t s;
    fill_stat(&s, s_mode);
    trace_send_stat(&s);
}

uint8_t alert_ui_state(void)     { return s_state; }
uint8_t alert_ui_last_tier(void) { return s_tier; }

uint32_t alert_ui_snooze_remaining_ms(void)
{
    if (s_state != TRACE_ALERT_SNOOZED) {
        return 0;
    }
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    const int32_t left = (int32_t)(s_snooze_until_ms - now);
    return left > 0 ? (uint32_t)left : 0;
}

uint16_t alert_ui_press_count(void) { return s_press_count; }

/* Which page the operator is on. Read by the frame loop, which fills the page
 * it is about to be asked to draw - so a page nobody is looking at costs
 * nothing to keep current. */
uint8_t alert_ui_page(void) { return s_page; }
