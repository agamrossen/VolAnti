/*
 * alert_ui.h - the alert / snooze state machine and the operator surface.
 *
 *     IDLE --trigger--> ALERTING --burst over--> COOLDOWN --> IDLE
 *                          |  \--press--------> SNOOZED --expiry--> IDLE
 *     any state --press--> SNOOZED(10 s)
 *     any state --hold 2 s--> power-off requested (the caller does the work)
 *
 * Three constraints this file exists to honour:
 *
 *   * Detection is never gated by the UI. Snooze silences outputs. Records,
 *     scores, chains and alert records continue exactly as they would with no
 *     actuators fitted. A snoozed device is a quiet device, not a deaf one.
 *
 *   * O(1) per frame. The whole integration into the armed mode is one
 *     alert_ui_tick() call after back_end(), reading a flag the detector
 *     already computed. Nothing here touches front_end / combiner / back_end
 *     or their inputs, and nothing here allocates.
 *
 *   * No blocking. The LED is written only when its colour changes and the
 *     e-paper is only ever enqueued, never waited on. A panel refresh takes
 *     ~2 s against a 32 ms frame; the two must never meet.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ui_config.h"

/* The outputs are a burst, not a siren.
 *
 *   ALERT_MAX_MS   how long the buzzer and motor run for one detection
 *   COOLDOWN_MS    enforced silence afterwards before anything can fire again
 *   SNOOZE_MS      the operator's own silence window, on a button press
 *
 * A continuous alarm at close range is unusable - you cannot think, and you
 * certainly cannot work on the rig. The burst is long enough to be unmissable
 * and short enough to be tolerable, and the cooldown stops a sustained source
 * from producing a solid tone. Detection is never paused by any of it, so a
 * cooled-down device is a quiet device, not a deaf one.
 *
 * Snooze is ten seconds, not thirty: it is a "stop that so I can listen"
 * button rather than a mute switch, and a long silence window on a device
 * whose miss cost far exceeds its false-alarm cost is the wrong default. Both
 * values are overridable at runtime by alert_ui_set_timing().
 *
 * The values themselves live in ui_config.h with every other operator-facing
 * constant. These two names are kept because the persisted settings default
 * names ALERT_MAX_MS, and a stored blob must go on meaning what it meant. */
#define ALERT_MAX_MS   ALERT_OUTPUT_MS
#define COOLDOWN_MS    5000u    /* then silence for this long                 */

/* Hold this long to request power-off. Comfortably longer than any tap and
 * comfortably shorter than the patience of someone holding a button down.
 * The value itself is BTN_HOLD_MS in ui_config.h. */
#define LONG_PRESS_MS  BTN_HOLD_MS

/* ---- the off ritual ------------------------------------------------------
 *
 * On the PCB the slide switch gates the regulator enable. It is a hard cut:
 * the firmware gets no warning, cannot draw anything, and the 3V3 rail
 * collapses in milliseconds against a ~2 s panel refresh. E-paper keeps its
 * image with no power, so a slider thrown during a guard leaves a device
 * sitting in a drawer showing its resting screen while being completely off.
 * For a device whose whole job is to be believed when it says it is
 * listening, that is the worst failure on the board, and no firmware can
 * outrun a collapsing rail.
 *
 * So the operator performs a ritual before the switch: hold the button for
 * LONG_PRESS_MS, the panel draws OFF with the reason under it, the guard
 * stops. Then the slider is thrown against an honest screen.
 *
 * A hold rather than a tap sequence, because a tap also turns the display
 * page: tapping through three pages is three taps in a rhythm, and any
 * tap-count ritual would turn the device off in the hands of somebody who was
 * only reading it. A hold cannot be produced by fumbled paging, it is one
 * gesture rather than a rhythm so it works with gloves on in the dark, and it
 * is what every other device in the operator's pocket already does.
 *
 * It does not arm during an alert. While the buzzer is going every press is a
 * snooze and nothing else, because the one moment an operator must not have
 * to think about button grammar is the moment the buzzer is going.
 *
 * The DevKit has no slider (BOARD_SLIDE_HARD_CUT 0) and the ritual still
 * works there - one behaviour, both boards. Only the words on the screen
 * differ, and on a board that cannot deep-sleep it parks instead. */

/* Which tier raised it. The alarm is the OR of the tiers and there is exactly
 * one buzzer and one dismissal, because an operator being alerted does not
 * care which statistic fired. What they care about afterwards is which one it
 * was - Tier-3 fires on fans, insects and vehicles as readily as on a rotor -
 * so the tier reaches the panel and the trace even though it never reaches
 * the buzzer. */
#define ALERT_TIER_NONE 0u
#define ALERT_TIER_V1   1u
#define ALERT_TIER_T2   2u
#define ALERT_TIER_T3   3u
#define ALERT_TIER_T4   4u
/* Not a tier: a peer heard something and said so over the radio. It travels
 * the same path as the four statistics, but it sounds different, because
 * "something is near you" and "something is near somebody else" call for
 * different actions and must be tellable apart without looking at the panel. */
#define ALERT_TIER_REMOTE 5u

/* A local alert is a continuous burst; a remote one is a repeating double
 * pulse. The numbers are chosen to be unmistakable through a jacket pocket at
 * ten metres. */
#define REMOTE_PULSE_MS   150u   /* each of the two pulses                  */
#define REMOTE_GAP_MS     150u   /* between them                            */
#define REMOTE_PERIOD_MS 1000u   /* and then silence to the next pair       */

/* Lazily configures the button pin. `mode_letter` is stamped into STAT
 * records so a capture says which mode produced it. */
void alert_ui_begin(char mode_letter);

/* Outputs off, state back to IDLE, actuators safe. Always paired with begin. */
void alert_ui_end(void);

/* Call once per frame (armed mode) or once per tick (drill).
 *   trigger  - true while any tier has an event open
 *   tier     - ALERT_TIER_*, which one opened it (ignored when !trigger)
 *   now_ms   - monotonic milliseconds
 * Returns true if outputs are currently being driven. */
bool alert_ui_tick(bool trigger, uint8_t tier, uint32_t now_ms);

/* True while the buzzer or motor is actually running. Tier-3 freezes its own
 * statistic while the device's outputs are audible, and this is what it asks. */
bool alert_ui_outputs_active(void);

/* The long press, latched so a caller polling once per frame cannot miss it.
 * alert_ui does not act on it: shutting the device down means stopping the
 * detector and the I2S buses, which belongs to whoever owns them. */
bool alert_ui_power_off_requested(void);
void alert_ui_clear_power_request(void);


/* Runtime timings, from the persisted settings. 0 leaves a value alone. */
/* Running with no battery: interleave the buzzer and the motor rather than
 * driving both at once, because together they brown out a USB-only board.
 * Measured, not assumed - see outputs_cadence(). */
void alert_ui_set_no_cell(bool none);

void alert_ui_set_timing(uint32_t alert_max_ms, uint32_t snooze_ms);

/* Which page the operator is on: EPAPER_PAGE_MAIN / _STATUS / _RECENT.
 *
 * The button lives here, so the page lives here. The frame loop reads it to
 * decide which page's text to keep filled, and the panel is told which screen
 * to draw by this module's own refresh pump - the two never disagree, because
 * there is one variable. */
uint8_t alert_ui_page(void);

/* ---- the button as an event source ---------------------------------------
 *
 * The state machine consumes these and knows nothing about GPIOs, so it can
 * be driven by the real button or by `U btn <tap|hold>` from the console.
 * That is what makes the grammar provable on a bench, and it means a wiring
 * fault can never be mistaken for a firmware one.
 *
 *   TAP     short press, fires on release  -> snooze
 *   HOLD    held BTN_HOLD_MS               -> the off ritual
 */
#define ALERT_BTN_NONE   0u
#define ALERT_BTN_TAP    1u
#define ALERT_BTN_DOUBLE 2u   /* retired; the value is not reused */
#define ALERT_BTN_HOLD   3u

void alert_ui_inject_button(uint8_t ev);

/* The virtual button: `U btn down` / `U btn up`, so a hold of an exact length
 * can be driven from a console. level > 0 presses (and synthesises the edge
 * the ISR would have counted); level <= 0 releases the override entirely and
 * hands the button back to the real pad, which is the only safe resting
 * state - a latched "up" would mask a real press. */
void alert_ui_inject_level(int level);

/* Inject one alert at the exact point a real detection enters the alert path
 * (`U alert v1 433`). Writes the ring entry too, so the counter and the
 * RECENT page behave as they would for a real event. */
void alert_ui_inject_alert(uint8_t tier, float hz);

/* What the UI wants on the glass versus what is on it (`U scr ?`). The two
 * differ whenever a refresh is in flight, which is the window a test that
 * only asked "what state are you in" could never see. Either may be -1 for
 * "nothing yet". */
void alert_ui_screens(int *want, int *shown);

/* The same thing on a repeat, for the frame-budget gate, which needs an alert
 * every 30 s while the guard is live. 0 disables. Test hook, never persisted. */
void alert_ui_set_auto_alert(uint32_t period_s);
uint32_t alert_ui_auto_alert(void);

/* The resting LED pattern, driven even when no mode is running. Call from any
 * idle loop to keep the slow "alive and listening" flash going. `listening`
 * false parks the LED dark. Touches the LED and nothing else - in particular
 * it does not sample the button, so an idle loop cannot silently consume a
 * press that its own state machine was going to act on. */
/* True once after a hold on the INFO page turned the screen. The caller
 * persists epaper_get_rotation(); the panel has already moved. Settings
 * deliberately do not reach into alert_ui.c, so that every host test of the
 * button grammar runs without an NVS stub. */
bool alert_ui_take_rotation(void);

void alert_ui_idle_led(bool listening, uint32_t now_ms);

/* The fault flash: the device is armed and is not receiving audio.
 *
 * This is the failure that wastes a field trip. A microphone bus that stops
 * delivering leaves every other indication looking healthy - the panel says
 * LISTENING, the loop keeps running, the trace says nothing to anyone who is
 * not reading it - and the box quietly hears nothing for an hour. Magenta is
 * used for nothing else, so it cannot be confused with the alerting red or
 * the green heartbeat. */
void alert_ui_fault_led(uint32_t now_ms);

/* Standby: true when the operator has tapped the button to wake the device.
 * Returns false while the power-off press is still held, so letting go of the
 * hold that turned it off cannot instantly turn it back on. */
bool alert_ui_standby_press(uint32_t now_ms);

/* Draw the OFF screen and park every output. Returns once the panel has
 * finished, or after ~6 s. Only for the power-off path - it blocks. */
void alert_ui_power_down(void);

/* Emit one STAT record describing the current state. */
void alert_ui_emit_stat(void);

/* For the `U s` command outside any armed mode. */
void alert_ui_emit_stat_idle(void);

uint8_t  alert_ui_state(void);
uint8_t  alert_ui_last_tier(void);
uint32_t alert_ui_snooze_remaining_ms(void);
uint16_t alert_ui_press_count(void);

/* Repaints of the resting screen since power-on. Counted where the panel is
 * driven, not where a redraw is asked for: a request dropped by the dwell or
 * by the no-cell deferral is not a redraw. */
uint32_t alert_ui_home_redraws(void);

/* Which tier raised the running alert, ALERT_TIER_NONE when none is. */
uint8_t alert_ui_tier(void);

/* True while the frozen alert page is showing. The frame loop asks so it can
 * publish the frozen rows instead of the live ones, and suppress the live
 * page refresh. */
bool alert_ui_frozen(void);

/* The link test's feedback: `beeps` short beeps and one short pulse. Not an
 * alert - it cannot start an alert window or reach the ALERT screen. */
void alert_ui_link_feedback(uint8_t beeps);

/* Detections that arrived while the box was snoozed - heard, recorded, and
 * deliberately not sounded. Shown on the test page as QUIET. Snooze is the
 * only deliberate silence there is, and it is the operator's own: no page and
 * no mode suppresses an alert, because the page an operator watches during a
 * field test must not be the page on which the test cannot be heard. */
uint16_t alert_ui_quiet_count(void);

/* The deciding score and threshold, pushed in so ALERT_BEGIN records what
 * fired rather than a value re-derived here. */
void alert_ui_set_decision(float score, float thr);

/* Test mode. Two navigation rules change and nothing else: the page never
 * times out, and an alert returns to the page rather than to MAIN. Detection,
 * thresholds, alert timing and the outputs are untouched. */
void alert_ui_set_test_mode(bool on);

/* Presses that registered inside an alert window. Separate from the total
 * because this is the press that can turn a seven-second alarm into two
 * beeps. */
uint16_t alert_ui_btn_alert_presses(void);

/* The button's own instrument: registered presses, presses rejected for
 * arriving inside BTN_MIN_GAP_MS of the last release, and lows too short to
 * be a press at all. The last two say whether the optional 100 nF is worth
 * fitting - a soak with nobody touching the unit should register zero
 * presses, and the glitch count says how hard the debounce had to work to
 * achieve that. */
void alert_ui_btn_stats(uint16_t *presses, uint16_t *phantoms,
                        uint16_t *glitches);
