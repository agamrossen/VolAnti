/* led_pattern.c - see led_pattern.h. Pure arithmetic, no ESP-IDF. */

#include "led_pattern.h"

#include "ui_config.h"

static volatile uint8_t s_pattern = LED_PAT_OFF;

void led_pattern_set(uint8_t pattern) { s_pattern = pattern; }
uint8_t led_pattern(void)             { return s_pattern; }

void led_pattern_colour(uint8_t pattern, uint32_t now_ms, uint8_t out[3])
{
    uint8_t r = 0, g = 0, b = 0;

    switch (pattern) {
    case LED_PAT_GUARD:
        /* A clear pulse: 300 in 1200 is 25%. Both 3.5% and 7.5% were
         * reported from the field as a dead LED on boards that were flashing
         * exactly as designed. A blink nobody can catch is indistinguishable
         * from no blink, and the person holding the device is the only
         * instrument that can settle which is which. */
        if ((now_ms % LED_GUARD_PERIOD_MS) < LED_GUARD_ON_MS) {
            r = LED_GUARD_R; g = LED_GUARD_G; b = LED_GUARD_B;
        }
        break;

    case LED_PAT_SNOOZE:
        /* SOLID, for the whole window. The operator silenced the device; the
         * one thing they must be able to check at a glance is that it is
         * still silenced, and a pattern that resembled the heartbeat would
         * say the opposite of the truth. */
        r = LED_SNOOZE_R; g = LED_SNOOZE_G; b = LED_SNOOZE_B;
        break;

    case LED_PAT_ALERT:
        /* FAST RED, local and remote alike. The amber this replaced tried to
         * carry local-versus-remote in a HUE; across a dark field, at 96/255,
         * with no reference, amber and red are the same light. The
         * distinction that survives is the buzzer cadence and the panel. */
        if ((now_ms % (LED_ALERT_ON_MS + LED_ALERT_OFF_MS)) < LED_ALERT_ON_MS) {
            r = LED_ALERT_R; g = LED_ALERT_G; b = LED_ALERT_B;
        }
        break;

    case LED_PAT_FAULT:
        /* Magenta, and a slower, more even blink than the alarm's. It cannot
         * share fast red with alerting: one signal with two meanings would
         * make a dead microphone bus and a drone overhead the same light on a
         * hillside. */
        if ((now_ms % (LED_FAULT_ON_MS + LED_FAULT_OFF_MS)) < LED_FAULT_ON_MS) {
            r = LED_FAULT_R; g = LED_FAULT_G; b = LED_FAULT_B;
        }
        break;

    default:
        break;                          /* LED_PAT_OFF: dark, and driven low */
    }

    out[0] = g;                         /* GRB on this module */
    out[1] = r;
    out[2] = b;
}
