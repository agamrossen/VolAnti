/*
 * led_pattern.h - the state language as pure arithmetic.
 *
 * state + time -> three wire bytes, and nothing else. No ESP-IDF, no RMT, no
 * FreeRTOS, which is the same arrangement power_slice.c, lora_proto.c,
 * mic_cal.c and epaper_draw.c already use and for the same reason: it lets
 * tests/test_alert_ui.py compile and drive THE SHIPPED MAPPING rather than a
 * model of it.
 *
 * That matters more here than it looks. The four LED tests measure a CADENCE
 * over a whole period, not a colour at an instant, because the heartbeat this
 * project shipped once was "working" at a 3.5% duty cycle and was reported
 * from the field as a dead LED. A blink nobody can catch is indistinguishable
 * from no blink, and only a test that counts milliseconds can tell them
 * apart. Moving the pattern engine onto its own task would have put that
 * cadence out of reach of every host test; putting the arithmetic here keeps
 * it in reach.
 *
 * Who calls what. The UI writes a state with led_pattern_set(). The engine on
 * core 1 in led_ws2812.c is the only thing that turns it into light. Nothing
 * else may write the pixel while a pattern is running.
 */
#pragma once

#include <stdint.h>

#define LED_PAT_OFF     0u
#define LED_PAT_GUARD   1u   /* green, LED_GUARD_ON_MS in LED_GUARD_PERIOD_MS */
#define LED_PAT_SNOOZE  2u   /* solid LED_SNOOZE_*                            */
#define LED_PAT_ALERT   3u   /* fast red, local and remote alike              */
#define LED_PAT_FAULT   4u   /* magenta: armed but hearing nothing            */
/* THE ENGINE STANDS DOWN. `U l rgb` and `U g` write the pixel directly for a
 * bench check, and the engine re-sends every LED_REFRESH_MS, so without this
 * a typed colour is reverted within half a second and a repeated check could
 * never be performed. */
#define LED_PAT_MANUAL  5u

/* The state the UI wants shown. One byte, deliberately unlocked: a byte
 * cannot tear, and the worst a stale read can do is hold the previous pattern
 * for one engine tick. */
void    led_pattern_set(uint8_t pattern);
uint8_t led_pattern(void);

/* state + time -> the three bytes in WIRE ORDER for this module, which is
 * G, R, B: measured on the bench and recorded in the calibration file, not
 * guessed from a part number. */
void led_pattern_colour(uint8_t pattern, uint32_t now_ms, uint8_t out[3]);
