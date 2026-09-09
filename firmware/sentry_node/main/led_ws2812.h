/*
 * led_ws2812.h - one WS2812 pixel on GPIO16, driven by RMT TX.
 *
 * Raw wire order only. There is deliberately no colour-name mapping in the
 * firmware: WS2812 parts ship in both GRB and RGB orderings and which one is
 * seated on this board is a bench observation, not a compile-time fact. The
 * host sends three channel bytes in wire order and the runbook records what
 * was observed.
 *
 * The alert colour is WHITE (255,255,255), which is identical under either
 * ordering - so the colour-order discovery can never force a rebuild and a
 * re-proof of the golden gate.
 *
 * Lazy init: the RMT channel is created on first use, never at boot.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* Creates the RMT channel + encoder on first call. Returns ESP_OK once ready.
 * Safe to call repeatedly. */
esp_err_t led_init(void);

/* Raw channel values in WIRE order. Lazily initialises. */
/* Tear down the RMT channel so the next led_set_raw() rebuilds it, pad routing
 * and all. The only supported recovery after something has detached the RMT
 * output from the pad - see the comment on the definition. */
esp_err_t led_deinit(void);

esp_err_t led_set_raw(uint8_t c0, uint8_t c1, uint8_t c2);

/* COLOURS. The one place RGB becomes this module's G,R,B wire order; every
 * caller that thinks in colours goes through here. led_set_raw() stays for
 * raw wire-order testing. */
esp_err_t led_set_rgb(uint8_t r, uint8_t g, uint8_t b);

/* The self-healing re-send loop. Re-sends the cached
 * colour every LED_REFRESH_MS so a single mis-decoded frame heals within half
 * a second instead of persisting until the next state change. Stop it before
 * the OFF ritual and deep sleep. */
/* ---- the pattern engine -------------------------------------------------
 *
 * Who is allowed to touch RMT. Computing the blink pattern on the detector
 * task and writing the RMT peripheral from there, inside the 32 ms frame
 * loop, is the wrong place twice: it puts a peripheral write
 * on the one path that must never be delayed, and it means the LED stops
 * blinking the moment the frame loop is busy, which is exactly when an
 * operator most wants to know the device is alive.
 *
 * So the UI now writes a STATE and nothing else, and the engine on core 1
 * turns that state into a colour and is the only writer of the peripheral.
 * Nothing else may call led_set_rgb() or led_set_raw() while a pattern is
 * running. */
#include "led_pattern.h"


esp_err_t led_refresh_start(void);
void      led_refresh_stop(void);

/* (0,0,0). Never faults, and is a no-op if the channel was never created -
 * so it is safe inside actuators_safe_all_off() on any path. */
void led_off(void);

/* CACHE OF THE LAST VALUE WRITTEN. NOT AN OBSERVATION OF LIGHT - the WS2812
 * has no back channel, so nothing in firmware can know whether a photon was
 * emitted. Named for what it is, because the old name (led_is_on) was read as
 * evidence of light once already and closed this bug falsely. */
bool led_last_written_nonzero(void);
void led_get_raw(uint8_t *c0, uint8_t *c1, uint8_t *c2);

/*
 * What the engine is actually doing - readable from the device rather than
 * inferred by reading the code, because the LED has no back channel and every
 * failure below looks the same from outside.
 *
 * The three symptoms are: nothing ever written (the pixel holds its power-up
 * state, seen as CONSTANT WHITE), then written but too briefly to see, then
 * written and dark. Those have completely different causes and NONE of them
 * can be told apart by looking at the LED, which is the whole problem: the
 * only readout is the thing that is broken.
 *
 * So the engine counts what it does. `U ls` prints it.
 */
typedef struct {
    uint32_t sends_ok;        /* transmissions the RMT accepted             */
    uint32_t sends_failed;    /* transmissions it rejected                  */
    uint32_t ticks;           /* engine loop iterations, so a dead task     */
                              /* is distinguishable from a silent one       */
    uint32_t skipped_locked;  /* wanted to send, the lock was held          */
    uint8_t  pattern;         /* what led_pattern() said last tick          */
    uint8_t  last[3];         /* the last bytes handed to the wire          */
    bool     task_alive;      /* the refresh task exists                    */
} led_engine_stat_t;

void led_engine_stat(led_engine_stat_t *out);
