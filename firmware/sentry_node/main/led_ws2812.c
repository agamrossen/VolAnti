#include "led_ws2812.h"

#include <string.h>

#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

#include "board_pins.h"
#include "led_pattern.h"
#include "led_strip_encoder.h"
#include "ui_config.h"

/* 10 MHz => 0.1 us per tick, which is what led_strip_encoder.c assumes for its
 * WS2812 bit timings. Do not change one without the other. */
#define LED_RMT_RESOLUTION_HZ 10000000

/* Bounded wait for the transmission to drain. Watchdogs are disabled by
 * project design, so every wait in this build has a timeout: a stuck RMT
 * channel must return an error, not hang the command loop forever. */
#define LED_TX_TIMEOUT_MS 200

static rmt_channel_handle_t s_chan;
static rmt_encoder_handle_t s_enc;
static bool s_ready;
static uint8_t s_raw[3];

/* ---------------------------------------------------------------------------
 * Reliability, and what it is and is not.
 *
 * The fitted pixel is a WS2812B-V6. It has been SEEN to emit a colour nobody
 * commanded, which is the signature of a mis-decoded frame - the die is alive
 * and the pad is driven, but a frame boundary was not respected. The reset
 * length in led_strip_encoder.c is the correctness fix. Everything here is
 * TOLERANCE: repeats and a periodic re-send do not make a marginal pixel
 * correct, they make one bad frame invisible to a human because another is
 * 400 us or half a second behind it.
 *
 * None of it can raise a rail. If VDD sits at the part's rated floor, this
 * improves the odds and settles nothing.
 * ------------------------------------------------------------------------ */
static SemaphoreHandle_t s_lock;      /* foreground vs the refresh task     */
static TaskHandle_t      s_refresh_task;
static volatile bool     s_refresh_run;
/* THE INSTRUMENT. See led_ws2812.h: the LED cannot report its own faults
 * because the LED is the report. */
static volatile uint32_t s_st_ok, s_st_fail, s_st_ticks, s_st_locked;
static volatile uint8_t  s_st_pat;
static volatile uint8_t  s_st_last[3];
static bool              s_warmed;

/* Every channel is capped before it reaches the wire. Full white is about
 * 55 mA and sags a marginal rail hardest, which browns out the pixel's own
 * decoder - the very failure the repeats above are covering for. */
static inline uint8_t led_cap(uint8_t v)
{
    return (v > LED_MAX_CHANNEL) ? (uint8_t)LED_MAX_CHANNEL : v;
}

/* ONE transmission. No locking, no caching, no repeats - the callers below
 * layer those on. Separated so the warm start can use it from inside
 * led_init(), where the lock is already held or not yet needed. */
static esp_err_t led_tx_once(uint8_t c0, uint8_t c1, uint8_t c2)
{
    uint8_t buf[3] = {c0, c1, c2};
    rmt_transmit_config_t tx = {.loop_count = 0};
    esp_err_t e = rmt_transmit(s_chan, s_enc, buf, sizeof(buf), &tx);
    if (e != ESP_OK) {
        return e;
    }
    /* Bounded, per the no-unbounded-waits rule. */
    return rmt_tx_wait_all_done(s_chan, LED_TX_TIMEOUT_MS);
}

esp_err_t led_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }
    rmt_tx_channel_config_t tx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = PIN_LED_WS2812,
        .mem_block_symbols = 64,
        .resolution_hz = LED_RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    esp_err_t e = rmt_new_tx_channel(&tx_cfg, &s_chan);
    if (e != ESP_OK) {
        s_chan = NULL;
        return e;
    }
    led_strip_encoder_config_t enc_cfg = {
        .resolution = LED_RMT_RESOLUTION_HZ,
        /* 280 us, not the 50 us Espressif's ancient example shipped with.
         *
         * Not the fix for anything observed: a short latch cannot explain a
         * dark LED here, because
         * init_level and eot_level are both 0 and the driver programs the
         * channel's fixed idle level from them, so the line is held LOW
         * indefinitely between frames and every frame already gets a latch
         * thousands of times longer than any datasheet asks.
         *
         * It is here because it is CORRECT. LED1 on this board is a
         * WS2812B-V6 (Worldsemi, LCSC C52917433, read off the schematic), and
         * Espressif's own led_strip 3.0.3 uses 280 us with the comment that
         * WS2812B-V5 needs it. Shipping 50 us against V6 silicon would be
         * relying on a margin nobody has measured. The value itself lives in
         * ui_config.h with the rest of the tuning surface. */
        .reset_us = LED_RESET_US,
    };
    e = rmt_new_led_strip_encoder(&enc_cfg, &s_enc);
    if (e != ESP_OK) {
        rmt_del_channel(s_chan);
        s_chan = NULL;
        s_enc = NULL;
        return e;
    }
    e = rmt_enable(s_chan);
    if (e != ESP_OK) {
        rmt_del_encoder(s_enc);
        rmt_del_channel(s_chan);
        s_chan = NULL;
        s_enc = NULL;
        return e;
    }
    s_ready = true;

    /* WARM START. The pixel's shift register holds whatever it held; a first
     * frame sent into an unknown state is a first frame that can be
     * mis-decoded. Settle, blank it, and leave a clean boundary behind. */
    if (!s_warmed) {
        s_warmed = true;
        vTaskDelay(pdMS_TO_TICKS(2));
        (void)led_tx_once(0, 0, 0);
        ets_delay_us(LED_INTERFRAME_US);
    }
    return ESP_OK;
}

/* TEAR THE CHANNEL DOWN so the NEXT led_set_raw() rebuilds it from scratch.
 *
 * Why this has to exist. led_init() short-circuits on s_ready, so once the
 * channel is built the pad routing is never re-established. Anything that
 * detaches the RMT output from the pad - and gpio_config(GPIO_MODE_INPUT)
 * does exactly that, permanently, by setting oen_sel so the peripheral can
 * never re-assert output enable - leaves the driver returning ESP_OK forever
 * with nothing on the wire. There was no way back short of a reboot.
 *
 * This is that way back. */
esp_err_t led_deinit(void)
{
    if (!s_ready) {
        return ESP_OK;
    }
    esp_err_t e = rmt_disable(s_chan);
    if (e != ESP_OK) {
        return e;
    }
    e = rmt_del_encoder(s_enc);
    if (e != ESP_OK) {
        return e;
    }
    e = rmt_del_channel(s_chan);
    if (e != ESP_OK) {
        return e;
    }
    s_chan = NULL;
    s_enc = NULL;
    s_ready = false;
    return ESP_OK;
}

static esp_err_t led_send_locked(uint8_t c0, uint8_t c1, uint8_t c2)
{
    esp_err_t e = led_init();
    if (e != ESP_OK) {
        return e;
    }
    /* REPEAT EVERY FRAME. A pixel that mis-decodes one frame decodes the next;
     * the encoder's own reset covers most of the gap and the extra idle
     * guarantees the boundary. Under 2 ms, and off the audio path. */
    for (int i = 0; i < LED_SEND_REPEATS; i++) {
        if (i) {
            ets_delay_us(LED_INTERFRAME_US);
        }
        e = led_tx_once(c0, c1, c2);
        if (e != ESP_OK) {
            return e;
        }
    }
    s_raw[0] = c0;
    s_raw[1] = c1;
    s_raw[2] = c2;
    return ESP_OK;
}

esp_err_t led_set_raw(uint8_t c0, uint8_t c1, uint8_t c2)
{
    c0 = led_cap(c0);
    c1 = led_cap(c1);
    c2 = led_cap(c2);
    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const esp_err_t e = led_send_locked(c0, c1, c2);
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
    return e;
}

/* The only place RGB becomes wire order.
 *
 * This module is wired G, R, B - measured on the bench from what was actually
 * seen, not guessed from a part number. Every caller that thinks
 * in colours goes through here, so if a different module is ever fitted this
 * one line changes and nothing else does. `U l rgb <r> <g> <b>` exists so the
 * mapping can be checked by eye rather than believed. */
esp_err_t led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return led_set_raw(g, r, b);
}


void led_off(void)
{
    if (!s_ready) {
        /* Never created, therefore nothing is lit and nothing to do. This is
         * what makes actuators_safe_all_off() callable on any path. */
        s_raw[0] = s_raw[1] = s_raw[2] = 0;
        return;
    }
    (void)led_set_raw(0, 0, 0);
}

/* ---------------------------------------------------------------------------
 * The self-healing refresh loop.
 *
 * Re-sends the CURRENT cached colour every LED_REFRESH_MS, unconditionally.
 * This is the single biggest reliability win available on a marginal pixel:
 * any missed latch heals within half a second, so an intermittent LED reads to
 * a human as a steady one.
 *
 * It is deliberately dull. Priority 1 - below the detector and below the panel
 * task. It never touches RMT from an ISR. It takes the same lock the
 * foreground uses, so it can never interleave with a commanded change, and it
 * skips its turn rather than waiting if a frame is already in flight. And it
 * stops cleanly for the OFF ritual, so a device drawing OFF does not have a
 * task quietly re-lighting the LED behind the screen.
 * ------------------------------------------------------------------------ */
/* THE ONLY WRITER OF THE PERIPHERAL. Runs on core 1 at priority 1: below the
 * display task, and a long way below the detector, which must never wait for
 * a pixel. */
static void led_refresh_body(void *arg)
{
    (void)arg;
    uint8_t cur[3] = {0, 0, 0};
    bool have = false;
    uint32_t last_send_ms = 0;
    while (s_refresh_run) {
        vTaskDelay(pdMS_TO_TICKS(LED_TICK_MS));
        /* NOT `!s_ready`, AND THAT WAS THE BUG. led_send_locked() calls
         * led_init() itself, and led_init() is the ONLY thing that sets
         * s_ready. The old refresh task could test it because the foreground
         * had already initialised the pad by writing a colour; this engine is
         * now the only writer there is, so testing s_ready made it wait for an
         * initialisation that only it could ever trigger. It never wrote the
         * pixel at all, and a WS2812 that receives no frame simply holds
         * whatever state it powered up in, which presents as an LED that is
         * continuously on. */
        if (!s_refresh_run || !s_lock) {
            continue;
        }
        s_st_ticks++;
        const uint8_t pat = led_pattern();
        s_st_pat = pat;
        if (pat == LED_PAT_MANUAL) {
            /* Somebody is looking at a typed colour. Do not touch the pixel,
             * and forget what was last sent so the next real pattern is
             * written rather than skipped as unchanged. */
            have = false;
            continue;
        }
        const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        uint8_t want[3];
        led_pattern_colour(pat, now, want);

        /* THE BRIGHTNESS CAP, WHICH THIS ENGINE SKIPPED. led_set_raw() caps
         * every channel and the engine bypassed it by calling
         * led_send_locked() directly, so an alert went out at 255 where the
         * cap is 180. That is not merely brighter: full drive is about 55 mA
         * and sags a marginal rail hardest, which browns out the pixel's OWN
         * DECODER, and a mis-decoded frame latches a colour nobody asked for
         * until something sends again, which presents as an LED stuck on. */
        want[0] = led_cap(want[0]);
        want[1] = led_cap(want[1]);
        want[2] = led_cap(want[2]);

        /* THE SELF-HEAL IS INSIDE THE ENGINE, not beside it. A pixel that
         * mis-decoded one frame holds the wrong colour until something sends
         * again; re-sending the CURRENT pattern colour every LED_REFRESH_MS
         * bounds that at half a second without a second writer existing. */
        const bool changed = !have || memcmp(cur, want, 3) != 0;
        const bool heal = (uint32_t)(now - last_send_ms) >= LED_REFRESH_MS;
        if (!changed && !heal) {
            continue;
        }
        if (xSemaphoreTake(s_lock, 0) != pdTRUE) {
            s_st_locked++;
            continue;
        }
        if (led_send_locked(want[0], want[1], want[2]) == ESP_OK) {
            s_st_ok++;
        } else {
            s_st_fail++;
        }
        s_st_last[0] = want[0]; s_st_last[1] = want[1]; s_st_last[2] = want[2];
        s_raw[0] = want[0]; s_raw[1] = want[1]; s_raw[2] = want[2];
        xSemaphoreGive(s_lock);
        cur[0] = want[0]; cur[1] = want[1]; cur[2] = want[2];
        have = true;
        last_send_ms = now;
    }
    s_refresh_task = NULL;
    vTaskDelete(NULL);
}

void led_engine_stat(led_engine_stat_t *out)
{
    if (!out) return;
    out->sends_ok      = s_st_ok;
    out->sends_failed  = s_st_fail;
    out->ticks         = s_st_ticks;
    out->skipped_locked = s_st_locked;
    out->pattern       = s_st_pat;
    out->last[0] = s_st_last[0];
    out->last[1] = s_st_last[1];
    out->last[2] = s_st_last[2];
    out->task_alive    = (s_refresh_task != NULL);
}

esp_err_t led_refresh_start(void)
{
    if (s_refresh_task) {
        return ESP_OK;
    }
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        if (!s_lock) {
            return ESP_ERR_NO_MEM;
        }
    }
    s_refresh_run = true;
    /* PINNED TO CORE 1. The frame loop owns core 0 and must never share a
     * scheduler slot with a peripheral write it does not need. */
    if (xTaskCreatePinnedToCore(led_refresh_body, "led_pat", 2048, NULL, 1,
                                &s_refresh_task, 1) != pdPASS) {
        s_refresh_run = false;
        s_refresh_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void led_refresh_stop(void)
{
    s_refresh_run = false;
}

/* CACHE OF THE LAST VALUE WRITTEN. THIS IS NOT AN OBSERVATION OF LIGHT.
 *
 * The WS2812 has no back channel; nothing in firmware can know whether a
 * photon was emitted. This function reports the firmware's own intention back
 * to itself. Read as evidence of light it will close a real bug as working -
 * STAT samples it lit at the duty cycle it was told to output, and that is
 * evidence of nothing but itself. The name says so. */
bool led_last_written_nonzero(void)
{
    return (s_raw[0] | s_raw[1] | s_raw[2]) != 0;
}

void led_get_raw(uint8_t *c0, uint8_t *c1, uint8_t *c2)
{
    if (c0) { *c0 = s_raw[0]; }
    if (c1) { *c1 = s_raw[1]; }
    if (c2) { *c2 = s_raw[2]; }
}
