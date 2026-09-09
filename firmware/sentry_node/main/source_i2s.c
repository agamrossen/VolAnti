/*
 * source_i2s.c - the real microphone, behind the same source_read() the
 * golden vectors use.
 *
 * NOTHING in detector.c changes for this. If it ever has to, the abstraction
 * has been broken and the parity evidence is void.
 *
 * SAMPLE FORMAT - the part that is easy to get wrong. Full derivation in
 * PORTING_NOTES.md sec 10.
 *
 *   The INMP441 emits 24-bit two's-complement, MSB first, in a 32-bit slot,
 *   Philips timing (MSB one BCLK after the WS edge). The I2S peripheral
 *   assembles that into a 32-bit word as
 *
 *       bits 31..8  = the 24 data bits, sign-extended by position
 *       bits  7..0  = zero (the mic drives nothing there)
 *
 *   so the signed 24-bit value is (raw >> 8), and the pipeline's int16 is the
 *   TOP 16 BITS of that:  q = raw >> 16.
 *
 *   That is a pure re-scaling by 2^-16 and nothing else. NO DC removal, NO
 *   high-pass, NO gain. The Python reference has no such stage, and adding one
 *   here would be an algorithm change masquerading as plumbing - the DC offset
 *   is MEASURED instead, by meter mode.
 */

#include "source.h"

#include <string.h>

#include "driver/i2s_std.h"
#include "esp_log.h"

#include "board_pins.h"
#include "generated/detector_config.h"

/* An I2S peripheral is a singleton piece of hardware; this handle is the one
 * piece of module state in the project outside detector_state_t, and it is
 * hardware identity, not algorithm state. */
static i2s_chan_handle_t s_rx;
static uint32_t s_short_reads;
static uint32_t s_timeouts;

/* DMA sizing. The hop is 512 samples = 32 ms and the detector spends up to
 * ~19.6 ms of that computing, during which nothing is draining the DMA. Six
 * descriptors of 512 frames buffer 3072 samples = 192 ms, i.e. six whole hops
 * of slack. Mono 32-bit slot => 4 bytes per frame => 2048 B per descriptor,
 * comfortably under the 4092 B per-descriptor ceiling. */
#define I2S_DESC_NUM    6
#define I2S_FRAME_NUM   512

/* Scratch for one read. Sized to the largest single request the frame loop
 * makes (the initial n_fft-sample window fill). */
static int32_t s_raw[CFG_N_FFT];

static int i2s_read(source_t *s, int16_t *dst, size_t n)
{
    if (s_rx == NULL) {
        return -ESP_ERR_INVALID_STATE;
    }
    size_t done = 0;
    while (done < n) {
        const size_t want = (n - done > CFG_N_FFT) ? CFG_N_FFT : (n - done);
        size_t got = 0;
        /* Generous timeout: at 16 kHz, CFG_N_FFT samples take 128 ms. A
         * timeout here means the mic is not clocking, which is a wiring fault,
         * not a slow frame - so it is counted and surfaced, never retried
         * silently forever. */
        esp_err_t e = i2s_channel_read(s_rx, s_raw, want * sizeof(int32_t),
                                       &got, 1000);
        const size_t nsamp = got / sizeof(int32_t);
        if (e != ESP_OK) {
            s_timeouts++;
            return (int)done;
        }
        if (nsamp < want) {
            s_short_reads++;
        }
        for (size_t k = 0; k < nsamp; k++) {
            /* >> 16, arithmetic: 24-bit sample in bits 31..8, keep the top 16.
             * The cast is after the shift so the sign survives. */
            dst[done + k] = (int16_t)(s_raw[k] >> 16);
        }
        done += nsamp;
        if (nsamp == 0) {
            return (int)done;
        }
        s->pos += nsamp;
    }
    return (int)done;
}

esp_err_t source_i2s(source_t *s)
{
    memset(s, 0, sizeof(*s));
    s->name = "i2s:inmp441";
    s->preset = SENTRY_DEFAULT_PRESET;
    s->thr = DEFAULT_THRESHOLD;      /* live audio runs the DEPLOYMENT point */
    s->read = i2s_read;
    s->n_total = 0;                  /* unbounded */
    s_short_reads = 0;
    s_timeouts = 0;

    if (s_rx != NULL) {
        return ESP_OK;               /* already up */
    }

    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_DESC_NUM;
    chan_cfg.dma_frame_num = I2S_FRAME_NUM;
    chan_cfg.auto_clear = false;
    esp_err_t e = i2s_new_channel(&chan_cfg, NULL, &s_rx);
    if (e != ESP_OK) {
        s_rx = NULL;
        return e;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(CFG_FS),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,     /* INMP441 has no MCLK input */
            .bclk = PIN_I2S_BCLK,
            .ws   = PIN_I2S_WS,
            .dout = I2S_GPIO_UNUSED,     /* receive only */
            .din  = PIN_I2S_SD,
            .invert_flags = {
                .mclk_inv = false, .bclk_inv = false, .ws_inv = false,
            },
        },
    };
    /* L/R strapped to GND => the mic drives the LEFT slot. Std mode always
     * clocks two slots; this selects which one is stored. */
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    e = i2s_channel_init_std_mode(s_rx, &std_cfg);
    if (e != ESP_OK) {
        i2s_del_channel(s_rx);
        s_rx = NULL;
        return e;
    }
    e = i2s_channel_enable(s_rx);
    if (e != ESP_OK) {
        i2s_del_channel(s_rx);
        s_rx = NULL;
        return e;
    }
    return ESP_OK;
}

void source_i2s_stop(void)
{
    if (s_rx != NULL) {
        i2s_channel_disable(s_rx);
        i2s_del_channel(s_rx);
        s_rx = NULL;
    }
}

uint32_t source_i2s_short_reads(void)
{
    return s_short_reads;
}

uint32_t source_i2s_timeouts(void)
{
    return s_timeouts;
}
