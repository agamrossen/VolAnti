#include "source_i2s_quad.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"          /* IRAM_ATTR for the overflow ISR */
#include "driver/i2s_std.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#include "board_pins.h"
#include "generated/detector_config.h"

/* ---------------------------------------------------------------------------
 * The channel map lives here and nowhere else.
 *
 * Expectation (QUAD_I2S_NOTES.md Q4): Philips framing puts the LEFT slot first
 * in each frame, and L/R->GND makes a capsule drive the left slot. So bus A
 * yields [M1 M2] and bus B yields [M3 M4].
 *
 * That is an EXPECTATION, not a proof. The bench tap test is the ground truth.
 * If it shows the pairs swapped, the entire fix is these four integers - which
 * is exactly why they are isolated here rather than spread through the reader.
 * ------------------------------------------------------------------------ */
#define QUAD_CH_BUSA_LEFT   0    /* -> output channel 0 (M1, WEST)  */
#define QUAD_CH_BUSA_RIGHT  1    /* -> output channel 1 (M2, EAST)  */
#define QUAD_CH_BUSB_LEFT   2    /* -> output channel 2 (M3, NORTH) */
#define QUAD_CH_BUSB_RIGHT  3    /* -> output channel 3 (M4, SOUTH) */

/* A timeout here means a bus is not clocking, which is a wiring or routing
 * fault, not a slow frame. It is counted and surfaced, never retried silently
 * forever and never mistaken for end-of-stream. */
#define QUAD_READ_TIMEOUT_MS 200

static i2s_chan_handle_t s_rx_a;      /* I2S0, master, GPIO6 */
static i2s_chan_handle_t s_rx_b;      /* I2S1, slave,  GPIO7 */
static bool s_running;
static quad_stats_t s_stats;

/* One scratch buffer per bus: 256 frames x 2 slots x 4 bytes = 2048 B each. */
static int32_t s_raw_a[QUAD_FRAMES_PER_READ * 2];
static int32_t s_raw_b[QUAD_FRAMES_PER_READ * 2];

/* `id` is a plain int in the IDF v6 channel config (driver/i2s_common.h:60);
 * there is no i2s_port_t in the new driver's API. */
static i2s_chan_config_t chan_cfg(int port, i2s_role_t role)
{
    i2s_chan_config_t c = I2S_CHANNEL_DEFAULT_CONFIG(port, role);
    c.dma_desc_num = QUAD_DESC_NUM;
    c.dma_frame_num = QUAD_FRAMES_PER_READ;
    c.auto_clear = false;
    return c;
}

/* ISR CONTEXT. Nothing here may block, allocate or log: it increments one
 * counter and returns false (no task woken). */
static bool IRAM_ATTR on_ovf_a(i2s_chan_handle_t h, i2s_event_data_t *e, void *u)
{
    (void)h; (void)e; (void)u;
    s_stats.ovf[0]++;
    return false;
}

static bool IRAM_ATTR on_ovf_b(i2s_chan_handle_t h, i2s_event_data_t *e, void *u)
{
    (void)h; (void)e; (void)u;
    s_stats.ovf[1]++;
    return false;
}

static i2s_std_config_t std_cfg(gpio_num_t din, gpio_num_t bclk, gpio_num_t ws)
{
    i2s_std_config_t s = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(CFG_FS),
        /* Stereo, no slot mask: both slots are captured, which is how two
         * microphones share one data line. BCLK is unchanged at 1.024 MHz
         * because the mono config already clocked two slots, so the capsules
         * see electrically identical conditions either way. */
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = bclk,
            .ws = ws,
            .dout = I2S_GPIO_UNUSED,
            .din = din,
            .invert_flags = {
                .mclk_inv = false, .bclk_inv = false, .ws_inv = false,
            },
        },
    };
    return s;
}

esp_err_t source_i2s_quad_start(void)
{
    if (s_running) {
        return ESP_OK;
    }
    memset(&s_stats, 0, sizeof(s_stats));

    /* ---- I2S1 SLAVE FIRST -------------------------------------------------
     * Order matters. A slave RX channel has no clock of its own, so it cannot
     * latch a bit until the master starts driving BCLK/WS; creating and
     * enabling it first means it is armed and waiting when the first edge
     * appears. The driver routes GPIO4/5 as INPUTS for a slave and connects
     * I2S1I_BCK_IN / I2S1I_WS_IN through the matrix - see QUAD_I2S_NOTES.md Q2
     * for the citation. */
    i2s_chan_config_t cb = chan_cfg(I2S_NUM_1, I2S_ROLE_SLAVE);
    esp_err_t e = i2s_new_channel(&cb, NULL, &s_rx_b);
    if (e != ESP_OK) {
        s_rx_b = NULL;
        return e;
    }
    i2s_std_config_t sb = std_cfg(PIN_I2S_SD2, PIN_I2S_BCLK, PIN_I2S_WS);
    e = i2s_channel_init_std_mode(s_rx_b, &sb);
    if (e != ESP_OK) {
        goto fail;
    }

    /* ---- I2S0 MASTER SECOND ---------------------------------------------- */
    i2s_chan_config_t ca = chan_cfg(I2S_NUM_0, I2S_ROLE_MASTER);
    e = i2s_new_channel(&ca, NULL, &s_rx_a);
    if (e != ESP_OK) {
        s_rx_a = NULL;
        goto fail;
    }
    i2s_std_config_t sa = std_cfg(PIN_I2S_SD, PIN_I2S_BCLK, PIN_I2S_WS);
    e = i2s_channel_init_std_mode(s_rx_a, &sa);
    if (e != ESP_OK) {
        goto fail;
    }

    /* Re-assert the pad INPUT enable so the slave keeps seeing the clocks that
     * the master drives. Both inits call gpio_func_sel() on pads 4 and 5, and
     * the master ran second, so the input enable set by the slave init may
     * have been cleared.
     *
     * USE gpio_input_enable() AND NOTHING ELSE. This is not a style choice.
     * gpio_set_direction(..., GPIO_MODE_INPUT_OUTPUT) looks like the obvious
     * call and is CATASTROPHIC here: it routes through gpio_output_enable(),
     * whose first act is
     *
     *     gpio_hal_matrix_out_default(...)   // esp_driver_gpio/src/gpio.c:228
     *     // "No peripheral output signal routed to the pin, just as a simple
     *     //  GPIO output"
     *
     * which DISCONNECTS I2S0's BCLK/WS output signals from GPIO4/GPIO5 and
     * replaces them with a plain GPIO level. The clocks stop, and every
     * microphone on the board goes silent - including the one on bus A that
     * had nothing to do with the change. gpio_input_enable() touches only the
     * input-enable bit (gpio.c:209-214) and leaves the output matrix alone. */
    gpio_input_enable(PIN_I2S_BCLK);
    gpio_input_enable(PIN_I2S_WS);

    /* RECIPE B, applied unconditionally as belt-and-braces.
     *
     * The bench says Recipe A alone is not enough: bus A ran with zero
     * timeouts (so the master IS driving GPIO4/5) while bus B timed out
     * continuously - I2S1 was not seeing the clocks internally. The slave's
     * matrix in-routes, set during its own init, do not survive the master's
     * init of the same two pads.
     *
     * So re-establish them by hand, after both inits, using the signal indices
     * the driver itself would use for an RX slave:
     *     s_rx_bck_sig = I2S1I_BCK_IN_IDX (31)   esp_hal_i2s/esp32s3/i2s_periph.c:47
     *     s_rx_ws_sig  = I2S1I_WS_IN_IDX  (32)   ...:49
     * These calls only add an input route; they cannot disturb the master's
     * output routing, and they are idempotent if Recipe A did work. */
    esp_rom_gpio_connect_in_signal(PIN_I2S_BCLK, I2S1I_BCK_IN_IDX, false);
    esp_rom_gpio_connect_in_signal(PIN_I2S_WS, I2S1I_WS_IN_IDX, false);
    gpio_input_enable(PIN_I2S_BCLK);
    gpio_input_enable(PIN_I2S_WS);

    /* ---- enable: SLAVE first, then MASTER --------------------------------- */
    /* Register the overflow callbacks BEFORE enabling, so no window exists in
     * which the channel is running and a loss would go uncounted. A failure
     * here is not fatal - it costs the measurement, not the audio - but it is
     * reported, because an overflow counter that was never armed reads zero
     * for the same reason a healthy one does. */
    {
        const i2s_event_callbacks_t cb_a = { .on_recv_q_ovf = on_ovf_a };
        const i2s_event_callbacks_t cb_b = { .on_recv_q_ovf = on_ovf_b };
        s_stats.ovf_armed =
            (i2s_channel_register_event_callback(s_rx_a, &cb_a, NULL) == ESP_OK) &&
            (i2s_channel_register_event_callback(s_rx_b, &cb_b, NULL) == ESP_OK);
    }

    e = i2s_channel_enable(s_rx_b);
    if (e != ESP_OK) {
        goto fail;
    }
    e = i2s_channel_enable(s_rx_a);
    if (e != ESP_OK) {
        i2s_channel_disable(s_rx_b);
        goto fail;
    }

    s_running = true;
    return ESP_OK;

fail:
    if (s_rx_a) {
        i2s_del_channel(s_rx_a);
        s_rx_a = NULL;
    }
    if (s_rx_b) {
        i2s_del_channel(s_rx_b);
        s_rx_b = NULL;
    }
    return e;
}

void source_i2s_quad_stop(void)
{
    if (s_rx_a) {
        i2s_channel_disable(s_rx_a);
        i2s_del_channel(s_rx_a);
        s_rx_a = NULL;
    }
    if (s_rx_b) {
        i2s_channel_disable(s_rx_b);
        i2s_del_channel(s_rx_b);
        s_rx_b = NULL;
    }
    s_running = false;
}

/* Read one bus. Returns frames read; 0 on timeout (counted). */
static int read_bus(i2s_chan_handle_t h, int32_t *raw, int bus)
{
    size_t got = 0;
    const size_t want = QUAD_FRAMES_PER_READ * 2 * sizeof(int32_t);
    esp_err_t e = i2s_channel_read(h, raw, want, &got,
                                   QUAD_READ_TIMEOUT_MS);
    const int frames = (int)(got / (2 * sizeof(int32_t)));
    if (e != ESP_OK) {
        s_stats.timeouts[bus]++;
        return 0;
    }
    if (frames < QUAD_FRAMES_PER_READ) {
        s_stats.short_reads[bus]++;
    }
    s_stats.frames_total[bus] += (uint32_t)frames;
    return frames;
}

int source_i2s_quad_read(int16_t *dst)
{
    return source_i2s_quad_read_ex(dst, false);
}

int source_i2s_quad_read_ex(int16_t *dst, bool lenient)
{
    if (!s_running) {
        return 0;
    }
    const int na = read_bus(s_rx_a, s_raw_a, 0);
    const int nb = read_bus(s_rx_b, s_raw_b, 1);

    /* LENIENT is for the METER ONLY. A dead bus used to zero the whole frame,
     * which made all four channels read 0.0 and hid the fact that the other
     * bus was perfectly alive - the operator could not tell "both dead" from
     * "one dead". In lenient mode the live bus is reported and the dead one is
     * zero-filled; the per-bus timeout counters say which is which.
     *
     * The strict path (parity, guard) still refuses a partial frame: fabricated
     * samples would break the sample-exact premise and misalign every later
     * frame. */
    if (lenient && (na == 0) != (nb == 0)) {
        const int n = (na > nb) ? na : nb;
        for (int f = 0; f < n; f++) {
            int16_t *o = dst + (size_t)f * QUAD_N_CH;
            o[QUAD_CH_BUSA_LEFT]  = na ? (int16_t)(s_raw_a[2 * f]     >> 16) : 0;
            o[QUAD_CH_BUSA_RIGHT] = na ? (int16_t)(s_raw_a[2 * f + 1] >> 16) : 0;
            o[QUAD_CH_BUSB_LEFT]  = nb ? (int16_t)(s_raw_b[2 * f]     >> 16) : 0;
            o[QUAD_CH_BUSB_RIGHT] = nb ? (int16_t)(s_raw_b[2 * f + 1] >> 16) : 0;
        }
        return n;
    }
    /* Use the shorter of the two so a frame is only emitted when BOTH buses
     * contributed to it. A partial frame would silently misalign every later
     * one, which is precisely the class of bug the sequence numbers exist to
     * make visible. */
    const int n = (na < nb) ? na : nb;
    if (n <= 0) {
        return 0;
    }
    for (int f = 0; f < n; f++) {
        /* Same conversion rule as the parity-proven mono path: the 24-bit
         * sample sits in bits 31..8, and the pipeline's int16 is the top 16
         * bits. Arithmetic shift, cast AFTER the shift so the sign survives.
         * No DC removal, no gain - the reference has no such stage. */
        const int16_t l_a = (int16_t)(s_raw_a[2 * f]     >> 16);
        const int16_t r_a = (int16_t)(s_raw_a[2 * f + 1] >> 16);
        const int16_t l_b = (int16_t)(s_raw_b[2 * f]     >> 16);
        const int16_t r_b = (int16_t)(s_raw_b[2 * f + 1] >> 16);
        int16_t *o = dst + (size_t)f * QUAD_N_CH;
        o[QUAD_CH_BUSA_LEFT]  = l_a;
        o[QUAD_CH_BUSA_RIGHT] = r_a;
        o[QUAD_CH_BUSB_LEFT]  = l_b;
        o[QUAD_CH_BUSB_RIGHT] = r_b;
    }
    return n;
}

const quad_stats_t *source_i2s_quad_stats(void) { return &s_stats; }
bool source_i2s_quad_running(void) { return s_running; }
