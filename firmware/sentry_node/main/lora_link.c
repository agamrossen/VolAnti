#include "lora_link.h"

#include <stdio.h>
#include <string.h>

#include "boot_rec.h"
#include "settings.h"
#include "lora_proto.h"

#if BOARD_HAS_LORA

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "alert_ui.h"
#include "event_log.h"
#include "trace.h"

/* ---- SX1276 registers. Only the ones this driver touches. -------------- */
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_PA_CONFIG            0x09
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PREAMBLE_MSB         0x20
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4D

#define MODE_LONG_RANGE          0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05

#define IRQ_TX_DONE              0x08
#define IRQ_PAYLOAD_CRC_ERROR    0x20
#define IRQ_RX_DONE              0x40

#define SX1276_VERSION           0x12

/* BW125 / CR4-5, preamble 8, private sync word 0x12, CRC on, through
 * PA_BOOST.
 *
 * Spreading factor is chosen for field margin. Each step up buys receiver
 * sensitivity and costs airtime, and the airtime is computed rather than
 * waved at:
 *
 *   SF7 /BW125/CR4-5, 18 bytes:   51.5 ms a frame,  154 ms a three-frame burst
 *   SF9 /BW125/CR4-5, 18 bytes:  185.3 ms a frame,  556 ms a three-frame burst
 *   SF10/BW125/CR4-5, 18 bytes:  329.7 ms a frame,  989 ms a three-frame burst
 *
 * SF10 is about +8.5 dB of sensitivity over SF7 for no extra current. In free
 * space every 6 dB doubles the range, so the expectation is a large range
 * gain - which is a prediction, not a measurement: the pair have to be walked
 * apart to settle it.
 *
 * A second of airtime for an alert that has already sounded locally is a
 * trade worth making: the local buzzer never waits on the radio. LDRO is not
 * needed - SF10 at BW125 is 8.192 ms a symbol, under the 16 ms
 * rule. The TxDone poll below is 600 ms and MUST grow with this; it does.
 *
 * The other cause is not software. A quarter-wave whip radiates in a torus
 * with deep nulls off its ENDS, so two units with their antennas pointed at
 * each other tip-to-tip are aimed into each other's null. Keep both antennas
 * VERTICAL and parallel. No firmware change can fix an antenna null. */
#define LORA_SF                  10
#define LORA_SYNC_WORD           0x12
#define LORA_PREAMBLE            8
/* 17 dBm is the Ra-01H's rated PA_BOOST maximum without the +20 dBm PA_DAC
 * mode, which the datasheet limits to a 1% duty cycle. This stays at the
 * normal setting: no duty-cycle caveat, no extra failure mode on a battery. */
/* 13 dBm, not the module's rated 17. At 17 and SF10 the three frames of a
 * burst queue into about a second of continuous PA draw, and the board
 * records `reset=BROWNOUT` at `phase=TX2` on a healthy 4166 mV cell, on USB,
 * with the buzzer and the panel already sequenced clear of it. The radio
 * alone collapses the rail, so no alert completes and the operator hears one
 * faint beep.
 *
 * 13 dBm is -4 dB on the module's maximum and still well above the 10 it
 * replaced, and the frames are now spaced past their own airtime so the PA
 * rests between them. A peer link that prevents the local alarm is strictly
 * worse than no peer link - the local alarm is the entire product.
 *
 * This is a software mitigation of what is probably a hardware limit, namely
 * bulk decoupling at the module. The standing rule that a reset on a healthy
 * cell is a hardware question still holds; this buys a working device while
 * that is answered. */
#define LORA_TX_DBM              13

#define LORA_TASK_STACK          3072
#define LORA_TASK_PRIO           2
#define LORA_TASK_CORE           1     /* NOT the audio core */

/* One burst at a time: an alert onset while a burst is still in the air
 * replaces it, because the newer alert is the one worth sending. */
typedef struct {
    uint8_t  pkt[LORA_PKT_BYTES];
    uint8_t  left;                /* transmissions still owed */
    uint32_t due_ms;
} burst_t;

static spi_device_handle_t s_spi;
static bool     s_ready;
static bool     s_enabled = true;
static uint32_t s_freq_hz = BOARD_LORA_HZ_DEFAULT;
static uint16_t s_id;
static uint8_t  s_seq;
static uint8_t  s_version_read = 0xFF;

static TaskHandle_t  s_task;
static QueueHandle_t s_rxq;
static volatile bool s_task_stop;

static burst_t  s_burst;
static lora_dedup_t s_dedup;

static volatile uint32_t s_remote_until_ms;
static volatile uint16_t s_remote_id;
static volatile uint8_t  s_remote_tier;
/* The sender's score, x100, off the packet. 0 means the peer did not report
 * one, which is what an older image transmits. */
static volatile uint16_t s_remote_score;
static volatile uint16_t s_remote_thr1;

/* The link test's one-shot event for the UI, and who it involved. */
static volatile uint8_t  s_link_event;
static volatile uint16_t s_link_peer;

/* Counters. "the radio is receiving nothing" and "the radio is receiving
 * something it will not accept" are completely different faults, and until
 * these existed the two were indistinguishable from outside the box. */
static uint32_t s_n_tx, s_n_rx_ok, s_n_rx_bad, s_n_rx_dup, s_n_rx_self;

/* ---- link quality of the last frame -------------------------------------
 * Captured for every frame that passes CRC, including one that is then
 * dropped as a duplicate or as our own id. That is deliberate: "the packet
 * arrived and was discarded" and "no packet arrived" are the two answers a
 * range walk has to tell apart, and a counter that only moved on accepted
 * frames could not tell them apart at all. */
static volatile int16_t s_last_rssi_dbm;
static volatile int16_t s_last_snr_db;
static volatile bool    s_have_rx_quality;
/* INJECTED frames, counted APART FROM THE AIR - see admit_frame(). A bench
 * drill that advanced rx_ok would leave the next morning's two-board air test
 * judged against a counter that had already moved. */
static uint32_t s_n_rx_loop;

/* ---- SPI ---------------------------------------------------------------- */

static void cs(int level)
{
    gpio_set_level(PIN_LORA_CS, level);
}

static uint8_t xfer(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {reg, val};
    uint8_t rx[2] = {0, 0};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    cs(0);
    const esp_err_t e = spi_device_polling_transmit(s_spi, &t);
    cs(1);
    return (e == ESP_OK) ? rx[1] : 0xFFu;
}

static uint8_t rd(uint8_t reg)       { return xfer(reg & 0x7Fu, 0x00); }
static void    wr(uint8_t reg, uint8_t v) { (void)xfer(reg | 0x80u, v); }

static void set_mode(uint8_t m) { wr(REG_OP_MODE, MODE_LONG_RANGE | m); }

static void set_frequency(uint32_t hz)
{
    /* frf = hz * 2^19 / 32e6. Done in 64-bit because 868e6 << 19 is not a
     * number that fits in 32. */
    const uint64_t frf = ((uint64_t)hz << 19) / 32000000ull;
    wr(REG_FRF_MSB + 0, (uint8_t)(frf >> 16));
    wr(REG_FRF_MSB + 1, (uint8_t)(frf >> 8));
    wr(REG_FRF_MSB + 2, (uint8_t)(frf >> 0));
}

static void radio_configure(void)
{
    set_mode(MODE_SLEEP);               /* LoRa mode is only settable asleep */
    set_frequency(s_freq_hz);
    wr(REG_FIFO_TX_BASE_ADDR, 0);
    wr(REG_FIFO_RX_BASE_ADDR, 0);
    wr(REG_MODEM_CONFIG_1, 0x72);       /* BW125, CR4-5, explicit header */
    wr(REG_MODEM_CONFIG_2, (uint8_t)((LORA_SF << 4) | 0x04));  /* CRC on */
    wr(REG_MODEM_CONFIG_3, 0x04);       /* AGC on; LDRO off (SF7 @ BW125) */
    wr(REG_PREAMBLE_MSB, 0);
    wr(REG_PREAMBLE_MSB + 1, LORA_PREAMBLE);
    wr(REG_SYNC_WORD, LORA_SYNC_WORD);
    /* PA_BOOST, and there is no choice about it on this module - the Ra-01H
     * does not bond RFO, so clearing this bit transmits into nothing. */
    wr(REG_PA_CONFIG, (uint8_t)(0x80u | (LORA_TX_DBM - 2)));
    wr(REG_PA_DAC, 0x84);               /* normal, not the +20 dBm mode */
    set_mode(MODE_STDBY);
}

static void radio_rx(void)
{
    wr(REG_DIO_MAPPING_1, 0x00);        /* DIO0 = RxDone */
    set_mode(MODE_RX_CONTINUOUS);
}

/* ---- DIO0: an edge, and nothing else in the ISR ------------------------ */

static void IRAM_ATTR dio0_isr(void *arg)
{
    (void)arg;
    BaseType_t woke = pdFALSE;
    const uint8_t token = 1;
    /* The ISR posts a token and returns. Reading the FIFO means SPI
     * transactions, which must not happen in interrupt context on a bus the
     * e-paper task is also using. */
    if (s_rxq) {
        xQueueSendFromISR(s_rxq, &token, &woke);
    }
    if (woke) {
        portYIELD_FROM_ISR();
    }
}

/* ---- receive ------------------------------------------------------------ */

static void admit_frame(const uint8_t *buf, int n, uint32_t now_ms,
                        bool injected);
static void build_frame(uint8_t *dst, const lora_pkt_t *p);

static void admit_frame(const uint8_t *buf, int n, uint32_t now_ms,
                        bool injected);
/* Forward-declared: admit_frame() queues the link REPLY, and the burst
 * builder is defined further down beside the transmit path. */
static void queue_burst(uint8_t tier, uint8_t flags, float score);

static void handle_rx(uint32_t now_ms)
{
    const uint8_t irq = rd(REG_IRQ_FLAGS);
    wr(REG_IRQ_FLAGS, irq);             /* write-1-to-clear */
    if (!(irq & IRQ_RX_DONE)) {
        return;
    }
    if (irq & IRQ_PAYLOAD_CRC_ERROR) {
        s_n_rx_bad++;
        return;
    }
    /* RSSI AND SNR BEFORE THE FIFO IS DRAINED. Both registers describe the
     * packet that has just been received and are valid until the next one
     * starts; reading them after admit_frame() would race the next arrival.
     *
     * -157 is the HF port constant from the SX1276 datasheet (section 5.5.5),
     * which is the correct one at 868 MHz; -164 is the LF port and would
     * report this link 7 dB better than it is. SNR is a signed quarter-dB
     * count, so it is divided by 4 and not by anything else. */
    {
        const int rssi_raw = (int)rd(REG_PKT_RSSI_VALUE);
        const int snr_raw  = (int)(int8_t)rd(REG_PKT_SNR_VALUE);
        s_last_rssi_dbm    = (int16_t)(rssi_raw - 157);
        s_last_snr_db      = (int16_t)(snr_raw / 4);
        s_have_rx_quality  = true;
    }
    const uint8_t n = rd(REG_RX_NB_BYTES);
    wr(REG_FIFO_ADDR_PTR, rd(REG_FIFO_RX_CURRENT_ADDR));
    uint8_t buf[LORA_PKT_BYTES];
    const int take = (n < LORA_PKT_BYTES) ? n : LORA_PKT_BYTES;
    for (int i = 0; i < take; i++) {
        buf[i] = rd(REG_FIFO);
    }
    /* The LENGTH THAT ARRIVED, not the length that was read. A 40-byte frame
     * truncated into an 18-byte buffer must be rejected as the wrong length,
     * not accepted because the first eighteen bytes happened to parse. */
    admit_frame(buf, (int)n, now_ms, false);
}

/* ONE PLACE WHERE A FRAME BECOMES AN ALERT, whether it arrived over the air
 * or was injected by the loopback drill. Splitting it out is what lets the
 * drill exercise the real path instead of a copy of it - a drill that proved
 * its own copy worked would prove nothing at all. */
static void admit_frame(const uint8_t *buf, int n, uint32_t now_ms,
                        bool injected)
{
    lora_pkt_t p;
    const int why = lora_rx_admit(&s_dedup, buf, n, s_id, now_ms, &p);
    if (injected) {
        s_n_rx_loop++;
    }
    switch (why) {
    case LORA_RX_SELF:
        s_n_rx_self++;
        return;
    case LORA_RX_DUP:
        s_n_rx_dup++;                   /* the burst's 2nd or 3rd copy */
        return;
    case LORA_RX_OK:
        break;
    default:
        s_n_rx_bad++;
        return;
    }
    /* AN INJECTED FRAME IS COUNTED APART FROM THE AIR. If a bench drill
     * advanced rx_ok, the next morning's two-board air test would be judged
     * against a counter that had already moved - and a lab run that looks
     * like a field one is this project's own standing complaint. */
    if (!injected) {
        s_n_rx_ok++;
    }
    /* ---- THE LINK TEST, BEFORE THE ALERT PATH -------------------------
     * A link packet is NOT an alert: no alarm, no five-second window, no
     * ALERT screen, no remote latch. It returns here so none of that runs.
     *
     * A REQUEST is answered exactly once - the dedup ring above has already
     * dropped the burst's second and third copies, so one reply per exchange
     * comes for free rather than from a counter.
     *
     * A reply is never answered. That single line is the entire reason this
     * cannot become the relay the protocol forbids: two units exchange one
     * request and one reply and then fall silent, and no arrangement of
     * units, duplicates or reflections can produce a third message. */
    if (p.flags & LORA_FLAG_LINK) {
        s_link_peer = p.device_id;
        if (p.flags & LORA_FLAG_LINKREQ) {
            s_link_event = LORA_LINK_GOT_REQ;
            if (!injected) {
                queue_burst(ALERT_TIER_V1, LORA_FLAG_LINKACK, 0.0f);
            }
        } else {
            s_link_event = LORA_LINK_GOT_ACK;   /* the round trip closed */
        }
        {
            char lb[96];
            snprintf(lb, sizeof(lb), "LORA link %s from N-%04X%s\n",
                     (p.flags & LORA_FLAG_LINKREQ) ? "REQUEST" : "REPLY",
                     (unsigned)p.device_id,
                     injected ? "  (LOOPBACK)" : "");
            trace_text(lb);
        }
        return;
    }

    s_remote_id = p.device_id;
    s_remote_tier = p.tier;
    s_remote_score = p.score_x100;
    s_remote_thr1 = p.thr1_milli;
    s_remote_until_ms = now_ms + LORA_REMOTE_LATCH_MS;
    {
        char b[96];
        snprintf(b, sizeof(b), "LORA remote alert from N-%04X tier %u%s%s\n",
                 (unsigned)p.device_id, (unsigned)p.tier,
                 (p.flags & LORA_FLAG_TEST) ? "  (TEST)" : "",
                 injected ? "  (LOOPBACK - not off the air)" : "");
        trace_text(b);
    }
}

/* ---- THE LOOPBACK DRILL -------------------------------------------------
 *
 * Why it is a software injection and not a transmission. An SX1276 is half
 * duplex: do_tx() puts the part in MODE_TX and only returns it to receive
 * afterwards, so a board CANNOT HEAR ITSELF. `Ls` alone can therefore never
 * exercise the receive path on one board, and on a night with one board that
 * means the self-drop, the dedupe window, the remote latch and the whole
 * remote-alert path have no test at all.
 *
 * What it proves: parse, the self drop, the dedupe window, the ten-second
 * remote latch, tier_or = ALERT_TIER_REMOTE, the double-pulse cadence, the RM
 * row in the event ring, the REMOTE N-XXXX note on the panel, and that a
 * button press clears the latch without touching the dedupe ring.
 *
 * What it proves nothing about, and the report must say so: the radio.
 * Register writes, PA_BOOST, the antenna, the channel, the sync word, range,
 * or that two boards agree about any of it. That is the two-board air test on
 * battery day, and it is owed.
 * ---------------------------------------------------------------------- */
bool lora_link_loopback(bool as_peer, uint32_t now_ms)
{
    if (!s_ready) {
        return false;
    }
    uint8_t pkt[LORA_PKT_BYTES];
    /* as_peer flips our own id into one that cannot be ours, so ONE command
     * drives both arms of the test: as_peer=false must be dropped as SELF,
     * as_peer=true must latch.
     *
     * The frame is built by the shipped builder, through the same helper
     * queue_burst uses. A drill that assembled its own eighteen bytes would
     * be testing a copy of the protocol, which is worth nothing. */
    lora_pkt_t p = {
        .device_id = as_peer ? (uint16_t)(s_id ^ 0xFFFFu) : s_id,
        .seq = s_seq,
        .hop = 0,
        .tier = ALERT_TIER_V1,
        .flags = LORA_FLAG_TEST,
    };
    build_frame(pkt, &p);
    admit_frame(pkt, LORA_PKT_BYTES, now_ms, true);
    return true;
}

/* ---- transmit ----------------------------------------------------------- */

static void do_tx(const uint8_t *pkt)
{
    set_mode(MODE_STDBY);
    wr(REG_FIFO_ADDR_PTR, 0);
    for (int i = 0; i < LORA_PKT_BYTES; i++) {
        wr(REG_FIFO, pkt[i]);
    }
    wr(REG_PAYLOAD_LENGTH, LORA_PKT_BYTES);
    wr(REG_IRQ_FLAGS, 0xFF);
    set_mode(MODE_TX);
    /* Poll for TxDone. AT SF9 EIGHTEEN BYTES IS 185 ms, not the 46 this
     * comment used to claim for SF7 - and the old bound of 40 iterations was
     * 200 ms, which would have expired mid-transmission on the very first
     * SF9 frame and returned the radio to receive while it was still sending.
     * 600 ms is three times the airtime. The loop yields, because this is a
     * low-priority task on the non-audio core.
     *
     * At SF10 a frame is 329.7 ms, so 600 ms would be 1.8x the airtime
     * rather than three times it, and a slow frame would be cut off mid
     * transmission. 1200 ms restores the margin. This bound and LORA_SF have
     * to move together, which is why the airtime is written down at both
     * ends. */
    for (int i = 0; i < 240; i++) {
        vTaskDelay(pdMS_TO_TICKS(5));
        if (rd(REG_IRQ_FLAGS) & IRQ_TX_DONE) {
            break;
        }
    }
    wr(REG_IRQ_FLAGS, 0xFF);
    s_n_tx++;
    radio_rx();                         /* straight back to listening */
}

/* ---- the task ----------------------------------------------------------- */

static void lora_task(void *arg)
{
    (void)arg;
    while (!s_task_stop) {
        uint8_t token;
        /* Wake on a DIO0 edge, or every 20 ms to service the burst. */
        if (xQueueReceive(s_rxq, &token, pdMS_TO_TICKS(20)) == pdTRUE) {
            handle_rx((uint32_t)(esp_timer_get_time() / 1000));
        }
        const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (s_burst.left > 0 && (int32_t)(now - s_burst.due_ms) >= 0) {
            do_tx(s_burst.pkt);
            s_burst.left--;
            s_burst.due_ms = now + lora_tx_gap(esp_random());
        }
    }
    s_task = NULL;
    vTaskDelete(NULL);
}

/* ---- public ------------------------------------------------------------- */

void lora_link_configure(bool enabled, uint32_t freq_hz)
{
    s_enabled = enabled;
    if (freq_hz) {
        s_freq_hz = freq_hz;
    }
}

bool lora_link_begin(void)
{
    if (s_ready) {
        return true;
    }
    if (!s_enabled) {
        return false;
    }
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY) != ESP_OK) {
        (void)esp_read_mac(mac, ESP_MAC_WIFI_STA);
    }
    s_id = lora_id_from_mac(mac);

    gpio_config_t g = {
        .pin_bit_mask = (1ULL << PIN_LORA_CS) | (1ULL << PIN_LORA_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&g);
    cs(1);                              /* CS idles HIGH */

    /* SPI2 is ALREADY INITIALISED BY THE E-PAPER and must not be initialised
     * twice. Adding a second device to the same host is the supported way to
     * share it, and the driver serialises the transactions. If the panel has
     * not come up yet, initialise the bus here with the same pins - whichever
     * gets there first wins and the second call is a no-op. */
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_EPD_DIN,
        .miso_io_num = PIN_LORA_MISO,
        .sclk_io_num = PIN_EPD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    const esp_err_t be = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (be != ESP_OK && be != ESP_ERR_INVALID_STATE) {
        trace_text("LORA ERR could not bring up SPI2\n");
        return false;
    }
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 4 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,             /* CS driven here, not by the bus */
        .queue_size = 2,
    };
    if (spi_bus_add_device(SPI2_HOST, &dev, &s_spi) != ESP_OK) {
        trace_text("LORA ERR could not add the radio to SPI2\n");
        return false;
    }

    /* Reset: RST low 1 ms, then 10 ms to come up. */
    gpio_set_level(PIN_LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(PIN_LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    s_version_read = rd(REG_VERSION);
    if (s_version_read != SX1276_VERSION) {
        /* SAY IT AND STOP. A radio that does not answer its own version
         * register is not going to transmit, and a device that carried on
         * pretending it had a radio would be a device whose peers silently
         * never hear it. */
        char b[96];
        snprintf(b, sizeof(b),
                 "LORA ERR reg 0x42 = 0x%02X, expected 0x%02X - no radio\n",
                 (unsigned)s_version_read, SX1276_VERSION);
        trace_text(b);
        spi_bus_remove_device(s_spi);
        s_spi = NULL;
        return false;
    }
    radio_configure();

    lora_dedup_reset(&s_dedup);
    s_burst.left = 0;
    s_rxq = xQueueCreate(4, sizeof(uint8_t));
    if (!s_rxq) {
        return false;
    }
    gpio_config_t d = {
        .pin_bit_mask = 1ULL << PIN_LORA_DIO0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&d);
    esp_err_t e = gpio_install_isr_service(0);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        return false;                   /* alert_ui may have claimed it */
    }
    gpio_isr_handler_add(PIN_LORA_DIO0, dio0_isr, NULL);

    radio_rx();
    s_task_stop = false;
    if (!s_task) {
        (void)xTaskCreatePinnedToCore(lora_task, "lora", LORA_TASK_STACK,
                                      NULL, LORA_TASK_PRIO, &s_task,
                                      LORA_TASK_CORE);
    }
    s_ready = true;
    return true;
}

void lora_link_end(void)
{
    if (!s_ready) {
        return;
    }
    s_task_stop = true;
    for (int i = 0; i < 20 && s_task; i++) {
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    gpio_isr_handler_remove(PIN_LORA_DIO0);
    set_mode(MODE_SLEEP);
    if (s_spi) {
        spi_bus_remove_device(s_spi);
        s_spi = NULL;
    }
    if (s_rxq) {
        vQueueDelete(s_rxq);
        s_rxq = NULL;
    }
    s_ready = false;
}

/* THE ONLY PLACE lora_pkt_build IS CALLED. Both the real burst and the
 * loopback drill route through it, so tests/test_lora.py's "exactly one build
 * site" assertion still holds and, more to the point, the drill cannot drift
 * into testing a second implementation of the wire format. */
static void build_frame(uint8_t *dst, const lora_pkt_t *p)
{
    lora_pkt_build(dst, p);
}

static void queue_burst(uint8_t tier, uint8_t flags, float score)
{
    /* The epoch is the boot counter's low byte.
     * `seq` restarts at 0 on every boot, so a sender that resets on every
     * alert transmits the same seq every time and its second alert inside a
     * minute is dropped by the peer as a repeat. The boot counter moves when
     * seq does not, which is exactly the discriminator that was missing. */
    const uint16_t sc = (score > 0.0f && score < 655.0f)
                            ? (uint16_t)(score * 100.0f + 0.5f) : 0u;
    lora_pkt_t p = {
        .device_id = s_id,
        .seq = ++s_seq,
        .hop = 0,
        .tier = tier,
        .flags = flags,
        .epoch = (uint8_t)(bootrec_boots() & 0xFFu),
        .score_x100 = sc,
        /* The one constant two paired units can differ by. A remote alert has
         * to be self-describing about it, or the paired measurement cannot be
         * read off the other board. */
        .thr1_milli = (uint16_t)settings_get()->thr1_milli,
    };
    build_frame(s_burst.pkt, &p);
    /* A link test is a person waiting to hear a beep, not a time-critical
     * alert, so it gets twice the transmissions. See LORA_LINK_TX_REPEATS. */
    s_burst.left = (flags & LORA_FLAG_LINK) ? LORA_LINK_TX_REPEATS
                                            : LORA_TX_REPEATS;
    s_burst.due_ms = (uint32_t)(esp_timer_get_time() / 1000);
}

bool lora_link_last_rx_quality(int *rssi_dbm, int *snr_db)
{
    if (!s_have_rx_quality) {
        return false;
    }
    if (rssi_dbm) { *rssi_dbm = (int)s_last_rssi_dbm; }
    if (snr_db)   { *snr_db   = (int)s_last_snr_db; }
    return true;
}

uint32_t lora_link_tx_count(void) { return s_n_tx; }

uint16_t lora_link_remote_score_x100(void) { return s_remote_score; }
uint16_t lora_link_remote_thr1_milli(void) { return s_remote_thr1; }

bool lora_link_send_linktest(void)
{
    if (!s_ready || !s_enabled) {
        return false;
    }
    queue_burst(ALERT_TIER_V1, LORA_FLAG_LINKREQ, 0.0f);
    s_link_event = LORA_LINK_SENT;
    s_link_peer = 0u;
    return true;
}

uint8_t lora_link_take_link_event(uint16_t *peer)
{
    const uint8_t e = s_link_event;
    if (peer) { *peer = s_link_peer; }
    s_link_event = LORA_LINK_NONE;
    return e;
}

void lora_link_announce(uint8_t tier, float score)
{
    if (!s_ready) {
        return;
    }
    queue_burst(tier, 0, score);
}

bool lora_link_send_test(bool on)
{
    if (!s_ready) {
        return false;
    }
    /* A TEST packet is flagged so a receiver can tell a drill from a drone,
     * and it takes a sequence number of its own - so a drill can never make
     * a real alert look like a duplicate of it. */
    queue_burst(on ? ALERT_TIER_V1 : ALERT_TIER_NONE, LORA_FLAG_TEST,
                0.0f);
    return true;
}

bool lora_link_remote_active(uint32_t now_ms)
{
    if (!s_ready || s_remote_until_ms == 0u) {
        return false;
    }
    if ((int32_t)(now_ms - s_remote_until_ms) >= 0) {
        return false;
    }
    return true;
}

bool lora_link_ready(void)           { return s_ready; }
uint16_t lora_link_remote_id(void)   { return s_remote_id; }
uint8_t  lora_link_remote_tier(void) { return s_remote_tier; }
uint16_t lora_link_device_id(void)   { return s_id; }

void lora_link_snooze(void)
{
    /* LOCAL ONLY. It ends this device's remote latch so the outputs stop; it
     * does NOT transmit, does not tell the peer anything, and does not touch
     * the dedupe ring - so the very next alert from any peer is heard. */
    s_remote_until_ms = 0u;
}

bool lora_link_selftest(char *dst, int cap)
{
    if (!s_spi) {
        snprintf(dst, cap, "LORA not started (enabled=%d) - send S or reboot "
                           "with `U c lora 1`\n", (int)s_enabled);
        return false;
    }
    const uint8_t v = rd(REG_VERSION);
    snprintf(dst, cap,
             "LORA reg0x42=0x%02X (expect 0x%02X) %s  id=N-%04X "
             "freq=%u Hz\n"
             "     tx=%u rx_ok=%u rx_bad=%u rx_dup=%u rx_self=%u "
             "rx_loop=%u\n",
             (unsigned)v, SX1276_VERSION,
             v == SX1276_VERSION ? "OK" : "NO RADIO",
             (unsigned)s_id, (unsigned)s_freq_hz,
             (unsigned)s_n_tx, (unsigned)s_n_rx_ok, (unsigned)s_n_rx_bad,
             (unsigned)s_n_rx_dup, (unsigned)s_n_rx_self,
             (unsigned)s_n_rx_loop);
    return v == SX1276_VERSION;
}

void lora_link_describe(char *dst, int cap)
{
    snprintf(dst, cap,
             "lora: %s id=N-%04X freq=%u Hz sf=%d bw=125k cr=4/5 "
             "sync=0x%02X pa=BOOST %d dBm\n",
             s_ready ? "up" : (s_enabled ? "DOWN" : "disabled"),
             (unsigned)s_id, (unsigned)s_freq_hz, LORA_SF,
             LORA_SYNC_WORD, LORA_TX_DBM);
}

#else  /* !BOARD_HAS_LORA */

/* THE BREADBOARD HAS NO RADIO, and GPIO48 - where the PCB puts DIO0 - is the
 * DevKitC-1 v1.0's onboard RGB LED. That is exactly why this compiles out
 * rather than remapping onto a free pin: a remap is how a driver ends up
 * driving the wrong thing on the wrong board. */

bool lora_link_begin(void) { return false; }
bool lora_link_ready(void) { return false; }
void lora_link_end(void) {}
void lora_link_announce(uint8_t tier, float score)
{ (void)tier; (void)score; }
bool lora_link_last_rx_quality(int *rssi_dbm, int *snr_db)
{
    (void)rssi_dbm; (void)snr_db;
    return false;
}
uint32_t lora_link_tx_count(void) { return 0u; }
uint16_t lora_link_remote_score_x100(void) { return 0u; }
uint16_t lora_link_remote_thr1_milli(void) { return 0u; }
bool lora_link_send_linktest(void) { return false; }
uint8_t lora_link_take_link_event(uint16_t *peer)
{ if (peer) { *peer = 0u; } return LORA_LINK_NONE; }
bool lora_link_remote_active(uint32_t now_ms) { (void)now_ms; return false; }
uint16_t lora_link_remote_id(void) { return 0; }
uint8_t  lora_link_remote_tier(void) { return 0; }
uint16_t lora_link_device_id(void) { return 0; }
void lora_link_snooze(void) {}
void lora_link_configure(bool enabled, uint32_t freq_hz)
{
    (void)enabled;
    (void)freq_hz;
}

bool lora_link_selftest(char *dst, int cap)
{
    snprintf(dst, cap, "LORA this board has no radio (%s)\n", BOARD_NAME);
    return false;
}

bool lora_link_send_test(bool on) { (void)on; return false; }
bool lora_link_loopback(bool as_peer, uint32_t now_ms)
{
    (void)as_peer;
    (void)now_ms;
    return false;
}

void lora_link_describe(char *dst, int cap)
{
    snprintf(dst, cap, "lora: absent on %s\n", BOARD_NAME);
}

#endif /* BOARD_HAS_LORA */
