#include "trace.h"

#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool     s_drop;
static uint32_t s_dropped_bytes;

void trace_set_drop_mode(bool drop)   { s_drop = drop; }
bool trace_drop_mode(void)            { return s_drop; }
uint32_t trace_dropped_bytes(void)    { return s_dropped_bytes; }

void trace_link_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 8192,
        .rx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
}

void trace_write(const void *buf, size_t n)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (n) {
        /* Drop mode: ZERO ticks. On a charger there is no host draining the
         * ring, so the 2 s blocking path would park the frame loop on the
         * first full buffer and never leave. Losing telemetry is a cost;
         * losing the detector is a defect. */
        int w = usb_serial_jtag_write_bytes(p, n, s_drop ? 0 : pdMS_TO_TICKS(2000));
        if (w <= 0) {
            if (s_drop) {
                s_dropped_bytes += (uint32_t)n;
            }
            return;             /* host vanished; the sentinel will be missing */
        }
        p += w;
        n -= (size_t)w;
        if (s_drop && n) {
            /* Partial write with nobody reading: abandon the rest of THIS
             * record rather than spin. The host resynchronises on the next
             * magic word, which is what the check field is for. */
            s_dropped_bytes += (uint32_t)n;
            return;
        }
    }
}

void trace_text(const char *s)
{
    trace_write(s, strlen(s));
}

int trace_read_line(char *dst, int cap, uint32_t ms)
{
    int n = 0;
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ms);
    while (n < cap - 1) {
        uint8_t c;
        int r = usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(50));
        if (r == 1) {
            if (c == '\n' || c == '\r') {
                if (n > 0) {
                    dst[n] = 0;
                    return n;
                }
                continue;
            }
            dst[n++] = (char)c;
        } else if (ms != portMAX_DELAY && xTaskGetTickCount() > deadline) {
            return -1;
        }
    }
    dst[n] = 0;
    return n;
}

/* Accumulates until a newline. The buffer survives between calls, so a
 * command that arrives one byte at a time - which is what a 115200-baud human
 * typing looks like to a 240 MHz loop - is assembled rather than lost. */
int trace_try_line(char *dst, int cap)
{
    static char  buf[64];
    static int   n;

    for (;;) {
        uint8_t ch;
        if (usb_serial_jtag_read_bytes(&ch, 1, 0) != 1) {
            return 0;                       /* nothing more right now */
        }
        if (ch == '\n' || ch == '\r') {
            if (n == 0) {
                continue;                   /* blank line: keep waiting */
            }
            int len = n < cap - 1 ? n : cap - 1;
            memcpy(dst, buf, (size_t)len);
            dst[len] = 0;
            n = 0;
            return len;
        }
        if (n < (int)sizeof(buf) - 1) {
            buf[n++] = (char)ch;
        }
        /* An over-long line is truncated at the buffer, not allowed to run
         * away; the terminating newline still ends it. */
    }
}

int trace_poll_byte(void)
{
    uint8_t c;
    return (usb_serial_jtag_read_bytes(&c, 1, 0) == 1) ? (int)c : -1;
}

/* FNV-1a. Not a CRC: this only has to catch a dropped or duplicated byte in a
 * cable, and it is cheap enough to leave outside the timed region. */
uint32_t trace_chk(const void *buf, size_t n)
{
    uint32_t h = 2166136261u;
    const uint8_t *p = (const uint8_t *)buf;
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

#define SEND(T, V, FIELD)                                            \
    do {                                                             \
        (V)->FIELD = trace_chk((V), sizeof(T) - sizeof(uint32_t));   \
        trace_write((V), sizeof(T));                                 \
    } while (0)

void trace_send_hdr(trace_hdr_t *h) { SEND(trace_hdr_t, h, chk); }
void trace_send_met(trace_met_t *m) { SEND(trace_met_t, m, chk); }
void trace_send_alt(trace_alt_t *a) { SEND(trace_alt_t, a, chk); }
void trace_send_ton(trace_ton_t *t) { SEND(trace_ton_t, t, chk); }

/* Variable-length record: header, payload, then one check field over both, so
 * a truncated payload is detected rather than silently parsed as samples. */
void trace_send_pcm(uint32_t seq, uint32_t first_index,
                    const int16_t *q, uint16_t n)
{
    trace_pcm_hdr_t h = {.magic = TRACE_MAGIC_PCM, .seq = seq,
                         .first_index = first_index, .n = n};
    uint32_t chk = trace_chk(&h, sizeof(h));
    /* continue the FNV over the payload */
    const uint8_t *p = (const uint8_t *)q;
    for (size_t i = 0; i < (size_t)n * sizeof(int16_t); i++) {
        chk ^= p[i];
        chk *= 16777619u;
    }
    trace_write(&h, sizeof(h));
    trace_write(q, (size_t)n * sizeof(int16_t));
    trace_write(&chk, sizeof(chk));
}
void trace_send_rec(trace_rec_t *r) { SEND(trace_rec_t, r, chk); }
void trace_send_prb(trace_prb_t *p) { SEND(trace_prb_t, p, chk); }
void trace_send_end(trace_end_t *e) { SEND(trace_end_t, e, chk); }

/* ---- alert-output + quad-acquisition records ----------------------------
 * These sizes are the wire format. scripts/trace_proto.py declares the same
 * layouts independently, and a silent divergence between the two would be
 * parsed as garbage rather than reported - so pin them here. If one of these
 * fires, fix the struct AND trace_proto.py together, never just one. */
_Static_assert(sizeof(trace_ack_t) == 28,  "ACK  wire size != trace_proto.ACK");
_Static_assert(sizeof(trace_stat_t) == 28, "STAT wire size != trace_proto.STA");
_Static_assert(sizeof(trace_qmt_t) == 160, "QMT  wire size != trace_proto.QMT");
_Static_assert(sizeof(trace_pc4_hdr_t) == 16,
               "PCM4 header wire size != trace_proto.PC4_HDR");
_Static_assert(sizeof(trace_t2r_t) == 60, "T2R  wire size != trace_proto.T2R");
_Static_assert(sizeof(trace_al2_t) == 44, "AL2  wire size != trace_proto.AL2");
_Static_assert(sizeof(trace_t3r_t) == 72, "T3R  wire size != trace_proto.T3R");
_Static_assert(sizeof(trace_al3_t) == 44, "AL3  wire size != trace_proto.AL3");

void trace_send_ack(trace_ack_t *a)  { SEND(trace_ack_t, a, chk); }
void trace_send_stat(trace_stat_t *s) { SEND(trace_stat_t, s, chk); }
void trace_send_qmt(trace_qmt_t *q)  { SEND(trace_qmt_t, q, chk); }
/* Tier-2. Additive: same macro, same framing, same version. */
void trace_send_t2r(trace_t2r_t *r)  { SEND(trace_t2r_t, r, chk); }
void trace_send_al2(trace_al2_t *a)  { SEND(trace_al2_t, a, chk); }
/* Tier-3. Additive again; the version does not move. */
void trace_send_t3r(trace_t3r_t *r)  { SEND(trace_t3r_t, r, chk); }
void trace_send_al3(trace_al3_t *a)  { SEND(trace_al3_t, a, chk); }

void trace_ack(const char *cmd, uint32_t status)
{
    trace_ack_t a = {.magic = TRACE_MAGIC_ACK, .status = status};
    /* strncpy, but without pulling in the warning about no NUL: the field is
     * fixed width and the host treats it as NUL-padded bytes, not a string. */
    for (int i = 0; i < (int)sizeof(a.cmd); i++) {
        a.cmd[i] = (cmd && cmd[i]) ? cmd[i] : '\0';
        if (cmd && !cmd[i]) {
            cmd = NULL;                 /* pad the remainder */
        }
    }
    trace_send_ack(&a);
}

/* Variable length, same discipline as trace_send_pcm: one check field over
 * header AND payload, so a truncated payload is detected rather than parsed
 * as samples. */
void trace_send_pcm4(uint32_t seq, uint32_t base_frame,
                     const int16_t *q, uint16_t n_frames, uint16_t n_ch)
{
    trace_pc4_hdr_t h = {.magic = TRACE_MAGIC_PC4, .seq = seq,
                         .base_frame = base_frame, .n_frames = n_frames,
                         .n_ch = n_ch};
    const size_t n_samp = (size_t)n_frames * (size_t)n_ch;
    uint32_t chk = trace_chk(&h, sizeof(h));
    const uint8_t *p = (const uint8_t *)q;
    for (size_t i = 0; i < n_samp * sizeof(int16_t); i++) {
        chk ^= p[i];
        chk *= 16777619u;
    }
    trace_write(&h, sizeof(h));
    trace_write(q, n_samp * sizeof(int16_t));
    trace_write(&chk, sizeof(chk));
}
