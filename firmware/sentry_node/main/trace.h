/*
 * trace.h - the host bridge: one binary record per frame, over USB.
 *
 * Binary, not text. PicoLibC's printf has no float support, and even with it
 * a decimal round-trip would throw away exactly the low-order bits that decide
 * a marginal frame. Every field is sent as its native machine representation.
 *
 * The stream is self-synchronising: every record carries a magic word and an
 * FNV-1a check field, so a stray log byte costs one record instead of the run.
 * Field ORDER follows device_config.json "trace_fields" exactly.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "detector.h"

#define TRACE_MAGIC_HDR 0xA5C31A0Fu
#define TRACE_MAGIC_REC 0x5EC0DE01u
#define TRACE_MAGIC_PRB 0x9B10BE55u
#define TRACE_MAGIC_END 0xE0F5EA1Du
#define TRACE_MAGIC_MET 0x3E7E12A0u   /* meter: level of the live mic       */
#define TRACE_MAGIC_ALT 0xA1E27F00u   /* alert: the detector fired          */
#define TRACE_MAGIC_PCM 0x9C40DA7Au   /* raw int16 samples, VARIABLE length */
#define TRACE_MAGIC_TON 0x70E4B142u   /* averaged spectrum peak (tone check) */
/* ---- the alert-output and quad-acquisition records -----------------------
 * Same style throughout: a 32-bit magic and a trailing FNV-1a. TRACE_VERSION
 * deliberately stays 1, because these are additive record types and the host
 * parser skips magics it does not know - so old tools keep reading new
 * captures and new tools keep reading old ones. */
#define TRACE_MAGIC_ACK 0x0ACC0DE5u   /* command acknowledgement            */
#define TRACE_MAGIC_STA 0x57A75AFEu   /* device status snapshot             */
#define TRACE_MAGIC_QMT 0x9AD4E7E5u   /* quad meter: 4 channels + 2 buses   */
#define TRACE_MAGIC_PC4 0x9C4D4A7Au   /* quad PCM, VARIABLE length          */
/* ---- the Tier-2 records --------------------------------------------------
 * Same style again, and TRACE_VERSION still 1. A version bump would say "old
 * tools cannot read this", which is false: the parser resynchronises on magic
 * words and skips the ones it does not know, so an older tool reads a Tier-2
 * capture and simply sees no T2 records. The host tests assert that against a
 * real pre-Tier-2 capture. */
#define TRACE_MAGIC_T2R 0x72E12A05u   /* Tier-2 per-frame record            */
#define TRACE_MAGIC_AL2 0xA1E27F02u   /* Tier-2 alert                       */
/* ---- the Tier-3 records --------------------------------------------------
 * Same style a third time, and TRACE_VERSION still 1. Tier-3 updates once
 * every four frames, so T3R is a sparse record: the absence of one on a frame
 * means no update was due, never that the tier is silent. */
#define TRACE_MAGIC_T3R 0x73E13A05u   /* Tier-3 per-update record           */
#define TRACE_MAGIC_AL3 0xA1E27F03u   /* Tier-3 alert                       */
#define TRACE_VERSION   1u

/* ACK status codes. */
#define TRACE_ACK_OK              0u
#define TRACE_ACK_NOT_IMPLEMENTED 1u
#define TRACE_ACK_FAULT           2u
#define TRACE_ACK_BAD_ARG         3u

/* STAT epaper_state values. */
#define TRACE_EPD_UNINIT  0u
#define TRACE_EPD_READY   1u
#define TRACE_EPD_BUSY    2u
#define TRACE_EPD_FAULTED 3u

/* STAT alert_state values (the alert_ui state machine). */
#define TRACE_ALERT_IDLE     0u
#define TRACE_ALERT_ALERTING 1u
#define TRACE_ALERT_SNOOZED  2u
#define TRACE_ALERT_COOLDOWN 3u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    char     name[32];
    uint32_t n_samples;
    uint32_t n_frames;
    double   threshold;
    uint32_t fs;
    uint32_t n_fft;
    uint32_t hop;
    uint32_t n_bins;
    uint32_t n_f0;
    uint32_t rec_size;
    uint32_t chk;
} trace_hdr_t;

/* device_config.json trace_fields, in order:
 * frame, t_s, score, f0_bin, f0_hz, f0_raw_hz, teeth, floor_fast, reanch,
 * n_held_bins, above_thr, cont_accepted, chain, fired */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t frame;
    double   t_s;
    float    score;
    uint16_t f0_bin;
    double   f0_hz;
    double   f0_raw_hz;
    uint16_t teeth;
    uint8_t  floor_fast;
    uint8_t  reanch;
    uint16_t n_held_bins;
    uint8_t  above_thr;
    uint8_t  cont_accepted;
    int32_t  chain;
    uint8_t  fired;
    uint8_t  is_octave;
    uint32_t us_frame;      /* esp_timer, compute only: no serial I/O inside */
    uint32_t chk;
} trace_rec_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t n_frames;
    uint8_t  verdict_fires;
    uint32_t n_events;
    float    peak_score;
    int32_t  longest_chain;
    uint32_t n_floor_fast;
    uint32_t n_reanch;
    uint32_t n_held_frames;
    uint64_t total_us;
    uint32_t max_us;
    uint32_t p99_us;
    uint64_t us_fft;        /* front_end, summed over the run */
    uint64_t us_mag;
    uint64_t us_floor;
    uint64_t us_score;
    uint8_t  chain_overflow;
    uint32_t chk;
} trace_end_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t frame;
    uint32_t win_checksum;
    uint16_t probe_bin[8];
    float    probe_mag[8];
    double   probe_floor[8];
    float    probe_S[8];
    double   e;
    double   e_slow;
    double   flat;
    uint8_t  rising;
    uint8_t  fast;
    uint16_t argmax_bin;
    float    argmax_score;
    uint32_t chk;
} trace_prb_t;

/* Meter: what the microphone is actually doing, every 500 ms. RMS, peak and
 * DC are in int16 units - the pipeline's own scale - rather than dBFS, because
 * a unit conversion on the device is a place for a bug to hide and the host
 * can divide. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint64_t t_us;
    uint32_t n_samples;
    double   rms;
    double   dc_offset;      /* mean sample value - measured, never removed */
    int32_t  peak_abs;
    int32_t  vmin;
    int32_t  vmax;
    uint32_t short_reads;    /* I2S returned less than asked */
    uint32_t timeouts;       /* I2S returned nothing - a wiring fault */
    uint32_t chk;
} trace_met_t;

/* Alert: emitted once on the frame the tracker latches. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t frame;
    double   t_s;
    double   f0_hz;
    float    score;
    int32_t  chain;
    uint32_t n_events;
    uint32_t chk;
} trace_alt_t;

/* PCM: raw int16 samples, exactly as handed to the pipeline. Variable length:
 * the payload follows the header and the check field follows the payload, so
 * the parser must read n before it can find the end. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint32_t first_index;    /* sample index of payload[0] in the stream */
    uint16_t n;              /* samples that follow */
} trace_pcm_hdr_t;

/* Tone check: the averaged magnitude spectrum's strongest bins. A plumbing
 * probe that answers "does a known frequency land in the right FFT bin" and
 * nothing else. It is not detection and says nothing about range. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t n_avg;          /* frames averaged */
    uint32_t peak_bin;
    double   peak_hz;
    double   bin_hz;         /* fs / n_fft */
    uint16_t top_bin[8];
    float    top_mag[8];
    double   dc_mag;
    uint32_t chk;
} trace_ton_t;

/* ==========================================================================
 * Records added by the alert-output + quad-acquisition build.
 * ========================================================================== */

/* ACK: every `U` / `D` command answers with exactly one of these, so a host
 * tool never has to infer success from silence. `cmd` echoes the command bytes
 * verbatim (NUL padded) so a reply cannot be mis-attributed to the wrong
 * command when several are pipelined. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    char     cmd[16];
    uint32_t status;         /* TRACE_ACK_* */
    uint32_t chk;
} trace_ack_t;

/* STAT: one snapshot of everything the operator can observe without a scope.
 * Emitted on request (`U s`), on every alert state transition, and once per
 * second while a drill or the armed mode is running. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t uptime_ms;
    uint8_t  mode;                  /* mode letter as ASCII; 0 = idle       */
    uint8_t  buzzer;                /* commanded state, 0/1                 */
    uint8_t  motor;
    /* The last value written, not an observation of light. The WS2812 has no
     * back channel, so nothing in firmware can know whether a photon was
     * emitted; the name says commanded so it cannot be read as evidence that
     * the LED lit. */
    uint8_t  led_cmd;               /* any channel nonzero, AS COMMANDED     */
    uint8_t  led_c0, led_c1, led_c2;/* raw wire order, never colour names   */
    uint8_t  alert_state;           /* TRACE_ALERT_*                        */
    uint32_t snooze_remaining_ms;
    uint8_t  epaper_state;          /* TRACE_EPD_*                          */
    uint8_t  button_level;          /* raw pin level: 1 = released (pull-up) */
    uint16_t press_count;           /* debounced presses since boot          */
    uint32_t chk;
} trace_stat_t;

/* QMT: the quad meter. Per-channel level and per-bus health, because a dead
 * bus must be loudly visible rather than looking like silence. Channel order
 * is always [M1 M2 M3 M4]; bus order is always [A (GPIO6), B (GPIO7)]. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint64_t t_us;
    uint32_t n_samples;       /* per channel, this block */
    double   rms[4];
    double   dc[4];           /* measured, never removed - same rule as mono */
    int32_t  peak[4];
    int32_t  vmin[4];
    int32_t  vmax[4];
    uint32_t timeouts[2];     /* per bus, cumulative */
    uint32_t short_reads[2];
    uint32_t frames_total[2];
    uint32_t chk;
} trace_qmt_t;

/* PCM4: interleaved 4-channel int16, variable length, same shape as PCM.
 * `seq` is mandatory here: 4 ch x 16 kHz x 2 B = 128 kB/s over USB-Serial-JTAG
 * is unproven, so the host must be able to detect loss exactly and use only
 * contiguous stretches rather than silently splicing. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;             /* consecutive from 0; gaps == dropped records */
    uint32_t base_frame;      /* frame index of payload[0] since capture start */
    uint16_t n_frames;
    uint16_t n_ch;            /* 4 */
} trace_pc4_hdr_t;

/* ==========================================================================
 * Tier-2 records.
 * ========================================================================== */

/* T2R: one per Tier-2 step, which at half rate is one per two frames.
 * Everything its decision reads is on the wire - the score, the winner, the
 * ring occupancy, the track age, and the coherence telemetry it deliberately
 * does not gate on - so the tier is as auditable as v1. `us_t2` is Tier-2's
 * own cost, separate from the frame timer, so the tier can be priced apart
 * from the pipeline. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t frame;
    double   t_s;
    float    score2;
    double   f02_hz;
    uint16_t f02_row;        /* row within the T2 slice, 0 .. T2_N_F0-1 */
    uint16_t teeth2;
    uint8_t  hit;
    uint8_t  fired2;
    uint8_t  excluded;       /* f02 fell in a persistent-source band     */
    uint8_t  pad;
    int32_t  hits;           /* ones in the ring, i.e. the M of M-of-N   */
    int32_t  n2;             /* ring length in STEPS, so N is on the wire */
    int32_t  track_age;
    float    kappa;          /* 1.0 for mono - coherence is undefined    */
    uint32_t us_t2;
    uint32_t chk;
} trace_t2r_t;

/* AL2: emitted once on the frame Tier-2 latches. A separate magic from ALT so
 * a reader can tell which tier alerted without inferring it from timing - the
 * two fire on different evidence, and which one earns its keep is the question
 * the record exists to answer. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t frame;
    double   t_s;
    double   f02_hz;
    float    score2;
    int32_t  hits;
    float    kappa;
    uint32_t n_events;
    uint32_t chk;
} trace_al2_t;

/* T3R: one per Tier-3 update, which is one frame in four. `W` is the wash
 * statistic and `r` the rate that won inside the firing band; `W_any` and
 * `r_any` are the winner over the whole grid and are telemetry only. The pair
 * exists because a rate that wins outside the firing band is the most useful
 * thing to know when the tier is silent on something audible.
 *
 * `us_t3` is the cost since the previous update, not since the run started:
 * four frames of biquads and decimation plus one envelope transform and one
 * 611-rate score. Read it as the tier's amortised cost per update, and divide
 * by four for its share of a single 32 ms hop.
 *
 * `frozen` is not a fault. Tier-3 hunts amplitude modulation between 100 and
 * 320 Hz, and the vibration motor is an ERM at 100-200 Hz bolted to the same
 * board; the tier freezes itself while the device's own outputs are running so
 * it cannot detect its own alarm. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t frame;
    double   t_s;
    double   r_hz;
    double   W;
    double   r_any_hz;
    double   W_any;
    int32_t  hits;
    int32_t  n3;
    int32_t  track_age;
    uint8_t  hit;
    uint8_t  fired3;
    uint8_t  frozen;
    uint8_t  pad;
    uint32_t us_t3;
    uint32_t chk;
} trace_t3r_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t frame;
    double   t_s;
    double   r_hz;
    double   W;
    int32_t  hits;
    uint32_t n_events;
    uint32_t chk;
} trace_al3_t;

/* ---- write policy, and why a field build needs a second one --------------
 * The default is blocking with a 2 s timeout: a lab capture must not lose
 * records, and a host that stops reading for a moment must not corrupt a
 * gate. That policy is wrong for a device on a phone charger, where nothing
 * ever drains the USB ring: it fills, and every subsequent write parks the
 * frame loop for two seconds, which does not merely lose the trace but stops
 * the detector. Drop mode writes with a zero timeout and discards what will
 * not fit, so telemetry degrades and detection does not. The standalone loop
 * turns it on; every lab mode leaves it off.
 *
 * Records are self-framing (magic + FNV check field), so a host that plugs in
 * mid-run resynchronises on the next record. A dropped record costs one
 * record. */
void trace_set_drop_mode(bool drop);
bool trace_drop_mode(void);
uint32_t trace_dropped_bytes(void);

void trace_link_init(void);
void trace_write(const void *buf, size_t n);
void trace_text(const char *s);
int  trace_read_line(char *dst, int cap, uint32_t ms);
int  trace_poll_byte(void);   /* non-blocking: byte, or -1 if none */

/* Non-blocking and non-destructive at the line level: accumulates bytes into
 * an internal buffer and returns > 0 only when a whole line has arrived, which
 * it copies into dst. Returns 0 while a line is still incomplete.
 *
 * trace_poll_byte() eats one byte to decide that the host wants something, and
 * that byte is the command letter - so a standalone loop exited that way would
 * hand the dispatcher "0" out of "R 0" and answer with an unknown-command
 * error. Autostart is only safe with a reader that loses nothing. */
int  trace_try_line(char *dst, int cap);

uint32_t trace_chk(const void *buf, size_t n);
void trace_send_hdr(trace_hdr_t *h);
void trace_send_rec(trace_rec_t *r);
void trace_send_prb(trace_prb_t *p);
void trace_send_end(trace_end_t *e);
void trace_send_met(trace_met_t *m);
void trace_send_alt(trace_alt_t *a);
void trace_send_ton(trace_ton_t *t);
void trace_send_pcm(uint32_t seq, uint32_t first_index,
                    const int16_t *q, uint16_t n);

void trace_send_ack(trace_ack_t *a);
void trace_send_stat(trace_stat_t *s);
void trace_send_qmt(trace_qmt_t *q);
void trace_send_pcm4(uint32_t seq, uint32_t base_frame,
                     const int16_t *q, uint16_t n_frames, uint16_t n_ch);
void trace_send_t2r(trace_t2r_t *r);
void trace_send_al2(trace_al2_t *a);
void trace_send_t3r(trace_t3r_t *r);
void trace_send_al3(trace_al3_t *a);
/* Convenience: build and send an ACK from a command string + status. */
void trace_ack(const char *cmd, uint32_t status);
