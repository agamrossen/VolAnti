/*
 * source_i2s_quad.h - four INMP441s as four aligned int16 channels.
 *
 * Acquisition and evidence only. There is deliberately no beamforming, no
 * bearing, no weighting and no combiner interaction anywhere in this module.
 * It produces raw indexed samples so the host tools can measure what the array
 * actually does; every interpretation lives host-side.
 *
 * Topology:
 *   I2S0  MASTER, std Philips, 32-bit, STEREO, din = GPIO6, drives BCLK GPIO4
 *         and WS GPIO5.               -> M1 (left slot), M2 (right slot)
 *   I2S1  SLAVE,  std Philips, 32-bit, STEREO, din = GPIO7, consuming the SAME
 *         physical GPIO4/GPIO5 as clock inputs via the GPIO matrix.
 *                                     -> M3 (left slot), M4 (right slot)
 *
 * One clock domain for all four microphones. The only expected discrepancy is
 * a possible CONSTANT integer-frame start offset between the two RX engines,
 * which is measured at the bench and compensated host-side (decision D6) - not
 * designed around, and never hidden by bookkeeping here.
 *
 * The mono path in source_i2s.c is untouched and remains the parity-proven
 * single-microphone route. Mono and quad are never active at once because they
 * share I2S0.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define QUAD_N_CH        4
#define QUAD_N_BUS       2
/* 256 frames/descriptor: a STEREO 32-bit frame is 8 bytes, so 256 frames is
 * 2048 B, comfortably under the 4092 B per-descriptor ceiling that the mono
 * sizing (512 frames) would now exceed.
 *
 * Six descriptors, not twelve. 6 x 256 = 1536 frames = 96 ms of slack per
 * controller against a 32 ms hop, which is still three hops of cushion. The
 * slack is what the driver holds, not what the guard needs, and twelve was
 * more DMA than the heap could spare once three tiers were running.
 *
 * What it buys: 2 buses x 6 x 2048 B = 24576 B, down from 49152. That 24 KB is
 * the whole reason all three tiers fit at once - the previous build was short
 * by about 10 KB with the pair, and no allocation order fixed it because the
 * shortfall was in the total, not the arrangement.
 *
 * What it costs: if the guard ever stalls for longer than 96 ms, a bus
 * overruns and source_i2s_quad_read reports a short read or a timeout, which
 * the frame loop already treats loudly. The worst measured frame is ~31 ms. */
#define QUAD_FRAMES_PER_READ 256
#define QUAD_DESC_NUM        6

typedef struct {
    uint32_t timeouts[QUAD_N_BUS];
    uint32_t short_reads[QUAD_N_BUS];
    uint32_t frames_total[QUAD_N_BUS];
    /* ---- actual audio loss, from the driver ------------------------------
     *
     * `DROP` counts frames whose compute exceeded the 32 ms hop, and a busy
     * session can read several percent that way. But a frame over the hop is
     * a warning, not a loss: the DMA holds
     * QUAD_DESC_NUM x QUAD_FRAMES_PER_READ = 96 ms per controller, three
     * whole hops, so a 33.5 ms frame eats 1.5 ms of slack and the next 25 ms
     * frame hands it back. Audio is lost only if the SUSTAINED MEAN exceeds
     * the hop, or if the driver's receive queue actually overflows.
     *
     * This is the second one, straight from the driver: i2s_channel_register_
     * event_callback()'s on_recv_q_ovf. Nonzero here is real, counted audio
     * loss. Zero here with DROP high means the budget is TIGHT and the slack
     * is absorbing it, which is a different problem with a different fix. */
    volatile uint32_t ovf[QUAD_N_BUS];
    /* False means the counters above were never armed, so their zero says
     * nothing. A counter that cannot move must never read as good news. */
    bool     ovf_armed;
} quad_stats_t;

/* Brings up both controllers. Enables the SLAVE first, then the MASTER, so the
 * slave is armed before the first clock edge exists. Idempotent. */
esp_err_t source_i2s_quad_start(void);

/* Tears both down and releases I2S0 so a mono mode can run afterwards. */
void source_i2s_quad_stop(void);

/* Reads QUAD_FRAMES_PER_READ frames from each bus and interleaves them as
 * [M1 M2 M3 M4] per frame into `dst` (4 * QUAD_FRAMES_PER_READ int16).
 * Returns the number of FRAMES written (0 on a bus timeout - the caller must
 * treat that loudly, never as a clean end of stream). */
int source_i2s_quad_read(int16_t *dst);

/* METER ONLY. With lenient=true a bus that timed out is zero-filled and the
 * live bus is still reported, so "one bus dead" is distinguishable from "both
 * dead". Never use this on the parity or guard paths - fabricated samples
 * would break the sample-exact premise. */
int source_i2s_quad_read_ex(int16_t *dst, bool lenient);

const quad_stats_t *source_i2s_quad_stats(void);
bool source_i2s_quad_running(void);
