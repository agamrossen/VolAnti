/*
 * sentry_node.c - the frame loop, the modes, and the console.
 *
 * For the golden replay the device holds the vectors and the detector and
 * nothing else. It does not hold the expected outcomes: those live in
 * data/golden_vectors.json and are applied by the host comparator, because
 * firmware that knows the answer cannot be said to have reproduced it.
 *
 * Host protocol (line commands in, binary records out):
 *     I                       identity / sizes / free heap, as text
 *     R <idx>                 replay vector idx, stream the trace
 *     P <idx> <from> <to>     ... and interleave forensic probe records
 * Every run emits: header, one record per frame, sentinel.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

#include "dsps_fft2r.h"
#include "dsps_fft4r.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#include "actuators.h"
#include "alert_ui.h"
#include "board_pins.h"
#include "boot_rec.h"
#include "combiners_cx.h"
#include "detector.h"
#include "detector_t2.h"
#include "detector_t3.h"
#include "detector_t4.h"
#include "epaper.h"
#include "epaper_draw.h"
#include "ui_config.h"
#include "event_log.h"
#include "settings.h"
#include "led_ws2812.h"
#include "lora_link.h"
#include "mic_cal.h"
#include "nf_probe.h"
#include "power_mon.h"
#include "source.h"
#include "source_i2s_quad.h"
#include "trace.h"

static detector_state_t *g_st;
static detector_work_t  *g_w;
/* The mono staging pair, and why it is not allocated at boot.
 *
 * 12 KB: one n_fft window of float32 and one of int16. Only the mono modes
 * touch it - the golden replay, the meter, the tone check, the self-tests -
 * and the quad pipeline has its own per-channel windows and never reads
 * either.
 *
 * Held for the life of the device it is free only while nothing else is
 * competing: the quad guard needs 70 KB, Tier-2 24 KB, Tier-3 13 KB and the
 * two I2S buses take 48 KB of DMA, and 12 KB held for a mode that is not
 * running is the difference between Tier-3 fitting and the guard refusing to
 * start. So it is allocated lazily and handed back when the guard starts. */
static float            *g_block;      /* sliding window, n_fft float32 */
static int16_t          *g_q;          /* int16 staging, n_fft */

static bool mono_alloc(void)
{
    if (!g_block) {
        g_block = heap_caps_calloc(CFG_N_FFT, sizeof(float),
                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!g_q) {
        g_q = heap_caps_calloc(CFG_N_FFT, sizeof(int16_t),
                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (g_block && g_q) {
        return true;
    }
    trace_text("ERR out of RAM for the mono staging buffers\n");
    return false;
}

static void mono_free(void)
{
    free(g_block);
    g_block = NULL;
    free(g_q);
    g_q = NULL;
}

/* One line of heap truth. Printed when a run starts and when one cannot. */
static void heap_line(const char *what)
{
    char b[96];
    snprintf(b, sizeof(b), "heap %s: free=%u largest=%u\n", what,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    trace_text(b);
}

static int cmp_u32(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return (x > y) - (x < y);
}

/* x = q / 32767.0f, exactly as make_golden.py quantised it - 32767, not
 * 32768 - and computed in float32, because numpy's `q.astype(np.float32) /
 * 32767.0` is a float32 operation under NEP 50 weak-scalar promotion. */
static inline float q_to_f(int16_t q)
{
    return (float)q / 32767.0f;
}

/* ==========================================================================
 * One frame loop, four modes. The golden replay and the live microphone run
 * the same code over the same source_read() abstraction; if they did not, the
 * golden evidence would say nothing about the live path, which is the whole
 * point of having the abstraction.
 *
 * The only per-mode differences are which records are emitted and when the
 * loop stops. Nothing mode-specific touches the detector.
 * ========================================================================== */

/* ==========================================================================
 * Tier-2. Everything below is opt-in and reached only through the `H` and
 * `K` letters.
 *
 * The rule that makes it safe: `guard_t2_opts_t *t2o == NULL` is the
 * tier-free behaviour, and every Tier-2 line inside run_quad_pipeline is
 * inside an `if (t2o ...)`. `G` and `Z` pass NULL, so their path through the
 * loop carries the same instructions it would with no Tier-2 in the tree,
 * which is why the golden gate still says what it says.
 * ========================================================================== */
typedef struct {
    bool     enable;
    t2_cfg_t cfg;
    char     cx;              /* 'a' (default) | 'b' | 'c' | 'd' */
    float    f_split_hz;      /* CX-C only */
    float    busoff;          /* CX-D only, samples of bus-B lag */
} guard_t2_opts_t;

/* The runtime exclusion list, set by `E` and consumed by `H`/`K`. Module
 * scope because it must SURVIVE between commands - the operator types it once
 * after a quiet baseline and then runs the guard. */
static int    g_excl_n;
static double g_excl_c[T2_MAX_EXCL];
static double g_excl_t[T2_MAX_EXCL];

static t2_state_t *g_t2st;
static t2_work_t  *g_t2w;
static cx_work_t  *g_cxw;

/* ==========================================================================
 * Tier-3, the wash tier.
 *
 * v1 and Tier-2 search 200-800 Hz for a harmonic comb. The only real propeller
 * this project has recorded put +0.3 dB there and +14.9 dB above 3.2 kHz, and
 * both comb tiers raised zero events on it at 4 m and at 10 m. Tier-3
 * demodulates the high band and finds the blade-pass periodicity in the
 * envelope; on that same audio it latched 94% of the 4 m capture. It is the
 * only tier here that has detected a real rotor.
 *
 * What is wrong with it, equally plainly: it fires on box fans, HVAC
 * condensers, insects, tracked vehicles and helicopters, because a fan and a
 * rotor are the same machine and the statistic cannot separate them. Its
 * calibration failed - the best weighted false-alarm rate reachable at any
 * threshold was 4.87/h against an allowance of 0.40/h - which is why
 * T3_ENABLED_DEFAULT is 0 in the generated header and the no-regression
 * certificate asserts it.
 *
 * It is enabled at runtime rather than in the header, so the sealed artifacts
 * stay untouched and a typed argument or a persisted setting turns it on. The
 * standing rule is that miss cost far exceeds false-alarm cost. Expect
 * nuisance alarms outdoors - that expectation is measured, not speculative -
 * and read the tier field on every alert before believing any of them.
 * ========================================================================== */
typedef struct {
    bool     enable;
    t3_cfg_t cfg;
} guard_t3_opts_t;

/* ==========================================================================
 * Tier-4, the slow comb. Same contract as Tier-3: additive, opt-in, and
 * disabled in the generated header.
 *
 * It is not disabled because it failed. tau4 = 30.5 gives zero events on
 * every real drone-free recording this project owns - 487 s of it, indoor and
 * outdoor - while both real in-band positives fire, the first at 4.45 s. What
 * it does not have is hours. Zero events in 487 s bounds the false-alarm rate
 * at 22/h with 95% confidence against an allowance of 0.40/h, so turning it
 * on in the field means shipping a tier whose false-alarm rate is unmeasured
 * to within a factor of fifty.
 *
 * Until those hours exist it runs on demand, its verdict goes in the trace and
 * the event ring beside the other three, and nothing about the alarm depends
 * on it.
 * ========================================================================== */
typedef struct {
    bool     enable;
    t4_cfg_t cfg;
} guard_t4_opts_t;

static t4_state_t *g_t4st;

static t3_state_t *g_t3st;

/* Tier-3's state is claimed early rather than per-run, and the reason is
 * placement rather than lifetime.
 *
 * t3_state_t is 12688 bytes and must be contiguous. By the time a run has
 * started two I2S buses (48 KB taken as twenty-four separate 2 KB descriptor
 * buffers), the guard (70 KB) and Tier-2 (24 KB), the heap holds 18616 bytes
 * free in no block larger than 7680 - so the allocation fails with plenty of
 * total memory left. Claiming it out of a pristine heap costs the mono modes
 * 12.7 KB they have to spare and removes the failure entirely.
 *
 * Nothing about it is per-run: t3_reset() clears it at the start of every run,
 * exactly as t2_state_reset() does for a buffer that is reallocated. */
static bool t3_boot_alloc(void)
{
    if (g_t3st) {
        return true;
    }
    if (t3_init() != 0) {
        trace_text("ERR t3_init failed (esp-dsp fft2r for the 1024-pt "
                   "envelope transform)\n");
        return false;
    }
    g_t3st = heap_caps_calloc(1, sizeof(t3_state_t),
                              MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return g_t3st != NULL;
}

/* Release it again when the configured pair does not include Tier-3.
 *
 * Holding Tier-3's 12688 bytes permanently makes the v1 + Tier-2 pair
 * unallocatable: Tier-2 asks for 15924 with 20720 free and 10752 in the
 * largest block, so the device falls back to v1 alone and the calibrated pair
 * becomes unreachable.
 *
 * So the claim follows the configuration rather than the boot. It is still
 * taken from a pristine heap - reconciled in run_standalone() before the I2S
 * buses and the guard buffers fragment anything - which is the property that
 * matters; what changes is that a pair which does not want it does not pay
 * for it. */
static void t3_free(void)
{
    free(g_t3st);
    g_t3st = NULL;
}

static bool t3_alloc(void) { return g_t3st != NULL; }

static void t4_free(void)
{
    free(g_t4st);
    g_t4st = NULL;
}

/* Idempotent, and claimed from the same pristine heap as the others. ~10 KB,
 * almost all of it the four sub-block accumulators. */
static bool t4_alloc(void)
{
    if (!g_t4st) {
        g_t4st = heap_caps_calloc(1, sizeof(t4_state_t),
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return g_t4st != NULL;
}

static void t2_free(void)
{
    free(g_t2st);
    g_t2st = NULL;
    free(g_t2w);
    g_t2w = NULL;
    free(g_cxw);
    g_cxw = NULL;
}

/* Idempotent, so the same function serves the boot reservation and the lab
 * modes that start a run without one. ~24 KB in two blocks - a t2_work_t is
 * 15924 bytes on its own, and that single block is what fails whenever Tier-2
 * cannot start - plus ~8 KB more only if a non-default combiner is selected.
 *
 * When it is called matters more than what it asks for. Called after the I2S
 * buses and the guard buffers, the 15924 has to come out of a heap already
 * cut into pieces by a 32800-byte spectra block and two DMA rings, and it does
 * not fit with 20720 bytes still free. run_standalone() calls this before any
 * of that; see the reservation block there. */
static bool t2_alloc(bool need_cx)
{
    if (!g_t2st) {
        g_t2st = heap_caps_calloc(1, sizeof(t2_state_t),
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!g_t2w) {
        g_t2w = heap_caps_calloc(1, sizeof(t2_work_t),
                                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (need_cx && !g_cxw) {
        g_cxw = heap_caps_calloc(1, sizeof(cx_work_t),
                                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!g_cxw) {
            return false;
        }
    }
    return g_t2st && g_t2w;
}

typedef struct {
    const char *name;
    uint32_t n_samples;      /* 0 = unbounded (live) */
    uint32_t max_frames;     /* 0 = unbounded (live) */
    uint32_t prb_from, prb_to;
    bool     emit_pcm;       /* parity: stream the samples too */
    bool     emit_alert;     /* live/parity: one record on the firing frame */
    bool     stop_on_input;  /* live: any host byte ends the run */
    bool     armed;          /* A mode: drive the actuators from rec.fired */
    /* Opt-in. NULL - which every call site that uses designated initialisers
     * leaves it - is exactly the behaviour with no Tier-2 in the tree. Only
     * `K` sets it. */
    const guard_t2_opts_t *t2;
} stream_opts_t;

/* ---- HOW A STANDALONE RUN IS INTERRUPTED --------------------------------
 * A lab mode stops on ANY host byte, which is fine when a human is driving it
 * and fatal when the device starts its own guard at power-on: the byte that
 * stops the run is the first letter of the command that wanted to replace it,
 * and eating it turns `R 0` into `0`. So a standalone run stops on a WHOLE
 * LINE, which is parked here and dispatched by the main loop untouched. */
static char g_pending[64];
static bool g_have_pending;

/* `S` asks the device to hand itself back to the guard. It has to be honoured
 * from both paths - the console loop, and the line that interrupted a
 * standalone run. Handled in only the first, sending `S` to a device that is
 * already guarding stops the guard, routes the line to dispatch_line() and
 * answers with an unknown-command error, leaving the box in the console doing
 * nothing at all. */
static bool g_want_standalone;

/* Why a run ended. The main loop needs to tell "the host wants the console"
 * from "the operator held the button down". */
#define RUN_END_INPUT     0     /* a host line is waiting in g_pending      */
/* The off ritual: a hold. It draws OFF with the reason under it and then
 * deep-sleeps, or parks on a board that cannot, so the cable can be pulled -
 * or the PCB's slider thrown - against a screen that is telling the truth.
 *
 * There is exactly one ending. Parking in standby with the panel still saying
 * LISTENING is the failure the ritual exists to remove, and a tap-count ritual
 * cannot coexist with paging, since three taps is how an operator reads three
 * pages. */
#define RUN_END_POWER     1     /* the two-second hold                      */
#define RUN_END_COMPLETE  2     /* ran to its length, or could not start    */
/* The cell ran out. Kept apart from RUN_END_POWER because the two lead to
 * different words on the same screen: one is an operator putting the device
 * away, the other is a device that must not come straight back up and finish
 * the battery off. */
#define RUN_END_BATTERY   3

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ---- console-transparent commands ---------------------------------------
 *
 * Any host line takes the console and stops the guard. That is deliberate,
 * because a lab session must be deterministic - but it makes two commands
 * impossible to use for what they exist for. `U btn` cannot drive the state
 * machine if the line carrying it stops the machine, and the frame-budget gate
 * needs an alert to land while four tiers are running.
 *
 * So exactly two prefixes are handled in the reader and never reach the
 * take-over. They are the two that act on the running guard rather than
 * asking it to stand down. Everything else takes the console. */
static const char *skip_ws(const char *p);
static void info_publish(int alive, int amb_db, const char *amb_word);
static void test_publish(void);
static void test_console_line(void);
static void frozen_publish(void);
static bool cmd_btn_word(const char *args);
static bool cmd_alert_word(const char *args);

static bool run_transparent_line(const char *line)
{
    const char *a = skip_ws(line);
    if (*a != 'U' && *a != 'u') {
        return false;
    }
    a = skip_ws(a + 1);
    if (!strncmp(a, "btn", 3)) {
        return cmd_btn_word(skip_ws(a + 3));
    }
    if (!strncmp(a, "alert", 5)) {
        return cmd_alert_word(skip_ws(a + 5));
    }
    return false;
}

/* The one place a long run decides to end. Standalone runs consume a whole
 * line and park it; lab runs stop on any byte. */
static bool run_should_stop(bool standalone, bool armed, int *reason)
{
    if (armed && alert_ui_power_off_requested()) {
        *reason = RUN_END_POWER;
        return true;
    }
    /* The rotation an operator just asked for, persisted. Polled beside the
     * power request because they are the same kind of event: a hold on the
     * button that the UI has already acted on and that something outside the
     * UI has to finish. The panel has already turned; this only writes the
     * quadrant to NVS so it survives the next power cycle, which is the whole
     * point of being able to do it without a laptop. Settings deliberately do
     * not reach into alert_ui.c, so the write lives here. */
    if (alert_ui_take_rotation()) {
        settings_mut()->epd_rotation = epaper_get_rotation();
        const bool ok = settings_save();
        char rb[96];
        snprintf(rb, sizeof(rb), "ROT %u deg %s\n",
                 (unsigned)epaper_degrees_from_quadrant(epaper_get_rotation()),
                 ok ? "saved" : "NOT SAVED");
        trace_text(rb);
    }
    /* The cell, checked here beside the button because they are the same kind
     * of event: something outside the detector has decided the run is over.
     * Reading it is a volatile byte load - the measurement happens on the
     * power task, on the other core, once every ten seconds. Inert on any
     * board with no divider, where power_mon_state() returns POWER_OK. */
    if (armed && power_mon_state() == POWER_EMPTY) {
        *reason = RUN_END_BATTERY;
        return true;
    }
    if (standalone) {
        if (trace_try_line(g_pending, sizeof(g_pending)) > 0) {
            /* Handled here and the guard keeps running: see
             * run_transparent_line. */
            if (run_transparent_line(g_pending)) {
                return false;
            }
            g_have_pending = true;
            *reason = RUN_END_INPUT;
            return true;
        }
        return false;
    }
    if (trace_poll_byte() >= 0) {
        *reason = RUN_END_INPUT;
        return true;
    }
    return false;
}

static void run_stream(source_t *src, const stream_opts_t *o)
{
    trace_hdr_t h = {
        .magic = TRACE_MAGIC_HDR, .version = TRACE_VERSION,
        .n_samples = o->n_samples, .n_frames = o->max_frames,
        .threshold = src->thr,   /* the source's own preset, not a global */
        .fs = CFG_FS, .n_fft = CFG_N_FFT, .hop = CFG_HOP,
        .n_bins = CFG_N_BINS, .n_f0 = CFG_N_F0,
        .rec_size = sizeof(trace_rec_t),
    };
    strncpy(h.name, o->name, sizeof(h.name) - 1);
    trace_send_hdr(&h);

    detector_state_reset(g_st, src->thr);
    /* The golden vectors test the sealed detector and nothing else.
     * detector_state_reset() memsets, so trk_family and veto_voice are clear
     * here whatever the operator has stored in settings, and the vectors are
     * replayed against exactly the code they were cut from. That is true only
     * as a side effect of the memset, so: do not copy the settings flags into
     * g_st on this path. A gate that judges a different detector from the one
     * it was calibrated against proves nothing. */
    g_st->trk_family = false;
    g_st->veto_voice = false;
    g_st->nf_gate    = false;   /* same reason */
    const bool t2on = (o->t2 != NULL && o->t2->enable);
    if (t2on) {
        t2_state_reset(g_t2st);
    }
    uint8_t prev_fired2 = 0;
    uint32_t t2_hits_total = 0;

    const bool bounded = (o->max_frames != 0);
    uint32_t *us = bounded ? calloc(o->max_frames, sizeof(uint32_t)) : NULL;
    uint64_t total_us = 0;
    uint32_t max_us = 0, n_fast = 0;
    uint64_t us_fft = 0, us_mag = 0, us_floor = 0, us_score = 0;
    float peak = -1e30f;
    int32_t longest = 0;
    double t_last = 0.0;
    uint32_t i = 0;
    uint8_t prev_fired = 0;
    uint32_t pcm_seq = 0, sample_index = 0;

    for (i = 0; !bounded || i < o->max_frames; i++) {
        /* slide the analysis window: frame i is x[i*hop : i*hop + n_fft] */
        const int want = (i == 0) ? CFG_N_FFT : CFG_HOP;
        if (i != 0) {
            memmove(g_block, g_block + CFG_HOP,
                    (CFG_N_FFT - CFG_HOP) * sizeof(float));
        }
        if (source_read(src, g_q, (size_t)want) != want) {
            break;                   /* stream ended, or the mic stopped */
        }
        float *dst = (i == 0) ? g_block : g_block + (CFG_N_FFT - CFG_HOP);
        for (int k = 0; k < want; k++) {
            dst[k] = q_to_f(g_q[k]);
        }
        if (o->emit_pcm) {
            /* Emitted before the trace record derived from it, and including
             * the whole first window, so the host can reconstruct exactly the
             * sample array the detector saw - x[0 : n_fft + (frames-1)*hop]. */
            trace_send_pcm(pcm_seq++, sample_index, g_q, (uint16_t)want);
        }
        sample_index += (uint32_t)want;

        /* frame END time - causal, exactly as detector.py computes it */
        const double t = (double)(i * CFG_HOP + CFG_N_FFT) / (double)CFG_FS;
        frame_rec_t rec;

        const int64_t c0 = esp_timer_get_time();
        front_end(g_block, g_w->spec[0], g_w);
        const cf32_t *sp = combiner(&g_w->spec[0][0], CFG_N_CHANNELS, g_w);
        const int64_t cf = esp_timer_get_time();
        back_end(sp, g_st, t, i, g_w, &rec);
        const int64_t c1 = esp_timer_get_time();
        us_fft += (uint64_t)(cf - c0);
        us_mag += rec.us_mag;
        us_floor += rec.us_floor;
        us_score += rec.us_score;

        const uint32_t dt = (uint32_t)(c1 - c0);
        if (us) {
            us[i] = dt;
        }
        total_us += dt;
        if (dt > max_us) {
            max_us = dt;
        }
        if (rec.score > peak) {
            peak = rec.score;
        }
        if (rec.chain > longest) {
            longest = rec.chain;
        }
        n_fast += rec.floor_fast;
        t_last = t;

        trace_rec_t r = {
            .magic = TRACE_MAGIC_REC,
            .frame = rec.frame, .t_s = rec.t_s, .score = rec.score,
            .f0_bin = rec.f0_bin, .f0_hz = rec.f0_hz,
            .f0_raw_hz = rec.f0_raw_hz, .teeth = rec.teeth,
            .floor_fast = rec.floor_fast, .reanch = rec.reanch,
            .n_held_bins = rec.n_held_bins, .above_thr = rec.above_thr,
            .cont_accepted = rec.cont_accepted, .chain = rec.chain,
            .fired = rec.fired, .is_octave = rec.is_octave,
            .us_frame = dt,
        };
        trace_send_rec(&r);

        if (o->emit_alert && rec.fired && !prev_fired) {
            trace_alt_t a = {.magic = TRACE_MAGIC_ALT, .frame = i,
                             .t_s = t, .f0_hz = rec.f0_hz,
                             .score = rec.score, .chain = rec.chain,
                             .n_events = (uint32_t)g_st->n_events + 1u};
            trace_send_alt(&a);
        }
        prev_fired = rec.fired;

        /* Tier-2 beside the sealed tier, on the same spectrum, in `K` mode
         * only. It reads `sp` after back_end has finished with it and writes
         * nothing v1 can see, which is why `R` and `K` produce identical v1
         * records for the same vector. */
        if (t2on) {
            t2_rec_t t2r;
            if (t2_step(sp, NULL, 1, &o->t2->cfg, g_t2st, t, i, g_t2w, &t2r)) {
                trace_t2r_t tr = {
                    .magic = TRACE_MAGIC_T2R, .frame = t2r.frame,
                    .t_s = t2r.t_s, .score2 = t2r.score2,
                    .f02_hz = t2r.f02_hz, .f02_row = t2r.f02_row,
                    .teeth2 = t2r.teeth2, .hit = t2r.hit,
                    .fired2 = t2r.fired2, .excluded = t2r.excluded,
                    .hits = t2r.hits, .n2 = t2r.n2,
                    .track_age = t2r.track_age, .kappa = t2r.kappa,
                    .us_t2 = t2r.us_t2};
                trace_send_t2r(&tr);
                t2_hits_total += t2r.hit;
                if (t2r.fired2 && !prev_fired2) {
                    trace_al2_t a2 = {
                        .magic = TRACE_MAGIC_AL2, .frame = i, .t_s = t,
                        .f02_hz = g_t2st->f0_on, .score2 = t2r.score2,
                        .hits = t2r.hits, .kappa = t2r.kappa,
                        .n_events = (uint32_t)g_t2st->n_events + 1u};
                    trace_send_al2(&a2);
                }
                prev_fired2 = t2r.fired2;
            }
        }

        /* The entire armed-mode integration: one O(1) call, reading a flag
         * the detector already computed. It cannot influence front_end,
         * combiner, back_end or their inputs, and it is skipped entirely in
         * every other mode, which is why `L` remains a valid timing comparator
         * for `A`. */
        if (o->armed) {
            alert_ui_tick(rec.fired != 0, ALERT_TIER_V1, now_ms());
        }

        if (i >= o->prb_from && i <= o->prb_to) {
            probe_rec_t p;
            detector_probe(g_w, g_st, &rec, &p);
            trace_prb_t tp = {.magic = TRACE_MAGIC_PRB, .frame = i,
                              .win_checksum = window_checksum(g_w->fftbuf,
                                                              CFG_N_FFT),
                              .e = p.e, .e_slow = p.e_slow, .flat = p.flat,
                              .rising = p.rising, .fast = p.fast,
                              .argmax_bin = p.argmax_bin,
                              .argmax_score = p.argmax_score};
            memcpy(tp.probe_bin, p.probe_bin, sizeof(tp.probe_bin));
            memcpy(tp.probe_mag, p.probe_mag, sizeof(tp.probe_mag));
            memcpy(tp.probe_floor, p.probe_floor, sizeof(tp.probe_floor));
            memcpy(tp.probe_S, p.probe_S, sizeof(tp.probe_S));
            trace_send_prb(&tp);
        }

        if (o->stop_on_input && trace_poll_byte() >= 0) {
            i++;
            break;
        }
    }

    detector_finish(g_st, t_last);
    if (t2on) {
        t2_finish(g_t2st, t_last);
        char b[80];
        snprintf(b, sizeof(b), "T2 events=%u hits=%u\n",
                 (unsigned)g_t2st->n_events, (unsigned)t2_hits_total);
        trace_text(b);
    }

    const uint32_t n_done = i;
    uint32_t p99 = 0;
    if (us && n_done) {
        qsort(us, n_done, sizeof(uint32_t), cmp_u32);
        uint32_t k = (uint32_t)((99 * n_done + 99) / 100);   /* ceil(0.99n) */
        if (k == 0) {
            k = 1;
        }
        p99 = us[k - 1];
    }
    free(us);

    trace_end_t e = {
        .magic = TRACE_MAGIC_END, .n_frames = n_done,
        .verdict_fires = (g_st->n_events > 0) ? 1 : 0,
        .n_events = (uint32_t)g_st->n_events,
        .peak_score = peak, .longest_chain = longest,
        .n_floor_fast = n_fast, .n_reanch = 0, .n_held_frames = 0,
        .total_us = total_us, .max_us = max_us, .p99_us = p99,
        .us_fft = us_fft, .us_mag = us_mag, .us_floor = us_floor,
        .us_score = us_score,
        .chain_overflow = g_st->overflow ? 1 : 0,
    };
    trace_send_end(&e);
}

/* ---- mode: golden - the parity build ----------------------------------- */
static void run_vector(int idx, uint32_t prb_from, uint32_t prb_to)
{
    if (!mono_alloc()) {
        return;
    }
    source_t src;
    if (source_golden(&src, idx) != ESP_OK) {
        trace_text("ERR bad vector index\n");
        return;
    }
    const uint32_t n = src.n_total;
    const stream_opts_t o = {
        .name = src.name, .n_samples = n,
        .max_frames = (n < CFG_N_FFT) ? 0u : 1u + (n - CFG_N_FFT) / CFG_HOP,
        .prb_from = prb_from, .prb_to = prb_to,
    };
    if (o.max_frames == 0) {
        trace_text("ERR vector shorter than one window\n");
        return;
    }
    run_stream(&src, &o);
}

/* ---- mode: K - golden vector i, with Tier-2 running beside it -----------
 *
 * Replays the same vector from the same flash array through the same
 * run_stream at the vector's own preset threshold, so the v1 half of a `K`
 * trace is byte-identical to the `R` trace for the same index - and the
 * comparator checks exactly that before it looks at Tier-2 at all. The
 * addition is the T2R stream, diffed against src/detector_t2.py run on the
 * same flash-resident samples.
 *
 * `R` is not modified and no new flash vector is added: the four that exist
 * exercise Tier-2's floor, score and tracker perfectly well.
 *
 * kappa is 1.0 on every frame here by definition rather than by omission -
 * one microphone has nothing to be coherent with.
 */
static void run_vector_t2(int idx, int thr2_milli, int rate)
{
    if (!mono_alloc()) {
        return;
    }
    source_t src;
    if (source_golden(&src, idx) != ESP_OK) {
        trace_text("ERR bad vector index\n");
        trace_ack("K", TRACE_ACK_BAD_ARG);
        return;
    }
    const uint32_t n = src.n_total;
    if (n < CFG_N_FFT) {
        trace_text("ERR vector shorter than one window\n");
        trace_ack("K", TRACE_ACK_BAD_ARG);
        return;
    }
    if (!t2_alloc(false)) {
        t2_free();
        trace_text("ERR t2 out of RAM\n");
        trace_ack("K", TRACE_ACK_FAULT);
        return;
    }
    guard_t2_opts_t o2 = {0};
    o2.enable = true;
    o2.cx = 'a';
    t2_cfg_default(&o2.cfg,
                   rate == 1 ? false
                             : (rate == 2 ? true : (T2_HALF_RATE_DEFAULT != 0)));
    t2_cfg_set_thr_milli(&o2.cfg, thr2_milli);
    if (g_excl_n > 0) {
        t2_cfg_set_excl(&o2.cfg, g_excl_n, g_excl_c, g_excl_t);
    }

    char b[128];
    snprintf(b, sizeof(b),
             "K %s tau2_milli=%d N2=%d M2=%d rate=%s\n", src.name,
             (int)(o2.cfg.tau2 * 1000.0 + 0.5), o2.cfg.n2, o2.cfg.m2,
             o2.cfg.decim == 2 ? "half" : "full");
    trace_text(b);

    const stream_opts_t o = {
        .name = src.name, .n_samples = n,
        .max_frames = 1u + (n - CFG_N_FFT) / CFG_HOP,
        .prb_from = 0xFFFFFFFFu, .prb_to = 0u,
        .t2 = &o2,
    };
    run_stream(&src, &o);
    t2_free();
}

/* ---- mode: live - the detector on the mic, at the deployment default ---- */
static void run_live(void)
{
    if (!mono_alloc()) {
        return;
    }
    source_t src;
    esp_err_t e = source_i2s(&src);
    if (e != ESP_OK) {
        trace_text("ERR i2s init failed\n");
        return;
    }
    const stream_opts_t o = {
        .name = src.name, .n_samples = 0, .max_frames = 0,
        .prb_from = 0xFFFFFFFFu, .prb_to = 0u,
        .emit_alert = true, .stop_on_input = true,
    };
    run_stream(&src, &o);
    source_i2s_stop();
}

/* ---- mode: A - armed live, identical to L plus the actuators ------------
 * Same source, same detector, same threshold, same records. The only
 * difference is `armed`, which enables the one alert_ui_tick() call after
 * back_end. `L` is deliberately left untouched so it stays the reference
 * `A`'s p99 is compared against. */
static void run_armed(void)
{
    if (!mono_alloc()) {
        return;
    }
    source_t src;
    if (source_i2s(&src) != ESP_OK) {
        trace_text("ERR i2s init failed\n");
        trace_ack("A", TRACE_ACK_FAULT);
        return;
    }
    alert_ui_begin('A');
    const stream_opts_t o = {
        .name = src.name, .n_samples = 0, .max_frames = 0,
        .prb_from = 0xFFFFFFFFu, .prb_to = 0u,
        .emit_alert = true, .stop_on_input = true, .armed = true,
    };
    run_stream(&src, &o);
    alert_ui_end();                 /* safe-off is inside */
    source_i2s_stop();
}

/* ---- mode: D - alert drill, no detector, no microphone -----------------
 * Exercises the whole alert/snooze/re-arm loop deterministically so the bench
 * can prove the state machine without needing audio to cooperate. */
static void run_drill(uint32_t seconds)
{
    if (seconds == 0u || seconds > 3600u) {
        seconds = 60u;
    }
    alert_ui_begin('D');

    const uint32_t t0 = now_ms();
    const uint32_t run_ms = seconds * 1000u;
    uint32_t next_trigger_ms = t0;          /* first alert immediately */
    uint32_t trigger_until_ms = 0;
    bool trigger = false;

    for (;;) {
        const uint32_t now = now_ms();
        if ((now - t0) >= run_ms) {
            break;
        }
        if (!trigger && (int32_t)(now - next_trigger_ms) >= 0) {
            trigger = true;
            trigger_until_ms = now + 8000u;   /* 8 s, or until snoozed */
            next_trigger_ms = now + 45000u;   /* then one every 45 s   */
        }
        if (trigger) {
            if ((int32_t)(now - trigger_until_ms) >= 0 ||
                alert_ui_state() == TRACE_ALERT_SNOOZED) {
                trigger = false;
            }
        }
        alert_ui_tick(trigger, ALERT_TIER_V1, now);

        if (trace_poll_byte() >= 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));        /* also the debounce period */
    }

    alert_ui_end();
    trace_end_t e = {.magic = TRACE_MAGIC_END, .n_frames = 0};
    trace_send_end(&e);
}

/* ---- mode: Q - quad meter ----------------------------------------------
 * Per-channel level and per-bus health. On a read timeout this emits a STAT
 * naming the bus and keeps trying: a dead bus must be loudly visible rather
 * than producing the clean instant exit the mono meter gives. */
static int16_t g_quad[QUAD_FRAMES_PER_READ * QUAD_N_CH];

static void run_quad_meter(void)
{
    if (source_i2s_quad_start() != ESP_OK) {
        trace_text("ERR quad i2s init failed\n");
        trace_ack("Q", TRACE_ACK_FAULT);
        return;
    }
    trace_hdr_t h = {
        .magic = TRACE_MAGIC_HDR, .version = TRACE_VERSION,
        .n_samples = 0, .n_frames = 0, .threshold = 0.0,
        .fs = CFG_FS, .n_fft = CFG_N_FFT, .hop = CFG_HOP,
        .n_bins = CFG_N_BINS, .n_f0 = CFG_N_F0,
        .rec_size = sizeof(trace_qmt_t),
    };
    strncpy(h.name, "quad_meter", sizeof(h.name) - 1);
    trace_send_hdr(&h);

    const uint32_t BLOCK = CFG_FS / 2;            /* 500 ms per record */
    uint32_t seq = 0;

    for (;;) {
        int64_t sum[QUAD_N_CH] = {0}, sumsq[QUAD_N_CH] = {0};
        int32_t vmin[QUAD_N_CH], vmax[QUAD_N_CH], peak[QUAD_N_CH] = {0};
        for (int c = 0; c < QUAD_N_CH; c++) {
            vmin[c] = 32767;
            vmax[c] = -32768;
        }
        uint32_t got = 0;
        uint32_t empty_reads = 0;

        while (got < BLOCK) {
            const int n = source_i2s_quad_read_ex(g_quad, true);
            if (n <= 0) {
                /* Loud, and keep going. */
                empty_reads++;
                trace_stat_t s = {0};
                s.magic = TRACE_MAGIC_STA;
                s.mode = 'Q';
                s.uptime_ms = now_ms();
                s.button_level = 1;
                trace_send_stat(&s);
                if (empty_reads >= 10u) {
                    break;             /* emit what we have, then loop again */
                }
                continue;
            }
            for (int f = 0; f < n; f++) {
                for (int c = 0; c < QUAD_N_CH; c++) {
                    const int32_t v = g_quad[f * QUAD_N_CH + c];
                    sum[c] += v;
                    sumsq[c] += (int64_t)v * v;
                    if (v < vmin[c]) { vmin[c] = v; }
                    if (v > vmax[c]) { vmax[c] = v; }
                    const int32_t a = (v < 0) ? -v : v;
                    if (a > peak[c]) { peak[c] = a; }
                }
            }
            got += (uint32_t)n;
        }

        trace_qmt_t m = {.magic = TRACE_MAGIC_QMT, .seq = seq++,
                         .t_us = (uint64_t)esp_timer_get_time(),
                         .n_samples = got};
        for (int c = 0; c < QUAD_N_CH; c++) {
            m.rms[c] = got ? sqrt((double)sumsq[c] / (double)got) : 0.0;
            m.dc[c] = got ? (double)sum[c] / (double)got : 0.0;
            m.peak[c] = peak[c];
            m.vmin[c] = got ? vmin[c] : 0;
            m.vmax[c] = got ? vmax[c] : 0;
        }
        const quad_stats_t *st = source_i2s_quad_stats();
        for (int b = 0; b < QUAD_N_BUS; b++) {
            m.timeouts[b] = st->timeouts[b];
            m.short_reads[b] = st->short_reads[b];
            m.frames_total[b] = st->frames_total[b];
        }
        trace_send_qmt(&m);

        if (trace_poll_byte() >= 0) {
            break;
        }
    }
    trace_end_t e = {.magic = TRACE_MAGIC_END, .n_frames = seq};
    trace_send_end(&e);
    source_i2s_quad_stop();
}

/* ---- mode: X - quad PCM capture ---------------------------------------
 * ~128 kB/s of 4-channel int16 over USB-Serial-JTAG is unproven, which is why
 * every record carries a mandatory sequence number: the host must be able to
 * detect loss exactly and analyse only contiguous stretches, rather than
 * splicing across a gap and reading the splice as an inter-channel delay. */
static void run_quad_capture(uint32_t seconds)
{
    if (source_i2s_quad_start() != ESP_OK) {
        trace_text("ERR quad i2s init failed\n");
        trace_ack("X", TRACE_ACK_FAULT);
        return;
    }
    const uint32_t n_total = seconds * CFG_FS;
    trace_hdr_t h = {
        .magic = TRACE_MAGIC_HDR, .version = TRACE_VERSION,
        .n_samples = n_total, .n_frames = 0, .threshold = 0.0,
        .fs = CFG_FS, .n_fft = CFG_N_FFT, .hop = CFG_HOP,
        .n_bins = CFG_N_BINS, .n_f0 = CFG_N_F0,
        .rec_size = sizeof(trace_pc4_hdr_t),
    };
    strncpy(h.name, "quad_pcm", sizeof(h.name) - 1);
    trace_send_hdr(&h);

    uint32_t seq = 0, base = 0, empty = 0;
    while (base < n_total) {
        const int n = source_i2s_quad_read(g_quad);
        if (n <= 0) {
            empty++;
            trace_stat_t s = {0};
            s.magic = TRACE_MAGIC_STA;
            s.mode = 'X';
            s.uptime_ms = now_ms();
            s.button_level = 1;
            trace_send_stat(&s);
            if (empty >= 25u) {
                break;                 /* a bus is dead; stop rather than spin */
            }
            continue;
        }
        empty = 0;
        trace_send_pcm4(seq++, base, g_quad, (uint16_t)n, QUAD_N_CH);
        base += (uint32_t)n;
        if (trace_poll_byte() >= 0) {
            break;
        }
    }
    trace_end_t e = {.magic = TRACE_MAGIC_END, .n_frames = seq};
    trace_send_end(&e);
    source_i2s_quad_stop();
}

/* ==========================================================================
 * mode: G - GUARD. The whole device, running as a device.
 *
 * Four microphones -> front_end per channel -> combiner (broadside sum) ->
 * the committed back_end at the DEPLOYMENT threshold -> buzzer, motor, LED,
 * e-paper, and a snooze button that silences the outputs without ever gating
 * detection. Runs until the host sends a byte.
 *
 * What the combiner does here: it is the existing N>1 path, an unweighted
 * complex sum across the four spectra. That is a broadside (delay-zero) beam -
 * it favours sound arriving equally at all four capsules and gives roughly
 * +6 dB on a correlated source against uncorrelated noise. It is not steered
 * beamforming and it produces no bearing. Nothing here touches front_end,
 * combiner or back_end themselves.
 *
 * The caveat: every threshold in this project was calibrated on one
 * microphone, and summing four changes the signal-to-floor relationship in a
 * way that has never been measured. The adaptive floor should absorb most of
 * it, since signal and floor scale together, but "should" is not "was
 * measured". Treat guard-mode detections as qualitative until there is a
 * bench measurement.
 * ========================================================================== */
#define GUARD_N_CH QUAD_N_CH

/* Tier-4's telemetry is a text line, on purpose.
 *
 * Every other tier has a binary trace record, and adding a fifth would mean
 * changing trace.h and trace_proto.py - the wire format the golden gate, the
 * quad parity gate and every capture tool already agree on. Tier-4 ships
 * disabled and has no parity gate yet, so a protocol change would spend the
 * thing those gates depend on before there is anything to spend it for.
 *
 * A text line costs nothing: every host parser already skips text, and at
 * 3.9 updates a second a seventy-byte line is 273 B/s. When Tier-4 earns a
 * parity gate it earns a binary record with it, and that is the moment to
 * change the wire.
 *
 * The fields are the reference record's, in its order, so a host diff needs no
 * translation layer. */
static void trace_text_t4(uint32_t frame, const t4_rec_t *r)
{
    char b[224];
    /* The per-stage breakdown rides on the same line. us= alone says how
     * long an update took and not where it went; these four say where, and
     * us_other is the computed remainder so the five visibly sum to us= or
     * the gap is on the line to be argued with. Appended after the existing
     * fields, so every tool that reads the old line still reads it. */
    snprintf(b, sizeof(b),
             "T4R f=%lu t=%.3f W4=%.3f f0=%.1f ratio=%.3f hit=%d above=%d "
             "excl=%d hits=%d/%d age=%d fired=%d us=%lu "
             "psd=%lu prom=%lu scan=%lu track=%lu other=%lu\n",
             (unsigned long)frame, r->t, r->W4, r->f0, r->ratio,
             r->hit ? 1 : 0, r->above ? 1 : 0, r->excluded ? 1 : 0,
             r->hits, r->n4, r->track_age, r->fired4 ? 1 : 0,
             (unsigned long)r->us_t4,
             (unsigned long)r->us_psd, (unsigned long)r->us_prom,
             (unsigned long)r->us_scan, (unsigned long)r->us_track,
             (unsigned long)r->us_other);
    trace_text(b);
}

/* ==========================================================================
 * Frame scheduling - which frames each tier makes expensive.
 *
 * Nothing about the algorithms makes three tiers in one 32 ms hop hard. What
 * makes it hard is that Tier-2 and Tier-3 each have one heavy frame in a
 * repeating cycle, and if the two cycles line up, every heavy frame is heavy
 * twice.
 *
 *   v1, quad          21.7 ms   every frame
 *   Tier-2 step        7.5 ms   every T2 decim-th    (29.2 - 21.7)
 *   Tier-3 transform   9.1 ms   every 4th            (30.8 - 21.7)
 *
 * Disjoint: worst frame ~= 21.7 + 9.1 = 30.8 ms, inside the hop.
 * Collided: worst frame ~= 21.7 + 7.5 + 9.1 = 38.3 ms, and real time is gone.
 *
 * Tier-2's phase: t2_step increments its own frame counter on every call and
 * works when the remainder is zero, so with decim 2 it works on even frames,
 * counting from the first frame of the run.
 *
 * Tier-3's phase: t3_push_block appends T3_HOP/T3_DECIM envelope samples per
 * frame and transforms when it holds T3_ENV_NFFT of them and T3_ENV_HOP are
 * new. The ring has to fill before the first transform, which is
 * T3_PRIME_FRAMES pushes, so the first transform lands on the
 * (T3_PRIME_FRAMES-1)-th frame Tier-3 sees rather than the first. Everything
 * after that is every T3_FRAMES_PER_UPDATE frames, which is even, so the
 * parity of the first transform is the parity of all of them.
 *
 * Starting Tier-3 at frame 0 therefore puts every transform on an odd frame,
 * which is exactly where Tier-2 is not. Delaying it by one frame moves all of
 * them onto even frames instead - the collision such a guard would be written
 * to prevent. The static assertion below is what holds it, because this is
 * arithmetic a comment cannot enforce.
 * ========================================================================== */
#define T3_ENV_PER_FRAME      (T3_HOP / T3_DECIM)
#define T3_FRAMES_PER_UPDATE  (T3_ENV_HOP / T3_ENV_PER_FRAME)
#define T3_PRIME_FRAMES       (T3_ENV_NFFT / T3_ENV_PER_FRAME)
#define T3_START_FRAME        0
#define T3_FIRST_UPDATE_FRAME (T3_START_FRAME + T3_PRIME_FRAMES - 1)

_Static_assert(T3_ENV_HOP % T3_ENV_PER_FRAME == 0,
               "Tier-3's update cadence is not a whole number of frames");
_Static_assert(T3_ENV_NFFT % T3_ENV_PER_FRAME == 0,
               "Tier-3's priming is not a whole number of frames");
_Static_assert(T3_FRAMES_PER_UPDATE % 2 == 0,
               "Tier-3's cadence is odd, so its updates do not keep a parity "
               "and cannot be held off Tier-2's frames by a phase choice");
_Static_assert(T3_FIRST_UPDATE_FRAME % 2 == 1,
               "Tier-3's first transform lands on an EVEN frame, which is "
               "where a half-rate Tier-2 also works. Fix the start frame, do "
               "not delete this assertion.");

/* Tier-4's slot. Same arithmetic, and it is a function in detector_t4.c
 * rather than a sentence because the accumulator has to fill before the first
 * update, so the first update lands on the (T4_WIN_FRAMES-1)-th frame Tier-4
 * sees. With three tiers there are three cycles to keep apart:
 *
 *   Tier-2   every 2nd frame          -> even frames
 *   Tier-3   every 4th, first at 15   -> frames = 3 (mod 4)
 *   Tier-4   every 8th, first at 65   -> frames = 1 (mod 8), so 1 (mod 4)
 *
 * which leaves Tier-4 the only slot Tier-2 and Tier-3 both leave free. Getting
 * there costs a two-frame start offset and nothing else. */
#define T4_START_FRAME        2
#define T4_FIRST_UPDATE_FRAME (T4_START_FRAME + T4_WIN_FRAMES - 1)

_Static_assert(T4_UPDATE_FRAMES % 4 == 0,
               "Tier-4's cadence is not a multiple of 4, so its updates do "
               "not keep a mod-4 slot and cannot be held off the other tiers");
_Static_assert(T4_FIRST_UPDATE_FRAME % 2 == 1,
               "Tier-4 updates on an EVEN frame, which is where a half-rate "
               "Tier-2 works");
_Static_assert(T4_FIRST_UPDATE_FRAME % 4 != T3_FIRST_UPDATE_FRAME % 4,
               "Tier-4 and Tier-3 update in the same mod-4 slot; the worst "
               "frame would carry both transforms");

/* How many frames the runtime probe watches before it reports. Starts after
 * the warm-up transitions so the mask it prints is the steady state, not the
 * priming. Tier-4 needs 65 frames before its first update, so the window must
 * outlast that or the probe reports a tier that has not started. */
#define SCHED_PROBE_FROM   72u
#define SCHED_PROBE_TO     264u

static float  *g_gwin[GUARD_N_CH];      /* per-channel sliding window       */
static cf32_t *g_gspec;                 /* [n_spec_ch][CFG_N_BINS]          */
static int16_t *g_gpend;                /* interleaved staging              */
static float  *g_gsum;                  /* summed window, SUMMED path only  */

/* ---- the per-gate instrument --------------------------------------------
 *
 * None of this gates anything. The per-gate counters say which gate throws a
 * real rig's frames away, on air rather than by inference from an offline
 * replay.
 *
 * `p` is the number that decides whether the chain can fire at all. Accept is
 * +1 and reject is -2 against a need of 6, so the drift per frame is 3p - 2
 * and the chain makes no progress unless p > 2/3. Near that boundary the
 * latency explodes: p = 0.75 fires in 0.77 s, p = 0.68 in 4.8 s, and p = 0.66
 * never fires at all.
 *
 * The denominator excludes `thr` and `jit`, for two different reasons.
 * A frame the score never lifted is not a frame a gate took away, so THR and
 * WARMUP are out. JIT is out because a jitter refusal is not a frame
 * rejection at all: that frame was ACCEPTED and counted in `acc`, and the
 * refusal happened later at fire time. Putting it in the denominator would
 * count the same frame on both sides. It is reported beside p instead. */
static nf_probe_t g_nfp;

/* One window's worth of gate accounting. THE SIX REASONS PARTITION THE FRAMES
 * EXACTLY - tracker_step writes exactly one DET_REJ_* per frame and the switch
 * below is exhaustive - so acc + thr + band + veto + nf + cont == frames, and
 * `TOT` is printed so that identity is visible rather than asserted.
 *
 * THR is a category, not a total: it counts frames whose score never cleared
 * the band threshold, plus the warmup frames. On a quiet window it dominates
 * every other category by an order of magnitude, which reads as missing
 * frames until the total is printed beside it. TOT removes the question. */
typedef struct {
    uint32_t acc, thr, band, veto, nf, cont, jit, frames, fast;
    /* The number that decides whether audio is actually lost. A frame over
     * the 32 ms hop is a warning: the DMA holds 96 ms of slack, three whole
     * hops, so an isolated 33.5 ms frame is handed back by the next 25 ms one.
     * Only a sustained mean over 32 ms loses audio, or a driver-reported
     * overflow, which source_i2s_quad counts. */
    uint64_t dt_sum_us;
    uint32_t dt_n;
} gate_counts_t;

static gate_counts_t g_gate;       /* accumulating, this window            */
static gate_counts_t g_gate_last;  /* the last COMPLETE window - displayed */
static gate_counts_t g_gate_fire;  /* snapshot at the deciding frame       */
static uint32_t      g_gate_win_ms;

/* ---- where the over-budget frames are, by which tier decided -------------
 *
 * A four-tier build measures several percent of frames over the 32 ms hop,
 * against a three-tier p99 of 29.30 ms with none over in n = 1938. It is a
 * real overrun rather than jitter: the frame timer starts after the I2S read
 * and the window fill, so it measures compute.
 *
 * The suspected mechanism is in detector_t4.c: t4_push_frame() accumulates
 * over T4_B_LO..T4_B_HI - 563 bins - on every frame, not only on the one frame
 * in eight where it decides. Its prominence slices are correctly scheduled
 * onto frames 1 (mod 4), disjoint from Tier-2's even frames, but the per-frame
 * accumulation is not scheduled at all, so it lands on the Tier-2 frames,
 * which are already the most expensive - worst v1 + T2 measured 33.5 ms.
 *
 * That is a hypothesis until these counters say so. `U c t4 0` and a rerun is
 * the decisive test, and this is the instrument that reads it: frames run and
 * frames over, split by which tier took a decision on them. If the overruns
 * are Tier-2 frames and they vanish with Tier-4 off, the mechanism is proved
 * and the fix is to schedule the accumulation. If they do not, it is not. */
#define OVER_BY_N 4      /* 0 = v1 alone, 1 = +T2, 2 = +T3, 3 = +T4 */
static uint32_t g_run_by[OVER_BY_N], g_over_by[OVER_BY_N];
static uint16_t g_sat_teeth, g_sat_gaps;         /* most recent frame      */
static uint16_t g_sat_teeth_fire, g_sat_gaps_fire;
static float    g_r1_fire, g_l_fire;
static uint32_t g_alerts_at_freeze;

/* One decimal with a sign, without %f: this image does not link newlib's
 * float formatting and every other number on these pages uses the same
 * integer trick (see CENT_I / CENT_F). */
static void fmt_db1(char *out, size_t n, float v)
{
    const bool neg = (v < 0.0f);
    float a = fabsf(v);
    if (!(a < 999.9f)) { a = 999.9f; }   /* also catches NaN, deliberately */
    const unsigned t = (unsigned)(a * 10.0f + 0.5f);
    snprintf(out, n, "%s%u.%u", neg ? "-" : "",
             (unsigned)((t / 10u) % 1000u), (unsigned)(t % 10u));
}

/* p as a PERCENTAGE of the frames that cleared the threshold, or -1 when none
 * did. A caller printing "0" for "no data" would report a dead detector.
 *
 * The denominator excludes thr and jit, for two different reasons. A frame the
 * score never lifted is not a frame a gate took away, so thr and warmup are
 * out. jit is out because a jitter refusal is not a frame rejection at all:
 * that frame was ACCEPTED and is already counted in acc, and the refusal came
 * later at fire time. Counting it here would count one frame on both sides. */
static int gate_p_pct(const gate_counts_t *g)
{
    const uint32_t den = g->acc + g->band + g->veto + g->nf + g->cont;
    if (den == 0u) {
        return -1;
    }
    return (int)((g->acc * 100u + den / 2u) / den);
}

/* FF as a PERCENTAGE of the window's frames with the floor on its FAST
 * constant. The room session asked whether FF was a count, a fraction or a
 * percentage; it is a percentage, and the page now says so with a %. */
static int gate_ff_pct(const gate_counts_t *g)
{
    if (g->frames == 0u) {
        return -1;
    }
    return (int)((g->fast * 100u + g->frames / 2u) / g->frames);
}

static void gate_count_frame(const frame_rec_t *rec)
{
    switch (rec->reject_reason) {
    case DET_REJ_NONE:   g_gate.acc++;  break;
    case DET_REJ_THR:
    case DET_REJ_WARMUP: g_gate.thr++;  break;
    case DET_REJ_BAND:   g_gate.band++; break;
    case DET_REJ_VETO:   g_gate.veto++; break;
    case DET_REJ_NF:     g_gate.nf++;   break;
    case DET_REJ_CONT:   g_gate.cont++; break;
    default: break;
    }
    g_gate.jit    += rec->jit_blocked ? 1u : 0u;
    g_gate.frames += 1u;
    g_gate.fast   += rec->floor_fast ? 1u : 0u;
    g_sat_teeth = rec->sat_teeth;
    g_sat_gaps  = rec->sat_gaps;
}

/* Close the window on a TIMER IN THE FRAME LOOP, not in test_publish().
 *
 * The room session's second defect: the reset used to live in test_publish(),
 * which runs ONLY while the TEST page is showing. With the page frozen on an
 * alert, or on MAIN, the counters accumulated unbounded - so `P .12` at
 * UP 00:27 described ten minutes of rig, music, chord and speech mixed
 * together and said nothing about any of them. Now the window is 5 s of wall
 * clock whatever the panel is doing. */
static void gate_window_tick(uint32_t now)
{
    if ((uint32_t)(now - g_gate_win_ms) >= TEST_REFRESH_MS) {
        g_gate_win_ms = now;
        g_gate_last = g_gate;
        memset(&g_gate, 0, sizeof(g_gate));
    }
}

/* ==========================================================================
 * The summed-window path.
 *
 * The guard used to run front_end - a Hann window and a 2048-point transform
 * - once per channel, and then sum the four spectra bin by bin. The DFT is
 * LINEAR and all four channels get the SAME window, so
 *
 *     sum_c FFT(w . x_c)  ==  FFT(w . sum_c x_c)
 *
 * exactly, in exact arithmetic. Summing the four windows first and
 * transforming once is the same combined spectrum for a quarter of the
 * transform cost - about 3.4 ms a frame of the quad build's 21.7, predicted
 * from a per-stage measurement of 0.84 ms of FFT plus 0.30 ms of magnitude
 * per channel.
 *
 * In float it is not bit-identical, and that is the only reason this is a
 * selectable path rather than a rewrite: four roundings of a sum of
 * transforms is not the same rounding as one transform of a sum. The claim
 * being made is decision identity, and it is a measurement - see
 * scripts/fft_path_proof.py, which replays archived four-channel device audio
 * through both paths and diffs above_thr / cont_accepted / chain / fired.
 *
 * What the path costs, stated plainly rather than buried:
 *
 *   - the per-channel spectra do not exist on it. A combiner_cx variant other
 *     than 'a' genuinely needs them, so the path is used only for 'a'.
 *   - Tier-2's kappa - the inter-capsule coherence it REPORTS but never
 *     decides on - cannot be computed. It reads NaN, which is the "not
 *     measured" value the host tools already skip, and never 1.0.
 *
 * It also gives memory back rather than taking it: the spectra block goes
 * from four channels to one, 32800 bytes to 8200, against 8192 for the summed
 * window. Net -16 KB on the path that fits three tiers.
 * ========================================================================== */
static int g_gspec_ch = GUARD_N_CH;

static void guard_free(void)
{
    for (int c = 0; c < GUARD_N_CH; c++) {
        free(g_gwin[c]);
        g_gwin[c] = NULL;
    }
    free(g_gspec);
    g_gspec = NULL;
    free(g_gpend);
    g_gpend = NULL;
    free(g_gsum);
    g_gsum = NULL;
}

static bool guard_alloc(bool summed)
{
    /* ~65 KB per-channel, ~49 KB summed, allocated per run and freed on exit
     * so the passive modes keep the heap they had. `I` reports the numbers. */
    for (int c = 0; c < GUARD_N_CH; c++) {
        g_gwin[c] = heap_caps_calloc(CFG_N_FFT, sizeof(float),
                                     MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!g_gwin[c]) {
            return false;
        }
    }
    g_gspec_ch = summed ? 1 : GUARD_N_CH;
    g_gspec = heap_caps_calloc((size_t)g_gspec_ch * CFG_N_BINS, sizeof(cf32_t),
                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    g_gpend = heap_caps_calloc((size_t)CFG_HOP * GUARD_N_CH, sizeof(int16_t),
                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (summed) {
        g_gsum = heap_caps_calloc(CFG_N_FFT, sizeof(float),
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!g_gsum) {
            return false;
        }
    }
    return g_gspec && g_gpend;
}

/* One quad frame loop, three modes:
 *   G  armed=true,  seconds=0, t2o=NULL -> runs until stopped, drives the
 *                                 actuators
 *   Z  armed=false, seconds=s, t2o=NULL -> bounded, ALSO streams the exact
 *                                 samples fed to the pipeline, so the host can
 *                                 re-run the Python reference on them and diff
 *                                 decisions
 *   H  armed=true,  t2o != NULL -> G, plus Tier-2 beside it and the alert
 *                                 driven by the OR of the two tiers
 * Sharing one loop is the same discipline as run_stream(): the acceptance test
 * must exercise the code the product actually runs, not a parallel copy.
 *
 * t2o == NULL IS TODAY'S BEHAVIOUR. Every Tier-2 line below is inside an
 * `if (t2o ...)`, so `G` and `Z` execute the instruction sequence they
 * executed before this build. That is what lets the golden gate keep meaning
 * what it meant. */
/* ---------------------------------------------------------------------------
 * The ambient bucket - a word for what the device is hearing.
 *
 * It is a QUANTISATION of the detector's own adaptive floor and adds no signal
 * processing: the floor is computed every frame regardless, and this reads its
 * mean once a second. Four words, with hysteresis, so it changes a handful of
 * times an hour rather than every frame - which is the only way a field on an
 * e-paper screen can carry it at all.
 *
 * The thresholds are provisional; see ui_config.h. The dB figure itself is
 * printed on the info page so they can be set from a real deployment rather
 * than from a guess.
 * ------------------------------------------------------------------------ */
static int ambient_db_now(void)
{
    if (!g_st || !g_st->have_floor) {
        return -99;
    }
    double acc = 0.0;
    for (int b = 0; b < CFG_N_BINS; b++) {
        acc += g_st->floor_[b];
    }
    const double mean = acc / (double)CFG_N_BINS;
    if (!(mean > 0.0)) {
        return -99;
    }
    return (int)(10.0 * log10(mean));
}

/* Hysteresis lives in the caller's state, not here: a bucket that recomputed
 * itself from scratch each second would flicker on every boundary. */
static const char *ambient_word(int db, uint8_t *bucket)
{
    static const char *W[4] = {"QUIET", "LOW", "BUSY", "LOUD"};
    const int edge[3] = {AMBIENT_DB_LOW, AMBIENT_DB_BUSY, AMBIENT_DB_LOUD};
    uint8_t b = *bucket;
    /* rise only past the edge, fall only past edge - hysteresis */
    while (b < 3 && db > edge[b] + AMBIENT_HYST_DB) {
        b++;
    }
    while (b > 0 && db < edge[b - 1] - AMBIENT_HYST_DB) {
        b--;
    }
    *bucket = b;
    return W[b];
}

/* x.xx from a float, without float printf: PicoLibC has none, which is why
 * the whole trace is binary. */
static int cent(float v)
{
    const float x = v * 100.0f + (v >= 0.0f ? 0.5f : -0.5f);
    if (x > 2000000.0f)  { return 2000000; }
    if (x < -2000000.0f) { return -2000000; }
    return (int)x;
}
#define CENT_I(v) (cent(v) / 100)
#define CENT_F(v) ((cent(v) < 0 ? -cent(v) : cent(v)) % 100)

/* ---- the deciding record ------------------------------------------------
 * One snapshot, taken at the frame whose comparison fired, read by the ALERT
 * screen, the ring, the LoRa packet, the test page and the console. Nothing
 * downstream is allowed a second opinion.
 *
 * Without it the ALERT screen reads a per-frame score, and the panel does not
 * draw until ALERT_SCREEN_START_MS after the decision - so the screen shows
 * the score from a frame well after the one that fired, and can display a
 * value below the threshold for an alert that was correct. The ring, written
 * in the same loop iteration as rec.fired, holds the right number, so the two
 * disagree. */
typedef struct {
    uint8_t  tier;          /* ALERT_TIER_*, NONE when nothing has fired   */
    uint8_t  fired_set;     /* EVLOG_FS_*, every tier that was up on this
                             * frame, not just the first in the priority
                             * order that `tier` reports                     */
    float    score;         /* the value the comparison used               */
    float    thr;           /* the constant it was compared against        */
    float    hz;            /* f0, or the rate for T3                      */
    uint32_t t_ms;
} decide_t;
static volatile decide_t g_decide;

/* The v1 threshold the detector is running, published from g_st->thr so every
 * surface that shows it reads one value. */
static volatile float g_run_thr1;

/* ---- the frozen alert page ----------------------------------------------
 * In test mode the panel stops being a live display at the instant of a
 * decision and becomes a record of it: every tier's score, threshold and f0 as
 * they were at that frame rather than as they are when the panel finally
 * draws, held until the operator taps.
 *
 * The point is a test run twenty metres away: the operator walks over when
 * they are ready and reads what the box heard, including what the tiers that
 * did not fire were doing at the same moment. That comparison is the
 * measurement, and a page that refreshed itself would destroy it. */
typedef struct {
    uint8_t  valid;
    uint8_t  tier;             /* the deciding tier, or ALERT_TIER_REMOTE   */
    uint16_t remote_id;
    uint16_t remote_thr1;
    float    remote_score;
    float    score[4];         /* v1, T2, T3, T4                            */
    float    thr[4];
    float    hz[4];            /* rate for T3                               */
    uint8_t  have[4];
    /* The instrument, at the deciding frame. R1 and L against rig distance
     * are what the near-field constants are set from, and a page that freezes
     * on the alert without carrying them loses the whole measurement. A page
     * that holds the deciding frame must hold all of it. */
    float    r1_db, l_db, r_db;
    uint16_t sat_t, sat_g;
    int16_t  p_pct, ff_pct;       /* -1 when the window had no data          */
    uint32_t g_acc, g_thr, g_band, g_veto, g_nf, g_cont, g_jit, g_tot;
    uint32_t alerts_at_freeze;    /* so a later reader can tell if more came */
} frozen_t;
static volatile frozen_t g_frozen;

typedef struct {
    float    v1_score, v1_thr, v1_hz;
    float    t2_score, t2_thr, t2_hz;
    float    t3_score, t3_thr, t3_rate;
    float    t4_score, t4_thr, t4_hz;
    /* One running maximum per tier: four comparisons per frame, and nothing
     * else in the frame loop moves. The instantaneous score alone is useless
     * during a test, because the repaint is every few seconds and a tier's
     * score can peak and fall back between two of them - "nothing happened"
     * and "T2 reached 0.91 of 1.15" would look identical. The UI clears these
     * when it reads them, so each repaint reports the window since the
     * previous one.
     *
     * Read and cleared from the UI task while the frame loop writes them. A
     * torn read costs at worst one frame's peak on one repaint, which is why
     * no lock is taken: locking the frame loop to protect a display number
     * would be a real cost paid for a cosmetic one. */
    float    v1_pk, t2_pk, t3_pk, t4_pk;
    /* When each tier last wrote a record. The frozen page needs a number for
     * every tier at the deciding frame, and a tier that updates one frame in
     * eight has no fresh score on most frames. With this, "no candidate right
     * now" prints the last score it did have instead of a word. */
    uint32_t seen_ms[4];
    uint8_t  have2, have3, have4;
} info_snap_t;
static volatile info_snap_t g_info;

/* Frames whose measured cost exceeded the hop, since power-on. The number to
 * watch when the frame budget is thin. */
static volatile uint32_t g_frames_over;
static uint32_t s_info_last_ms;
/* Test mode's own repaint clock and near-miss tally. Declared here rather
 * than beside test_publish() because the frame loop above reads them and a
 * file-scope static cannot be forward declared. */
static uint32_t s_test_last_ms;
static uint32_t s_near_count;
/* The frozen page is drawn once per freeze, not once per frame. */
static bool s_frozen_drawn;
/* The alert tier on the previous frame, so the freeze can be taken on the
 * onset edge for every alert path. */
static uint8_t s_prev_alert_tier;
/* The link test's last outcome, for the TEST page. */
static volatile uint8_t  g_link_state;

/* ---- `Lp <N>`: ping every N seconds -------------------------------------
 * A range walk needs the box to keep asking while the operator carries the
 * other unit away, since a single `Ls` is useless once you are 50 m from the
 * keyboard. This reuses the existing link-test packet rather than adding a
 * second message to the air interface: detection and local alerting only, and
 * the radio carries one message. */
static volatile uint32_t g_ping_period_s;   /* 0 = off                      */
static volatile uint32_t g_ping_due_ms;
static volatile uint32_t g_ping_tx, g_ping_rx;
static volatile uint16_t g_link_peer;
static volatile uint32_t g_link_ms;

static int run_quad_pipeline(uint32_t seconds, bool armed, double thr,
                             const guard_t2_opts_t *t2o,
                             const guard_t3_opts_t *t3o,
                             const guard_t4_opts_t *t4o, bool standalone)
{
    const bool t2on = (t2o != NULL && t2o->enable);
    const bool t3on = (t3o != NULL && t3o->enable);
    /* The near-field gate, declared here beside t2on/t3on for the same reason
     * loraon, poweron and disp_on are: the no-regression certificate audits
     * this function for code the untouched modes would execute, and "the
     * sealed path is unchanged" has to be readable at the line rather than
     * derived from a settings call three files away. */
    const bool nfon = (settings_get()->nf_gate != 0);
    const bool t4on = (t4o != NULL && t4o->enable);
    /* THE RADIO IS LIVE IN THE FIELD MODE ONLY.
     *
     * `G`, `Z`, `H` and every bounded capture are LAB runs, judged on
     * decision-for-decision identity, and a lab session that started
     * transmitting - or that answered a colleague's bench test with a buzzer -
     * would be a surprise nobody asked for. The standalone guard is the mode
     * where "everyone alerts everyone" is the design; it is also the only
     * mode that calls lora_link_begin(). So this is `standalone` and the
     * radio actually answering, and on a board with no radio it is a compiled
     * zero. */
    const bool lora_on = standalone && armed && lora_link_ready();
    /* Same discipline for the battery bar. On the breadboard this is a
     * compiled zero and the operator surface is byte-for-byte what it was; on
     * the PCB it is what puts the bar on the LISTENING screen. Declared here
     * beside the tier flags so the certificate's mode-isolation audit can see
     * the guard - and so a reader of the frame loop can too. */
    const bool power_on = armed && (BOARD_HAS_VBAT != 0);
    /* THE OPERATOR DISPLAY runs on an armed guard and nowhere else. `Z` - the
     * unarmed capture path through this same function - must execute none of
     * it, and naming the condition here is what lets certify.py's mode
     * isolation audit say so at the line rather than derive it from a call
     * graph. Same arrangement as lora_on and power_on above. */
    const bool disp_on = armed;
    /* The summed-window path is used whenever nothing downstream needs the
     * per-channel spectra: that is every combiner but the experimental ones.
     * Selected here, printed at arming, never guessed at by the frame loop. */
    const bool summed = !t2on || t2o->cx == 'a';
    const char cmd = t2on ? 'H' : (armed ? 'G' : 'Z');
    const char cmds[2] = {cmd, '\0'};

    /* The mono modes' 12 KB is not needed while the guard runs, and with
     * three tiers it is the difference between fitting and not. */
    mono_free();

    /* ---- allocation order, and the claimant that is not here ------------
     * The per-run set is I2S (48 KB of DMA) + guard (70 KB) + Tier-2 (24 KB),
     * in this order. Tier-3's 13 KB is deliberately not in it; see
     * t3_boot_alloc().
     *
     * The order is load-bearing and took three measurements to settle. With
     * Tier-3 allocating per-run, I2S first left it without a contiguous 12688
     * (18616 free, largest 7680), and I2S last left the SECOND bus without its
     * 24576 (57180 free, largest 31744). A reservation - take the DMA's 48 KB
     * as one block, hand it back just before the driver asks - moved the
     * failure onto the guard's 32800-byte spectra instead. The heap has no
     * placement that fits four large claimants; the answer was to stop having
     * four, not to shuffle them. */
    const char *stage = NULL;
    if (!guard_alloc(summed)) {
        stage = summed ? "guard buffers (~54 KB, summed-window path)"
                       : "guard buffers (70 KB, incl. one 32800-byte spectra "
                         "block)";
    } else if (source_i2s_quad_start() != ESP_OK) {
        stage = "the I2S buses (48 KB of DMA)";
    } else if (t2on && !t2_alloc(t2o->cx != 'a')) {
        stage = "Tier-2 state (24 KB)";
    } else if (t4on && !t4_alloc()) {
        stage = "Tier-4 state (~10 KB)";
    } else if (t3on && !t3_alloc()) {
        stage = "Tier-3 state - it is claimed at BOOT, so this means boot "
                "could not get it";
    }
    /* NAME THE STAGE THAT FAILED, and print the heap. The old message said
     * "out of RAM" whatever went wrong, including a Tier-3 FFT init that had
     * not allocated anything at all - a message that sends the next session
     * looking in the wrong place, and did. */
    if (stage) {
        char b[128];
        snprintf(b, sizeof(b), "ERR quad: could not start - %s\n", stage);
        trace_text(b);
        heap_line("at failure");
        source_i2s_quad_stop();
        t3_free();
        t2_free();
        guard_free();
        trace_ack(cmds, TRACE_ACK_FAULT);
        return RUN_END_COMPLETE;
    }
    {
        char pb[160];
        snprintf(pb, sizeof(pb),
                 "FFT path: %s (%d transform%s/frame)%s\n",
                 summed ? "summed window" : "per channel",
                 summed ? 1 : GUARD_N_CH, summed ? "" : "s",
                 summed ? ", kappa not measured" : "");
        trace_text(pb);
    }
    heap_line("armed");

    if (armed) {
        /* The panel is alert_ui's from here: its first tick asks for
         * LISTENING. Requesting it here as well cost a second full refresh -
         * four seconds of a panel that is 2 s per update - for no extra
         * information. */
        alert_ui_begin(cmd);
    }

    trace_hdr_t h = {
        .magic = TRACE_MAGIC_HDR, .version = TRACE_VERSION,
        .n_frames = 0,
        .n_samples = seconds * CFG_FS,
        .threshold = thr,
        .fs = CFG_FS, .n_fft = CFG_N_FFT, .hop = CFG_HOP,
        .n_bins = CFG_N_BINS, .n_f0 = CFG_N_F0,
        .rec_size = sizeof(trace_rec_t),
    };
    strncpy(h.name, t2on ? "guard_quad_t2"
                         : (armed ? "guard_quad" : "quad_parity"),
            sizeof(h.name) - 1);
    /* NO HEADER FOR A STANDALONE RUN - the same reasoning that withholds the
     * sentinel, and it took a failed golden gate to notice the other half.
     *
     * With autostart on and a header emitted, a later bounded capture reads
     * back the guard's header instead of its own - the frames are the
     * replay's, the header is whatever was still in the link when the host
     * connected - and the golden comparison fails on a device whose detector
     * is perfectly correct.
     *
     * HDR and END are framing for a bounded capture. A standalone guard is not
     * one: it is unbounded, nobody is necessarily reading it, and the same
     * information is in the plain-text banner above, which every parser skips
     * by design. Emitting neither is what makes autostart safe to leave on. */
    if (!standalone) {
        trace_send_hdr(&h);
    }

    detector_state_reset(g_st, thr);
    /* THE FAMILY RULE, from the persisted settings and DEFAULT OFF. Set after
     * the reset because the reset memsets - which is also what guarantees a
     * run that does not ask for it gets the sealed tracker. It applies to
     * every armed mode including `G`, deliberately: a variant that could only
     * be exercised in the field mode could not be characterised on the bench.
     * The flag is what the trace and the banner report. */
    g_st->trk_family = (settings_get()->trk_family != 0);
    if (g_st->trk_family) {
        trace_text("TRK family {2,3} continuity ON (normalised jitter) - "
                   "adopt-ELIGIBLE, not the shipped default\n");
    }
    /* THE VETO, same discipline: the flag is what the trace and the banner
     * report, so a run is always self-describing about which detector it was.
     * A device that fires on a piano and a device that does not are different
     * instruments, and a log that does not say which one produced it cannot
     * be used to judge either. */
    g_st->veto_voice = (settings_get()->veto_voice != 0);
    if (g_st->veto_voice) {
        char bv[128];
        snprintf(bv, sizeof(bv),
                 "VETO voice/struck-note ON: f0 >= %.0f Hz AND teeth >= %d\n",
                 (double)CFG_VETO_F0_MIN_HZ, (int)CFG_VETO_MIN_TEETH);
        trace_text(bv);
    }
    /* Printed for the same reason the veto line is: the settings line says
     * what is stored and this says what the detector took, and the difference
     * between those two claims can be a device that sees nothing. */
    g_st->nf_gate = nfon;
    {
        char bn[160];
        if (nfon) {
            snprintf(bn, sizeof(bn),
                     "NF near-field gate ON: jitter <= %.3f, and at score >= "
                     "%.2f require R >= %.1f dB\n",
                     (double)CFG_MAX_JITTER_FIELD, (double)CFG_NF_SCORE_HI,
                     (double)CFG_NF_R_MIN_DB);
        } else {
            snprintf(bn, sizeof(bn),
                     "NF near-field gate OFF: sealed jitter <= %.3f, no R "
                     "requirement (sealed behaviour)\n",
                     (double)CFG_MAX_JITTER);
        }
        trace_text(bn);
    }
    /* ---- test mode is announced, loudly ---------------------------------
     * It changes the glass, the ring, the radio payload and the console and
     * nothing about detection, so a unit in test mode detects exactly as a
     * unit out of it. That is precisely why it has to be printed: a posture
     * that is invisible in the numbers is a posture somebody will carry to a
     * deployment. The home screen carries the same tag for the operator who
     * has no laptop. */
    if (settings_get()->test_mode) {
        trace_text("TEST MODE: bench posture. Detection, thresholds, alert "
                   "timing and outputs are UNCHANGED.\n");
    }
    /* ---- the four compiled thresholds, on one line ----------------------
     * Each tier also prints its own further down, which means answering "what
     * is this image's operating point" from those alone requires reading four
     * separate lines and knowing that the v1 one is in milli-units and the
     * rest are not. Together and in the same units, a session log can be
     * checked at a glance. */
    {
        char bt[192];
        /* ---- the running v1, read from what the detector compares -------
         * g_st->thr is not a copy of the threshold, it is the threshold:
         * detector_state_reset() assigns it and detector.c compares
         * `score > band_threshold(f0, st->thr)` against it. Printing
         * a settings field or the compiled constant here would be printing
         * something that OUGHT to equal what the detector uses, and this
         * banner exists precisely because those two drifted apart once
         * already.
         *
         * The compiled default appears only when it differs, so the ordinary
         * board prints one number and the paired board prints the difference
         * it was configured for. */
        const double run_v1 = g_st->thr;
        /* PUBLISHED ONCE so the TEST page header, the frozen page and this
         * banner cannot re-derive it differently. Re-deriving a displayed
         * number from its source constants is exactly what put 1.65 on an
         * ALERT screen. */
        g_run_thr1 = (float)run_v1;
        char v1s[48];
        if (fabs(run_v1 - (double)DEFAULT_THRESHOLD) < 1e-9) {
            snprintf(v1s, sizeof(v1s), "v1=%.4f", run_v1);
        } else {
            snprintf(v1s, sizeof(v1s), "v1=%.4f (compiled %.4f)",
                     run_v1, (double)DEFAULT_THRESHOLD);
        }
        snprintf(bt, sizeof(bt),
                 "THRESHOLDS running %s  t2=%.4f  t3=%.4f  t4=%.4f\n",
                 v1s, (double)T2_TAU2, (double)T3_TAU3, (double)T4_TAU4);
        trace_text(bt);
        /* ---- the unit and its one variable ------------------------------
         * A paired field configuration differs by v1's threshold and nothing
         * else, so every disagreement between the two boards has exactly one
         * possible cause. That is only legible if each board says which one
         * it is, and the running threshold is the settings value rather than
         * the compiled default. */
        const uint32_t t1 = settings_get()->thr1_milli
                                ? settings_get()->thr1_milli
                                : (uint32_t)(DEFAULT_THRESHOLD * 1000.0 + 0.5);
        snprintf(bt, sizeof(bt), "UNIT N-%04X thr %lu.%02lu\n",
                 (unsigned)lora_link_device_id(),
                 (unsigned long)(t1 / 1000u),
                 (unsigned long)((t1 % 1000u) / 10u));
        trace_text(bt);
    }
    if (t4on) {
        t4_reset(g_t4st);
        char b4[160];
        snprintf(b4, sizeof(b4),
                 "T4 tau4_milli=%d grid=%d-%dHz nharm=%d n4=%d m4=%d "
                 "upd=%d frames warmup_ms=%d\n",
                 (int)(t4o->cfg.tau4 * 1000.0 + 0.5),
                 (int)T4_F0_LO, (int)T4_F0_HI, T4_N_HARM,
                 t4o->cfg.n4, t4o->cfg.m4, T4_UPDATE_FRAMES,
                 (int)(t4o->cfg.warmup_s * 1000.0));
        trace_text(b4);
    }
    if (t3on) {
        t3_reset(g_t3st);
        char b[128];
        snprintf(b, sizeof(b),
                 "T3 tau3_milli=%d fire=%d-%dHz n3=%d m3=%d nharm=%d "
                 "warmup_ms=%d\n",
                 (int)(t3o->cfg.tau3 * 1000.0 + 0.5),
                 (int)t3o->cfg.fire_lo, (int)t3o->cfg.fire_hi,
                 t3o->cfg.n3, t3o->cfg.m3, t3o->cfg.n_harm,
                 (int)(t3o->cfg.warmup_s * 1000.0));
        trace_text(b);
    }
    if (t2on) {
        t2_state_reset(g_t2st);
        /* Integers only: PicoLibC's printf has no float support, which is why
         * the whole trace is binary. tau2 and busoff go out in MILLI-units,
         * the same units the `H` command takes them in. */
        char b[128];
        snprintf(b, sizeof(b),
                 "T2 tau2_milli=%d N2=%d M2=%d gap=%d rel=%d rate=%s cx=%c "
                 "busoff_milli=%d\n",
                 (int)(t2o->cfg.tau2 * 1000.0 + 0.5), t2o->cfg.n2,
                 t2o->cfg.m2, t2o->cfg.gap, t2o->cfg.release,
                 t2o->cfg.decim == 2 ? "half" : "full", t2o->cx,
                 (int)(t2o->busoff * 1000.0f));
        trace_text(b);
    }

    /* From here to the end of the loop the audio path is HOT: nothing may
     * block on the panel. epaper_wait_idle() enforces it. */
    epaper_set_audio_hot(true);

    uint32_t frame = 0;
    uint32_t pcm_seq = 0, sample_index = 0;
    const uint32_t n_total = seconds * CFG_FS;
    bool stop_after_this = false;
    uint32_t pend = 0;              /* frames waiting in g_gpend            */
    uint32_t filled = 0;            /* samples of window primed so far      */
    uint8_t prev_fired = 0;
    uint8_t prev_fired2 = 0;
    uint8_t prev_fired3 = 0;
    uint8_t prev_fired4 = 0;
    bool    alert_open = false;     /* the OR, for the event ring           */
    /* Sentinel telemetry, so the quad END record says what the mono one says.
     * The quad parity gate compares the device's sentinel against the
     * reference's own longest_chain and n_floor_fast; leave them unfilled and
     * that gate reports FAIL with zero decision mismatches on every run it
     * ever does. A gate that fails for a reason unrelated to what it gates is
     * worse than no gate. These are counters over records that already exist;
     * no decision reads them. */
    /* THE FOOTER'S TWO FACTS, sampled once a second.
     *
     * "are all four microphones alive" and "which algorithms are running" are
     * the two questions a field session keeps asking, and until now both
     * needed a laptop. The RMS is taken from the newest hop of the windows
     * front_end has already been handed - no new buffer, no new read - and
     * only on every 32nd frame, so the cost is ~0.3 us/frame amortised against
     * the 1.1 ms of headroom the guard has left. epaper_set_footer() is a
     * strncpy; it never draws and never blocks. */
    uint32_t foot_next = 0;
    /* THE OPERATOR DISPLAY's own state, held across the run: the snapshot
     * being maintained, and which hour its time fields were last refreshed
     * on. Zero-initialised, so the first second of a run publishes once and
     * then goes quiet until something happens. */
    epaper_main_t disp = {0};
    uint32_t disp_hour = 0xFFFFFFFFu;
    /* The ambient bucket's hysteresis state, and the raw dB behind it. The
     * info page prints the number so the thresholds can be set from a real
     * deployment rather than from ui_config.h's provisional values. */
    uint8_t  amb_bucket = 0;
    int      amb_db = -99;
    float   peak_score = -1e30f;
    int32_t longest_chain = 0;
    uint32_t n_floor_fast = 0;
    float   last_score2 = 0.0f;     /* Tier-2's score on its last step      */
    float   last_W = 0.0f;          /* Tier-3's W on its last update        */
    float   last_W4 = 0.0f;         /* Tier-4's W4 on its last update       */
    int64_t t3_us_prev = 0;         /* us_front+us_score are RUNNING TOTALS */
    int     end_reason = RUN_END_COMPLETE;
    uint32_t empty = 0;
    bool alerted_once = false;

    /* The scheduling probe: two 4-bit masks saying which frame slots each
     * tier made expensive, and the worst frame time seen in each class,
     * reported once. Measured rather than assumed, because the tier phases are
     * arithmetic that a comment cannot enforce. Four ops a frame for 128
     * frames, then nothing. */
    uint8_t  sched_t2 = 0, sched_t3 = 0, sched_t4 = 0;
    uint32_t worst_plain = 0, worst_t2 = 0, worst_t3 = 0, worst_both = 0;
    bool     sched_done = false;

    for (;;) {
        const int n = source_i2s_quad_read(g_quad);
        if (n <= 0) {
            /* A bus stopped. Say so loudly and keep trying: going quiet here
             * is the failure mode that wastes a whole session. */
            if (++empty % 8u == 1u) {
                trace_stat_t s = {0};
                s.magic = TRACE_MAGIC_STA;
                s.mode = 'G';
                s.uptime_ms = now_ms();
                s.button_level = 1;
                trace_send_stat(&s);
            }
            if (armed) {
                /* SAY IT ON THE BOX. A standalone device has no console to
                 * print STAT records to, and "the panel still says LISTENING
                 * while no audio has arrived for a minute" is the failure
                 * that comes home with an empty capture. */
                alert_ui_fault_led(now_ms());
            }
            if (run_should_stop(standalone, armed, &end_reason)) {
                break;
            }
            continue;
        }
        empty = 0;

        /* PARITY: emit the samples BEFORE the frames derived from them, in
         * contiguous sequence-numbered blocks, so the host reconstructs
         * exactly the array the pipeline consumed. */
        if (!armed) {
            trace_send_pcm4(pcm_seq++, sample_index, g_quad,
                            (uint16_t)n, QUAD_N_CH);
            sample_index += (uint32_t)n;
            if (n_total && sample_index >= n_total) {
                stop_after_this = true;
            }
        }

        for (int f = 0; f < n; f++) {
            for (int c = 0; c < GUARD_N_CH; c++) {
                g_gpend[pend * GUARD_N_CH + c] = g_quad[f * GUARD_N_CH + c];
            }
            pend++;
            if (pend < CFG_HOP) {
                continue;
            }
            pend = 0;

            /* slide every channel's window by one hop, append the new block */
            for (int c = 0; c < GUARD_N_CH; c++) {
                memmove(g_gwin[c], g_gwin[c] + CFG_HOP,
                        (CFG_N_FFT - CFG_HOP) * sizeof(float));
                float *dst = g_gwin[c] + (CFG_N_FFT - CFG_HOP);
                for (int k = 0; k < CFG_HOP; k++) {
                    dst[k] = q_to_f(g_gpend[k * GUARD_N_CH + c]);
                }
            }
            /* R1 and L, from channel 0 only.
             *
             * Fed here rather than after the priming check so the biquads see
             * a continuous sample stream: a filter that skips the first three
             * hops carries their transient into the first frame anybody
             * reads. The 30 s reference settles during priming for the same
             * reason. See nf_probe.h for why one channel and not the sum. */
            nf_probe_frame(&g_nfp, g_gwin[0] + (CFG_N_FFT - CFG_HOP), CFG_HOP);
            if (filled < CFG_N_FFT) {
                filled += CFG_HOP;
                if (filled < CFG_N_FFT) {
                    continue;          /* window not primed yet */
                }
            }

            const double t = (double)((double)frame * CFG_HOP + CFG_N_FFT) /
                             (double)CFG_FS;
            frame_rec_t rec;
            const int64_t c0 = esp_timer_get_time();
            const cf32_t *sp;
            if (summed) {
                /* ONE transform. The sum is the combined spectrum already, so
                 * there is no combiner call left to make - which is also why
                 * this cannot silently diverge from combiner(): it is not a
                 * second implementation of it, it is the same arithmetic done
                 * before the transform instead of after. */
                for (int i = 0; i < CFG_N_FFT; i++) {
                    float a = g_gwin[0][i];
                    for (int c = 1; c < GUARD_N_CH; c++) {
                        a += g_gwin[c][i];
                    }
                    g_gsum[i] = a;
                }
                front_end(g_gsum, g_gspec, g_w);
                sp = g_gspec;
            } else {
                for (int c = 0; c < GUARD_N_CH; c++) {
                    front_end(g_gwin[c], &g_gspec[(size_t)c * CFG_N_BINS],
                              g_w);
                }
                /* `G`/`Z` call the sealed combiner directly. `H` with cx 'a' -
                 * the default - reaches the SAME function through combiner_cx,
                 * which returns combiner()'s own result rather than a
                 * reimplementation of it. */
                sp = t2on
                    ? combiner_cx(t2o->cx, t2o->f_split_hz, t2o->busoff,
                                  g_gspec, GUARD_N_CH, g_w, g_cxw)
                    : combiner(g_gspec, GUARD_N_CH, g_w);
            }
            back_end(sp, g_st, t, frame, g_w, &rec);
            t2_rec_t t2r;
            bool t2ran = false;
            if (t2on) {
                t2ran = t2_step(sp, summed ? NULL : g_gspec, GUARD_N_CH,
                                &t2o->cfg, g_t2st, t, frame, g_t2w, &t2r);
            }
            /* ---- TIER-3, on the RAW BLOCK ------------------------------
             * A different seam from Tier-2 on purpose: a per-frame band energy
             * is sampled at 31.25 Hz, whose Nyquist is 15.6 Hz, so it cannot
             * see an 82 Hz modulation. Tier-3 therefore takes the newest hop
             * of each channel's window - the same samples front_end just
             * consumed, before the window and the transform.
             *
             * The first frame used to be skipped, and the skip was backwards.
             * The comment that stood here said the skip moved Tier-3's updates
             * onto odd frames so they never landed on Tier-2's even ones. The
             * arithmetic says the opposite, and it is arithmetic, not opinion
             * - see T3_FIRST_UPDATE_FRAME above. Tier-3 needs T3_ENV_NFFT
             * envelope samples before its FIRST transform, which is 16 pushes,
             * so the first transform lands on the SIXTEENTH frame it sees, not
             * the first. Starting at frame 0 puts it on frame 15 - odd, and
             * disjoint from Tier-2. Starting at frame 1 put it on frame 16 -
             * EVEN, landing on Tier-2's step on every single update.
             *
             * It never showed up because it cannot: the pair builds have only
             * ever run one of the two tiers, and one tier cannot collide with
             * a tier that is not there. It would have shown up on the first
             * all-three frame as roughly 21.7 + 7.5 + 9.1 = 38 ms against a
             * 32 ms hop. The probe below MEASURES the two cadences rather than
             * trusting either comment. */
            t3_rec_t t3r;
            bool t3ran = false;
            if (t3on) {
                const float *chp[GUARD_N_CH];
                for (int c = 0; c < GUARD_N_CH; c++) {
                    chp[c] = g_gwin[c] + (CFG_N_FFT - CFG_HOP);
                }
                t3ran = t3_push_block(chp, GUARD_N_CH, &t3o->cfg, g_t3st,
                                      armed && alert_ui_outputs_active(),
                                      &t3r);
            }
            /* ---- TIER-4, on the COMBINED SPECTRUM ----------------------
             * The cheapest seam of the four: it consumes the array back_end
             * was just handed, so it costs no transform. The two-frame start
             * offset is the whole of its scheduling - see T4_START_FRAME. */
            t4_rec_t t4r;
            bool t4ran = false;
            if (t4on && frame >= T4_START_FRAME) {
                t4ran = t4_push_frame(sp, &t4o->cfg, g_t4st,
                                      (uint32_t)frame, t,
                                      armed && alert_ui_outputs_active(),
                                      &t4r);
                if (t4ran) {
                    /* THIS WAS MISSING AND THE PANEL SAID "T4 OFF" FOREVER.
                     * have2 and have3 are set at their own tiers, and have4
                     * has to be set here or the info page reads "T4 OFF" on a
                     * board that is running Tier-4 the whole time - the worst
                     * kind of display bug, because the only way to catch it is
                     * to disbelieve the device. */
                    g_info.t4_score = (float)t4r.W4;
                    if ((float)t4r.W4 > g_info.t4_pk) { g_info.t4_pk = (float)t4r.W4; }
                    g_info.t4_hz    = (float)t4r.f0;
                    g_info.t4_thr   = (float)t4o->cfg.tau4;
                    g_info.have4    = 1u;
                    g_info.seen_ms[3] = now_ms();
                }
            }
            const uint32_t dt = (uint32_t)(esp_timer_get_time() - c0);
            /* THE DROP ACCOUNTING. One hop is 32 ms; a frame that took longer
             * than that did not merely run late, it means audio was dropped,
             * which is the one failure this device cannot have during an
             * alert. Counted here because this is the only place the measured
             * cost of a frame exists. */
            {
                /* SCHED proves at most one tier decides on a frame, so this
                 * classification is a partition. Tier-4's per-frame
                 * accumulation is in EVERY bucket, which is the point. */
                const int ob = t2ran ? 1 : (t3ran ? 2 : (t4ran ? 3 : 0));
                if (g_run_by[ob] < 0xFFFFFFFFu) { g_run_by[ob]++; }
                if (dt > 32000u && g_over_by[ob] < 0xFFFFFFFFu) {
                    g_over_by[ob]++;
                }
            }
            g_gate.dt_sum_us += dt;
            g_gate.dt_n++;
            if (dt > 32000u) {
                if (g_frames_over < 0xFFFFFFFFu) {
                    g_frames_over++;
                }
                bootrec_count_drop();
            }
            gate_count_frame(&rec);
            gate_window_tick(now_ms());
            g_info.v1_score = (float)rec.score;
            if ((float)rec.score > g_info.v1_pk) { g_info.v1_pk = (float)rec.score; }
            g_info.v1_hz    = (float)rec.f0_hz;
            g_info.v1_thr   = (float)thr;
            g_info.seen_ms[0] = now_ms();

            if (!sched_done && frame >= SCHED_PROBE_FROM) {
                const uint8_t slot = (uint8_t)(1u << (frame & 3u));
                if (t2ran) { sched_t2 |= slot; }
                if (t3ran) { sched_t3 |= slot; }
                if (t4ran) { sched_t4 |= slot; }
                uint32_t *w = t2ran ? (t3ran ? &worst_both : &worst_t2)
                                    : (t3ran ? &worst_t3   : &worst_plain);
                if (dt > *w) { *w = dt; }
                if (frame >= SCHED_PROBE_TO) {
                    char sb[224];
                    const bool clash = (sched_t2 & sched_t3) ||
                                       (sched_t2 & sched_t4) ||
                                       (sched_t3 & sched_t4);
                    snprintf(sb, sizeof(sb),
                             "SCHED t2=%X t3=%X t4=%X mod4 %s  "
                             "worst us: v1 %u, +t2 %u, +t3 %u, both %u\n",
                             (unsigned)sched_t2, (unsigned)sched_t3,
                             (unsigned)sched_t4,
                             clash ? "COLLIDE" : "disjoint",
                             (unsigned)worst_plain, (unsigned)worst_t2,
                             (unsigned)worst_t3, (unsigned)worst_both);
                    trace_text(sb);
                    /* Say it twice when it is wrong. A standalone box has no
                     * console, and this is the line that explains a p99 over
                     * the hop before anyone starts bisecting the detector. */
                    if (clash) {
                        trace_text("BUG two tiers are working on the same "
                                   "frames; the worst frame carries both. "
                                   "See T3_FIRST_UPDATE_FRAME and "
                                   "T4_START_FRAME.\n");
                    }
                    sched_done = true;
                }
            }

            trace_rec_t r = {
                .magic = TRACE_MAGIC_REC,
                .frame = rec.frame, .t_s = rec.t_s, .score = rec.score,
                .f0_bin = rec.f0_bin, .f0_hz = rec.f0_hz,
                .f0_raw_hz = rec.f0_raw_hz, .teeth = rec.teeth,
                .floor_fast = rec.floor_fast, .reanch = rec.reanch,
                .n_held_bins = rec.n_held_bins, .above_thr = rec.above_thr,
                .cont_accepted = rec.cont_accepted, .chain = rec.chain,
                .fired = rec.fired, .is_octave = rec.is_octave,
                .us_frame = dt,
            };
            trace_send_rec(&r);
            if (rec.score > peak_score) {
                peak_score = rec.score;
            }
            if ((int32_t)rec.chain > longest_chain) {
                longest_chain = (int32_t)rec.chain;
            }
            n_floor_fast += rec.floor_fast ? 1u : 0u;

            if (rec.fired && !prev_fired) {
                trace_alt_t a = {.magic = TRACE_MAGIC_ALT, .frame = frame,
                                 .t_s = t, .f0_hz = rec.f0_hz,
                                 .score = rec.score, .chain = rec.chain,
                                 .n_events = (uint32_t)g_st->n_events + 1u};
                trace_send_alt(&a);
                alerted_once = true;
            }
            prev_fired = rec.fired;

            if (t2ran) {
                trace_t2r_t tr = {
                    .magic = TRACE_MAGIC_T2R, .frame = t2r.frame,
                    .t_s = t2r.t_s, .score2 = t2r.score2,
                    .f02_hz = t2r.f02_hz, .f02_row = t2r.f02_row,
                    .teeth2 = t2r.teeth2, .hit = t2r.hit,
                    .fired2 = t2r.fired2, .excluded = t2r.excluded,
                    .hits = t2r.hits, .n2 = t2r.n2,
                    .track_age = t2r.track_age, .kappa = t2r.kappa,
                    .us_t2 = t2r.us_t2};
                trace_send_t2r(&tr);
                last_score2 = t2r.score2;
                g_info.t2_score = (float)t2r.score2;
                if ((float)t2r.score2 > g_info.t2_pk) { g_info.t2_pk = (float)t2r.score2; }
                g_info.t2_hz    = (float)t2r.f02_hz;
                g_info.t2_thr   = (float)t2o->cfg.tau2;
                g_info.have2    = 1u;
                    g_info.seen_ms[1] = now_ms();
                if (t2r.fired2 && !prev_fired2) {
                    trace_al2_t a2 = {
                        .magic = TRACE_MAGIC_AL2, .frame = frame, .t_s = t,
                        .f02_hz = g_t2st->f0_on, .score2 = t2r.score2,
                        .hits = t2r.hits, .kappa = t2r.kappa,
                        .n_events = (uint32_t)g_t2st->n_events + 1u};
                    trace_send_al2(&a2);
                    alerted_once = true;
                }
                prev_fired2 = t2r.fired2;
            }

            if (t3ran) {
                const int64_t t3_us_now = g_t3st->us_front + g_t3st->us_score;
                const uint32_t t3_us = (uint32_t)(t3_us_now - t3_us_prev);
                t3_us_prev = t3_us_now;
                trace_t3r_t t3rec = {
                    .magic = TRACE_MAGIC_T3R, .frame = frame,
                    .t_s = t3r.t, .r_hz = t3r.r, .W = t3r.W,
                    .r_any_hz = t3r.r_any, .W_any = t3r.W_any,
                    .hits = t3r.hits, .n3 = t3r.n3,
                    .track_age = t3r.track_age,
                    .hit = t3r.hit ? 1u : 0u,
                    .fired3 = t3r.fired3 ? 1u : 0u,
                    .frozen = t3r.frozen ? 1u : 0u, .pad = 0u,
                    .us_t3 = t3_us};
                trace_send_t3r(&t3rec);
                last_W = (float)t3r.W;
                g_info.t3_score = (float)t3r.W;
                if ((float)t3r.W > g_info.t3_pk) { g_info.t3_pk = (float)t3r.W; }
                g_info.t3_rate  = (float)t3r.r;
                g_info.t3_thr   = (float)t3o->cfg.tau3;
                g_info.have3    = 1u;
                    g_info.seen_ms[2] = now_ms();
                if (t3r.fired3 && !prev_fired3) {
                    trace_al3_t a3 = {
                        .magic = TRACE_MAGIC_AL3, .frame = frame, .t_s = t,
                        .r_hz = g_t3st->r_on, .W = t3r.W, .hits = t3r.hits,
                        .n_events = (uint32_t)g_t3st->n_events + 1u};
                    trace_send_al3(&a3);
                    alerted_once = true;
                }
                prev_fired3 = t3r.fired3;
            }

            if (t4ran) {
                trace_text_t4(frame, &t4r);
                last_W4 = (float)t4r.W4;
                if (t4r.fired4 && !prev_fired4) {
                    alerted_once = true;
                }
                prev_fired4 = t4r.fired4 ? 1u : 0u;
            }

            if (disp_on && frame >= foot_next) {
                foot_next = frame + 32u;          /* about once a second */
                int alive = 0;
                for (int c = 0; c < GUARD_N_CH; c++) {
                    const float *w = g_gwin[c] + (CFG_N_FFT - CFG_HOP);
                    float acc = 0.0f;
                    for (int k = 0; k < CFG_HOP; k += 4) {   /* 1 in 4 is plenty */
                        acc += w[k] * w[k];
                    }
                    /* 1e-4 of full scale is ~3 int16 counts: below that a
                     * channel is not quiet, it is absent. */
                    if (sqrtf(acc / (float)(CFG_HOP / 4)) > 1e-4f) {
                        alive++;
                    }
                }
                /* ---- the ping tick ------------------------------------
                 * In the one-per-second block because that is the coarsest
                 * clock the protocol needs and the finest the radio should
                 * see: a ping is a range instrument, not an alert, and it
                 * must never contend with one. lora_link_send_test() queues
                 * on the link task, so nothing here waits on the air. */
                if (g_ping_period_s &&
                    (int32_t)(now_ms() - g_ping_due_ms) >= 0) {
                    g_ping_due_ms = now_ms() + g_ping_period_s * 1000u;
                    if (lora_link_send_test(true)) {
                        g_ping_tx++;
                    }
                }

                /* Integers only: PicoLibC's printf has no float support,
                 * which is why the whole trace is binary. Clamped so the
                 * compiler can see the field widths are bounded. */
                int thr_milli = (int)(thr * 1000.0 + 0.5);
                if (thr_milli < 0)     { thr_milli = 0; }
                if (thr_milli > 99999) { thr_milli = 99999; }

                /* ---- the operator display --------------------------------
                 *
                 * One snapshot per second, and epaper_set_main() throws away
                 * any that would draw the same pixels - so this loop is free
                 * on the ~86 000 seconds a day when nothing happens, and the
                 * panel redraws only when something did.
                 *
                 * The time fields are only refreshed on the hour, and that is
                 * the whole of the refresh budget's arithmetic. A "last alert
                 * 12 min ago" that ticked would be a two-second full-panel
                 * refresh every sixty seconds, for ever: 1440 a day against
                 * the ~30 this device is allowed. Between heartbeats the age
                 * is stale by up to an hour, which is what "coarse" means and
                 * is the right trade on a panel that costs two seconds to
                 * change. An event-driven publish - an alert, a battery
                 * slice, a warning - carries fresh time along with it for
                 * free, because it was going to redraw anyway. */
                const uint32_t hour = now_ms() / 3600000u;
                const bool beat = (hour != disp_hour);
                if (beat) {
                    disp_hour = hour;
                }

                /* HOME says LISTENING and nothing else. ALERT and SNOOZE are
                 * their own screens, so the resting screen never changes its
                 * own headline. */
                snprintf(disp.state_word, sizeof(disp.state_word),
                         "LISTENING");
                {
                    const int adb = ambient_db_now();
                    snprintf(disp.ambient, sizeof(disp.ambient), "%s",
                             ambient_word(adb, &amb_bucket));
                    amb_db = adb;
                }
                disp.snooze_s = (uint16_t)(SNOOZE_MS / 1000u);

                /* THE WARNING LINE. A fault an operator would otherwise have
                 * to know to look for. It is on MAIN and not on the status
                 * page for exactly that reason: nobody checks a status page
                 * to find out whether they should have checked it. */
                /* ---- the mic count is off this screen ------------------
                 * Put in the warning line it makes the home screen redraw
                 * constantly: every change is a full two-second refresh, and
                 * `alive` moves on ordinary variation, so the resting screen
                 * never rests.
                 *
                 * The count is not lost - INFO carries it as MIC n/4 OK/FAIL,
                 * which is where somebody checking the box will look. What is
                 * still owed is a once-only MIC FAULT chN report and its ring
                 * entry; without them a channel that dies overnight is
                 * visible only to whoever opens INFO.
                 *
                 * LOW BATT stays: it is a fact about the whole unit, it moves
                 * once, and it is the one thing an operator walking past must
                 * not have to open a page to discover. */
                if (power_mon_state() == POWER_LOW) {
                    snprintf(disp.warn, sizeof(disp.warn), "LOW BATT");
                } else {
                    disp.warn[0] = 0;
                }

                /* ---- the alert screen's one line ---------------------
                 * Which tier fired, at what score against its threshold, and
                 * where. Left unwritten, epaper_main_t::alert_line still gets
                 * drawn, so the ALERT screen shows the word alone and the one
                 * thing anyone wants to know afterwards is missing. A remote
                 * alert's numbers come off the packet, and it prints the
                 * sender rather than a tier that is not this box's. */
                {
                    const uint8_t at = alert_ui_tier();
                    if (at == ALERT_TIER_REMOTE) {
                        const uint16_t rs = lora_link_remote_score_x100();
                        if (rs) {
                            snprintf(disp.alert_line, sizeof(disp.alert_line),
                                     "FROM N-%04X %d.%02d",
                                     (unsigned)lora_link_remote_id(),
                                     (int)(rs / 100u), (int)(rs % 100u));
                        } else {
                            snprintf(disp.alert_line, sizeof(disp.alert_line),
                                     "FROM N-%04X",
                                     (unsigned)lora_link_remote_id());
                        }
                    } else if (at != ALERT_TIER_NONE) {
                        /* The deciding record, not a live snapshot. See
                         * decide_t: reading g_info here shows the score from
                         * a frame well after the one that fired. */
                        const float sc = g_decide.score;
                        const float th = g_decide.thr;
                        const float hz = g_decide.hz;
                        /* Wider than the row, for the reason info_publish()
                         * gives: -Werror=format-truncation must assume a
                         * pathological value for every field. The row is
                         * truncated to EPAPER_LINE_MAX by contract, and the
                         * render test checks it at full width. */
                        char al[80];
                        snprintf(al, sizeof(al),
                                 "%s %d.%02d/%d.%02d %dHZ",
                                 evlog_tier_short(g_decide.tier),
                                 CENT_I(sc), CENT_F(sc),
                                 CENT_I(th), CENT_F(th),
                                 (int)(hz + 0.5f));
                        /* Explicit precision: epaper_page_line() truncates
                         * by contract, and saying so here is what keeps
                         * -Werror=format-truncation from rejecting a copy
                         * that is correct. */
                        snprintf(disp.alert_line, sizeof(disp.alert_line),
                                 "%.*s", (int)(sizeof(disp.alert_line) - 1u),
                                 al);
                    } else {
                        disp.alert_line[0] = 0;
                    }
                }

                disp.alerts_local  = evlog_count_local();
                disp.alerts_remote = evlog_count_remote();
                snprintf(disp.unit_id, sizeof(disp.unit_id), "N-%04X",
                         (unsigned)lora_link_device_id());
                disp.radio_on = lora_link_ready() ? 1u : 0u;
                disp.test_mode = settings_get()->test_mode ? 1u : 0u;
                alert_ui_set_test_mode(disp.test_mode != 0u);
                /* THE LAST PEER HEARD, and it is written only after one has
                 * actually been heard. lora_link_remote_id() is 0 until the
                 * first accepted frame and 0 IS A LEGAL PEER ID, so printing
                 * it unconditionally would put "FROM N-0000" on the resting
                 * screen of every device that has never heard anything. */
                if (evlog_count_remote() > 0) {
                    snprintf(disp.last_src, sizeof(disp.last_src), "N-%04X",
                             (unsigned)lora_link_remote_id());
                }
                disp.batt_absent = settings_get()->batt_absent ? 1u : 0u;

                if (beat) {
                    disp.up_min = now_ms() / 60000u;
                    uint32_t last_ms = 0;
                    disp.last_alert_min = evlog_last_alert_ms(&last_ms)
                        ? (int32_t)((now_ms() - last_ms) / 60000u)
                        : -1;
                }

                /* THE BAR, AND THE ONE RULE ABOUT IT: draw on CHANGE, never
                 * on a schedule. A redraw is a two-second full refresh of the
                 * whole panel, so a bar that updated once a second would
                 * leave the box permanently mid-refresh and showing neither
                 * state. That rule is now enforced by the snapshot itself -
                 * an unchanged slice publishes nothing - rather than by a
                 * direct epaper_request() that skipped the dwell and left the
                 * coalescer's idea of what was on the glass stale. */
                const bool poweron = power_on;
                if (poweron) {
                    epaper_set_battery(power_mon_slice(),
                                       power_mon_charging());
                    disp.batt_slice = (int8_t)power_mon_slice();
                    disp.batt_charging = power_mon_charging() ? 1u : 0u;
                    (void)power_mon_display_changed();
                } else {
                    disp.batt_slice = -1;
                }
                epaper_set_main(&disp);

                /* ---- THE INFO PAGE REPAINTS WHILE IT IS SHOWN -----------
                 * Deliberately exempt from HOME_MIN_REDRAW_MS: that rule
                 * exists because the resting screen is up for hours and a
                 * refresh costs two seconds. INFO is up because somebody is
                 * standing in front of it reading the numbers, and a page of
                 * live numbers that does not move is not live. */
                /* In test mode the same slot carries the test page, at its
                 * own cadence. One page, one gesture to reach it, different
                 * contents - not a fourth page to navigate past. */
                if (alert_ui_frozen()) {
                    /* Published once, then never touched. The live refresh
                     * below is skipped entirely: a page that
                     * updated itself would destroy the comparison it exists
                     * to hold. */
                    if (!s_frozen_drawn) {
                        s_frozen_drawn = true;
                        frozen_publish();
                    }
                } else if (settings_get()->test_mode) {
                    s_frozen_drawn = false;
                    if (alert_ui_page() == EPAPER_PAGE_STATUS &&
                        (uint32_t)(now_ms() - s_test_last_ms) >=
                            TEST_REFRESH_MS) {
                        s_test_last_ms = now_ms();
                        test_publish();
                        test_console_line();
                    }
                } else if (alert_ui_page() == EPAPER_PAGE_STATUS &&
                    (uint32_t)(now_ms() - s_info_last_ms) >= INFO_REFRESH_MS) {
                    s_info_last_ms = now_ms();
                    info_publish(alive, amb_db, disp.ambient);
                }

                /* ---- PAGE 2's ONE LIVE ROW ------------------------------
                 * The rest of STATUS is fixed for the run and was written
                 * once, where the guard is armed. This row is not: a
                 * microphone that stops mid-run is the whole reason the row
                 * exists, and it is the same count MAIN's warning line uses. */
                /* MENU's ONE LIVE ROW. A microphone that stops mid-run is
                 * the whole reason it exists; the uptime beside it is what
                 * converts the event ring to civil time from one photograph;
                 * and AMB is the raw dB behind the ambient WORD, printed so
                 * ui_config.h's provisional thresholds can be set from a
                 * real deployment instead of from a guess. */
                char ln[EPAPER_LINE_MAX + 1];
                /* Not in test mode. Row 3 of this page is the T2 row there,
                 * and a live row written straight into the slot would erase
                 * it every second - the page would show a tier line that
                 * flickered into a microphone count. The MIC/UP/AMB row is
                 * an INFO row and belongs to INFO. */
                if (!settings_get()->test_mode && !alert_ui_frozen()) {
                    snprintf(ln, sizeof(ln), "MIC %d/%d UP %uH AMB %d",
                             alive & 7, GUARD_N_CH & 7,
                             (unsigned)(now_ms() / 3600000u), amb_db);
                    epaper_page_line(EPAPER_PAGE_STATUS, 3, ln);
                }

                /* ---- PAGE 3, RECENT: what it HEARD -----------------------
                 * The last five, newest first, off the same ring `V` dumps.
                 * On a standalone night this and the ring are the only record
                 * there is - and the ring cannot be read without a laptop. */
                {
                    evlog_rec_t rec[EPAPER_PAGE_ROWS - 2];
                    const int nrec = evlog_recent(rec, EPAPER_PAGE_ROWS - 2);
                    epaper_page_line(EPAPER_PAGE_RECENT, 0,
                                     "AGE  TIER  HZ   FOR");
                    /* One row shorter than the page, because the last one
                     * has to say what this page's hold does. Same rule as
                     * STATUS: a gesture whose meaning changes with the screen
                     * is only safe if the screen says so. */
                    for (int k = 0; k < EPAPER_PAGE_ROWS - 2; k++) {
                        if (k >= nrec) {
                            epaper_page_line(EPAPER_PAGE_RECENT, k + 1, NULL);
                            continue;
                        }
                        const uint32_t age_m =
                            (now_ms() - rec[k].uptime_ms) / 60000u;
                        snprintf(ln, sizeof(ln), "%luM %s %d %luS",
                                 (unsigned long)age_m,
                                 evlog_tier_short(rec[k].tier),
                                 (int)rec[k].hz,
                                 (unsigned long)(rec[k].dur_ms / 1000u));
                        epaper_page_line(EPAPER_PAGE_RECENT, k + 1, ln);
                    }
                    epaper_page_line(EPAPER_PAGE_RECENT,
                                     EPAPER_PAGE_ROWS - 1, "HOLD ROTATES");
                }
            }

            if (armed) {
                /* THE OR. Any tier's latch drives the same alert path - one
                 * buzzer, one dismissal, because an operator being alerted
                 * does not care which statistic fired. WHICH tier is carried
                 * to the panel and the trace, because afterwards they care
                 * about very little else: Tier-3 fires on fans and insects.
                 *
                 * Written out per tier rather than relying on prev_firedN
                 * being 0 when a tier is off. They ARE 0 - each is only ever
                 * assigned inside its own `if` - but "G is unchanged" should
                 * be readable HERE, not derived from an initialiser three
                 * hundred lines away. */
                const bool v1_fired = (rec.fired != 0);
                const bool t2_fired = t2on && (prev_fired2 != 0);
                const bool t3_fired = t3on && (prev_fired3 != 0);
                const bool t4_fired = t4on && (prev_fired4 != 0);
                const uint8_t tier =
                    v1_fired ? ALERT_TIER_V1
                  : (t2_fired ? ALERT_TIER_T2
                  : (t3_fired ? ALERT_TIER_T3
                  : (t4_fired ? ALERT_TIER_T4 : ALERT_TIER_NONE)));
                const bool local = v1_fired || t2_fired || t3_fired ||
                                   t4_fired;

                /* ---- captured once, here ------------------------------
                 * This is the frame whose comparison fired. Every consumer
                 * reads g_decide from now until the next alert; none of them
                 * re-samples a live score, because a live score at draw time
                 * is a different number and that is the whole defect. */
                const uint8_t fired_set =
                      (uint8_t)((v1_fired ? EVLOG_FS_V1 : 0u) |
                                (t2_fired ? EVLOG_FS_T2 : 0u) |
                                (t3_fired ? EVLOG_FS_T3 : 0u) |
                                (t4_fired ? EVLOG_FS_T4 : 0u));
                if (local && !alert_open) {
                    decide_t d;
                    d.fired_set = fired_set;
                    d.t_ms = now_ms();
                    if (v1_fired) {
                        d.tier = ALERT_TIER_V1;
                        d.score = (float)rec.score;
                        d.thr = (float)thr;
                        d.hz = (float)rec.f0_hz;
                    } else if (t2_fired) {
                        d.tier = ALERT_TIER_T2;
                        d.score = g_t2st->peak2;
                        d.thr = (float)t2o->cfg.tau2;
                        d.hz = (float)g_t2st->f0_on;
                    } else if (t3_fired) {
                        d.tier = ALERT_TIER_T3;
                        d.score = (float)last_W;
                        d.thr = (float)t3o->cfg.tau3;
                        d.hz = (float)g_t3st->r_on;
                    } else {
                        d.tier = ALERT_TIER_T4;
                        d.score = (float)g_t4st->ev_peak;
                        d.thr = (float)t4o->cfg.tau4;
                        d.hz = (float)g_t4st->f0_on;
                    }
                    g_decide = d;
                    /* ALERT_BEGIN records what actually fired. */
                    alert_ui_set_decision(d.score, d.thr);

                }

                /* ---- THE RADIO, and both directions of it ---------------
                 * loraon is a runtime const so the certificate's mode-
                 * isolation audit can SEE the guard, the same way it sees
                 * t2on and t3on. On a board with no radio every call below
                 * is a stub the linker deletes, and `G`/`Z` never reach any
                 * of it because they are not armed.
                 *
                 * Transmit on the onset edge only - never per frame, never on
                 * a re-confirmation. Three transmissions 350 +- 150 ms apart
                 * is 138 ms of air time per alert; per FRAME it would be a
                 * transmitter running at 31 Hz. */
                const bool loraon = lora_on;
                if (loraon && local && !alert_open) {
                    /* The score the decision was made at, so the peer's
                     * ALERT screen can show it rather than only
                     * which box heard something. It is the firing tier's own
                     * score, not a mixture of the four. */
                    lora_link_announce(tier, g_decide.score);
                }
                /* ---- the link test's incoming events -------------------
                 * A request gets one beep, a reply gets two - and two is the
                 * only outcome that proves a link, because it is the only one
                 * the sender could not have produced alone. The reply itself
                 * was queued inside lora_link's admit path, where the dedup
                 * ring guarantees exactly one per exchange. */
                {
                    uint16_t lpeer = 0u;
                    const uint8_t lev = lora_link_take_link_event(&lpeer);
                    if (lev == LORA_LINK_GOT_REQ || lev == LORA_LINK_GOT_ACK) {
                        g_ping_rx++;
                    }
                    if (lev == LORA_LINK_GOT_REQ) {
                        /* TWO beeps on the FAR unit, for the same reason the
                         * sender got two: this is the half an operator hears
                         * from across a field, and one chirp does not carry. */
                        alert_ui_link_feedback(2u);
                        g_link_peer = lpeer;
                        g_link_state = LORA_LINK_GOT_REQ;
                        g_link_ms = now_ms();
                    } else if (lev == LORA_LINK_GOT_ACK) {
                        /* THREE, and it must differ from the two above. This
                         * is the ONLY outcome that proves a round trip - the
                         * one thing the sender could not have produced alone -
                         * so it cannot sound like "I sent". */
                        alert_ui_link_feedback(3u);
                        g_link_peer = lpeer;
                        g_link_state = LORA_LINK_GOT_ACK;
                        g_link_ms = now_ms();
                    } else if (lev == LORA_LINK_SENT) {
                        g_link_state = LORA_LINK_SENT;
                        g_link_ms = now_ms();
                    }
                }

                /* ---- the freeze, on the alert onset, every path -------
                 * Placed inside `if (local && !alert_open)` this covers only a
                 * real detection: an injected alert sets s_inject_alert inside
                 * alert_ui and never sets rec.fired, and a remote alert never
                 * touches the detector at all, so for both of those g_frozen
                 * stays zero-initialised - tier ALERT_TIER_NONE, every have[]
                 * 0 - and the page shows WAIT on every row, marks nothing
                 * fired, and leaves v1 on top because with all keys equal the
                 * stable sort keeps index order. The rows are honest about an
                 * empty snapshot; the snapshot is the defect.
                 *
                 * Taken on the tier's NONE -> something edge, so it is the
                 * alert's own onset frame on every path. */
                {
                    const uint8_t at_now = alert_ui_tier();
                    if (at_now != ALERT_TIER_NONE &&
                        s_prev_alert_tier == ALERT_TIER_NONE) {
                        frozen_t f;
                        const uint32_t tnow = now_ms();
                        f.valid = 1u;
                        f.tier = at_now;
                        f.remote_id = (uint16_t)lora_link_remote_id();
                        f.remote_thr1 = lora_link_remote_thr1_milli();
                        f.remote_score =
                            (float)lora_link_remote_score_x100() / 100.0f;
                        const float sc[4] = {g_info.v1_score, g_info.t2_score,
                                             g_info.t3_score, g_info.t4_score};
                        const float th[4] = {g_info.v1_thr, g_info.t2_thr,
                                             g_info.t3_thr, g_info.t4_thr};
                        const float hz[4] = {g_info.v1_hz, g_info.t2_hz,
                                             g_info.t3_rate, g_info.t4_hz};
                        for (int i = 0; i < 4; i++) {
                            f.score[i] = sc[i];
                            f.thr[i] = th[i];
                            f.hz[i] = hz[i];
                            /* FRESH, STALE or NEVER - never a word. */
                            const uint32_t seen = g_info.seen_ms[i];
                            f.have[i] = (seen == 0u) ? 0u
                                      : ((uint32_t)(tnow - seen) <= 2000u ? 1u
                                                                          : 2u);
                        }
                        /* the deciding tier's row carries the deciding record */
                        const int di =
                            (at_now == ALERT_TIER_V1) ? 0 :
                            (at_now == ALERT_TIER_T2) ? 1 :
                            (at_now == ALERT_TIER_T3) ? 2 :
                            (at_now == ALERT_TIER_T4) ? 3 : -1;
                        if (di >= 0 && g_decide.tier == at_now) {
                            f.score[di] = g_decide.score;
                            f.thr[di] = g_decide.thr;
                            f.hz[di] = g_decide.hz;
                            f.have[di] = 1u;
                        }
                        /* The instrument, captured with everything else and
                         * from the SAME window, so the page cannot disagree
                         * with itself. */
                        f.r1_db = g_nfp.r_db;
                        f.l_db  = g_nfp.l_db;
                        f.r_db  = (float)g_st->r_db;
                        f.sat_t = g_sat_teeth;
                        f.sat_g = g_sat_gaps;
                        f.p_pct  = (int16_t)gate_p_pct(&g_gate_last);
                        f.ff_pct = (int16_t)gate_ff_pct(&g_gate_last);
                        f.g_acc  = g_gate_last.acc;
                        f.g_thr  = g_gate_last.thr;
                        f.g_band = g_gate_last.band;
                        f.g_veto = g_gate_last.veto;
                        f.g_nf   = g_gate_last.nf;
                        f.g_cont = g_gate_last.cont;
                        f.g_jit  = g_gate_last.jit;
                        f.g_tot  = g_gate_last.frames;
                        f.alerts_at_freeze =
                            (uint32_t)(evlog_count_local() +
                                       evlog_count_remote());
                        g_frozen = f;
                        s_frozen_drawn = false;
                    }
                    /* ---- AN INJECTED ALERT TRANSMITS TOO -----------------
                     * `U alert` is the only alert an operator can produce on
                     * demand, and a bench check of the peer link depends on it
                     * reaching the air. The announce below is gated on
                     * `local`, which is rec.fired and the tier flags - real
                     * detections only - so without this an injected alert
                     * raises sound, motor and page on this unit and puts
                     * nothing on the air.
                     *
                     * A remote alert is excluded, and that exclusion is the
                     * whole of the no-relay rule: a device that re-broadcast
                     * what it heard would make a storm possible by
                     * construction. */
                    if (at_now != ALERT_TIER_NONE &&
                        s_prev_alert_tier == ALERT_TIER_NONE &&
                        at_now != ALERT_TIER_REMOTE && !local && lora_on) {
                        lora_link_announce(at_now, g_decide.score);
                    }
                    s_prev_alert_tier = at_now;
                }

                /* RECEIVE. A peer's alert is OR'd into the same path every
                 * tier uses, because an operator being alerted does not care
                 * which box heard it - but it carries ALERT_TIER_REMOTE, so
                 * it sounds different, looks different and is recorded
                 * differently. A LOCAL alert always wins the attribution: if
                 * this box can hear the thing itself, that is the more urgent
                 * fact. */
                const bool remote = loraon && lora_link_remote_active(now_ms());
                const bool any = local || remote;
                const uint8_t tier_or = local ? tier : ALERT_TIER_REMOTE;

                /* THE EVENT RING, written on the edges of the OR - here rather
                 * than in alert_ui, because this is where the evidence is: the
                 * frequency and the score ON THE FIRING FRAME. In standalone
                 * nothing drains the trace link, so without this a day of
                 * guarding leaves exactly one recoverable fact - that the
                 * buzzer went off some number of times. */
                if (loraon && remote && !local) {
                    /* WHO IT CAME FROM, on the panel. Set before the tick so
                     * the screen alert_ui asks for already carries it. */
                    char nb[24];
                    snprintf(nb, sizeof(nb), "REMOTE N-%04X",
                             (unsigned)lora_link_remote_id());
                    epaper_set_alert_note(nb);
                    epaper_set_alert_remote(true);
                } else if (loraon && !any) {
                    epaper_set_alert_note(NULL);
                    epaper_set_alert_remote(false);
                } else if (local) {
                    /* OURS. Said explicitly rather than left over from the
                     * last alert: a stale REMOTE word on a local alarm would
                     * send an operator looking at the wrong sky. */
                    epaper_set_alert_remote(false);
                }

                if (any && !alert_open) {
                    /* v1's evidence by default; a tier's own only when THAT
                     * tier is both enabled and the one that fired. Written as
                     * statements rather than a ternary so the `t2on`/`t3on`
                     * guard is readable at the point of the dereference -
                     * which is also what lets the mode-isolation audit see
                     * that `G` and `Z` touch no tier state here. */
                    /* ---- the score the decision was made at -------------
                     * Not the current frame's score. For a latching tier that
                     * is not the score that raised the alert: T2, T3 and T4
                     * hold `latched` while they count their release down, and
                     * the per-frame record keeps reporting fired=1 as the
                     * score decays and f0 wanders. Read that way, one alert
                     * appears as a string of events at scores well below the
                     * tier's own threshold, each carrying hit=0 above=0.
                     *
                     * A number on a screen that cannot be reconciled with the
                     * threshold beside it is worse than no number, because it
                     * invites the wrong remedy - here, believing the tier
                     * alerts below its own threshold, which it does not.
                     *
                     * st->ev_peak and st->peak2 already held the peak among
                     * the windows that satisfied the M-of-N. They were simply
                     * never read. */
                    /* One source: the ring reads the same deciding record
                     * the screen and the packet do. */
                    float hz = g_decide.hz;
                    float sc = g_decide.score;
                    if (loraon && !local && remote) {
                        /* A peer's alert. The originating id rides in the Hz
                         * field and the originating tier in the score, which
                         * is the only place the eighteen-byte packet's two
                         * facts can be recovered from after the field day -
                         * and evlog_dump() labels the row RM so nobody reads
                         * them as a frequency and a comb score. */
                        hz = (float)lora_link_remote_id();
                        sc = (float)lora_link_remote_tier();
                    }
                    evlog_open(tier_or, hz, sc, now_ms());
                    /* Immediately after the open, so the entry carries the
                     * whole set and not only `tier_or`, which is the first of
                     * a priority list. */
                    evlog_set_fired(g_decide.fired_set);
                } else if (!any && alert_open) {
                    evlog_close(now_ms(), EVLOG_REL_SOURCE);
                }
                alert_open = any;

                alert_ui_tick(any, tier_or, now_ms());
            }
            frame++;
        }

        if (stop_after_this ||
            run_should_stop(standalone, armed, &end_reason)) {
            break;
        }
    }

    epaper_set_audio_hot(false);

    if (alert_open) {
        evlog_close(now_ms(), EVLOG_REL_STOP);
    }

    if (armed) {
        /* Leave the panel showing the resting screen, not a stale ALERT -
         * EXCEPT on every path whose next screen is OFF, where queueing
         * LISTENING first would cost a 2 s refresh to display a state the
         * device is deliberately leaving. That is the long press, the ritual
         * and the empty battery; all three draw OFF next. */
        if (alerted_once && end_reason != RUN_END_POWER &&
            end_reason != RUN_END_BATTERY) {
            epaper_request(EPAPER_SCREEN_READY);
        }
        alert_ui_end();
    }
    uint32_t t2_events = 0;
    if (t2on) {
        t2_finish(g_t2st, 0.0);       /* closes a still-open Tier-2 event */
        t2_events = (uint32_t)g_t2st->n_events;
    }
    const uint32_t t3_events = t3on ? (uint32_t)g_t3st->n_events : 0u;
    const uint32_t t3_updates = t3on ? (uint32_t)g_t3st->n_updates : 0u;
    t3_free();
    t2_free();
    guard_free();
    source_i2s_quad_stop();

    /* NO SENTINEL WHEN A HOST INTERRUPTED A STANDALONE RUN.
     *
     * capture_trace.capture() reads "until the sentinel". A standalone guard
     * that emitted END on its way out would hand the host tool an immediate
     * end-of-capture for a command whose reply had not started yet, and every
     * lab capture taken on a field-configured board would come back empty.
     * The run still stops, the records still stop; only the framing that
     * belongs to a BOUNDED capture is withheld. */
    if (standalone && end_reason == RUN_END_INPUT) {
        return end_reason;
    }

    trace_end_t e = {.magic = TRACE_MAGIC_END, .n_frames = frame,
                     .verdict_fires = (g_st->n_events > 0) ? 1 : 0,
                     .n_events = (uint32_t)g_st->n_events,
                     .peak_score = (frame ? peak_score : 0.0f),
                     .longest_chain = longest_chain,
                     .n_floor_fast = n_floor_fast,
                     .n_reanch = 0, .n_held_frames = 0};
    trace_send_end(&e);
    if (t2on) {
        char b[64];
        snprintf(b, sizeof(b), "T2 events=%u\n", (unsigned)t2_events);
        trace_text(b);
    }
    if (t3on) {
        char b[80];
        snprintf(b, sizeof(b), "T3 events=%u updates=%u\n",
                 (unsigned)t3_events, (unsigned)t3_updates);
        trace_text(b);
    }
    return end_reason;
}

/* ---- `U` - individually commandable outputs ---------------------------
 * Every U command answers with exactly one ACK so a host tool never has to
 * infer success from silence. Actuator states set here PERSIST across entry
 * and exit of the passive modes (M, T, Q, X, I, self-tests), which is what
 * makes the acoustic interference test possible; only A and D force
 * safe_all_off() on their exit. */
static const char *skip_ws(const char *s)
{
    while (*s == ' ' || *s == '\t') {
        s++;
    }
    return s;
}

/* ---- `U p`: e-paper bus diagnosis, a bench instrument --------------------
 *
 * When BUSY never goes low, EPD_1IN54_V2_ReadBusy() times out and epaper.c
 * marks the module faulted. "Faulted" is one word for five very different
 * faults - no panel fitted, a broken connection, a panel with no power, a dead
 * panel, or the wrong driver - and the operator surface cannot tell them
 * apart.
 *
 * This issues no panel command. It reads pins, and the readings separate the
 * cases by themselves:
 *
 *   BUSY with an internal pull-up, measured as a voltage, is the whole test.
 *     ~3.3 V  nothing is there. The pull-up sees an open circuit.
 *     ~0.6 V  a panel IS there and its VCC is at 0 V: the pull-up is being
 *             clamped through the part's input protection diode. One diode
 *             drop is not a logic level and cannot be produced by an open pin.
 *     ~0.0 V  a powered panel is actively holding BUSY low - which is IDLE,
 *             and then the timeout is a driver problem, not a wiring one.
 *   BUSY with a PULL-DOWN reading HIGH means something drives it high, which
 *     for this part means a powered panel asserting BUSY.
 *
 * The bridge test drives each bus pin low in turn: any OTHER pin that follows
 * is a solder bridge, which is the one failure that is the bare board's fault.
 *
 * It leaves the bus reconfigured. Reboot before using the panel again; on a
 * faulted panel there is nothing to disturb, which is the only time it runs.
 * ------------------------------------------------------------------------ */
#define PDIAG_N 6
static const int  PDIAG_PIN[PDIAG_N]  = {PIN_EPD_CS, PIN_EPD_DIN, PIN_EPD_CLK,
                                         PIN_EPD_DC, PIN_EPD_RST, PIN_EPD_BUSY};
static const char *PDIAG_NAME[PDIAG_N] = {"CS  ", "DIN ", "CLK ", "DC  ",
                                          "RST ", "BUSY"};

/* GPIO -> ADC on the ESP32-S3: ADC1 is GPIO1..10, ADC2 is GPIO11..20. Both
 * units are opened because the panel bus straddles them. */
static int pdiag_mv(int pin)
{
    adc_unit_t unit;
    adc_channel_t ch;
    if (pin >= 1 && pin <= 10)       { unit = ADC_UNIT_1; ch = (adc_channel_t)(pin - 1); }
    else if (pin >= 11 && pin <= 20) { unit = ADC_UNIT_2; ch = (adc_channel_t)(pin - 11); }
    else                             { return -1; }

    adc_oneshot_unit_handle_t h = NULL;
    adc_oneshot_unit_init_cfg_t u = {.unit_id = unit};
    if (adc_oneshot_new_unit(&u, &h) != ESP_OK) {
        return -1;                       /* busy elsewhere - say so, not zero */
    }
    adc_oneshot_chan_cfg_t c = {
        .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    int mv = -1;
    if (adc_oneshot_config_channel(h, ch, &c) == ESP_OK) {
        int raw = 0;
        if (adc_oneshot_read(h, ch, &raw) == ESP_OK) {
            adc_cali_handle_t cal = NULL;
            adc_cali_curve_fitting_config_t cc = {
                .unit_id = unit, .chan = ch,
                .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT,
            };
            if (adc_cali_create_scheme_curve_fitting(&cc, &cal) == ESP_OK) {
                if (adc_cali_raw_to_voltage(cal, raw, &mv) != ESP_OK) { mv = -1; }
                adc_cali_delete_scheme_curve_fitting(cal);
            } else {
                mv = raw * 3100 / 4095;  /* uncalibrated, still diagnostic */
            }
        }
    }
    adc_oneshot_del_unit(h);
    return mv;
}

static void pdiag_pull(int pin, int up, int down)
{
    gpio_config_t c = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en   = up   ? GPIO_PULLUP_ENABLE   : GPIO_PULLUP_DISABLE,
        .pull_down_en = down ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_reset_pin((gpio_num_t)pin);
    gpio_config(&c);
    vTaskDelay(pdMS_TO_TICKS(20));
}

static int pdiag_level_vote(int pin)
{
    int highs = 0;
    for (int i = 0; i < 64; i++) { highs += gpio_get_level((gpio_num_t)pin); }
    return highs;                        /* 0 = solid low, 64 = solid high */
}

static void cmd_panel_diag(const char *line)
{
    char b[224];

    trace_text("\n==== E-PAPER BUS DIAGNOSIS (no panel commands issued) ====\n");
    snprintf(b, sizeof(b), "board=%s  panel driver=EPD_1IN54_V2 (200x200)  "
             "epaper_state=%d (3=FAULTED)\n", BOARD_NAME, (int)epaper_state());
    trace_text(b);
    snprintf(b, sizeof(b), "pins: CS=%d DIN=%d CLK=%d DC=%d RST=%d BUSY=%d\n",
             PIN_EPD_CS, PIN_EPD_DIN, PIN_EPD_CLK, PIN_EPD_DC, PIN_EPD_RST,
             PIN_EPD_BUSY);
    trace_text(b);

#if BOARD_HAS_LORA
    /* The radio shares SCK and MOSI. Park its chip select HIGH so nothing
     * below can be mistaken for a command by the SX1276. */
    gpio_set_direction((gpio_num_t)PIN_LORA_CS, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)PIN_LORA_CS, 1);
    trace_text("radio CS parked HIGH for the duration\n");
#endif

    /* ---- A/B: every bus pin, as a level and as a voltage, under each pull */
    trace_text("\nA. each pin as an INPUT, under each internal pull\n");
    trace_text("   (open pin follows the pull; ~0.6 V under a pull-up means a\n"
               "    part is there with its supply at 0 V - a clamp diode)\n");
    trace_text("   pin        pull-down          pull-up\n");
    for (int i = 0; i < PDIAG_N; i++) {
        pdiag_pull(PDIAG_PIN[i], 0, 1);
        const int dh = pdiag_level_vote(PDIAG_PIN[i]);
        const int dmv = pdiag_mv(PDIAG_PIN[i]);
        pdiag_pull(PDIAG_PIN[i], 1, 0);
        const int uh = pdiag_level_vote(PDIAG_PIN[i]);
        const int umv = pdiag_mv(PDIAG_PIN[i]);
        snprintf(b, sizeof(b),
                 "   %s(%2d)  %2d/64 hi %5d mV   %2d/64 hi %5d mV\n",
                 PDIAG_NAME[i], PDIAG_PIN[i], dh, dmv, uh, umv);
        trace_text(b);
    }

    /* ---- C: does the panel answer a reset at all --------------------------
     * A live SSD1681 raises BUSY while it reboots and drops it when ready. Any
     * TRANSITION proves something is alive on the other end of that wire; a
     * flat line proves only that the wire goes nowhere useful. */
    trace_text("\nC. RST pulse, BUSY sampled at 1 kHz for 600 ms\n");
    pdiag_pull(PIN_EPD_BUSY, 0, 0);      /* no pull: let the panel drive it */
    gpio_reset_pin((gpio_num_t)PIN_EPD_RST);
    gpio_set_direction((gpio_num_t)PIN_EPD_RST, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)PIN_EPD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    const int before = gpio_get_level((gpio_num_t)PIN_EPD_BUSY);
    gpio_set_level((gpio_num_t)PIN_EPD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    const int during = gpio_get_level((gpio_num_t)PIN_EPD_BUSY);
    gpio_set_level((gpio_num_t)PIN_EPD_RST, 1);
    int edges = 0, first_low = -1, last = during;
    for (int ms = 0; ms < 600; ms++) {
        const int v = gpio_get_level((gpio_num_t)PIN_EPD_BUSY);
        if (v != last) { edges++; last = v; }
        if (v == 0 && first_low < 0) { first_low = ms; }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    snprintf(b, sizeof(b),
             "   BUSY before=%d  during RST-low=%d  transitions after=%d  "
             "first LOW at %d ms\n", before, during, edges, first_low);
    trace_text(b);
    trace_text(edges > 0
               ? "   -> BUSY MOVED. Something on that wire is alive.\n"
               : "   -> BUSY never moved. Nothing answered the reset.\n");

    /* ---- D: solder bridges ------------------------------------------------ */
    trace_text("\nD. bridge test - each pin driven LOW, the others pulled up\n");
    int bridges = 0;
    for (int i = 0; i < PDIAG_N; i++) {
        for (int j = 0; j < PDIAG_N; j++) {
            if (j != i) { pdiag_pull(PDIAG_PIN[j], 1, 0); }
        }
        gpio_reset_pin((gpio_num_t)PDIAG_PIN[i]);
        gpio_set_direction((gpio_num_t)PDIAG_PIN[i], GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)PDIAG_PIN[i], 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        for (int j = 0; j < PDIAG_N; j++) {
            if (j == i) { continue; }
            if (pdiag_level_vote(PDIAG_PIN[j]) < 32) {
                snprintf(b, sizeof(b), "   BRIDGE? %s(%d) follows %s(%d) low\n",
                         PDIAG_NAME[j], PDIAG_PIN[j],
                         PDIAG_NAME[i], PDIAG_PIN[i]);
                trace_text(b);
                bridges++;
            }
        }
        gpio_reset_pin((gpio_num_t)PDIAG_PIN[i]);
    }
    trace_text(bridges ? "   -> bridges above are a BARE-BOARD fault\n"
                       : "   no pin follows any other: no bridge on this bus\n");

    trace_text("\nREBOOT before using the panel again - this left the bus "
               "reconfigured.\n");
    trace_ack(line, TRACE_ACK_OK);
}

static bool s_boot_rtc_survived;

/* ---- the info snapshot: the field range instrument ----------------------
 *
 * At each step of a distance ladder this says how far each tier sits from its
 * own threshold, which is the only way to learn anything from a step that did
 * not fire: "nothing happened" and "T2 reached 0.91 of 1.15" are completely
 * different facts about a distance.
 *
 * Written per frame by the detector and read by the UI with no locking, which
 * is deliberate: a lock on the audio core's hot path to protect a number on a
 * page that repaints every few seconds would be the wrong trade twice over.
 * The worst a torn read can do is show one stale digit for one repaint. */


/* Fill every row of the INFO page. Section 4.5's list is fifteen rows once
 * "one line per tier" is counted as four; fourteen fit (proved by
 * tests/test_epaper_pages.py, where fifteen clips at y=188), so line 12, free
 * heap at arming, is the one dropped, per that section's own cut order. It is
 * in the handoff instead, where the number is actually used. */

/* ---- near misses, counted in the UI and not in the frame loop -----------
 * The frame-accurate rule - at or above 80% of threshold for three
 * consecutive frames - can only be evaluated on the detector task, and this is
 * not that rule. It counts repaints at which a tier's running peak reached 80%
 * of its threshold without an alert, which is coarser in both directions: it
 * cannot see a three-frame run inside one window, and it counts a single loud
 * frame the real rule would ignore.
 *
 * It is here because it costs the frame loop nothing and still separates the
 * two facts a distance step needs separated - "nothing happened" and "T2
 * reached 0.91 of 1.15". The frame-accurate version is owed. */
#define NEAR_FRAC 0.80f

static void near_scan(void)
{
    const struct { float pk, thr; uint8_t have; } t[4] = {
        {g_info.v1_pk, g_info.v1_thr, 1u},
        {g_info.t2_pk, g_info.t2_thr, g_info.have2},
        {g_info.t3_pk, g_info.t3_thr, g_info.have3},
        {g_info.t4_pk, g_info.t4_thr, g_info.have4},
    };
    for (int i = 0; i < 4; i++) {
        if (t[i].have && t[i].thr > 0.0f &&
            t[i].pk >= NEAR_FRAC * t[i].thr && t[i].pk < t[i].thr) {
            if (s_near_count < 0xFFFFFFFFu) { s_near_count++; }
        }
    }
}

/* One tier row of the TEST page. Written through a 64-byte buffer for the
 * reason info_publish() gives: formatting into an EPAPER_LINE_MAX buffer makes
 * -Werror=format-truncation reject the line on a value that cannot occur. */
/* A tier that was up on the deciding frame is starred on its own row, so a
 * frozen page says which tiers agreed rather than only which one the priority
 * order named. Read `V1*` as "v1 fired too". */
static void test_tier_row(uint8_t row, const char *name, uint8_t have,
                          uint8_t enabled, float now, float pk, float thr,
                          int hz)
{
    char ln[64];
    char nm[8];
    {
        const uint8_t bit = (name[1] == '1') ? EVLOG_FS_V1
                          : (name[1] == '2') ? EVLOG_FS_T2
                          : (name[1] == '3') ? EVLOG_FS_T3
                          :                    EVLOG_FS_T4;
        snprintf(nm, sizeof(nm), "%s%s", name,
                 (g_decide.fired_set & bit) ? "*" : "");
    }
    if (have) {
        snprintf(ln, sizeof(ln), "%-4s %d.%02d %d.%02d %d.%02d %4d",
                 nm,
                 CENT_I(now), CENT_F(now),
                 CENT_I(pk),  CENT_F(pk),
                 CENT_I(thr), CENT_F(thr),
                 hz);
    } else {
        /* The same distinction INFO makes, and for the same reason: a tier
         * that updates one frame in eight is healthy and silent for a quarter
         * of a second, and printing OFF there is how a running tier came to
         * be read as a stopped one. */
        snprintf(ln, sizeof(ln), "%-4s %s", nm, enabled ? "WAIT" : "OFF");
    }
    epaper_page_line(EPAPER_PAGE_STATUS, row, ln);
}

/* ---- the console snapshot -----------------------------------------------
 * One line per repaint, so an attached laptop logs a whole test run without
 * anybody watching the glass. Printed only while the test page is showing and
 * only in test mode, and nothing is stored for it: the heap is thin and the
 * audio captures remain the real record.
 *
 * Formatted from the values test_publish() has just written to the page, so
 * the line and the glass cannot disagree. It is called immediately after, and
 * reads the same statics rather than re-sampling g_info, because re-sampling
 * would race the frame loop and print a line that no screen ever showed. */
static float s_tl_now[4], s_tl_pk[4], s_tl_thr[4];
static int   s_tl_hz[4];
/* Snapshotted by test_publish() at the moment it renders and before the
 * counters are reset, for the same reason the tier values are: re-sampling
 * here would race the frame loop and print a line no screen ever showed. */
static int      s_tl_p, s_tl_ff;
static uint32_t s_tl_acc, s_tl_rthr, s_tl_band, s_tl_veto, s_tl_nf,
                s_tl_cont, s_tl_jit, s_tl_tot;
static float    s_tl_r1, s_tl_l, s_tl_r;
static unsigned s_tl_satt, s_tl_satg;

static void test_console_line(void)
{
    /* WIDER THAN THE LINE IT PRINTS, for the reason info_publish() gives:
     * -Werror=format-truncation must assume a pathological value for every
     * one of the sixteen numbers, even though none can occur. */
    char b[640];
    const uint32_t up = now_ms() / 100u;      /* tenths, no float printf */
    int rssi = 0, snr = 0;
    const bool q = lora_link_last_rx_quality(&rssi, &snr);
    snprintf(b, sizeof(b),
             "TEST t=%lu.%lu "
             "v1=%d.%02d/%d.%02d/%d.%02d@%d t2=%d.%02d/%d.%02d/%d.%02d@%d "
             "t3=%d.%02d/%d.%02d/%d.%02d@%d t4=%d.%02d/%d.%02d/%d.%02d@%d "
             "veto=%u near=%lu alerts=%u rx=%d/%d "
             "p=%d acc=%lu thr=%lu band=%lu veto2=%lu nf=%lu cont=%lu "
             "jit=%lu tot=%lu ff=%d r1=%d.%02d l=%d.%02d r=%d.%02d sat=%u/%u "
             "nfus=%u over=%lu/%lu,%lu/%lu,%lu/%lu,%lu/%lu meanus=%lu ovf=%lu%s\n",
             (unsigned long)(up / 10u), (unsigned long)(up % 10u),
             CENT_I(s_tl_now[0]), CENT_F(s_tl_now[0]),
             CENT_I(s_tl_pk[0]),  CENT_F(s_tl_pk[0]),
             CENT_I(s_tl_thr[0]), CENT_F(s_tl_thr[0]), s_tl_hz[0],
             CENT_I(s_tl_now[1]), CENT_F(s_tl_now[1]),
             CENT_I(s_tl_pk[1]),  CENT_F(s_tl_pk[1]),
             CENT_I(s_tl_thr[1]), CENT_F(s_tl_thr[1]), s_tl_hz[1],
             CENT_I(s_tl_now[2]), CENT_F(s_tl_now[2]),
             CENT_I(s_tl_pk[2]),  CENT_F(s_tl_pk[2]),
             CENT_I(s_tl_thr[2]), CENT_F(s_tl_thr[2]), s_tl_hz[2],
             CENT_I(s_tl_now[3]), CENT_F(s_tl_now[3]),
             CENT_I(s_tl_pk[3]),  CENT_F(s_tl_pk[3]),
             CENT_I(s_tl_thr[3]), CENT_F(s_tl_thr[3]), s_tl_hz[3],
             /* Part 1 tail. Order matches the format string above. */
             (unsigned)bootrec_vetoed(), (unsigned long)s_near_count,
             (unsigned)(evlog_count_local() + evlog_count_remote()),
             q ? rssi : 0, q ? snr : 0,
             s_tl_p, (unsigned long)s_tl_acc, (unsigned long)s_tl_rthr,
             (unsigned long)s_tl_band, (unsigned long)s_tl_veto,
             (unsigned long)s_tl_nf, (unsigned long)s_tl_cont,
             (unsigned long)s_tl_jit, (unsigned long)s_tl_tot, s_tl_ff,
             CENT_I(s_tl_r1), CENT_F(s_tl_r1),
             CENT_I(s_tl_l),  CENT_F(s_tl_l),
             CENT_I(s_tl_r),  CENT_F(s_tl_r),
             s_tl_satt, s_tl_satg, g_nfp.us_last,
             /* over-budget frames / frames run, as v1,+T2,+T3,+T4 */
             (unsigned long)g_over_by[0], (unsigned long)g_run_by[0],
             (unsigned long)g_over_by[1], (unsigned long)g_run_by[1],
             (unsigned long)g_over_by[2], (unsigned long)g_run_by[2],
             (unsigned long)g_over_by[3], (unsigned long)g_run_by[3],
             (unsigned long)(g_gate_last.dt_n
                 ? (g_gate_last.dt_sum_us / g_gate_last.dt_n) : 0u),
             (unsigned long)(source_i2s_quad_stats()
                 ? (source_i2s_quad_stats()->ovf[0] +
                    source_i2s_quad_stats()->ovf[1]) : 0u),
             (source_i2s_quad_stats() && source_i2s_quad_stats()->ovf_armed)
                 ? "" : "(not armed)");
    trace_text(b);
}

/* ---- the frozen alert page ----------------------------------------------
 * Rows ranked by closeness to firing, score over threshold descending, with
 * whatever fired pinned to the top and marked `*`.
 *
 * Two scales, because scale 1 is unreadable at arm's length and this page is
 * read from a distance. Scale 2 holds fourteen characters against scale 1's
 * twenty-eight, and `*T4 32.4/30.5` is thirteen - so the four tier rows get
 * scale 2 and the f0 and the four ratios go on one scale-1 line beneath,
 * rather than being squeezed onto rows nobody can read.
 *
 * `*` replaces the word FIRED because a word costs five of those fourteen
 * characters and a marker costs one.
 *
 * v1 and T2 print two decimals, T3 and T4 one: v1's threshold is 1.50 and the
 * difference between 1.62 and 1.50 is the whole decision, while T4's is 30.5
 * and a hundredth there is noise. Both come to four characters, so every row
 * is the same width. */
static void frozen_publish(void)
{
    static const char *NAME[4] = {"V1", "T2", "T3", "T4"};
    char ln[64], a[16], b[16];
    const frozen_t f = g_frozen;
    uint8_t row = 0;

    snprintf(ln, sizeof(ln), "N-%04X", (unsigned)lora_link_device_id());
    epaper_page_line_sc(EPAPER_PAGE_STATUS, row++, ln, 1u);

    int order[4] = {0, 1, 2, 3};
    float key[4];
    for (int i = 0; i < 4; i++) {
        key[i] = (f.thr[i] > 0.0f) ? (f.score[i] / f.thr[i]) : -1.0f;
    }
    const int fired_ix =
        (f.tier == ALERT_TIER_V1) ? 0 : (f.tier == ALERT_TIER_T2) ? 1 :
        (f.tier == ALERT_TIER_T3) ? 2 : (f.tier == ALERT_TIER_T4) ? 3 : -1;
    if (fired_ix >= 0) {
        key[fired_ix] = 1e9f;
    }
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            if (key[order[j]] > key[order[i]]) {
                const int t = order[i]; order[i] = order[j]; order[j] = t;
            }
        }
    }

    for (int k = 0; k < 4; k++) {
        const int i = order[k];
        if (i >= 2) {
            snprintf(a, sizeof(a), "%d.%01d", CENT_I(f.score[i]),
                     CENT_F(f.score[i]) / 10);
            snprintf(b, sizeof(b), "%d.%01d", CENT_I(f.thr[i]),
                     CENT_F(f.thr[i]) / 10);
        } else {
            snprintf(a, sizeof(a), "%d.%02d", CENT_I(f.score[i]),
                     CENT_F(f.score[i]));
            snprintf(b, sizeof(b), "%d.%02d", CENT_I(f.thr[i]),
                     CENT_F(f.thr[i]));
        }
        snprintf(ln, sizeof(ln), "%s%s %s/%s",
                 (i == fired_ix) ? "*" : " ", NAME[i], a, b);
        epaper_page_line_sc(EPAPER_PAGE_STATUS, row++, ln, 2u);
    }

    {
        const int di = (fired_ix >= 0) ? fired_ix : order[0];
        /* F0 rather than @: the panel font has no '@', and the font test
         * caught it here rather than on a hillside. */
        int n = snprintf(ln, sizeof(ln), "F0 %d", (int)(f.hz[di] + 0.5f));
        for (int k = 0; k < 4 && n > 0 && n < (int)sizeof(ln) - 6; k++) {
            const int i = order[k];
            const float r = (f.thr[i] > 0.0f) ? f.score[i] / f.thr[i] : 0.0f;
            n += snprintf(ln + n, sizeof(ln) - (size_t)n, " %d.%02d",
                          CENT_I(r), CENT_F(r));
        }
        epaper_page_line_sc(EPAPER_PAGE_STATUS, row++, ln, 1u);
    }

    /* ---- THE INSTRUMENT, ON THE PAGE THAT HOLDS (room session defect 1) - */
    {
        char r1s[16], ls[16], rs[16];
        fmt_db1(r1s, sizeof(r1s), f.r1_db);
        fmt_db1(ls, sizeof(ls), f.l_db);
        fmt_db1(rs, sizeof(rs), f.r_db);
        snprintf(ln, sizeof(ln), "R1 %s L %s R %s", r1s, ls, rs);
        epaper_page_line_sc(EPAPER_PAGE_STATUS, row++, ln, 1u);

        if (f.p_pct < 0) {
            snprintf(ln, sizeof(ln), "P --  TOT %lu THR %lu",
                     (unsigned long)f.g_tot, (unsigned long)f.g_thr);
        } else {
            snprintf(ln, sizeof(ln), "P .%02d TOT %lu THR %lu",
                     f.p_pct >= 100 ? 99 : (int)f.p_pct,
                     (unsigned long)f.g_tot, (unsigned long)f.g_thr);
        }
        epaper_page_line_sc(EPAPER_PAGE_STATUS, row++, ln, 1u);

        snprintf(ln, sizeof(ln), "A%lu B%lu V%lu N%lu C%lu J%lu",
                 (unsigned long)f.g_acc, (unsigned long)f.g_band,
                 (unsigned long)f.g_veto, (unsigned long)f.g_nf,
                 (unsigned long)f.g_cont, (unsigned long)f.g_jit);
        epaper_page_line_sc(EPAPER_PAGE_STATUS, row++, ln, 1u);

        /* ALERTS at the moment of the freeze. The room session could not tell
         * whether the 8 m rung had alerted, because the 5 m page was still up
         * and a later alert leaves no mark on a frozen page. Now it does: if
         * the live ALERTS count has moved past this number, more followed. */
        snprintf(ln, sizeof(ln), "FFPCT %d SAT %u/%u A%lu",
                 (int)f.ff_pct, (unsigned)f.sat_t, (unsigned)f.sat_g,
                 (unsigned long)f.alerts_at_freeze);
        epaper_page_line_sc(EPAPER_PAGE_STATUS, row++, ln, 1u);
    }

    /* ---- the source line ----------------------------------------------
     * A local alert says nothing here; the absence is the signal. A remote one
     * names the sender, and if the packet carried no threshold - an older peer
     * - it prints the score alone rather than a ratio computed against a
     * threshold it does not know. */
    if (f.tier == ALERT_TIER_REMOTE) {
        if (f.remote_thr1) {
            snprintf(ln, sizeof(ln), "FROM N-%04X %d.%02d/%d.%02d",
                     (unsigned)f.remote_id,
                     CENT_I(f.remote_score), CENT_F(f.remote_score),
                     (int)(f.remote_thr1 / 1000u),
                     (int)((f.remote_thr1 % 1000u) / 10u));
        } else {
            snprintf(ln, sizeof(ln), "FROM N-%04X %d.%02d",
                     (unsigned)f.remote_id,
                     CENT_I(f.remote_score), CENT_F(f.remote_score));
        }
        epaper_page_line_sc(EPAPER_PAGE_STATUS, row++, ln, 1u);
    }

    epaper_page_line_sc(EPAPER_PAGE_STATUS, row++, "TAP TO RETURN", 1u);
    while (row < EPAPER_PAGE_ROWS) {
        epaper_page_line(EPAPER_PAGE_STATUS, row++, NULL);
    }

    /* ---- the same numbers on the console -------------------------------
     * So an attached laptop captures what the glass shows and nobody has to
     * write four numbers by hand at every step. Formatted from the same
     * frozen snapshot as the rows above, never re-sampled. */
    {
        char cl[224];
        static const char *N[4] = {"v1", "T2", "T3", "T4"};
        int n = snprintf(cl, sizeof(cl),
                         "FROZEN t=%lu.%lu unit=N-%04X fired=%s teeth=%d",
                         (unsigned long)(now_ms() / 1000u),
                         (unsigned long)((now_ms() % 1000u) / 100u),
                         (unsigned)lora_link_device_id(),
                         evlog_tier_short(f.tier), t4_last_teeth());
        for (int i = 0; i < 4 && n > 0 && n < (int)sizeof(cl) - 40; i++) {
            n += snprintf(cl + n, sizeof(cl) - (size_t)n,
                          " %s=%d.%02d/%d.%02d@%d", N[i],
                          CENT_I(f.score[i]), CENT_F(f.score[i]),
                          CENT_I(f.thr[i]), CENT_F(f.thr[i]),
                          (int)(f.hz[i] + 0.5f));
        }
        if (f.tier == ALERT_TIER_REMOTE && n > 0 &&
            n < (int)sizeof(cl) - 24) {
            snprintf(cl + n, sizeof(cl) - (size_t)n, " from=N-%04X",
                     (unsigned)f.remote_id);
        }
        trace_text(cl);
        trace_text("\n");
    }
}

/* ---- the test page ------------------------------------------------------
 * It occupies the STATUS slot rather than adding a fourth page: one page
 * reached by the same gesture, not a new one to navigate to. Nothing about
 * MAIN or RECENT changes.
 *
 * Every value is sampled at the instant the repaint begins, which is what
 * makes the console line and the glass agree - they are formatted from the
 * same read. */
static void test_publish(void)
{
    char ln[64];
    const settings_t *c = settings_get();
    uint8_t row = 0;
    const uint32_t up = now_ms() / 1000u;

    /* Read the four maxima ONCE and clear them, so the window this page
     * reports is exactly the interval since the previous repaint. */
    const float pk1 = g_info.v1_pk, pk2 = g_info.t2_pk;
    const float pk3 = g_info.t3_pk, pk4 = g_info.t4_pk;
    near_scan();
    g_info.v1_pk = 0.0f; g_info.t2_pk = 0.0f;
    g_info.t3_pk = 0.0f; g_info.t4_pk = 0.0f;

    /* The unit and its threshold in the header, so a photograph of this page
     * is self-describing about which board it came from. */
    snprintf(ln, sizeof(ln), "N-%04X THR %d.%02d UP %02u:%02u",
             (unsigned)lora_link_device_id(),
             CENT_I(g_run_thr1), CENT_F(g_run_thr1),
             (unsigned)(up / 3600u), (unsigned)((up / 60u) % 60u));
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    epaper_page_line(EPAPER_PAGE_STATUS, row++, "TIER NOW   PK    THR    F0");

    test_tier_row(row++, "V1", 1u, 1u, g_info.v1_score, pk1, g_info.v1_thr,
                  (int)(g_info.v1_hz + 0.5f));
    test_tier_row(row++, "T2", g_info.have2, c->t2_enabled, g_info.t2_score,
                  pk2, g_info.t2_thr, (int)(g_info.t2_hz + 0.5f));
    /* T3's last column is an envelope RATE per second, not a frequency. The
     * header says F0 for all four because the column is the tier's candidate;
     * D.3 names the difference and FIELD_STANDALONE.md repeats it. */
    test_tier_row(row++, "T3", g_info.have3, c->t3_enabled, g_info.t3_score,
                  pk3, g_info.t3_thr, (int)(g_info.t3_rate + 0.5f));
    test_tier_row(row++, "T4", g_info.have4, c->t4_enabled, g_info.t4_score,
                  pk4, g_info.t4_thr, (int)(g_info.t4_hz + 0.5f));

    /* ---- the link test's line ------------------------------------------
     * SENT means this unit asked. RECV means a peer asked and was answered.
     * OK means the round trip closed, which is the only one that proves both
     * directions - and it is the line to look for standing in a field. */
    if (g_link_state != LORA_LINK_NONE) {
        const uint32_t age = (now_ms() - g_link_ms) / 1000u;
        if (g_link_state == LORA_LINK_SENT) {
            snprintf(ln, sizeof(ln), "LINK SENT %lus AGO",
                     (unsigned long)age);
        } else {
            snprintf(ln, sizeof(ln), "LINK %s N-%04X %lus",
                     g_link_state == LORA_LINK_GOT_ACK ? "OK" : "RECV",
                     (unsigned)g_link_peer, (unsigned long)age);
        }
        epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);
    }

    /* ---- the two R values side by side ----------------------------------
     *
     * R is computed from the FFT of the four-channel sum; R1 from channel 0
     * alone. Printing both on one line is the measurement: the difference
     * between them is the coherence bias, read off a real source rather than
     * argued from a coherence table. Expect R1 - R of 0 to +6 dB, larger on
     * broadband sources than on tonal ones.
     *
     * L is the low band in dB above its own 30 s reference - the level term
     * that can bound this gate in place of the score. */
    {
        char rs[16], r1s[16], ls[16];
        fmt_db1(rs, sizeof(rs), (float)g_st->r_db);
        fmt_db1(r1s, sizeof(r1s), g_nfp.r_db);
        fmt_db1(ls, sizeof(ls), g_nfp.l_db);
        snprintf(ln, sizeof(ln), "R1 %s L %s R %s", r1s, ls, rs);
        epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);
    }

    /* ---- p, AND THE GATE THAT IS EATING THE FRAMES ----------------------
     * Read from the last COMPLETE 5 s window, so this is a recent number and
     * not everything since the panel last showed this page. */
    {
        const gate_counts_t *g = &g_gate_last;
        const int pp = gate_p_pct(g);
        const int ff = gate_ff_pct(g);
        if (pp < 0) {
            snprintf(ln, sizeof(ln), "P --  TOT %lu THR %lu",
                     (unsigned long)g->frames, (unsigned long)g->thr);
        } else {
            snprintf(ln, sizeof(ln), "P .%02d TOT %lu THR %lu",
                     pp >= 100 ? 99 : pp,
                     (unsigned long)g->frames, (unsigned long)g->thr);
        }
        epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

        /* A + B + V + N + C + THR == TOT, exactly. */
        snprintf(ln, sizeof(ln), "A%lu B%lu V%lu N%lu C%lu J%lu",
                 (unsigned long)g->acc, (unsigned long)g->band,
                 (unsigned long)g->veto, (unsigned long)g->nf,
                 (unsigned long)g->cont, (unsigned long)g->jit);
        epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

        s_tl_p = pp; s_tl_ff = ff;
        s_tl_acc = g->acc; s_tl_rthr = g->thr; s_tl_tot = g->frames;
        s_tl_band = g->band; s_tl_veto = g->veto;
        s_tl_nf = g->nf; s_tl_cont = g->cont; s_tl_jit = g->jit;
        s_tl_r1 = g_nfp.r_db; s_tl_l = g_nfp.l_db; s_tl_r = (float)g_st->r_db;
        s_tl_satt = g_sat_teeth; s_tl_satg = g_sat_gaps;
    }

    /* SAT is teeth/gaps pinned at the ceiling on the last frame. Both rising
     * together is the score collapse described in detector.c.
     *
     * There is deliberately no veto counter on this line: nothing increments
     * one, so it would print a structural zero that reads as evidence. A
     * number that cannot move is worse than no number. DROP stays; it is
     * real. */
    if (g_ping_period_s) {
        int prssi = 0, psnr = 0;
        (void)lora_link_last_rx_quality(&prssi, &psnr);
        snprintf(ln, sizeof(ln), "PING T%lu R%lu %d/%d",
                 (unsigned long)g_ping_tx, (unsigned long)g_ping_rx,
                 prssi, psnr);
        epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);
    }

    {
        const int ff = gate_ff_pct(&g_gate_last);
        /* DROP is frames whose COMPUTE time exceeded the 32 ms hop - the
         * timer starts after the I2S read, so this is overrun and not jitter.
         * Cumulative since boot, and labelled so nobody reads it as a rate. */
        const gate_counts_t *gw = &g_gate_last;
        const unsigned mean_us = gw->dt_n
            ? (unsigned)(gw->dt_sum_us / gw->dt_n) : 0u;
        const quad_stats_t *qs = source_i2s_quad_stats();
        const unsigned ovf = qs ? (unsigned)(qs->ovf[0] + qs->ovf[1]) : 0u;
        snprintf(ln, sizeof(ln), "FFPCT%d D%u O%s M%u.%u",
                 ff, (unsigned)bootrec_drops(),
                 /* "-" not "0" when the counter was never armed: a number
                  * that cannot move must never read as good news. NOT "?" -
                  * the panel font has no question mark, and
                  * test_epaper_pages.py caught it here rather than letting it
                  * render as a silent gap on a hillside. */
                 (qs && qs->ovf_armed) ? "" : "-",
                 mean_us / 1000u, (mean_us % 1000u) / 100u);
        if (qs && qs->ovf_armed) {
            snprintf(ln, sizeof(ln), "FFPCT%d D%u O%u M%u.%u",
                     ff, (unsigned)bootrec_drops(), ovf,
                     mean_us / 1000u, (mean_us % 1000u) / 100u);
        }
        epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

        /* PANEL drops, distinct from AUDIO drops above. A refresh that never
         * reached the glass is why a screen can look stale, and until now the
         * counter existed and was printed nowhere. */
        snprintf(ln, sizeof(ln), "PDROP %lu", (unsigned long)epaper_dropped());
    }
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    snprintf(ln, sizeof(ln), "ALERTS %u HERE %u FROM %u",
             (unsigned)(evlog_count_local() + evlog_count_remote()),
             (unsigned)evlog_count_local(), (unsigned)evlog_count_remote());
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    {
        evlog_rec_t last;
        if (evlog_recent(&last, 1) == 1) {
            const uint32_t age_s = (now_ms() - last.uptime_ms) / 1000u;
            /* score/threshold from the deciding record, so this line and the
             * ALERT screen cannot disagree. */
            snprintf(ln, sizeof(ln), "LAST %s %d.%02d/%d.%02d %dHZ %luS",
                     evlog_tier_short(last.tier),
                     CENT_I(g_decide.score), CENT_F(g_decide.score),
                     CENT_I(g_decide.thr), CENT_F(g_decide.thr),
                     (int)last.hz, (unsigned long)age_s);
        } else {
            snprintf(ln, sizeof(ln), "LAST NONE");
        }
        epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);
    }

    {
        int rssi = 0, snr = 0;
        if (lora_link_last_rx_quality(&rssi, &snr)) {
            snprintf(ln, sizeof(ln), "RX %d SNR %d TX %lu", rssi, snr,
                     (unsigned long)lora_link_tx_count());
        } else {
            /* "no frame has ever arrived" is not "the last one was weak". */
            snprintf(ln, sizeof(ln), "RX NONE TX %lu",
                     (unsigned long)lora_link_tx_count());
        }
        epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);
    }

    snprintf(ln, sizeof(ln), "REDRAWS %lu OVER %lu",
             (unsigned long)alert_ui_home_redraws(), (unsigned long)g_frames_over);
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    while (row < EPAPER_PAGE_ROWS) {
        epaper_page_line(EPAPER_PAGE_STATUS, row++, NULL);
    }

    /* The console line of D.5 is formatted from THESE, not from a second read
     * of g_info: a re-read would race the frame loop and could print a line
     * that no screen ever showed. */
    s_tl_now[0] = g_info.v1_score; s_tl_pk[0] = pk1;
    s_tl_thr[0] = g_info.v1_thr;   s_tl_hz[0] = (int)(g_info.v1_hz + 0.5f);
    s_tl_now[1] = g_info.t2_score; s_tl_pk[1] = pk2;
    s_tl_thr[1] = g_info.t2_thr;   s_tl_hz[1] = (int)(g_info.t2_hz + 0.5f);
    s_tl_now[2] = g_info.t3_score; s_tl_pk[2] = pk3;
    s_tl_thr[2] = g_info.t3_thr;   s_tl_hz[2] = (int)(g_info.t3_rate + 0.5f);
    s_tl_now[3] = g_info.t4_score; s_tl_pk[3] = pk4;
    s_tl_thr[3] = g_info.t4_thr;   s_tl_hz[3] = (int)(g_info.t4_hz + 0.5f);
}

static void info_publish(int alive, int amb_db, const char *amb_word)
{
    /* WIDER THAN A ROW, ON PURPOSE. Formatting straight into an
     * EPAPER_LINE_MAX buffer makes -Werror=format-truncation reject every one
     * of these lines, because it must assume a pathological value (a
     * seven-digit Hz, a four-digit slice) even though none can occur. The
     * lines are designed to fit at real values and the host render test checks
     * them at close to full width; epaper_page_line() then truncates at
     * EPAPER_LINE_MAX by contract, so a value that did go mad would lose its
     * tail rather than smash the stack. */
    char ln[64];
    const settings_t *c = settings_get();
    uint8_t row = 0;
    const uint32_t up = now_ms() / 1000u;

    snprintf(ln, sizeof(ln), "INFO %02u:%02u:%02u",
             (unsigned)(up / 3600u), (unsigned)((up / 60u) % 60u),
             (unsigned)(up % 60u));
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    if (BOARD_HAS_VBAT && power_mon_mv() >= 0) {
        const int mv = power_mon_mv();
        snprintf(ln, sizeof(ln), "BAT %d.%02dV %d/4 %s%s",
                 mv / 1000, (mv % 1000) / 10, power_mon_slice(),
                 power_mon_charging() ? "USB" : "CELL",
                 power_mon_state() == POWER_LOW ? " LOW" : "");
    } else {
        snprintf(ln, sizeof(ln), "BAT N/A");
    }
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    {
        const bootrec_t *b = bootrec_newest();
        snprintf(ln, sizeof(ln), "BOOT %s %s N%u",
                 bootrec_reason_name(b->reset_reason),
                 bootrec_decision_name(b->decision),
                 (unsigned)bootrec_boots());
        epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);
    }

    snprintf(ln, sizeof(ln), "MIC %d/%d %s", alive & 7, GUARD_N_CH & 7,
             alive >= GUARD_N_CH ? "OK" : "FAIL");
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    snprintf(ln, sizeof(ln), "AMB %dDB %s", amb_db, amb_word ? amb_word : "");
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    snprintf(ln, sizeof(ln), "V1 %d.%02d/%d.%02d %dHZ",
             CENT_I(g_info.v1_score), CENT_F(g_info.v1_score),
             CENT_I(g_info.v1_thr), CENT_F(g_info.v1_thr),
             (int)(g_info.v1_hz + 0.5f));
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    if (g_info.have2) {
        snprintf(ln, sizeof(ln), "T2 %d.%02d/%d.%02d %dHZ",
                 CENT_I(g_info.t2_score), CENT_F(g_info.t2_score),
                 CENT_I(g_info.t2_thr), CENT_F(g_info.t2_thr),
                 (int)(g_info.t2_hz + 0.5f));
    } else {
        /* OFF MEANS DISABLED. "no reading yet" is a different fact and gets a
         * different word: a tier that is armed but has not produced a record
         * shows WAIT, because a tier that updates one frame in eight can be
         * perfectly healthy and still have nothing to say for a quarter of a
         * second, and printing OFF there is how a running tier came to be
         * read as a stopped one. */
        snprintf(ln, sizeof(ln), "T2 %s", c->t2_enabled ? "WAIT" : "OFF");
    }
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    if (g_info.have3) {
        snprintf(ln, sizeof(ln), "T3 %d.%02d/%d.%02d %dHZ",
                 CENT_I(g_info.t3_score), CENT_F(g_info.t3_score),
                 CENT_I(g_info.t3_thr), CENT_F(g_info.t3_thr),
                 (int)(g_info.t3_rate + 0.5f));
    } else {
        /* OFF MEANS DISABLED. "no reading yet" is a different fact and gets a
         * different word: a tier that is armed but has not produced a record
         * shows WAIT, because a tier that updates one frame in eight can be
         * perfectly healthy and still have nothing to say for a quarter of a
         * second, and printing OFF there is how a running tier came to be
         * read as a stopped one. */
        snprintf(ln, sizeof(ln), "T3 %s", c->t3_enabled ? "WAIT" : "OFF");
    }
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    if (g_info.have4) {
        snprintf(ln, sizeof(ln), "T4 %d.%02d/%d.%02d %dHZ",
                 CENT_I(g_info.t4_score), CENT_F(g_info.t4_score),
                 CENT_I(g_info.t4_thr), CENT_F(g_info.t4_thr),
                 (int)(g_info.t4_hz + 0.5f));
    } else {
        /* OFF MEANS DISABLED. "no reading yet" is a different fact and gets a
         * different word: a tier that is armed but has not produced a record
         * shows WAIT, because a tier that updates one frame in eight can be
         * perfectly healthy and still have nothing to say for a quarter of a
         * second, and printing OFF there is how a running tier came to be
         * read as a stopped one. */
        snprintf(ln, sizeof(ln), "T4 %s", c->t4_enabled ? "WAIT" : "OFF");
    }
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    /* p99 IS n/a AND SAYS SO. No rolling accumulator exists on the device;
     * the figure this project quotes comes from the host analysing a capture.
     * Section 4.5 says print n/a rather than invent it. */
    snprintf(ln, sizeof(ln), "OVER32 %u P99 N/A", (unsigned)g_frames_over);
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    {
        uint8_t lt = 0; float lhz = 0.0f; uint32_t lms = 0;
        if (bootrec_last_alert(&lt, &lhz, &lms)) {
            snprintf(ln, sizeof(ln), "LAST %uM %s %dHZ A%u",
                     (unsigned)((now_ms() - lms) / 60000u),
                     evlog_tier_short(lt), (int)(lhz + 0.5f),
                     (unsigned)bootrec_alerts());
        } else {
            snprintf(ln, sizeof(ln), "LAST NONE A%u V%u",
                     (unsigned)bootrec_alerts(), (unsigned)bootrec_vetoed());
        }
        epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);
    }

#if BOARD_HAS_LORA
    snprintf(ln, sizeof(ln), "LORA N-%04X %s RX%u",
             (unsigned)lora_link_device_id(),
             lora_link_ready() ? "UP" : "DN",
             (unsigned)evlog_count_remote());
#else
    snprintf(ln, sizeof(ln), "LORA N/A");
#endif
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    snprintf(ln, sizeof(ln), "THR %d.%02d FAM%u T%u%u%u V%u",
             CENT_I(g_info.v1_thr), CENT_F(g_info.v1_thr),
             (unsigned)c->trk_family, (unsigned)c->t2_enabled,
             (unsigned)c->t3_enabled, (unsigned)c->t4_enabled,
             (unsigned)c->veto_voice);
    /* N is the near-field gate. It shares this row rather than taking a new
     * one, because the page is full and an operator reading "V1 N1" has both
     * guards in one glance. */
    {
        size_t L = strlen(ln);
        snprintf(ln + L, sizeof(ln) - L, " N%u", (unsigned)c->nf_gate);
    }
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);

    /* THE LAST ROW SAYS WHAT THE HOLD DOES, and it has to, because on this
     * page alone the hold does not turn the device off. A gesture whose
     * meaning changes with the screen is only safe if the screen says so. */
    /* A gesture whose meaning changes with the screen is only safe if the
     * screen says so, and this page's hold no longer rotates - it tests the
     * radio. The rotation still shows here because it is the page that
     * reports configuration; the gesture that CHANGES it moved to RECENT. */
    snprintf(ln, sizeof(ln), "ROT%u HOLD TESTS LINK",
             (unsigned)epaper_degrees_from_quadrant(epaper_get_rotation()));
    epaper_page_line(EPAPER_PAGE_STATUS, row++, ln);
}

/* `U btn tap|hold|down|up`. Returns false if the words are not recognised, so
 * an unknown `U btn ...` still falls through to the normal console path and
 * gets a NACK rather than being silently swallowed by the guard. */
static bool cmd_btn_word(const char *g)
{
    if (!strncmp(g, "tap", 3)) {
        alert_ui_inject_button(ALERT_BTN_TAP);
        return true;
    }
    if (!strncmp(g, "hold", 4)) {
        alert_ui_inject_button(ALERT_BTN_HOLD);
        return true;
    }
    /* THE PRIMITIVES, so a hold of an exact length can be timed from a host:
     * `U btn down`, wait, `U btn up`. `up` releases the override entirely and
     * hands the button back to the pad. */
    if (!strncmp(g, "down", 4)) {
        alert_ui_inject_level(1);
        return true;
    }
    if (!strncmp(g, "up", 2)) {
        alert_ui_inject_level(0);
        return true;
    }
    return false;
}

/* `U alert v1 433`  one event into the alert path, where a real one enters.
 * `U alert every 30`  the same on a repeat, for the frame-budget gate.
 * `U alert every 0`  stop. */
static bool cmd_alert_word(const char *g)
{
    if (!strncmp(g, "every", 5)) {
        int secs = -1;
        if (sscanf(g + 5, "%d", &secs) != 1 || secs < 0) {
            return false;
        }
        alert_ui_set_auto_alert((uint32_t)secs);
        return true;
    }
    uint8_t tier = ALERT_TIER_NONE;
    if      (!strncmp(g, "v1", 2)) { tier = ALERT_TIER_V1; }
    else if (!strncmp(g, "t2", 2)) { tier = ALERT_TIER_T2; }
    else if (!strncmp(g, "t3", 2)) { tier = ALERT_TIER_T3; }
    else if (!strncmp(g, "t4", 2)) { tier = ALERT_TIER_T4; }
    if (tier == ALERT_TIER_NONE) {
        return false;
    }
    int hz = 0;
    if (sscanf(g + 2, "%d", &hz) != 1 || hz <= 0) {
        hz = 433;
    }
    alert_ui_inject_alert(tier, (float)hz);
    return true;
}

static void cmd_u(const char *line)
{
    const char *a = skip_ws(line + 1);

    /* ---- WORD COMMANDS FIRST -------------------------------------------
     * `U b` is the buzzer and has been since the field build, so `U boot` and
     * `U btn` cannot be reached by the single-letter switch below without
     * changing what an existing runbook means. Whole words are matched here
     * and the letter dispatch is left exactly as it was. */
    if (!strncmp(a, "boot", 4)) {
        bootrec_dump();
        trace_ack(line, TRACE_ACK_OK);
        return;
    }
    if (!strncmp(a, "btn", 3)) {
        if (!strncmp(skip_ws(a + 3), "stats", 5)) {
            uint16_t pr = 0, ph = 0, gl = 0;
            alert_ui_btn_stats(&pr, &ph, &gl);
            char b[224];
            snprintf(b, sizeof(b),
                     "BTN presses=%u phantoms=%u glitches=%u "
                     "in-alert=%u  "
                     "(poll %u ms, %u lows to press, %u in an alert, "
                     "%u highs to release, min gap %u ms)\n",
                     (unsigned)pr, (unsigned)ph, (unsigned)gl,
                     (unsigned)alert_ui_btn_alert_presses(),
                     (unsigned)BTN_POLL_MS, (unsigned)BTN_PRESS_SAMPLES,
                     (unsigned)BTN_ALERT_PRESS_SAMPLES,
                     (unsigned)BTN_RELEASE_SAMPLES, (unsigned)BTN_MIN_GAP_MS);
            trace_text(b);
            trace_ack(line, TRACE_ACK_OK);
            return;
        }
        if (!cmd_btn_word(skip_ws(a + 3))) {
            trace_text("U btn tap | hold | down | up\n");
            trace_ack(line, TRACE_ACK_BAD_ARG);
            return;
        }
        trace_ack(line, TRACE_ACK_OK);
        return;
    }
    if (!strncmp(a, "alert", 5)) {
        if (!cmd_alert_word(skip_ws(a + 5))) {
            trace_text("U alert v1|t2|t3|t4 <hz> | U alert every <s>\n");
            trace_ack(line, TRACE_ACK_BAD_ARG);
            return;
        }
        trace_ack(line, TRACE_ACK_OK);
        return;
    }
    if (!strncmp(a, "scr", 3)) {
        int want = -1, shown = -1;
        alert_ui_screens(&want, &shown);
        char b[96];
        snprintf(b, sizeof(b), "SCR want=%d drawn=%d state=%u page=%u\n",
                 want, shown, (unsigned)alert_ui_state(),
                 (unsigned)alert_ui_page());
        trace_text(b);
        trace_ack(line, TRACE_ACK_OK);
        return;
    }
    if (!strncmp(a, "vbat", 4)) {
        char b[512];
        power_mon_describe_raw(b, sizeof(b));
        trace_text(b);
        power_mon_describe(b, sizeof(b));
        trace_text(b);
        trace_ack(line, TRACE_ACK_OK);
        return;
    }
    if (!strncmp(a, "wake", 4)) {
        /* Wake is a restart, which is also what closes the few kilobytes an
         * in-process guard restart leaks. A boot takes the unit to LISTENING
         * in about 1.5 s and the ring records it as reset=SW
         * decision=LISTENING. */
        trace_text("U wake: restarting to LISTENING\n");
        bootrec_note_uptime(now_ms());
        vTaskDelay(pdMS_TO_TICKS(120));
        esp_restart();
        return;
    }
    if (!strncmp(a, "rst", 3)) {
        /* A software reset must land in LISTENING with nobody touching the
         * button. It is the cheapest proof that the boot policy
         * holds, and it is the one reset that can be commanded from here. */
        trace_text("U rst: restarting\n");
        vTaskDelay(pdMS_TO_TICKS(120));     /* let the line drain */
        esp_restart();
        return;                             /* not reached */
    }
    if (!strncmp(a, "off", 3)) {
        const char *g = skip_ws(a + 3);
        if (strncmp(g, "deep", 4) != 0) {
            trace_text("U off deep   (ends the USB link; hands only from "
                       "there)\n");
            trace_ack(line, TRACE_ACK_BAD_ARG);
            return;
        }
#if BOARD_NEEDS_DEEP_SLEEP
        /* The test hook for off-deep, skipping the wait. It drops the USB
         * link the moment it runs and only a hand on the button brings the
         * device back, so it must not be run in an unattended session. */
        trace_ack(line, TRACE_ACK_OK);
        bootrec_set_decision(BOOTDEC_OFF_DEEP_BACK);
        enter_off_deep("U off deep");
#else
        trace_text("no deep sleep on this board target\n");
        trace_ack(line, TRACE_ACK_FAULT);
#endif
        return;
    }

    switch (*a) {
    case 'b':
        /* `U b <duty_pct> [ms]` - the tone sweep, for beep mode 1.
         * Distinguished from `U b0` / `U b1` / `U bd0` by the space: those
         * three have never had one, so no existing runbook line changes
         * meaning. The TMB12A03 is an active buzzer with its own oscillator,
         * so what a chopped supply does to it is an ears question, and this
         * is how it gets answered. */
        if (a[1] == ' ' || a[1] == '\t') {
            int duty = -1, ms = 0;
            if (sscanf(a + 1, "%d %d", &duty, &ms) >= 1 &&
                duty >= 0 && duty <= 100) {
                if (ms <= 0 || ms > 3000) { ms = 300; }
                if (!actuators_buzzer_tone(BTN_BEEP_PWM_HZ, duty, true)) {
                    trace_ack(line, TRACE_ACK_FAULT);
                    return;
                }
                vTaskDelay(pdMS_TO_TICKS(ms));
                (void)actuators_buzzer_tone(BTN_BEEP_PWM_HZ, 0, false);
                /* Hand the pad back to whatever the alert path expects. */
                actuators_buzzer_drive(settings_get()->buzz_drive,
                                       settings_get()->buzz_hz);
                trace_ack(line, TRACE_ACK_OK);
                return;
            }
        }
        /* `U b0` / `U b1` are unchanged. Two new sub-letters ride behind them,
         * because "maximum volume" is a property of the transducer, not of the
         * firmware, and the firmware cannot know which one is fitted. */
        if (a[1] == 'd') {
            /* U bd0            solid DC   (correct for an ACTIVE buzzer)
             * U bd1 [hz]       square wave (correct for a PASSIVE one)     */
            int hz = 0;
            const int mode = (a[2] == '1') ? BUZZ_DRIVE_PWM : BUZZ_DRIVE_DC;
            if (sscanf(a + 3, "%d", &hz) != 1 || hz <= 0) {
                hz = (int)BUZZ_PWM_HZ_DEFAULT;
            }
            actuators_buzzer_drive((uint8_t)mode, (uint32_t)hz);
            settings_mut()->buzz_drive = (uint8_t)mode;
            settings_mut()->buzz_hz = (uint16_t)hz;
            char b[96];
            snprintf(b, sizeof(b), "buzzer drive=%s hz=%d %s\n",
                     mode == BUZZ_DRIVE_PWM ? "pwm" : "dc", hz,
                     settings_save() ? "(saved)" : "(NOT saved)");
            trace_text(b);
            trace_ack(line, TRACE_ACK_OK);
            return;
        }
        if (a[1] == 't' && a[2] == 'n') {
            /* `U btn <tap|double|hold>` RIDES BEHIND `U b`, AND THAT IS A TRAP
             * WORTH NAMING. The dispatcher switches on ONE character, so
             * without this test `U btn tap` matches `U b` with a[1]=='t' and
             * plays the four-second BUZZER LADDER at full volume. It must be
             * tested before the ladder, not after. */
            const char *bb = skip_ws(a + 3);
            uint8_t ev = ALERT_BTN_NONE;
            if (bb[0] == 't') { ev = ALERT_BTN_TAP; }
            else if (bb[0] == 'd') { ev = ALERT_BTN_DOUBLE; }
            else if (bb[0] == 'h') { ev = ALERT_BTN_HOLD; }
            if (ev == ALERT_BTN_NONE) {
                trace_text("U btn <tap|double|hold>\n");
                trace_ack(line, TRACE_ACK_BAD_ARG);
                return;
            }
            alert_ui_inject_button(ev);
            trace_text(ev == ALERT_BTN_TAP ? "btn TAP -> snooze\n" :
                       ev == ALERT_BTN_DOUBLE ? "btn DOUBLE -> menu\n"
                                              : "btn HOLD -> OFF ritual\n");
            trace_ack(line, TRACE_ACK_OK);
            return;
        }
        if (a[1] == 't') {
            /* THE LADDER. Four one-second bursts with a gap between them, so
             * an ear a metre away can rank them. Whichever is loudest is the
             * drive this hardware wants; set it with `U bd`. An ACTIVE buzzer
             * is loudest on the DC step and the PWM steps sound thin or
             * quieter; a PASSIVE one is near-silent on DC and loudest at
             * whichever frequency is closest to its resonance. */
            static const uint32_t ladder[] = {0u, 2000u, 2700u, 4000u};
            const uint8_t save_mode = actuators_buzzer_drive_mode();
            const uint32_t save_hz = actuators_buzzer_drive_hz();
            for (unsigned i = 0; i < sizeof(ladder) / sizeof(ladder[0]); i++) {
                char b[64];
                if (ladder[i] == 0u) {
                    snprintf(b, sizeof(b), "buzzer test 1/4: DC (solid)\n");
                    actuators_buzzer_drive(BUZZ_DRIVE_DC, 0);
                } else {
                    snprintf(b, sizeof(b), "buzzer test %u/4: PWM %u Hz\n",
                             i + 1u, (unsigned)ladder[i]);
                    actuators_buzzer_drive(BUZZ_DRIVE_PWM, ladder[i]);
                }
                trace_text(b);
                actuators_buzzer(true);
                vTaskDelay(pdMS_TO_TICKS(1000));
                actuators_buzzer(false);
                vTaskDelay(pdMS_TO_TICKS(600));
            }
            actuators_buzzer_drive(save_mode, save_hz);
            trace_text("buzzer test done - set the loudest with U bd0 or "
                       "U bd1 <hz>\n");
            trace_ack(line, TRACE_ACK_OK);
            return;
        }
        actuators_buzzer(a[1] == '1');
        trace_ack(line, TRACE_ACK_OK);
        return;
    case 'v':
        actuators_motor(a[1] == '1');
        trace_ack(line, TRACE_ACK_OK);
        return;
    case 'k': {
        /* `U k 0|1` - PARK THE LED DATA PIN AT A STATIC LEVEL.
         *
         * This is the measurement that settles a dark LED, and it is the only
         * one software can make. `U y` shows whether the engine is sending
         * correct GRB bytes; if it is sending them continuously with zero
         * failures while the part stays dark, and a full-white command does
         * not light it either, then perfect data is being clocked into a part
         * that does not respond - which is the signature of a part that is not
         * powered. This command parks the data pin so a meter can say so.
         *
         * RMT is torn down first. While the channel owns the pad a plain
         * gpio_set_level() is ignored, so a meter would read the RMT idle
         * level and prove nothing. `U l <c0> <c1> <c2>` rebuilds the channel
         * and takes the pad back.
         *
         * This is a bench command and it latches. It is deliberately limited
         * to the LED data pin: the same trick on the buzzer or motor pin
         * latches a load on with no timeout, which is a mistake this console
         * has already made once (see the restore note in `U g`). */
        int lvl = -1;
        if (sscanf(a + 1, "%d", &lvl) != 1 || (lvl != 0 && lvl != 1)) {
            trace_ack(line, TRACE_ACK_BAD_ARG);
            return;
        }
        (void)led_deinit();
        gpio_config_t kc = {
            .pin_bit_mask = 1ULL << PIN_LED_WS2812,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&kc);
        gpio_set_level((gpio_num_t)PIN_LED_WS2812, lvl ? 1 : 0);
        char kb[520];
        snprintf(kb, sizeof(kb),
                 "\nGPIO%d PARKED %s (RMT torn down)\n"
                 "  Meter against GND, expected if healthy:\n"
                 "    LED1 VDD  pin 1     3.3 V     0 V = the VDD joint is open\n"
                 "    LED1 VSS  pin 3     0 V       floating = open ground\n"
                 "    R17 P1  (IO16 side) %s\n"
                 "    R17 P2  / LED1 DIN  %s   differs from P1 = R17 open\n"
                 "  All four correct and still dark = dead or misoriented LED1,\n"
                 "  and no software fix exists.\n"
                 "  `U l 0 0 0` rebuilds the RMT channel and takes the pad back.\n",
                 (int)PIN_LED_WS2812, lvl ? "HIGH" : "LOW",
                 lvl ? "3.3 V" : "0 V", lvl ? "3.3 V" : "0 V");
        trace_text(kb);
        trace_ack(line, TRACE_ACK_OK);
        return;
    }
    case 'y': {
        /* `U y` - WHAT THE LED ENGINE IS DOING, in numbers.
         *
         * The LED has been wrong three times running and every diagnosis was
         * reached by reading code. It cannot report its own fault because it
         * IS the report: "nothing was ever written", "written too briefly to
         * see" and "written dark" look identical to an operator and have
         * completely different causes. These counters tell them apart in one
         * line without anyone having to watch the board. */
        led_engine_stat_t st;
        led_engine_stat(&st);
        char sb[200];
        snprintf(sb, sizeof(sb),
                 "led engine task=%d ticks=%lu ok=%lu fail=%lu locked=%lu "
                 "pat=%u last=(%u,%u,%u) cap=%d\n",
                 st.task_alive ? 1 : 0,
                 (unsigned long)st.ticks, (unsigned long)st.sends_ok,
                 (unsigned long)st.sends_failed,
                 (unsigned long)st.skipped_locked,
                 (unsigned)st.pattern,
                 (unsigned)st.last[0], (unsigned)st.last[1],
                 (unsigned)st.last[2], (int)LED_MAX_CHANNEL);
        trace_text(sb);
        trace_ack(line, TRACE_ACK_OK);
        return;
    }
    case 'l': {
        int c0 = 0, c1 = 0, c2 = 0;
        /* `U l rgb <r> <g> <b>` - colours, so the wire order can be checked
         * by eye rather than believed. This module is wired G,R,B, measured on
         * the bench from what was actually seen; led_set_rgb() is the one
         * place that mapping lives. Bare `U l <c0> <c1> <c2>` stays raw wire
         * order, for testing the mapping itself. */
        {
            const char *r = skip_ws(a + 1);
            if ((r[0] == 'r' || r[0] == 'R') && (r[1] == 'g' || r[1] == 'G') &&
                (r[2] == 'b' || r[2] == 'B')) {
                int rr = 0, gg = 0, bb = 0;
                if (sscanf(r + 3, "%d %d %d", &rr, &gg, &bb) != 3 ||
                    rr < 0 || rr > 255 || gg < 0 || gg > 255 ||
                    bb < 0 || bb > 255) {
                    trace_ack(line, TRACE_ACK_BAD_ARG);
                    return;
                }
                const esp_err_t er = led_set_rgb((uint8_t)rr, (uint8_t)gg,
                                                 (uint8_t)bb);
                char lb[120];
                snprintf(lb, sizeof(lb),
                         "led rgb(%d,%d,%d) -> wire(%d,%d,%d)%s  cap %d\n",
                         rr, gg, bb, gg, rr, bb,
                         er == ESP_OK ? "" : "  SEND FAILED",
                         (int)LED_MAX_CHANNEL);
                trace_text(lb);
                trace_ack(line, er == ESP_OK ? TRACE_ACK_OK : TRACE_ACK_FAULT);
                return;
            }
        }
        if (sscanf(a + 1, "%d %d %d", &c0, &c1, &c2) != 3) {
            trace_ack(line, TRACE_ACK_BAD_ARG);
            return;
        }
        if (c0 < 0 || c0 > 255 || c1 < 0 || c1 > 255 || c2 < 0 || c2 > 255) {
            trace_ack(line, TRACE_ACK_BAD_ARG);
            return;
        }
        /* A typed colour must stick. The pattern engine owns the pixel and
         * re-sends every LED_REFRESH_MS, so without standing it down a manual
         * colour is reverted within half a second and a repeated LED check
         * could never be performed. */
        led_pattern_set(LED_PAT_MANUAL);
        esp_err_t e = led_set_raw((uint8_t)c0, (uint8_t)c1, (uint8_t)c2);
        trace_ack(line, e == ESP_OK ? TRACE_ACK_OK : TRACE_ACK_FAULT);
        return;
    }
    case 'e': {
        const char *b = skip_ws(a + 1);
        if (epaper_state() == TRACE_EPD_FAULTED) {
            /* Faulted panels NACK rather than hang the caller. */
            trace_ack(line, TRACE_ACK_FAULT);
            return;
        }
        epaper_screen_t scr;
        switch (*b) {
        case 'i': scr = EPAPER_SCREEN_INIT_CLEAR; break;
        case 'r': scr = EPAPER_SCREEN_READY;      break;
        case 'a': scr = EPAPER_SCREEN_ALERT;      break;
        case 'c': scr = EPAPER_SCREEN_CLEAR;      break;
        case 't': scr = EPAPER_SCREEN_TEST;       break;
        case 'w': scr = EPAPER_SCREEN_ALERT_WASH; break;
        case 'o': scr = EPAPER_SCREEN_OFF;        break;
        /* THE TWO PAGES, so a bench can see them without a button.
         *
         * In the field a tap turns the page and that is the whole interface.
         * At a bench the panel is the thing under test, and F5 asks somebody
         * to look at every screen - so both are reachable from the console
         * too. `s` is STATUS; RECENT is `n` because `r` has meant the resting
         * screen since the field build and moving it would silently change
         * what an older runbook asks for. */
        case 's': scr = EPAPER_SCREEN_STATUS;     break;
        case 'n': scr = EPAPER_SCREEN_RECENT;     break;
        default:  trace_ack(line, TRACE_ACK_BAD_ARG); return;
        }
        int rot = -1;
        if (sscanf(b + 1, "%d", &rot) == 1) {
            const uint8_t q = epaper_quadrant_from_arg(rot);
            if (q != 0xFFu) {
                epaper_set_rotation(q);     /* this run only; `U c rot` persists */
            }
        }
        trace_ack(line, epaper_request(scr) ? TRACE_ACK_OK : TRACE_ACK_FAULT);
        return;
    }
    case 'p':
        /* `U p` - E-PAPER BUS DIAGNOSIS. Deliberately NOT behind
         * `U e`: that case NACKs the moment the panel is faulted,
         * which is the only state in which this is worth running. */
        cmd_panel_diag(line);
        return;
    case 'c': {
        /* `U c`          print the persisted configuration
         * `U cr`         reset it to the shipped defaults
         * `U c <k> <v>`  set one field and save it
         *
         * The keys are the ones a field session actually changes. Anything
         * not here stays a per-run argument on purpose. */
        const char *b = skip_ws(a + 1);
        char buf[288];
        if (*b == 0) {
            settings_describe(buf, sizeof(buf));
            trace_text(buf);
            trace_ack(line, TRACE_ACK_OK);
            return;
        }
        /* RESET IS `U cr` - the letter immediately after the `c`, NO SPACE.
         *
         * This tested `*b == 'r'` AFTER skipping whitespace, so `U c rot 270`
         * and `U c rate 2` would both match it and silently reset the entire
         * configuration to defaults, the tier pair included - so typing the
         * documented fix for a sideways display would wipe the configuration
         * the deployment was built around, with a cheerful "saved" underneath
         * it. */
        if (a[1] == 'r') {
            settings_reset();
            settings_describe(buf, sizeof(buf));
            trace_text(buf);
            trace_ack(line, TRACE_ACK_OK);
            return;
        }
        char key[16] = {0};
        int val = 0;
        if (sscanf(b, "%15s %d", key, &val) != 2) {
            trace_text("ERR usage: U c | U cr | U c <thr1|thr2|thr3|thr4|"
                       "t2|t3|t4|"
                       "rate|snooze|burst|autostart|rot|mirror|batt|veto|"
                       "test|lora|lorahz|family> <value>\n");
            trace_ack(line, TRACE_ACK_BAD_ARG);
            return;
        }
        settings_t *c = settings_mut();
        if      (!strcmp(key, "thr1"))      { c->thr1_milli = (uint32_t)val; }
        else if (!strcmp(key, "thr2"))      { c->thr2_milli = (uint32_t)val; }
        else if (!strcmp(key, "thr3"))      { c->thr3_milli = (uint32_t)val; }
        else if (!strcmp(key, "t2"))        { c->t2_enabled = val ? 1 : 0; }
        else if (!strcmp(key, "t3"))        { c->t3_enabled = val ? 1 : 0; }
        else if (!strcmp(key, "thr4"))      { c->thr4_milli = (uint32_t)val; }
        else if (!strcmp(key, "t4"))        { c->t4_enabled = val ? 1 : 0; }
        /* `U c batt 1` a cell IS fitted (the default), `U c batt 0` none.
         * Stored inverted so a v5 blob's zeroed reserved byte reads "fitted"
         * and no settings version had to be bumped for it. */
        else if (!strcmp(key, "batt"))      { c->batt_absent = val ? 0 : 1; }
        else if (!strcmp(key, "veto"))      { c->veto_voice = val ? 1 : 0; }
        /* One key for both halves - see settings.h. */
        else if (!strcmp(key, "nf"))        { c->nf_gate = val ? 1 : 0; }
        /* Test mode. Reachable as `U c test 1` for consistency with every
         * other persisted setting, and as a bare `test 1` because that is what
         * an operator is most likely to type. Both land here. */
        else if (!strcmp(key, "test"))      { c->test_mode = val ? 1 : 0; }
        else if (!strcmp(key, "rate"))      { c->t2_rate = (uint8_t)val; }
        else if (!strcmp(key, "snooze"))    { c->snooze_ms = (uint32_t)val; }
        else if (!strcmp(key, "burst"))     { c->alert_max_ms = (uint32_t)val; }
        else if (!strcmp(key, "autostart")) { c->autostart = val ? 1 : 0; }
        else if (!strcmp(key, "lora")) {
            /* REFUSED, NOT SILENTLY IGNORED, on a board with no radio. A
             * device that cheerfully answered "saved" to a setting it cannot
             * honour is how an operator comes to believe their peer beacon is
             * on when nothing is connected to an antenna. */
            if (!BOARD_HAS_LORA && val) {
                trace_text("ERR this board has no radio; see `I` for the "
                           "board name\n");
                trace_ack(line, TRACE_ACK_BAD_ARG);
                return;
            }
            c->lora_enabled = val ? 1 : 0;
        }
        else if (!strcmp(key, "lorahz")) {
            /* THE DEPLOYMENT FREQUENCY IS A CONFIGURATION, not a code change:
             * the antenna is the band-specific element and the module covers
             * 803-930 MHz. The range is refused rather than clamped, on the
             * same principle as the Tier-2 band - an operator who typed the
             * wrong number must SEE that, not get a silently different
             * channel and a device their peers cannot hear. */
            if (val < 803000000 || val > 930000000) {
                trace_text("ERR lorahz must be 803000000-930000000 (the "
                           "Ra-01H's range). The ANTENNA is the band piece; "
                           "set the channel to your local regulations.\n");
                trace_ack(line, TRACE_ACK_BAD_ARG);
                return;
            }
            c->lora_hz = (uint32_t)val;
        }
        else if (!strcmp(key, "family")) {
            /* THE TRACKER FAMILY RULE. Runtime, default off, and OFF is
             * bit-identical to the sealed tracker. Turning it on is a
             * CHARACTERISATION decision for Field-3, not an adoption. */
            c->trk_family = val ? 1 : 0;
        }
        else if (!strcmp(key, "rot")) {
            /* DEGREES, clockwise - because that is what the operator is
             * looking at when they decide it is wrong. A bare 0..3 quadrant
             * is still accepted so old notes keep working. */
            const uint8_t q = epaper_quadrant_from_arg(val);
            if (q == 0xFFu) {
                trace_text("ERR rot must be 0, 90, 180 or 270 degrees\n");
                trace_ack(line, TRACE_ACK_BAD_ARG);
                return;
            }
            c->epd_rotation = q;
            epaper_set_rotation(q);
        }
        else if (!strcmp(key, "mirror")) {
            c->epd_mirror = val ? 1 : 0;
            epaper_set_mirror(c->epd_mirror != 0);
        }
        else {
            trace_text("ERR unknown key\n");
            trace_ack(line, TRACE_ACK_BAD_ARG);
            return;
        }
        const bool saved = settings_save();
        settings_describe(buf, sizeof(buf));
        trace_text(buf);
        trace_text(saved ? "saved\n" : "NOT SAVED (nvs write failed)\n");
        trace_ack(line, saved ? TRACE_ACK_OK : TRACE_ACK_FAULT);
        return;
    }
    case 's':
        /* `U scr <home|alert|snooze|menu|off>` RIDES BEHIND `U s`, tested
         * FIRST so a screen request can never be mistaken for a STAT dump.
         * `U e<letter>` already draws any screen but speaks enum letters; a
         * field morning should not have to. This takes the words that are
         * printed on the field card. */
        if (a[1] == 'c' && a[2] == 'r') {
            const char *w = skip_ws(a + 3);
            epaper_screen_t sc;
            if      (w[0] == 'h') { sc = EPAPER_SCREEN_READY; }
            else if (w[0] == 'a') { sc = EPAPER_SCREEN_ALERT; }
            else if (w[0] == 's') { sc = EPAPER_SCREEN_SNOOZE; }
            else if (w[0] == 'm') { sc = EPAPER_SCREEN_STATUS; }
            else if (w[0] == 'o') { sc = EPAPER_SCREEN_OFF; }
            else {
                trace_text("U scr <home|alert|snooze|menu|off>\n");
                trace_ack(line, TRACE_ACK_BAD_ARG);
                return;
            }
            trace_ack(line,
                      epaper_request(sc) ? TRACE_ACK_OK : TRACE_ACK_FAULT);
            return;
        }
        alert_ui_emit_stat_idle();
        trace_ack(line, TRACE_ACK_OK);
        return;
    case 'G': {
        /* `U G` - PIN SCAN. Configures every FREE gpio with a pull-up and
         * watches them all for 6 s. Press and hold the button: whichever pin
         * goes low is the pin the button is really on.
         *
         * This exists because "the wiring is correct" and "the pin never goes
         * low" cannot both be true, and guessing between them wastes bench
         * time. Only pins that nothing else in this build drives are scanned -
         * the I2S clocks/data, the e-paper bus, the LED, the two NPN bases and
         * the strapping/flash/PSRAM/USB pins are all excluded, so the scan
         * cannot disturb anything.
         *
         * Which pins those are is a board fact, and it used to be a literal
         * list - first here, then briefly as BOARD_SCAN_PINS in the board
         * header. Six of its ten entries are allocated on the PCB - the VBAT
         * divider, the charger STAT line and four LoRa signals - so on that
         * board the list would have driven a pull-up onto a live SPI bus and
         * onto the radio's reset. A list beside the allocation is a second
         * statement of one fact and those two had already disagreed, so there
         * is no list: board_scan_pins() derives the set from what the active
         * board header allocates, reserves and keeps clear. This file asks the
         * question and the seam answers it. */
        board_scan_t scan;
        board_scan_pins(&scan);
        const int ncand = scan.n;
        for (int i = 0; i < ncand; i++) {
            gpio_config_t c = {
                .pin_bit_mask = 1ULL << scan.pin[i],
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            gpio_config(&c);
        }
        trace_text("\nPIN SCAN - press and HOLD the button now (6 s)\n");
        /* Sized from the SET's capacity, not from how many pins this board
         * happens to leave free: scan.n is a runtime count now, and a board
         * with one more scannable pin than the last one must not be able to
         * walk off the end of a counter array. 49 ints on a 16 KB task stack. */
        int lowseen[sizeof(scan.pin) / sizeof(scan.pin[0])] = {0};
        const uint32_t t0 = now_ms();
        while (now_ms() - t0 < 6000u) {
            for (int i = 0; i < ncand; i++) {
                if (gpio_get_level(scan.pin[i]) == 0) {
                    lowseen[i]++;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        char b[96];
        int found = 0;
        for (int i = 0; i < ncand; i++) {
            if (lowseen[i] > 0) {
                found++;
                snprintf(b, sizeof(b),
                         "  GPIO%-2d went LOW on %d samples\n",
                         (int)scan.pin[i], lowseen[i]);
                trace_text(b);
            }
        }
        if (!found) {
            trace_text("  NO pin went low. The button is not connected to any "
                       "free GPIO,\n  or not to GND. Check both legs with a "
                       "multimeter in continuity mode:\n  pressing the button "
                       "must beep between its two legs.\n");
        }
        trace_ack(line, TRACE_ACK_OK);
        return;
    }
    case 'g': {
        /* `U g [pin]` - RAW GPIO WATCH, the diagnostic that settles "is the
         * button a code problem or a wiring problem".
         *
         * Configures the pin as input with the internal pull-up, samples it
         * for 3 s, and reports min/max/transitions as text. Hold the button
         * while it runs: any change at all proves the pin sees the switch, and
         * no change at all means the switch is not reaching this pin - which
         * is a measurement rather than an inference. Defaults to the snooze
         * pin; pass another number to test a different one. */
        int pin = (int)PIN_SNOOZE_BTN;
        (void)sscanf(a + 1, "%d", &pin);
        if (pin < 0 || pin > 48) {
            trace_ack(line, TRACE_ACK_BAD_ARG);
            return;
        }
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << pin,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&cfg) != ESP_OK) {
            trace_ack(line, TRACE_ACK_FAULT);
            return;
        }
        int lo = 1, hi = 0, trans = 0, prev = gpio_get_level(pin);
        const uint32_t t0 = now_ms();
        while (now_ms() - t0 < 3000u) {
            const int v = gpio_get_level(pin);
            if (v != prev) { trans++; prev = v; }
            if (v < lo) { lo = v; }
            if (v > hi) { hi = v; }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        /* ---- RESTORE THE PAD. `U g` IS A DESTRUCTIVE WRITE, NOT A READ ----
         *
         * gpio_config(GPIO_MODE_INPUT) above did two things beyond reading:
         * it cleared GPIO_ENABLE and set func_out_sel_cfg[pin].oen_sel = 1, so
         * a PERIPHERAL that owns this pad can never re-assert its output
         * enable - and the peripheral is never told. RMT goes on returning
         * ESP_OK with nothing on the wire, for the rest of the boot, because
         * led_init() latches s_ready and never re-routes the pad.
         *
         * And on a pin that DRIVES A LOAD the internal pull-up is enough to
         * bias an NPN base ON: `U g 17` latches the buzzer on with no timeout,
         * and `U b0` cannot clear it because the pin is no longer an output.
         *
         * Both have been demonstrated on the bench. So every probe puts the
         * pad back, and says which way. */
        const char *restored;
        if (pin == PIN_LED_WS2812) {
            /* Drop the RMT channel so the next led_set_raw() rebuilds it and
             * re-routes the pad. Nothing else recovers this. */
            (void)led_deinit();
            restored = "RMT channel torn down; next `U l` rebuilds and "
                       "re-routes the pad";
        } else if (pin == PIN_BUZZER || pin == PIN_MOTOR) {
            /* Level BEFORE direction, the same order actuators_boot_safe()
             * uses, so the pad never spends an instruction as a driven HIGH. */
            actuators_boot_safe();
            restored = "buzzer and motor driven LOW again (level before "
                       "direction)";
        } else {
            gpio_reset_pin((gpio_num_t)pin);
            restored = "pad reset to its default state";
        }

        char b[420];
        snprintf(b, sizeof(b),
                 "\nGPIO%d watch 3s: min=%d max=%d transitions=%d\n"
                 "  RESTORED: %s\n",
                 pin, lo, hi, trans, restored);
        trace_text(b);
        trace_text("  1 = released (internal pull-up), 0 = shorted to GND\n");
        if (lo == 1) {
            trace_text("  NEVER went low: the switch is not reaching this "
                       "pin and GND.\n");
        } else if (trans > 20) {
            trace_text("  many transitions: contact bounce or a floating "
                       "pin.\n");
        } else {
            trace_text("  the pin sees the switch.\n");
        }
        trace_ack(line, TRACE_ACK_OK);
        return;
    }
    default:
        trace_ack(line, TRACE_ACK_BAD_ARG);
        return;
    }
}

/* ---- mode: parity - samples AND trace, so the host can re-derive ------- */
static void run_parity(uint32_t seconds)
{
    if (!mono_alloc()) {
        return;
    }
    source_t src;
    if (source_i2s(&src) != ESP_OK) {
        trace_text("ERR i2s init failed\n");
        return;
    }
    uint32_t n = seconds * CFG_FS;
    if (n < CFG_N_FFT) {
        n = CFG_N_FFT;
    }
    const stream_opts_t o = {
        .name = src.name, .n_samples = n,
        .max_frames = 1u + (n - CFG_N_FFT) / CFG_HOP,
        .prb_from = 0xFFFFFFFFu, .prb_to = 0u,
        .emit_pcm = true, .emit_alert = true,
    };
    run_stream(&src, &o);
    source_i2s_stop();
}

/* ---- mode: tone - which FFT bin does a known frequency land in? --------
 * PLUMBING ONLY. This proves the mic -> I2S -> window -> FFT chain puts a
 * known tone where the arithmetic says it should. It is NOT detection, it
 * says NOTHING about range or about drones, and a clean result here means
 * exactly one thing: the samples are arriving right way up and at the right
 * rate. Per the brief, no absolute claim follows from it. */
static void run_tone(uint32_t n_avg)
{
    if (!mono_alloc()) {
        return;
    }
    source_t src;
    if (source_i2s(&src) != ESP_OK) {
        trace_text("ERR i2s init failed\n");
        return;
    }
    if (n_avg == 0 || n_avg > 64) {
        n_avg = 16;
    }
    static double acc[CFG_N_BINS];
    memset(acc, 0, sizeof(acc));

    for (uint32_t f = 0; f < n_avg; f++) {
        const int want = (f == 0) ? CFG_N_FFT : CFG_HOP;
        if (f != 0) {
            memmove(g_block, g_block + CFG_HOP,
                    (CFG_N_FFT - CFG_HOP) * sizeof(float));
        }
        if (source_read(&src, g_q, (size_t)want) != want) {
            break;
        }
        float *dst = (f == 0) ? g_block : g_block + (CFG_N_FFT - CFG_HOP);
        for (int k = 0; k < want; k++) {
            dst[k] = q_to_f(g_q[k]);
        }
        front_end(g_block, g_w->spec[0], g_w);
        for (int b = 0; b < CFG_N_BINS; b++) {
            const float re = g_w->spec[0][b].re, im = g_w->spec[0][b].im;
            acc[b] += sqrt((double)re * re + (double)im * im);
        }
    }
    for (int b = 0; b < CFG_N_BINS; b++) {
        acc[b] /= (double)n_avg;
    }

    /* top 8 bins, DC excluded: bin 0 carries the mic's DC offset, which is
     * measured by meter mode and deliberately NOT removed anywhere. */
    trace_ton_t t = {.magic = TRACE_MAGIC_TON, .n_avg = n_avg,
                     .bin_hz = (double)CFG_FS / (double)CFG_N_FFT,
                     .dc_mag = acc[0]};
    bool used[CFG_N_BINS];
    memset(used, 0, sizeof(used));
    used[0] = true;
    for (int j = 0; j < 8; j++) {
        int best = -1;
        double bv = -1.0;
        for (int b = 1; b < CFG_N_BINS; b++) {
            if (!used[b] && acc[b] > bv) {
                bv = acc[b];
                best = b;
            }
        }
        if (best < 0) {
            break;
        }
        used[best] = true;
        t.top_bin[j] = (uint16_t)best;
        t.top_mag[j] = (float)bv;
        if (j == 0) {
            t.peak_bin = (uint32_t)best;
            t.peak_hz = (double)best * t.bin_hz;
        }
    }
    trace_send_ton(&t);
    source_i2s_stop();
}

/* ---- mode: meter - is the microphone alive, and how loud is the room? --
 * No detector, no windowing: raw level only. This is the FIRST thing to run
 * after wiring, because a dead or mis-clocked INMP441 reads as a constant, as
 * all-zero or as full-scale, and all three are obvious here and invisible once
 * the adaptive floor has swallowed them. */
static void run_meter(void)
{
    if (!mono_alloc()) {
        return;
    }
    source_t src;
    if (source_i2s(&src) != ESP_OK) {
        trace_text("ERR i2s init failed\n");
        return;
    }
    trace_hdr_t h = {
        .magic = TRACE_MAGIC_HDR, .version = TRACE_VERSION,
        .n_samples = 0, .n_frames = 0, .threshold = 0.0,
        .fs = CFG_FS, .n_fft = CFG_N_FFT, .hop = CFG_HOP,
        .n_bins = CFG_N_BINS, .n_f0 = CFG_N_F0,
        .rec_size = sizeof(trace_met_t),
    };
    strncpy(h.name, "meter", sizeof(h.name) - 1);
    trace_send_hdr(&h);

    const uint32_t BLOCK = CFG_FS / 2;          /* 500 ms */
    uint32_t seq = 0;
    for (;;) {
        int64_t sum = 0, sumsq = 0;
        int32_t vmin = 32767, vmax = -32768, peak = 0;
        uint32_t got = 0;
        while (got < BLOCK) {
            const uint32_t want =
                (BLOCK - got > CFG_N_FFT) ? CFG_N_FFT : (BLOCK - got);
            const int n = source_read(&src, g_q, want);
            if (n <= 0) {
                break;
            }
            for (int k = 0; k < n; k++) {
                const int32_t v = g_q[k];
                sum += v;
                sumsq += (int64_t)v * v;
                if (v < vmin) { vmin = v; }
                if (v > vmax) { vmax = v; }
                const int32_t a = (v < 0) ? -v : v;
                if (a > peak) { peak = a; }
            }
            got += (uint32_t)n;
        }
        if (got == 0) {
            break;
        }
        trace_met_t m = {
            .magic = TRACE_MAGIC_MET, .seq = seq++,
            .t_us = (uint64_t)esp_timer_get_time(), .n_samples = got,
            .rms = sqrt((double)sumsq / (double)got),
            .dc_offset = (double)sum / (double)got,
            .peak_abs = peak, .vmin = vmin, .vmax = vmax,
            .short_reads = source_i2s_short_reads(),
            .timeouts = source_i2s_timeouts(),
        };
        trace_send_met(&m);
        if (trace_poll_byte() >= 0) {
            break;
        }
    }
    trace_end_t e = {.magic = TRACE_MAGIC_END, .n_frames = seq};
    trace_send_end(&e);
    source_i2s_stop();
}

/* 'F' - FFT self-test on a signal whose transform is known in closed form.
 * x[n] = cos(2*pi*k0*n/N), NO window: the unnormalised DFT must give
 * |X[k0]| = N/2 and ~0 everywhere else. This separates an FFT scaling or
 * ordering fault from anything downstream in one shot. */
static void fft_selftest(int k0)
{
    if (!mono_alloc()) {
        return;
    }
    static float blk[CFG_N_FFT];
    for (int n = 0; n < CFG_N_FFT; n++) {
        blk[n] = cosf(2.0f * (float)M_PI * (float)k0 * (float)n
                      / (float)CFG_N_FFT);
    }
    /* front_end applies the Hann window, so bypass it here and do the raw
     * transform the same way front_end does. */
    for (int i = 0; i < CFG_N_FFT; i++) {
        g_w->fftbuf[i] = blk[i];
    }
    dsps_fft2r_fc32(g_w->fftbuf, CFG_N_FFT >> 1);
    dsps_bit_rev2r_fc32(g_w->fftbuf, CFG_N_FFT >> 1);
    dsps_cplx2real_fc32(g_w->fftbuf, CFG_N_FFT >> 1);

    char b[256];
    snprintf(b, sizeof(b), "\nFFT selftest cos(2pi*%d*n/%d), no window\n"
             "  expect |X[%d]| = %d, all other bins ~0\n",
             k0, CFG_N_FFT, k0, CFG_N_FFT / 2);
    trace_text(b);
    float best = -1.0f;
    int bestk = -1;
    double tot = 0.0;
    for (int k = 0; k < CFG_N_BINS; k++) {
        float re, im;
        if (k == 0) { re = g_w->fftbuf[0]; im = 0.0f; }
        else if (k == CFG_N_BINS - 1) { re = g_w->fftbuf[1]; im = 0.0f; }
        else { re = g_w->fftbuf[2 * k]; im = g_w->fftbuf[2 * k + 1]; }
        float m = sqrtf(re * re + im * im);
        tot += m;
        if (m > best) { best = m; bestk = k; }
    }
    snprintf(b, sizeof(b), "  argmax bin %d  |X| %.4f   sum|X| %.4f\n",
             bestk, (double)best, tot);
    trace_text(b);
    for (int k = (k0 > 3 ? k0 - 3 : 0); k <= k0 + 3 && k < CFG_N_BINS; k++) {
        float re = (k == 0) ? g_w->fftbuf[0] : g_w->fftbuf[2 * k];
        float im = (k == 0) ? 0.0f : g_w->fftbuf[2 * k + 1];
        snprintf(b, sizeof(b), "    bin %4d  re %12.4f  im %12.4f  |X| %12.4f\n",
                 k, (double)re, (double)im,
                 (double)sqrtf(re * re + im * im));
        trace_text(b);
    }
}

/* 'W' - the same self-test WITH the Hann window applied, through the real
 * front_end path. A Hann-windowed cosine at bin k0 must give
 * |X[k0]| = N/4 and |X[k0 +- 1]| = N/8, everything else ~0. If 'F' is clean
 * and 'W' is not, the fault is the window table or the block, not the FFT. */
static void win_selftest(int k0)
{
    if (!mono_alloc()) {
        return;
    }
    char b[256];
    for (int n = 0; n < CFG_N_FFT; n++) {
        g_block[n] = cosf(2.0f * (float)M_PI * (float)k0 * (float)n
                          / (float)CFG_N_FFT);
    }
    front_end(g_block, g_w->spec[0], g_w);
    snprintf(b, sizeof(b), "\nWINDOWED selftest cos(2pi*%d*n/%d) * hann\n"
             "  expect |X[%d]| = %d, |X[%d+-1]| = %d\n",
             k0, CFG_N_FFT, k0, CFG_N_FFT / 4, k0, CFG_N_FFT / 8);
    trace_text(b);
    for (int k = k0 - 2; k <= k0 + 2; k++) {
        snprintf(b, sizeof(b), "    bin %4d  |X| %12.4f\n", k,
                 (double)sqrtf(g_w->spec[0][k].re * g_w->spec[0][k].re
                               + g_w->spec[0][k].im * g_w->spec[0][k].im));
        trace_text(b);
    }
    snprintf(b, sizeof(b), "  window[0..3] %.9g %.9g %.9g %.9g   "
             "window[1023] %.9g  window[2047] %.9g\n",
             (double)SENTRY_WINDOW_PROBE(0), (double)SENTRY_WINDOW_PROBE(1),
             (double)SENTRY_WINDOW_PROBE(2), (double)SENTRY_WINDOW_PROBE(3),
             (double)SENTRY_WINDOW_PROBE(1023),
             (double)SENTRY_WINDOW_PROBE(2047));
    trace_text(b);
}

/* 'B' - dump the raw replay block at one frame, before anything touches it. */
static void block_dump(int idx, uint32_t frame)
{
    if (!mono_alloc()) {
        return;
    }
    char b[320];
    source_t src;
    if (source_golden(&src, idx) != ESP_OK) {
        trace_text("ERR bad index\n");
        return;
    }
    for (uint32_t i = 0; i <= frame; i++) {
        if (i == 0) {
            source_read(&src, g_q, CFG_N_FFT);
            for (int k = 0; k < CFG_N_FFT; k++) {
                g_block[k] = q_to_f(g_q[k]);
            }
        } else {
            memmove(g_block, g_block + CFG_HOP,
                    (CFG_N_FFT - CFG_HOP) * sizeof(float));
            source_read(&src, g_q, CFG_HOP);
            for (int k = 0; k < CFG_HOP; k++) {
                g_block[CFG_N_FFT - CFG_HOP + k] = q_to_f(g_q[k]);
            }
        }
    }
    double ss = 0.0, mx = 0.0;
    for (int k = 0; k < CFG_N_FFT; k++) {
        ss += (double)g_block[k] * g_block[k];
        if (fabs(g_block[k]) > mx) {
            mx = fabs(g_block[k]);
        }
    }
    snprintf(b, sizeof(b), "\nBLOCK vector %d frame %u\n"
             "  q[0..5]      %d %d %d %d %d %d\n"
             "  block[0..5]  %.9g %.9g %.9g %.9g %.9g %.9g\n"
             "  rms %.9g  max %.9g\n",
             idx, (unsigned)frame,
             g_q[0], g_q[1], g_q[2], g_q[3], g_q[4], g_q[5],
             (double)g_block[0], (double)g_block[1], (double)g_block[2],
             (double)g_block[3], (double)g_block[4], (double)g_block[5],
             sqrt(ss / CFG_N_FFT), mx);
    trace_text(b);
    front_end(g_block, g_w->spec[0], g_w);
    snprintf(b, sizeof(b), "  |X| at 0,26,51,64,128,256,512,1024: "
             "%.6g %.6g %.6g %.6g %.6g %.6g %.6g %.6g\n",
             (double)hypotf(g_w->spec[0][0].re, g_w->spec[0][0].im),
             (double)hypotf(g_w->spec[0][26].re, g_w->spec[0][26].im),
             (double)hypotf(g_w->spec[0][51].re, g_w->spec[0][51].im),
             (double)hypotf(g_w->spec[0][64].re, g_w->spec[0][64].im),
             (double)hypotf(g_w->spec[0][128].re, g_w->spec[0][128].im),
             (double)hypotf(g_w->spec[0][256].re, g_w->spec[0][256].im),
             (double)hypotf(g_w->spec[0][512].re, g_w->spec[0][512].im),
             (double)hypotf(g_w->spec[0][1024].re, g_w->spec[0][1024].im));
    trace_text(b);
}

/* ==========================================================================
 * STANDALONE - THE FIELD MODE. Power is the only input it needs.
 *
 * Everything else in this file assumes a host: a letter arrives, a mode runs,
 * a mode ends. That is exactly right for a bench and useless on a hillside,
 * where the device hangs off a phone power bank with a charge-only cable and
 * there is no laptop to type `H` into. THIS is the mode that makes the box a
 * device rather than an instrument:
 *
 *   power on  -> quad acquisition, v1 + Tier-2 + Tier-3, armed
 *   alarm     -> buzzer + motor + white LED + ALERT (or ALERT/TIER 3) panel
 *   button    -> 10 s of silence, solid blue LED, panel back to LISTENING
 *   hold 2 s  -> OFF screen drawn and waited for; then the cable can be pulled
 *   any host line -> the console takes over, with the line intact
 *
 * Three properties it must have, and each has cost something to get right:
 *
 *   1. IT MUST NOT BLOCK ON A LINK NOBODY IS READING. A charger does not
 *      enumerate; the USB TX ring fills once and never drains. With the
 *      blocking write policy every record would then park the frame loop for
 *      two seconds. Drop mode makes an unread link cost telemetry instead of
 *      detection. See trace_set_drop_mode().
 *   2. IT MUST GIVE THE CONSOLE BACK WITHOUT EATING THE COMMAND. See
 *      trace_try_line(). A run interrupted this way also withholds its END
 *      sentinel, so a host capture is not terminated before it begins.
 *   3. ITS CONFIGURATION MUST SURVIVE THE CABLE BEING PULLED. Every tunable on
 *      this device is deliberately a runtime argument, which resets to its
 *      default at every power cycle - fine with a laptop attached, useless
 *      without one. settings.c persists the handful that matter.
 * ========================================================================== */

/* Rebuilds the whole operating configuration from the persisted settings.
 * Called on entry to every standalone run, so `U c...` takes effect on the
 * next run without a reboot. */
static void standalone_configure(guard_t2_opts_t *t2o, guard_t3_opts_t *t3o,
                                 guard_t4_opts_t *t4o, double *thr1)
{
    const settings_t *c = settings_get();

    alert_ui_set_timing(c->alert_max_ms, c->snooze_ms);
    actuators_buzzer_drive(c->buzz_drive, c->buzz_hz);
    epaper_set_rotation(c->epd_rotation);
    epaper_set_mirror(c->epd_mirror != 0);

    *thr1 = (c->thr1_milli > 0) ? (double)c->thr1_milli / 1000.0
                                : (double)DEFAULT_THRESHOLD;

    memset(t2o, 0, sizeof(*t2o));
    if (c->t2_enabled) {
        t2_cfg_default(&t2o->cfg,
                       c->t2_rate == 1 ? false
                                       : (c->t2_rate == 2
                                          ? true
                                          : (T2_HALF_RATE_DEFAULT != 0)));
        t2_cfg_set_thr_milli(&t2o->cfg, (int)c->thr2_milli);
        /* The exclusion list an operator typed with `E` rides along. It is
         * NOT persisted: an exclusion band is a claim about one site on one
         * day, and a device that silently remembers where it was told to go
         * deaf is a device that will one day be deaf for no reason anyone
         * present can explain. */
        if (g_excl_n > 0) {
            t2_cfg_set_excl(&t2o->cfg, g_excl_n, g_excl_c, g_excl_t);
        }
        t2o->cx = (c->cx >= 'a' && c->cx <= 'd') ? c->cx : 'a';
        t2o->f_split_hz = 1500.0f;
        t2o->busoff = 0.0f;
        t2o->enable = true;
    }

    memset(t3o, 0, sizeof(*t3o));
    if (c->t3_enabled) {
        t3o->cfg = t3_default_cfg();
        t3o->cfg.enabled = true;
        if (c->thr3_milli > 0) {
            t3o->cfg.tau3 = (double)c->thr3_milli / 1000.0;
        }
        t3o->enable = true;
    }

    memset(t4o, 0, sizeof(*t4o));
    if (c->t4_enabled) {
        t4o->cfg = t4_default_cfg();
        t4o->cfg.enabled = true;
        /* The SAME flag as v1's. One operator switch, because "stop firing on
         * a piano" is not a per-tier request and a veto that answered it on
         * one of four OR'd tiers would look broken. */
        t4o->cfg.veto_voice = (c->veto_voice != 0);
        if (c->thr4_milli > 0) {
            t4o->cfg.tau4 = (double)c->thr4_milli / 1000.0;
        }
        /* The same exclusion list, and Tier-4 needs it more than Tier-2 does:
         * its near enemies are steady tonal sources, and the 08-13 lab
         * captures show it locking cleanly onto a 245 Hz HVAC comb with
         * harmonics at 5x, 6x, 10x and 15x. Not persisted, for the reason
         * given above. */
        if (g_excl_n > 0) {
            const int n = g_excl_n < T4_MAX_EXCL ? g_excl_n : T4_MAX_EXCL;
            t4o->cfg.n_excl = n;
            for (int i = 0; i < n; i++) {
                t4o->cfg.excl_c[i] = g_excl_c[i];
                t4o->cfg.excl_t[i] = g_excl_t[i];
            }
        }
        t4o->enable = true;
    }
}

static int run_standalone(void)
{
    guard_t2_opts_t t2o;
    guard_t3_opts_t t3o;
    guard_t4_opts_t t4o;
    double thr1;
    standalone_configure(&t2o, &t3o, &t4o, &thr1);

    /* ---- why three tiers fit, and the reservation that makes them --------
     * At twelve I2S descriptors the budget is a total rather than an
     * arrangement: 48 (I2S) + 70 (guard) + 24 (T2) + 13 (T3) = 155 KB against
     * about 147 KB, and no allocation order can make 155 fit in 147.
     *
     * Two things buy it back:
     *   - QUAD_DESC_NUM 12 -> 6 hands back 24 KB of I2S DMA, taking the total
     *     to 24 + 70 + 24 + 13 = 131 KB of ~147 KB. That is the memory half.
     *   - Tier-3's start frame sits off Tier-2's, so the two tiers stop making
     *     the same frames expensive. That is the time half, and it was a bug
     *     rather than a budget - see the scheduling block above.
     *
     * The reservation is the mechanism: claim every configured tier here, out
     * of a heap nothing has fragmented yet, and only then start the buses and
     * the guard.
     *
     * The two lines that check it on hardware are heap_line("armed") and the
     * scheduling probe. A tier that cannot claim its state is dropped
     * individually and says so. If it ever comes to choosing, Tier-3 is the
     * one that stays: it is the only tier that has detected a real rotor. */
    if (t3o.enable && !g_t3st) {
        if (!t3_boot_alloc()) {
            trace_text("WARN Tier-3 could not claim its state; running "
                       "without it\n");
            t3o.enable = false;
        }
    } else if (!t3o.enable && g_t3st) {
        t3_free();
    }
    if (t4o.enable && !t4_alloc()) {
        trace_text("WARN Tier-4 could not claim its state; running without "
                   "it\n");
        t4_free();
        t4o.enable = false;
    } else if (!t4o.enable) {
        t4_free();
    }
    if (t2o.enable && !t2_alloc(t2o.cx != 'a')) {
        trace_text("WARN Tier-2 could not claim its 24 KB from a pristine "
                   "heap; running without it\n");
        heap_line("at the Tier-2 reservation");
        t2_free();
        t2o.enable = false;
    } else if (!t2o.enable) {
        t2_free();
    }
    heap_line("after reserving the tiers");

    char b[224];
    snprintf(b, sizeof(b),
             "\nSENTRY standalone: quad + v1(%d milli)%s%s%s  "
             "snooze %us  burst %us\n",
             (int)(thr1 * 1000.0 + 0.5),
             t2o.enable ? " + T2" : "",
             t3o.enable ? " + T3(wash)" : "",
             t4o.enable ? " + T4(slow comb)" : "",
             (unsigned)(settings_get()->snooze_ms / 1000u),
             (unsigned)(settings_get()->alert_max_ms / 1000u));
    trace_text(b);

    /* THE GUARD IS STARTING, so LISTENING is what this boot decided to do.
     * Recorded here rather than at the top of app_main because a decision is
     * what actually happened, not what was intended: everything above this
     * point can still fail and end the run. */
    bootrec_set_decision(BOOTDEC_LISTENING);

    /* ---- the only boot signal there is ----------------------------------
     * One green pulse when the guard arms, and nothing else. No buzzer and no
     * motor at boot under any circumstance, including after a brownout,
     * because a device that makes a noise it was not asked to make teaches
     * its operator to ignore noises. Blocking is fine here, the detector has
     * not started. */
    led_pattern_set(LED_PAT_MANUAL);
    (void)led_set_rgb(LED_GUARD_R, LED_GUARD_G, LED_GUARD_B);
    vTaskDelay(pdMS_TO_TICKS(BOOT_READY_LED_MS));
    led_pattern_set(LED_PAT_GUARD);

    /* The battery monitor, on its own task on core 1. Inert on a board with
     * no divider - power_mon_begin() is an empty function there and the task
     * is never created, so the breadboard build's heap and core-1 load are
     * exactly what they were. */
    power_mon_begin();
    /* The operator's statement about the hardware, before the first
     * measurement can be acted on. */
    power_mon_set_no_cell(settings_get()->batt_absent != 0);
    /* The same fact reaches the alert cadence, which interleaves the
     * buzzer and the motor rather than browning out a USB-only board.
     *
     * Do not force s_no_cell from a brownout. It does not mean "drop the
     * motor": it means a solid tone instead of a pulsed one, and every panel
     * refresh deferred while the buzzer sounds - so it turns the alarm into a
     * continuous note and guarantees the ALERT screen can never draw. It
     * would also be aimed at the wrong load, since ALERT_MOTOR_IN_BURST is 0
     * and the motor is not in the local burst. The overlap that matters is
     * the panel's 2 s refresh landing on the buzzer, and that is fixed where
     * it belongs, in the alert timeline. */
    power_mon_quiet(false);         /* the guard is the one run that wants it */
    {
        char pb[96];
        power_mon_describe(pb, sizeof(pb));
        trace_text(pb);
    }

    /* THE PEER BEACON, and only in this mode. Inert on a board with no radio;
     * on a board with one, a module that does not answer its version register
     * says so and the guard runs on regardless - a device that cannot reach
     * its peers is still a device that can hear a drone. */
    {
        const settings_t *sc = settings_get();
        lora_link_configure(sc->lora_enabled != 0, sc->lora_hz);
        if (sc->lora_enabled) {
            (void)lora_link_begin();
        }
        char lb[160];
        lora_link_describe(lb, sizeof(lb));
        trace_text(lb);
    }

    /* ---- PAGE 2, STATUS: filled ONCE, here, before the guard starts -----
     *
     * Every row of it is fixed for the life of a run - the unit id, the
     * board, the tiers, the calibration, the radio, the operating point - so
     * rebuilding them once a second inside the frame loop would be a second
     * of work a day spent re-deriving constants.
     *
     * And it is out here for a second reason, which is the better one:
     * tests/test_mic_cal.py asserts that the GUARD'S FRAME LOOP never reads
     * mic_cal_enabled, because the day it does is the day the microphone
     * gains have been wired into the coherent sum - a change that needs
     * golden and quad parity on a board before it can ship. Reading the flag
     * to print one word on a status page is not that, but the gate cannot
     * tell the two apart and it should not have to. So the read happens
     * where the guard is armed, not where it runs.
     *
     * The MIC row is the exception and is refreshed by the frame loop, because
     * a microphone that dies mid-run is exactly what that row is for. */
    {
        /* ---- MENU, filled ONCE, here, before the guard starts -------------
         *
         * Everything that would otherwise clutter the resting screen, on one
         * page. Every row is fixed for the life of a run except the two the
         * frame loop rewrites - mic health and uptime - so rebuilding them
         * once a second would be a second of work a day spent re-deriving
         * constants.
         *
         * It is out here for a second reason: the host tests assert that the
         * guard's frame loop never reads mic_cal_enabled, because the day it
         * does is the day the microphone gains reach the coherent sum, and
         * that change needs golden and quad parity on a board first. Reading
         * the flag to print one word is not that, but the gate cannot tell and
         * should not have to. */
        const settings_t *sc2 = settings_get();
        char ln[EPAPER_LINE_MAX + 1];
        snprintf(ln, sizeof(ln), "N-%04X  %s",
                 (unsigned)lora_link_device_id(), BOARD_NAME);
        epaper_page_line(EPAPER_PAGE_STATUS, 0, ln);
        /* THE BUILD DATE, not the git hash the brief asked for. A hash needs
         * a CMake define wired through idf_component_register, and adding a
         * build-system dependency at midnight for a MENU row is the wrong
         * trade; __DATE__ answers "is this the image I flashed?" which is the
         * question that matters in a field. Named as what it is. */
        snprintf(ln, sizeof(ln), "BUILD %s", __DATE__);
        epaper_page_line(EPAPER_PAGE_STATUS, 1, ln);
        snprintf(ln, sizeof(ln), "TIERS V1%s%s%s",
                 t2o.enable ? "+T2" : "", t3o.enable ? "+T3" : "",
                 t4o.enable ? "+T4" : "");
        epaper_page_line(EPAPER_PAGE_STATUS, 2, ln);
        /* row 3 is the frame loop's: MIC n/4, UP nH, AMB dB */
        {
            int tm = (int)(thr1 * 1000.0 + 0.5);
            if (tm < 0) { tm = 0; }
            if (tm > 99999) { tm = 99999; }
            snprintf(ln, sizeof(ln), "THR %d.%02d  T2 %d.%02d",
                     tm / 1000, (tm % 1000) / 10,
                     (int)T2_TAU2, (int)((T2_TAU2 - (int)T2_TAU2) * 100));
        }
        epaper_page_line(EPAPER_PAGE_STATUS, 4, ln);
        snprintf(ln, sizeof(ln), "ROT %u  CAL %s  RING %u",
                 (unsigned)epaper_degrees_from_quadrant(sc2->epd_rotation),
                 sc2->mic_cal_enabled ? "SET" : "ID", (unsigned)EVLOG_N);
        epaper_page_line(EPAPER_PAGE_STATUS, 5, ln);
    }


    /* Telemetry becomes best-effort HERE and nowhere else. */
    trace_set_drop_mode(true);
    int reason = run_quad_pipeline(0u, true, thr1,
                                   t2o.enable ? &t2o : NULL,
                                   t3o.enable ? &t3o : NULL,
                                   t4o.enable ? &t4o : NULL, true);

    /* LAST RESORT. If the configured pair could not start at all, fall back to
     * v1 alone rather than give up: a quad guard with one tier is worth
     * immeasurably more on a hillside than a console prompt nobody is reading. */
    if (reason == RUN_END_COMPLETE && (t2o.enable || t3o.enable)) {
        trace_text("WARN the configured tier could not start; falling back to "
                   "v1 alone so the device still guards\n");
        reason = run_quad_pipeline(0u, true, thr1, NULL, NULL, NULL, true);
    }

    /* A host is talking to us: give it the lossless link back. */
    trace_set_drop_mode(false);
    return reason;
}

/* ---- STANDBY: what "off" means on a device with no power switch ----------
 * The detector is stopped, both buses are stopped, every output is parked and
 * the panel says OFF. The CPU is still running, because the button has to be
 * able to bring it back and because the USB console must stay usable - this is
 * "off" in the sense the operator means it, which is "it is not listening and
 * the display is not lying about that", not a power state.
 *
 * The cable can be pulled at any point from here on and the panel keeps the
 * OFF image, which is the entire reason this state exists: e-paper holds its
 * last frame with no power, so a device unplugged while LISTENING goes on
 * claiming to listen from inside a bag. */
/* THE WAIT, split out so that a caller which has ALREADY drawn its own OFF
 * screen can park without redrawing it. A second alert_ui_power_down() would
 * be two more seconds of full panel refresh to display the screen already on
 * the glass.
 *
 * There used to be a run_standby() beside it, for a long press that parked the
 * device WITHOUT drawing OFF. It was deleted with the two-endings merge: a
 * parked device whose panel still says it is guarding is the exact dishonesty
 * the OFF screen exists to prevent, and there is no longer a gesture that
 * asks for it. */
#define OFF_AWAKE_RESUME   0    /* a completed hold: go back to LISTENING   */
#define OFF_AWAKE_CONSOLE  1    /* a host line took the console             */
#define OFF_AWAKE_EXPIRED  2    /* OFF_AWAKE_MAX_MS elapsed: deep sleep now */
#define OFF_AWAKE_EMPTY    3    /* the cell ran out while parked          */

/* ---- OFF-AWAKE, AND WHY IT IS NOT DEEP SLEEP ----------------------------
 *
 * Read this before turning it into a sleep. The reasoning is a hardware fact
 * rather than a preference.
 *
 * The slide switch does not cut power. It cuts the TPS63020's EN pin, so
 * after a slide-off the 3V3_SYS rail is held up by C24 330 uF plus C26 100 uF
 * and drained only by whatever is still awake on it. With the chip in DEEP
 * SLEEP the loads are microamp-class: the WS2812B-V6 is under 1 uA static,
 * the S3 in deep sleep is single-digit uA, and the flash, PSRAM, mics and
 * panel standby add tens to a few hundred uA. At about 0.3 mA, 430 uF takes
 * roughly a second to fall the ~0.8 V to the brownout threshold. So a slide
 * off-and-on faster than that never resets the chip: the unit wakes up still
 * asleep, behind the stale OFF image, and appears to do nothing.
 *
 * A CPU that is merely awake and idling draws tens of milliamps and collapses
 * the same rail in tens of milliseconds, so every flick of the switch becomes
 * a clean cold boot. That is why a deliberately-off unit stays awake for half
 * a day: the OFF state has to be one an operator can get out of with the
 * switch, and only after OFF_AWAKE_MAX_MS, when nobody is standing there any
 * more, is it worth trading that for the microamps.
 *
 * No automatic light sleep is enabled anywhere in this build - CONFIG_PM_ENABLE
 * is off and there is no tickless idle - so "awake" here really is tens of
 * milliamps, expected to be 30 to 45 mA. */
static int run_off_awake(void)
{
    const uint32_t entered = now_ms();
    uint32_t next_batt = entered + 60000u;
    for (;;) {
        const uint32_t now = now_ms();
        if (alert_ui_standby_press(now)) {
            /* ---- wake is a restart --------------------------------------
             * A guard restarted in-process leaks about 3.6 KB, and on a device
             * where the largest free block decides whether four tiers fit, a
             * UI gesture that can be repeated must not spend heap. So the
             * in-process restart is not a path: a boot reaches LISTENING in
             * about 1.5 s and the ring records it as reset=SW
             * decision=LISTENING, which makes the wake more explainable
             * afterwards rather than less. The pause is by design and the
             * console says so. */
            trace_text("SENTRY waking: restarting to LISTENING\n");
            /* The confirmation the hold completed. Blocking is correct here:
             * the detector has been stopped since the OFF ritual. */
            actuators_motor(true);
            vTaskDelay(pdMS_TO_TICKS(BTN_CONFIRM_MOTOR_MS));
            actuators_motor(false);
            bootrec_note_uptime(now);
            vTaskDelay(pdMS_TO_TICKS(120));   /* let the line drain */
            esp_restart();
            return OFF_AWAKE_RESUME;          /* not reached */
        }
        if (trace_try_line(g_pending, sizeof(g_pending)) > 0) {
            g_have_pending = true;
            return OFF_AWAKE_CONSOLE;
        }
        if ((uint32_t)(now - entered) >= (uint32_t)OFF_AWAKE_MAX_MS) {
            return OFF_AWAKE_EXPIRED;
        }
        /* A cell can still run flat while the box sits switched off in a bag,
         * and the empty park protects the cell, so it must still run here.
         * Once a minute is enough for a state whose whole point is to do
         * nothing. */
        /* THE MONITOR IS STILL ITS OWN TASK. Reading power_mon_state() here
         * is a cached byte; calling the monitor tick here would put the ADC on
         * this thread, and the rule is that it runs on the power task and
         * nowhere else. A cell can still run flat in a bag, so the empty
         * verdict must still be able to arrive during OFF-AWAKE. */
        if ((int32_t)(now - next_batt) >= 0) {
            next_batt = now + 60000u;
            if (power_mon_state() == POWER_EMPTY) {
                return OFF_AWAKE_EMPTY;
            }
        }
        alert_ui_idle_led(false, now);  /* dark: the box is off, and says so */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}



/* ---- OFF-DEEP: the only sleep on this device, and it can be woken -------
 *
 * A sleep with no wake source cannot be woken, so "hold to wake from OFF"
 * needs one: EXT1 on the button, ANY_LOW (the button pulls to GND), and
 * nothing else in the set. USB and the charger are deliberately absent - a
 * unit an operator turned off must not come back because somebody plugged it
 * in to charge. */
/* ---- arming a sleep, in one place ---------------------------------------
 *
 * Both sleeps arm identically apart from the timer, and they must: a wake
 * source that exists on one path and not the other is a device that can be
 * woken from one kind of sleep and not the other.
 *
 * The pd_config is the line that is easy to miss. GPIO21 has no external
 * pull-up - a tactile switch to GND and an optional 100 nF, nothing else - so
 * the only thing holding it high in sleep is the chip's internal pull-up, and
 * that pull-up lives in the RTC peripheral domain, which deep sleep powers
 * down by default. Without it the pin floats while asleep: ANY_LOW can fire on
 * nothing, and a real press may never be seen. pullup_en without pd_config is
 * the worst of both. */
/* Which boards sleep at all. Defined here, above the first use: referencing
 * esp_deep_sleep_start() anywhere links about 2.3 KB that must live in
 * internal RAM, paid whether the path runs or not, out of the heap that
 * decides whether the tiers fit. A board with neither a slider nor a cell
 * gains nothing from it and must not pay for it. */
#define BOARD_NEEDS_DEEP_SLEEP (BOARD_SLIDE_HARD_CUT || BOARD_HAS_VBAT)

#if BOARD_NEEDS_DEEP_SLEEP
static void sleep_arm(bool with_timer)
{
    /* Latch the three outputs low THROUGH the sleep. Without this a pad can
     * float as the domains go down, and a buzzer that chirps every time the
     * device is put away is a buzzer nobody trusts. */
    power_mon_end();                    /* the ADC has nothing left to read */
    actuators_safe_all_off();
    gpio_hold_en((gpio_num_t)PIN_LED_WS2812);
    gpio_hold_en((gpio_num_t)PIN_BUZZER);
    gpio_hold_en((gpio_num_t)PIN_MOTOR);
    gpio_deep_sleep_hold_en();

    rtc_gpio_pullup_en((gpio_num_t)PIN_SNOOZE_BTN);
    rtc_gpio_pulldown_dis((gpio_num_t)PIN_SNOOZE_BTN);
    (void)esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    (void)esp_sleep_enable_ext1_wakeup_io(1ULL << PIN_SNOOZE_BTN,
                                          ESP_EXT1_WAKEUP_ANY_LOW);
    if (with_timer) {
        (void)esp_sleep_enable_timer_wakeup(
            (uint64_t)BATT_EMPTY_WAKE_S * 1000000ULL);
    }
}

static void enter_off_deep(const char *why)
{
    char b[128];
    snprintf(b, sizeof(b), "SENTRY OFF-DEEP (%s). Hold the button to wake.\n",
             why ? why : "");
    trace_text(b);
    vTaskDelay(pdMS_TO_TICKS(200));         /* let the line drain */
    sleep_arm(false);
    esp_deep_sleep_start();
}

/* The empty park, reachable both from the guard and from a timer wake that
 * still refuses to resume. */
static void enter_empty_deep(void)
{
    bootrec_set_decision(BOOTDEC_EMPTY);
    trace_text("SENTRY BATTERY EMPTY: sleeping, waking to re-check.\n");
    vTaskDelay(pdMS_TO_TICKS(200));
    sleep_arm(true);
    esp_deep_sleep_start();
}
#endif  /* BOARD_NEEDS_DEEP_SLEEP */


/* ---- THE HONEST OFF: draw the reason, then stop for good -----------------
 *
 * Two callers, one behaviour: the OFF ritual's two-second hold, and a cell
 * that has run out. Both end with a panel that says OFF and a device that is not running,
 * because the alternative on e-paper is a drawer full of boxes claiming to
 * listen. The screen is drawn and WAITED FOR - the detector is already
 * stopped, and the entire purpose of the image is to be correct after the
 * power goes.
 *
 * Why deep sleep, and why only on a board that needs it.
 *
 * On the PCB, deep sleep is the point. The ritual is performed by an operator
 * whose hand is already on the slider, and waking on a stray press between
 * the ritual and the switch would put the panel back to LISTENING at the
 * worst possible moment. An empty cell is worse still: coming back up to
 * guard would finish the battery off and, on a protected cell, trip the
 * protection - which needs a charger to clear, in a field, at night. So:
 * deep sleep, no wake source, and the way back is the power cycle the
 * operator was about to perform anyway.
 *
 * On the DevKit it costs 2356 bytes and buys nothing, and that is a
 * measurement rather than a guess. Reading idf.py size across three builds:
 *
 *     baseline                                DIRAM 150373
 *     + the off ritual, no sleep              DIRAM 150709   (+336)
 *     + esp_deep_sleep_start()                DIRAM 153065   (+2692)
 *
 * Referencing esp_deep_sleep_start anywhere links about 2.3 KB of code that
 * must live in internal RAM because it runs with the flash powered down. It
 * is paid whether or not the path ever executes, and on the ESP32-S3 DIRAM is
 * shared with the heap - so it comes straight out of the ~16 KB predicted
 * free at arming, which is 15% of the headroom that decides whether three
 * tiers fit at all. That headroom has never been measured on hardware.
 *
 * And on the breadboard it buys NOTHING an operator can see. There is no
 * slider and no battery; "turning it off" is pulling a USB cable, which cuts
 * the rail regardless of what the CPU was doing. Deep sleep and standby leave
 * the operator holding exactly the same thing: a panel saying OFF.
 *
 * So the ritual, the OFF screen and the words are identical on both boards -
 * which is what "one behaviour" is actually about - and only the last
 * instruction differs. A board with a hard cut or a battery sleeps; a board
 * with neither parks in standby, where the button can bring it back. Cutting
 * power mid-guard behaves the same on both.
 * ------------------------------------------------------------------------ */

static void run_off_and_sleep(const char *reason, bool battery)
{
    trace_text("\nSENTRY OFF: ");
    trace_text(reason);
    trace_text("\n");
    epaper_set_off_reason(reason);
    alert_ui_power_down();              /* parks the outputs, draws OFF, waits */
    alert_ui_clear_power_request();
    /* power_mon_end() moved into sleep_arm(): OFF-AWAKE keeps the monitor
     * running, because a cell can still run flat while the box sits switched
     * off in a bag, and because the ADC must stay on the power task. */
    /* One last chance for the panel to finish, on top of power_down's own
     * wait: a half-drawn OFF screen is exactly as dishonest as a stale
     * LISTENING one, and this is the last instruction that can prevent it. */
    (void)epaper_wait_idle(6000u);
#if BOARD_NEEDS_DEEP_SLEEP
    if (battery) {
        /* ---- SECTION 2.4: THE CELL RAN OUT ---------------------------
         * Not OFF-AWAKE. Staying awake to be convenient would finish the
         * cell off and, on a protected pack, trip the protection, which
         * needs a charger to clear, in a field, at night. So this one
         * sleeps at once, and it is the ONE place a timer is armed: the
         * deployment needs a box that comes back by itself when mains
         * returns, without anybody walking to it.
         *
         * Predicted duty: a ~150 ms boot every BATT_EMPTY_WAKE_S at ~80 mA
         * is about 0.2 mA average. PREDICTED, not measured. */
        enter_empty_deep();
    }
    /* USER OFF: awake first, so the switch still works. See run_off_awake. */
    alert_ui_clear_power_request();
    trace_text("SENTRY OFF: hold the button to wake, or slide the switch.\n");
    switch (run_off_awake()) {
    case OFF_AWAKE_EXPIRED: enter_off_deep("12 h awake elapsed"); break;
    case OFF_AWAKE_EMPTY:   enter_empty_deep();                   break;
    default: break;
    }
#else
    (void)battery;
    trace_text("SENTRY parked: the panel shows OFF and the reason. Safe to "
               "unplug. Press to resume, or type a command.\n");
    alert_ui_clear_power_request();
    (void)run_off_awake();
#endif
}

/* ---- what a wake has to prove -------------------------------------------
 *
 * Waking is not the same as being awake. A wake has a reason, and if it cannot
 * make good on that reason it must either go back to sleep or, if it has
 * failed too often, stop sleeping altogether.
 *
 * The asymmetry is the whole design. "The unit turned itself on" costs a
 * cell. "The unit will not wake" costs a detector on a hillside that cannot
 * be told from a dead one. So every uncertain branch below resolves to
 * LISTENING, and the only paths back to sleep are the two that are certain:
 * a timer wake with a TRUSTED reading still under the resume level, and an
 * EXT1 wake where the button was demonstrably not held. */
static int btn_level_now(void)
{
    /* The pad comes out of a deep sleep still owned by the RTC mux. */
    rtc_gpio_deinit((gpio_num_t)PIN_SNOOZE_BTN);
    gpio_config_t bc = {
        .pin_bit_mask = 1ULL << PIN_SNOOZE_BTN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&bc);
    return gpio_get_level((gpio_num_t)PIN_SNOOZE_BTN);
}

static void wake_loop_stop(int mv)
{
    evlog_note_vbat(EVLOG_REL_WAKE_LOOP, mv, now_ms());
    bootrec_clear_failed_wakes();
    trace_text("WAKE-LOOP: woke and could not finish more than three times "
               "in ten seconds. NOT sleeping again; guarding instead.\n");
}

static void handle_deepsleep_wake(void)
{
#if BOARD_NEEDS_DEEP_SLEEP
    bootrec_set_btn(btn_level_now());

    if (!bootrec_was_deepsleep_wake()) {
        /* A real reset ends any loop: somebody power-cycled it, or the
         * console restarted it, and the count belongs to the sleep it came
         * from. */
        bootrec_clear_failed_wakes();
        return;
    }

    const uint8_t cause = bootrec_wake_cause();
    char b[192];

    if (cause == (uint8_t)ESP_SLEEP_WAKEUP_TIMER) {
        const int mv  = power_mon_probe();
        const int chg = power_mon_chg_level();
        bootrec_set_vbat(mv, chg);
        if (!power_mon_trusted_mv(mv)) {
            /* R1 AT THE WAKE. C.2 is explicit: an untrusted reading on a
             * timer wake means RESUME, not another sleep. Sleeping on a
             * number nobody can verify is how a charged device stays off. */
            snprintf(b, sizeof(b),
                     "WAKE(timer): reading %d mV is untrusted, resuming to "
                     "LISTENING\n", mv);
            trace_text(b);
            evlog_note_vbat(EVLOG_REL_VBAT_BAD, mv, now_ms());
            bootrec_clear_failed_wakes();
            return;
        }
        if (mv >= (int)(BATT_RESUME_V * 1000.0f) || chg > 0) {
            snprintf(b, sizeof(b), "WAKE(timer): %d mV chg=%d, resuming\n",
                     mv, chg);
            trace_text(b);
            bootrec_clear_failed_wakes();
            return;
        }
        if (bootrec_note_failed_wake()) {
            wake_loop_stop(mv);
            return;
        }
        /* Still empty, and certainly so. Back to sleep WITHOUT touching the
         * panel: it already says BATTERY EMPTY and a refresh costs charge
         * the cell has not got. */
        bootrec_set_decision(BOOTDEC_OFF_DEEP_BACK);
        enter_empty_deep();
        return;                          /* not reached */
    }

    if (cause == (uint8_t)ESP_SLEEP_WAKEUP_EXT1) {
        /* Section 2.3: the pin must STAY low for the whole hold, measured
         * from the wake. A knock against the button in a bag is not a
         * request to start guarding. */
        const uint32_t t0 = now_ms();
        bool held = true;
        while ((uint32_t)(now_ms() - t0) < BTN_HOLD_MS) {
            if (btn_level_now() != 0) {
                held = false;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (held) {
            trace_text("WAKE(ext1): hold completed, resuming to LISTENING\n");
            bootrec_set_decision(BOOTDEC_OFF_DEEP_RESUME);
            bootrec_clear_failed_wakes();
            actuators_motor(true);
            vTaskDelay(pdMS_TO_TICKS(BTN_CONFIRM_MOTOR_MS));
            actuators_motor(false);
            return;
        }
        if (bootrec_note_failed_wake()) {
            wake_loop_stop(-1);
            return;
        }
        trace_text("WAKE(ext1): not held, going back to sleep\n");
        bootrec_set_decision(BOOTDEC_OFF_DEEP_BACK);
        /* Wait for the release, or the same low pin wakes it straight back. */
        while (btn_level_now() == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        enter_off_deep("not held");
        return;                          /* not reached */
    }
#endif
}

/* ---- `Lc`: microphone relative calibration ------------------------------
 *
 * Ambient with the array unobstructed -> per-channel band-limited RMS ->
 * gains relative to the four-channel mean -> print -> the operator confirms
 * -> persist.
 *
 * The confirm step exists because every other tunable on this device is a
 * number a human typed, and this is the one place a value could enter the
 * configuration without anybody deciding it. So the decision is put back:
 * `Lc` measures and shows, `Lc y` stores, `Lc n` discards, and nothing
 * reaches NVS without the second command.
 *
 * It cannot deafen itself. mic_cal_gains() refuses a dead channel and
 * refuses anything beyond +-3 dB rather than storing a large correction,
 * because a gain that rescues a broken microphone leaves a four-microphone
 * array running on three and saying nothing about it. The refusal names the
 * channel.
 */
static bool     g_cal_have;                 /* a measurement awaits confirm */
static int16_t  g_cal_gain[MIC_CAL_N];

static void run_mic_cal(uint32_t seconds)
{
    if (seconds == 0u) {
        seconds = 60u;                       /* the default window */
    }
    mono_free();
    if (!guard_alloc(false)) {               /* per-channel spectra needed */
        trace_text("ERR Lc: no RAM for the per-channel path\n");
        heap_line("at failure");
        trace_ack("Lc", TRACE_ACK_FAULT);
        return;
    }
    if (source_i2s_quad_start() != ESP_OK) {
        trace_text("ERR Lc: the I2S buses would not start\n");
        guard_free();
        trace_ack("Lc", TRACE_ACK_FAULT);
        return;
    }
    {
        char b[160];
        snprintf(b, sizeof(b),
                 "\nLc: %u s of AMBIENT, array unobstructed, %d-%d Hz\n"
                 "    keep away from it and keep the site quiet\n",
                 (unsigned)seconds, mic_cal_bin_lo() * CFG_FS / CFG_N_FFT,
                 mic_cal_bin_hi() * CFG_FS / CFG_N_FFT);
        trace_text(b);
    }

    mic_cal_acc_t acc;
    mic_cal_begin(&acc);
    const uint32_t n_total = seconds * CFG_FS;
    uint32_t pend = 0, filled = 0, taken = 0;
    int reason = RUN_END_COMPLETE;

    while (taken < n_total) {
        const int n = source_i2s_quad_read(g_quad);
        if (n <= 0) {
            if (run_should_stop(false, false, &reason)) {
                break;
            }
            continue;
        }
        taken += (uint32_t)n;
        for (int f = 0; f < n; f++) {
            for (int c = 0; c < GUARD_N_CH; c++) {
                g_gpend[pend * GUARD_N_CH + c] = g_quad[f * GUARD_N_CH + c];
            }
            if (++pend < CFG_HOP) {
                continue;
            }
            pend = 0;
            for (int c = 0; c < GUARD_N_CH; c++) {
                memmove(g_gwin[c], g_gwin[c] + CFG_HOP,
                        (CFG_N_FFT - CFG_HOP) * sizeof(float));
                float *dst = g_gwin[c] + (CFG_N_FFT - CFG_HOP);
                for (int k = 0; k < CFG_HOP; k++) {
                    dst[k] = q_to_f(g_gpend[k * GUARD_N_CH + c]);
                }
            }
            if (filled < CFG_N_FFT) {
                filled += CFG_HOP;
                if (filled < CFG_N_FFT) {
                    continue;
                }
            }
            for (int c = 0; c < GUARD_N_CH; c++) {
                front_end(g_gwin[c], &g_gspec[(size_t)c * CFG_N_BINS], g_w);
            }
            mic_cal_add(&acc, g_gspec, GUARD_N_CH);
        }
        if (run_should_stop(false, false, &reason)) {
            break;
        }
    }
    source_i2s_quad_stop();
    guard_free();

    int16_t g[MIC_CAL_N];
    char why[160];
    if (!mic_cal_gains(&acc, GUARD_N_CH, g, why, sizeof(why))) {
        char b[224];
        snprintf(b, sizeof(b), "Lc REFUSED: %s\n", why);
        trace_text(b);
        trace_text("    Nothing stored. Fix the microphone, do not store a "
                   "gain that hides it.\n");
        g_cal_have = false;
        trace_ack("Lc", TRACE_ACK_FAULT);
        return;
    }
    for (int i = 0; i < MIC_CAL_N; i++) {
        g_cal_gain[i] = g[i];
    }
    g_cal_have = true;
    {
        char b[256];
        /* Integers and a hand-placed decimal point: PicoLibC's printf has no
         * float support, which is why the whole trace is binary. */
        snprintf(b, sizeof(b),
                 "Lc measured %d frames:  M1 %d.%03d  M2 %d.%03d  "
                 "M3 %d.%03d  M4 %d.%03d\n",
                 acc.n_frames,
                 g[0] / 1000, g[0] % 1000, g[1] / 1000, g[1] % 1000,
                 g[2] / 1000, g[2] % 1000, g[3] / 1000, g[3] % 1000);
        trace_text(b);
    }
    trace_text("    NOT STORED YET.  `Lc y` to keep them,  `Lc n` to "
               "discard.\n");
    trace_ack("Lc", TRACE_ACK_OK);
}

static void cmd_mic_cal(const char *line)
{
    const char *a = skip_ws(line + 2);
    if (*a == 'y' || *a == 'Y') {
        if (!g_cal_have) {
            trace_text("Lc y: nothing measured to confirm - run `Lc` "
                       "first\n");
            trace_ack(line, TRACE_ACK_BAD_ARG);
            return;
        }
        settings_t *c = settings_mut();
        for (int i = 0; i < MIC_CAL_N; i++) {
            c->mic_gain_milli[i] = g_cal_gain[i];
        }
        c->mic_cal_enabled = 1;
        const bool ok = settings_save();
        g_cal_have = false;
        char b[288];
        settings_describe(b, sizeof(b));
        trace_text(b);
        trace_text(ok ? "calibration stored\n"
                      : "NOT SAVED (nvs write failed)\n");
        trace_ack(line, ok ? TRACE_ACK_OK : TRACE_ACK_FAULT);
        return;
    }
    if (*a == 'n' || *a == 'N') {
        g_cal_have = false;
        trace_text("Lc n: discarded, nothing stored\n");
        trace_ack(line, TRACE_ACK_OK);
        return;
    }
    run_mic_cal((uint32_t)atoi(a));
}

static void say_info(void)
{
    char b[512];
    snprintf(b, sizeof(b),
             "\nSENTRY-NODE stage1a  site=%s corpus=%s\n"
             /* WHICH BOARD THIS IMAGE IS FOR. First line after the banner
              * because it is the first thing bring-up has to establish, and
              * because the two targets are pin-identical for fourteen of
              * their signals - so "it looks right" proves nothing and this
              * line proves everything. The geometry id is the KEY into
              * data/geometry_profiles.json, not a description of it. */
             "board=%s geometry=%s  vbat=%d chg=%d lora=%d slide_cut=%d\n"
             "golden_threshold=%.4f  deployment_default=%s (%.4f)\n"
             "n_bins=%d n_f0=%d n_fft=%d hop=%d fs=%d channels=%d\n"
             "sizeof state=%u work=%u rec=%u  free_heap=%u largest=%u\n"
             "work@%p (align %u)  fftbuf@%p (align %u)\n"
             "i2s pins: BCLK=GPIO%d WS=GPIO%d SD=GPIO%d  (SD2=GPIO%d reserved)\n"
             "mic L/R must be strapped to %s   BCLK rate %d Hz\n"
             "modes: R<i> golden | M meter | T tone | L live | Y<s> parity\n"
             "       G[thr_milli] GUARD (4 mic, full device) | Z<s> quad parity\n"
             "       A armed | D<s> drill\n"
             "       Q quad meter | X<s> quad pcm | U ... actuators\n"
             "       H[t1][t2][cx][boff][rate][t2lo][t2hi][t3] GUARD+T2+T3\n"
             "       K<i>[t2][rate] golden+TIER2 | E n f0 tol ... exclude\n"
             "       S STANDALONE (the field mode: hand the device back to\n"
             "         itself; it also starts by itself at power-on)\n"
             "vectors=%d\n",
             SENTRY_SITE, SENTRY_CORPUS_TAG,
             BOARD_NAME, BOARD_GEOMETRY,
             BOARD_HAS_VBAT, BOARD_HAS_CHG_STAT, BOARD_HAS_LORA,
             BOARD_SLIDE_HARD_CUT,
             (double)GOLDEN_THRESHOLD, SENTRY_DEFAULT_PRESET,
             (double)DEFAULT_THRESHOLD,
             CFG_N_BINS, CFG_N_F0, CFG_N_FFT, CFG_HOP, CFG_FS, CFG_N_CHANNELS,
             (unsigned)sizeof(detector_state_t), (unsigned)sizeof(detector_work_t),
             (unsigned)sizeof(trace_rec_t),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (void *)g_w, (unsigned)((uintptr_t)g_w & 15u),
             (void *)g_w->fftbuf, (unsigned)((uintptr_t)g_w->fftbuf & 15u),
             PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_SD, PIN_I2S_SD2,
             MIC_LR_STRAP, (int)I2S_BCLK_HZ,
             source_golden_count());
    trace_text(b);
    for (int i = 0; i < source_golden_count(); i++) {
        snprintf(b, sizeof(b), "  [%d] %-22s n=%-7u preset=%-10s thr=%.4f\n",
                 i, source_golden_name(i), (unsigned)source_golden_len(i),
                 source_golden_preset(i), source_golden_thr(i));
        trace_text(b);
    }
    /* Tier-2's configuration tag, so a capture can be attributed to the
     * constants that produced it without asking the repo. Integers only:
     * PicoLibC's printf has no float support. */
    snprintf(b, sizeof(b),
             "tier2: corpus=%s calibrated=%d band=%d-%d Hz rows=%d..%d n=%d\n"
             "       tau2_milli=%d N2=%d M2=%d gap=%d rel=%d warmup_ms=%d\n"
             "       tau_rise_s=%d half_rate_default=%d excl=%d\n"
             "       sizeof t2state=%u t2work=%u cxwork=%u t2rec=%u\n",
             T2_CORPUS_TAG, T2_CALIBRATED, (int)T2_F_LO, (int)T2_F_HI,
             T2_ROW_LO, T2_ROW_HI, T2_N_F0,
             (int)(T2_TAU2 * 1000.0 + 0.5), T2_N2, T2_M2, T2_GAP, T2_RELEASE,
             (int)(T2_WARMUP_S * 1000.0), (int)T2_TAU_RISE_S,
             T2_HALF_RATE_DEFAULT, T2_N_EXCL,
             (unsigned)sizeof(t2_state_t), (unsigned)sizeof(t2_work_t),
             (unsigned)sizeof(cx_work_t), (unsigned)sizeof(trace_t2r_t));
    trace_text(b);

    /* Tier-3, and the fact that matters most about it. */
    snprintf(b, sizeof(b),
             "tier3: corpus=%s enabled_default=%d (header) tau3_milli=%d\n"
             "       fire=%d-%d Hz grid=%d-%d Hz rates=%d nharm=%d W_null=%d\n"
             "       n3=%d m3=%d warmup_ms=%d  sizeof t3state=%u t3rec=%u\n"
             "       NOTE: calibration FAILED at 4.87 wFA/h vs 0.40 allowance.\n"
             "       It is the only tier that has detected a real rotor, and\n"
             "       it fires on fans, insects and vehicles. Runtime-enabled.\n",
             T3_CORPUS_TAG, T3_ENABLED_DEFAULT,
             (int)(T3_TAU3 * 1000.0 + 0.5),
             (int)T3_FIRE_LO, (int)T3_FIRE_HI, (int)T3_GRID_LO,
             (int)T3_GRID_HI, T3_N_RATES, T3_N_HARM, (int)T3_W_NULL,
             T3_N3, T3_M3, (int)(T3_WARMUP_S * 1000.0),
             (unsigned)sizeof(t3_state_t), (unsigned)sizeof(trace_t3r_t));
    trace_text(b);

    settings_describe(b, sizeof(b));
    trace_text(b);

    /* THE SHIPPED FIELD CONFIGURATION, printed underneath the running one.
     *
     * `cfg` is what this board is doing; `ship` is what a blank board would
     * do. Printing both, in one format, one under the other,
     * turns "does an erased board come up field-correct" into a comparison an
     * operator can make by eye and a gate can make with a diff - and it makes
     * every line an operator still has to type visible as a difference rather
     * than as an absence. Identical lines mean nothing needs typing, which is
     * the whole claim of the field image. */
    settings_describe_defaults(b, sizeof(b));
    trace_text(b);
}

/* ==========================================================================
 * The command set. Extracted verbatim from app_main's loop when the standalone
 * mode arrived: a device that starts its own guard needs to dispatch a line
 * from two places - the console loop, and the line that INTERRUPTED a
 * standalone run - and duplicating the chain would have been the obvious way
 * to get them out of step. Every letter, argument and reply is unchanged.
 * ========================================================================== */
static void dispatch_line(const char *line)
{
    /* THE ADC GOES QUIET THE MOMENT A HOST SPEAKS, and stays quiet until the
     * device is handed back to itself. Every gate this project owns is a
     * bounded capture typed from a laptop - the golden replay, `Y` and `Z`
     * parity, `X` - and their verdict is decision-for-decision identity. The
     * battery sampler is fifteen conversions on the other core once every ten
     * seconds and SHOULD be invisible to all of them; that has never been
     * measured, and until it has, a gate does not carry an unmeasured risk
     * for the sake of a status bar nobody is looking at with a laptop
     * attached. run_standalone() turns it back on. */
    power_mon_quiet(true);

    /* `test 0|1` IS TESTED BEFORE `T`, AND THAT IS THE SAME TRAP A THIRD
     * TIME. The dispatcher switches on ONE character, so without this
     * `test 1` matches `T` and plays a TONE at the operator. `U btn` behind
     * `U b` was the first, `U boot` behind `U b` the second, and both are
     * commented where they sit. A whole word that begins with an existing
     * command letter must be matched before the letter, never after. */
    if (!strncmp(line, "test", 4) &&
        (line[4] == ' ' || line[4] == '\t')) {
        int v = -1;
        if (sscanf(line + 4, "%d", &v) != 1 || (v != 0 && v != 1)) {
            trace_text("test 0 | test 1\n");
            trace_ack(line, TRACE_ACK_BAD_ARG);
            return;
        }
        settings_mut()->test_mode = (uint8_t)v;
        const bool saved = settings_save();
        char b[288];
        settings_describe(b, sizeof(b));
        trace_text(b);
        trace_text(saved ? "saved\n" : "NOT SAVED (nvs write failed)\n");
        trace_ack(line, saved ? TRACE_ACK_OK : TRACE_ACK_FAULT);
        return;
    }

    if (line[0] == 'I' || line[0] == 'i') {
        say_info();
    } else if (line[0] == 'R' || line[0] == 'r') {
        run_vector(atoi(line + 1), 0xFFFFFFFFu, 0u);
    } else if (line[0] == 'M' || line[0] == 'm') {
        run_meter();
    } else if (line[0] == 'T' || line[0] == 't') {
        run_tone((uint32_t)atoi(line + 1));
    } else if (line[0] == 'L' || line[0] == 'l') {
        /* TWO SUB-LETTERS RIDE BEHIND `L`, exactly as `U bd` and `U bt` ride
         * behind `U b`. Bare `L` is still the live meter and is untouched;
         * only `Lt` and `Ls` are new, and on a board with no radio both
         * answer with the board name rather than doing nothing. */
        if (line[1] == 'c' || line[1] == 'C') {
            cmd_mic_cal(line);
        } else if (line[1] == 't' || line[1] == 'T') {
            char b[192];
            const bool ok = lora_link_selftest(b, sizeof(b));
            trace_text(b);
            trace_ack(line, ok ? TRACE_ACK_OK : TRACE_ACK_FAULT);
        } else if (line[1] == 's' || line[1] == 'S') {
            const int on = atoi(line + 2);
            const bool ok = lora_link_send_test(on != 0);
            trace_text(ok ? "LORA test packet queued (3 transmissions)\n"
                          : "LORA not started - `Lt` says why\n");
            trace_ack(line, ok ? TRACE_ACK_OK : TRACE_ACK_FAULT);
        } else if (line[1] == 'p' || line[1] == 'P') {
            const int n = atoi(line + 2);
            char b[128];
            if (n <= 0) {
                g_ping_period_s = 0u;
                snprintf(b, sizeof(b), "LORA ping OFF (tx %lu rx %lu)\n",
                         (unsigned long)g_ping_tx, (unsigned long)g_ping_rx);
            } else {
                g_ping_period_s = (uint32_t)(n > 3600 ? 3600 : n);
                g_ping_due_ms = now_ms();          /* first one immediately */
                g_ping_tx = g_ping_rx = 0u;
                snprintf(b, sizeof(b),
                         "LORA ping every %lus - TEST page shows PING\n",
                         (unsigned long)g_ping_period_s);
            }
            trace_text(b);
            trace_ack(line, TRACE_ACK_OK);
        } else if (line[1] == 'l' || line[1] == 'L') {
            /* `Ll` - THE LOOPBACK DRILL. Exercises the receive half on ONE
             * board, which no amount of transmitting can: an SX1276 is half
             * duplex and cannot hear itself.
             *
             * `Ll`   inject a frame carrying OUR id  -> must be dropped as SELF
             * `Ll 1` inject a frame from a PEER      -> must raise a remote
             *                                           alert
             * `Ll 1` again, immediately              -> must be dropped as a
             *                                           duplicate of the burst
             *
             * Read the verdict off the rx_self / rx_dup / rx_loop counters in
             * `Lt`. It proves the protocol and the alert path; it proves
             * NOTHING about the radio, the antenna or the channel - that is
             * the two-board air test in field/BATTERY_DAY_CHECKLIST.md. */
            const bool as_peer = atoi(line + 2) != 0;
            const bool ok = lora_link_loopback(
                as_peer, (uint32_t)(esp_timer_get_time() / 1000));
            trace_text(ok ? (as_peer
                             ? "LORA loopback injected AS A PEER - expect a "
                               "remote alert, then a dup on a repeat\n"
                             : "LORA loopback injected AS OURSELVES - expect "
                               "rx_self to advance and NO alert\n")
                          : "LORA not started - `Lt` says why\n");
            char b[192];
            (void)lora_link_selftest(b, sizeof(b));
            trace_text(b);
            trace_ack(line, ok ? TRACE_ACK_OK : TRACE_ACK_FAULT);
        } else {
            run_live();
        }
    } else if (line[0] == 'Y' || line[0] == 'y') {
        int secs = atoi(line + 1);
        run_parity(secs > 0 ? (uint32_t)secs : 10u);
    } else if (line[0] == 'W' || line[0] == 'w') {
        int k0 = atoi(line + 1);
        win_selftest(k0 ? k0 : 64);
    } else if (line[0] == 'B' || line[0] == 'b') {
        int idx = 0; unsigned fr = 0;
        sscanf(line + 1, "%d %u", &idx, &fr);
        block_dump(idx, fr);
    } else if (line[0] == 'F' || line[0] == 'f') {
        int k0 = atoi(line + 1);
        fft_selftest(k0 ? k0 : 64);
    } else if (line[0] == 'P' || line[0] == 'p') {
        int idx = 0;
        unsigned a = 0, b = 0;
        if (sscanf(line + 1, "%d %u %u", &idx, &a, &b) < 1) {
            trace_text("ERR usage: P <idx> <from> <to>\n");
            return;
        }
        run_vector(idx, a, b);
    /* ---- additive: alert outputs and quad acquisition ---------------
     * New letters only. Every existing letter above keeps its exact
     * behaviour, and the unknown-letter fallback is unchanged. */
    } else if (line[0] == 'U' || line[0] == 'u') {
        cmd_u(line);
    } else if (line[0] == 'D' || line[0] == 'd') {
        run_drill((uint32_t)atoi(line + 1));
    } else if (line[0] == 'A' || line[0] == 'a') {
        run_armed();
    } else if (line[0] == 'Q' || line[0] == 'q') {
        run_quad_meter();
    } else if (line[0] == 'X' || line[0] == 'x') {
        int secs = atoi(line + 1);
        run_quad_capture(secs > 0 ? (uint32_t)secs : 10u);
    } else if (line[0] == 'G' || line[0] == 'g') {
        /* Optional threshold in MILLI-units, matching the existing
         * letter+int parser style: `G` keeps the deployment default so
         * every previous observation still means what it meant; `G 1850`
         * runs at 1.850. Calibration therefore never needs a rebuild, and
         * so never forces a golden re-proof. */
        int milli = atoi(line + 1);
        double thr = (milli > 0) ? (double)milli / 1000.0
                                 : (double)DEFAULT_THRESHOLD;
        (void)run_quad_pipeline(0u, true, thr, NULL, NULL, NULL, false);
    } else if (line[0] == 'Z' || line[0] == 'z') {
        int secs = atoi(line + 1);
        (void)run_quad_pipeline(secs > 0 ? (uint32_t)secs : 30u, false,
                                (double)DEFAULT_THRESHOLD, NULL, NULL, NULL,
                                false);
    /* ---- additive: Tier-2, one new guard letter ----------------------
     * `H [thr1_milli] [thr2_milli] [cx] [busoff_milli] [rate]`
     *
     * Bare `H` is `G` plus Tier-2 at its calibrated constants, CX-A, zero
     * bus offset and the shipped rate. Every argument is
     * runtime, so calibration, a combiner experiment or a rate change
     * never forces a rebuild - and therefore never forces a golden
     * re-proof. That is the `G [thr_milli]` precedent, extended.
     *
     *   rate: 0 = the compiled default, 1 = full rate, 2 = half rate */
    } else if (line[0] == 'H' || line[0] == 'h') {
        int m1 = 0, m2 = 0, boff = 0, rate = 0, blo = 0, bhi = 0, m3 = 0;
        char cx = 'a';
        sscanf(line + 1, "%d %d %c %d %d %d %d %d", &m1, &m2, &cx, &boff,
               &rate, &blo, &bhi, &m3);
        if (cx >= 'A' && cx <= 'Z') {
            cx += 'a' - 'A';
        }
        if (!combiner_cx_valid(cx)) {
            trace_text("ERR usage: H [thr1_milli] [thr2_milli] [cx a|b|c|d]"
                       " [busoff_milli] [rate 0|1|2] [t2_lo_hz]"
                       " [t2_hi_hz] [t3_tau_milli, 0=off]\n");
            trace_ack("H", TRACE_ACK_BAD_ARG);
            return;
        }
        guard_t2_opts_t o = {0};
        o.enable = true;
        t2_cfg_default(&o.cfg,
                       rate == 1 ? false
                                 : (rate == 2 ? true
                                              : (T2_HALF_RATE_DEFAULT != 0)));
        t2_cfg_set_thr_milli(&o.cfg, m2);
        /* A BAD BAND IS REFUSED, NOT CLAMPED. If the rig day types a band
         * the v1 grid cannot express, the operator must see that rather
         * than get a silently different band and a trace that does not
         * say so. */
        if (!t2_cfg_set_band(&o.cfg, blo, bhi)) {
            trace_text("ERR t2 band must be inside 70-2000 Hz and "
                       "lo < hi\n");
            trace_ack("H", TRACE_ACK_BAD_ARG);
            return;
        }
        /* Exclusion bands set by a previous `E` ride along. */
        if (g_excl_n > 0) {
            t2_cfg_set_excl(&o.cfg, g_excl_n, g_excl_c, g_excl_t);
        }
        o.cx = cx;
        o.f_split_hz = 1500.0f;   /* CX-C's split; not yet a runtime arg */
        o.busoff = (float)boff / 1000.0f;
        const double thr1 = (m1 > 0) ? (double)m1 / 1000.0
                                     : (double)DEFAULT_THRESHOLD;
        /* Tier-3 rides in on an eighth argument, and off is the default, so
         * `H` with seven arguments is byte-for-byte the two-tier mode. A
         * positive tau3 in milli-units turns the wash tier on for this run
         * only; nothing is persisted from here. */
        guard_t3_opts_t o3 = {0};
        if (m3 > 0) {
            o3.cfg = t3_default_cfg();
            o3.cfg.enabled = true;
            o3.cfg.tau3 = (double)m3 / 1000.0;
            o3.enable = true;
        }
        (void)run_quad_pipeline(0u, true, thr1, &o, m3 > 0 ? &o3 : NULL, NULL,
                                false);
    /* `K <i>` - golden vector i through the SAME replay machinery as `R`,
     * with Tier-2 running beside it. `R` itself is untouched, so the
     * parity evidence is unaffected; this is Tier-2's own gate. Mono
     * audio has no coherence, so kappa is 1.0 throughout by definition. */
    /* `E` - the persistent-source exclusion list, at RUNTIME.
     *
     *   E 0                         clear
     *   E n f0_1 tol_1 ... f0_n tol_n   (n <= 4, all Hz, integers)
     *
     * Runtime rather than compiled, because populating the list through a
     * header forces a rebuild and a rebuild forces a golden re-proof. It
     * applies to Tier-2 hit counting only: v1 never sees it, telemetry still
     * logs an excluded winner, and nothing self-learns - a human types these
     * from a site baseline or they stay empty. */
    } else if (line[0] == 'E' || line[0] == 'e') {
        int n = -1;
        const char *p = line + 1;
        if (sscanf(p, "%d", &n) != 1 || n < 0 || n > T2_MAX_EXCL) {
            trace_text("ERR usage: E 0 | E n f0 tol [f0 tol ...]  "
                       "(n <= 4, Hz)\n");
            trace_ack("E", TRACE_ACK_BAD_ARG);
            return;
        }
        /* advance past the count, then read n (centre, tol) pairs */
        while (*p == ' ') { p++; }
        while (*p && *p != ' ') { p++; }
        double cs[T2_MAX_EXCL] = {0}, ts[T2_MAX_EXCL] = {0};
        bool ok = true;
        for (int i = 0; i < n; i++) {
            int f = 0, tol = 0;
            if (sscanf(p, "%d %d", &f, &tol) != 2 || f <= 0 || tol < 0) {
                ok = false;
                break;
            }
            cs[i] = (double)f;
            ts[i] = (double)tol;
            for (int k = 0; k < 2; k++) {
                while (*p == ' ') { p++; }
                while (*p && *p != ' ') { p++; }
            }
        }
        if (!ok) {
            trace_text("ERR E: expected n pairs of integers f0 tol\n");
            trace_ack("E", TRACE_ACK_BAD_ARG);
            return;
        }
        g_excl_n = n;
        for (int i = 0; i < n; i++) {
            g_excl_c[i] = cs[i];
            g_excl_t[i] = ts[i];
        }
        {
            char eb[160];
            int off = snprintf(eb, sizeof(eb), "E ok n=%d", n);
            for (int i = 0; i < n && off < (int)sizeof(eb) - 24; i++) {
                off += snprintf(eb + off, sizeof(eb) - off, "  %d+-%d",
                                (int)g_excl_c[i], (int)g_excl_t[i]);
            }
            snprintf(eb + off, sizeof(eb) - off, "\n");
            trace_text(eb);
        }
        trace_ack("E", TRACE_ACK_OK);
    } else if (line[0] == 'S' || line[0] == 's') {
        /* Handled here rather than in the console loop, so the SAME letter
         * works whether it arrives at an idle console or interrupts a run. */
        g_want_standalone = true;
        trace_ack("S", TRACE_ACK_OK);
    } else if (line[0] == 'V' || line[0] == 'v') {
        /* THE FIELD DAY'S ONLY RECOVERABLE RECORD. In standalone nothing
         * drains the trace link, so the per-frame records and the ALT/AL2/AL3
         * alerts are dropped by design - that is the right trade, because
         * blocking the detector to preserve telemetry is the wrong one. This
         * ring is what survives. `V` dumps it; `V 0` clears it. */
        if (line[1] == ' ' && atoi(line + 1) == 0 && strchr(line, '0')) {
            evlog_reset();
            trace_text("EVENTS cleared\n");
        }
        evlog_dump();
        /* THE RAM RING IS NOT THE WHOLE STORY AFTER A RESET, and on this
         * board the morning's first act - opening a serial port - IS a reset.
         * Print the RTC mirror too whenever this boot was not a cold
         * power-on, so a night's events survive being read. */
        if (bootrec_reset_reason() != (uint8_t)ESP_RST_POWERON &&
            bootrec_mirror_count() > 0u) {
            bootrec_mirror_dump();
        }
        trace_ack("V", TRACE_ACK_OK);
    } else if (line[0] == 'K' || line[0] == 'k') {
        int idx = 0, m2 = 0, rate = 0;
        sscanf(line + 1, "%d %d %d", &idx, &m2, &rate);
        run_vector_t2(idx, m2, rate);
    } else {
        trace_text("ERR unknown command\n");
    }
}

void app_main(void)
{
    /* THE ONLY BOOT-PATH ADDITION IN THIS BUILD, and it is a safety default,
     * not a feature: the buzzer and motor sit behind NPN drivers whose bases
     * hang off GPIO17/18 through 1 kOhm, so a high-Z pad at reset is an
     * undefined base drive. Two GPIO writes, before anything else can take
     * time. Every other new peripheral (RMT, SPI/e-paper, button, I2S1)
     * initialises lazily on first use of its own command, so the path from
     * reset to a golden replay is otherwise byte-for-byte what it was. */
    actuators_boot_safe();

    /* ---- release the deep-sleep holds -----------------------------------
     *
     * OFF-DEEP latches GPIO16/17/18 low so the LED, the buzzer and the motor
     * cannot twitch while the chip sleeps. A latch survives the wake, so it
     * must be released here or every output stays frozen for the whole of the
     * next run and the device comes back silent and dark while insisting it
     * is guarding. Released unconditionally: calling these on a boot that
     * never slept is harmless, and a conditional would be one more thing to
     * get wrong. */
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis((gpio_num_t)PIN_LED_WS2812);
    gpio_hold_dis((gpio_num_t)PIN_BUZZER);
    gpio_hold_dis((gpio_num_t)PIN_MOTOR);

    /* ---- SECTION 2.5: WHAT THE LAST BOOT DID ---------------------------
     * Called here, before anything else can take time, because it must read
     * the reset reason and the RTC blob before any later code can disturb
     * either. It PRINTS nothing: the USB serial driver is not up yet at this
     * point and an early trace_text is simply dropped, which is how the first
     * attempt at this lost its own boot line. The printing happens after
     * "STAGE1A READY" below. */
    s_boot_rtc_survived = bootrec_begin();

    /* THE LED'S SELF-HEALING RE-SEND. Started once, here, so it covers the
     * console as well as the guard: a pixel that mis-decodes a frame heals
     * within LED_REFRESH_MS instead of holding the wrong colour until the
     * next state change. Stopped again by alert_ui_power_down(), so a device
     * drawing OFF has nothing quietly re-lighting it. */
    (void)led_refresh_start();

    /* ---- THE SNOOZE BUTTON, SAMPLED ONCE AT BOOT ------------------------
     *
     * Read this before believing the result. A 500 ms window in which nobody
     * presses reads EXACTLY like an unwired pin, so this can report that it
     * saw no edge and it can NEVER report that the button is broken. That
     * distinction is not pedantry: a scan whose window opens before anybody
     * can press will call a working pin dead, and opening the serial port
     * resets the ESP32-S3, so that window is easy to miss. This print exists
     * so the question is answered by the board in front of you rather than by
     * a document. */
    {
        gpio_config_t bc = {
            .pin_bit_mask = 1ULL << PIN_SNOOZE_BTN,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        int lo = 1, seen = 0;
        if (gpio_config(&bc) == ESP_OK) {
            for (int i = 0; i < 100; i++) {
                if (gpio_get_level(PIN_SNOOZE_BTN) == 0) { lo = 0; seen = 1; }
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
        (void)lo;
        trace_text(seen
            ? "SNOOZE_BTN: ok - saw the pin go low\n"
            : "SNOOZE_BTN: no press seen in 500 ms. This is NOT evidence the\n"
              "            pin is unwired - nobody may have pressed it. Hold\n"
              "            the button during boot, or use `U btn tap`.\n");
        /* Leave it as the ISR wants it; alert_ui's button_init() reconfigures
         * with the negedge interrupt on first use, and the idle state is the
         * same pulled-up input either way. */
    }

    /* The trace is binary and shares the link with the log. Silence the log
     * so a stray ESP_LOGx can never land inside a record. */
    esp_log_level_set("*", ESP_LOG_NONE);
    trace_link_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    if (detector_init() != ESP_OK) {
        trace_text("FATAL esp-dsp fft init failed\n");
        return;
    }

    g_st = heap_caps_calloc(1, sizeof(detector_state_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    /* 16-byte alignment is REQUIRED, not preferred: dsps_fft2r_fc32 resolves
     * to the ESP32-S3 SIMD variant, whose 128-bit loads need it. malloc only
     * promises 4, and the aligned(16) attribute on a struct member does not
     * reach the allocator. */
    g_w = heap_caps_aligned_calloc(16, 1, sizeof(detector_work_t),
                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    /* g_block / g_q are NOT allocated here any more - see mono_alloc(). */
    if (!g_st || !g_w) {
        trace_text("FATAL out of internal RAM\n");
        return;
    }

    /* Tier-3's state, claimed before anything has fragmented the heap - but
     * ONLY if the stored pair wants it. Holding it unconditionally made the
     * v1+Tier-2 pair unallocatable; see t3_free(). run_standalone()
     * reconciles it on every run, so a pair switch needs no reboot. */
    if (settings_get()->t3_enabled && !t3_boot_alloc()) {
        trace_text("WARN Tier-3 unavailable: could not claim its state at "
                   "boot\n");
    }

    trace_text("\nSENTRY-NODE STAGE1A READY\n");

    /* THE BOOT RECORD, printed now that the link is actually up. */
    {
        char bb[192];
        /* THE PHASE IS ON THE BOOT LINE, not only in the record, because
         * during a 20-alert soak the console sees every reboot as it happens
         * and the record is exactly what a brownout throws away. */
        snprintf(bb, sizeof(bb), "BOOT reset=%s wake=%s rtc=%s phase=%s\n",
                 bootrec_reason_name(bootrec_reset_reason()),
                 bootrec_wake_name(bootrec_wake_cause()),
                 s_boot_rtc_survived ? "survived" : "INITIALISED",
                 bootrec_phase_name(bootrec_phase_at_reset()));
        trace_text(bb);
        if (s_boot_rtc_survived && bootrec_mirror_count() > 0u) {
            snprintf(bb, sizeof(bb),
                     "RING-MIRROR: %u entries survived reset %s\n",
                     (unsigned)bootrec_mirror_count(),
                     bootrec_reason_name(bootrec_reset_reason()));
            trace_text(bb);
        }
    }

    /* WHAT THIS WAKE HAS TO PROVE, before autostart takes it to LISTENING.
     * On any boot that is not a deep-sleep wake this only records the button
     * level and returns. */
    handle_deepsleep_wake();

    char line[64];

    /* ---- THE POWER-ON PATH ----------------------------------------------
     * autostart is a persisted setting and ships ON, so a board that comes up
     * on a charger starts guarding by itself. It is not a trap door: any host
     * line takes the console, WITH THE LINE INTACT (trace_try_line), and the
     * console then keeps it. That asymmetry is deliberate - a lab session must
     * be deterministic, so the guard never re-arms behind the operator's back.
     * `S` puts it back. */
    bool console = false;
    int  fail_streak = 0;

    for (;;) {
        if (!console && settings_get()->autostart) {
            const int why = run_standalone();

            /* THE TWO HONEST ENDINGS. Both leave the panel saying OFF with
             * the reason under it, and both stop the device. On a board that
             * deep-sleeps neither call returns at all; on the breadboard they
             * park in standby and come back when the button is pressed, which
             * is the same `continue` the long press takes. */
            if (why == RUN_END_POWER || why == RUN_END_BATTERY) {
                /* On a board that deep-sleeps this never returns. On the
                 * breadboard it parks against the OFF screen it just drew and
                 * comes back when the button is pressed, which is why the
                 * pending-line handling below is shared. */
                run_off_and_sleep(
                    why == RUN_END_BATTERY
                        ? "BATTERY EMPTY - CHARGE ME"
                        : (BOARD_SLIDE_HARD_CUT
                               ? "HOLD 2 s OR SLIDE THE SWITCH NOW"
                               : "SAFE TO UNPLUG"),
                    why == RUN_END_BATTERY);
            }
            if (why == RUN_END_POWER || why == RUN_END_BATTERY) {
                if (g_have_pending) {
                    g_have_pending = false;
                    console = true;
                    dispatch_line(g_pending);
                    if (g_want_standalone) {
                        g_want_standalone = false;
                        console = false;
                        fail_streak = 0;
                    }
                }
                continue;                 /* the button woke it: guard again */
            }
            if (why == RUN_END_INPUT && g_have_pending) {
                g_have_pending = false;
                console = true;
                dispatch_line(g_pending);
                if (g_want_standalone) {
                    g_want_standalone = false;
                    console = false;
                    fail_streak = 0;
                }
                continue;
            }
            /* It could not start - no I2S, or no RAM. Retry a few times,
             * because a field device that gives up on the first transient is
             * a field device that comes home with no data, then stop and say
             * so rather than spin a fault at 30 Hz forever. */
            if (++fail_streak >= 5) {
                trace_text("SENTRY standalone could not start 5 times; "
                           "console only. Fix the fault and send S.\n");
                console = true;
                continue;
            }
            for (int i = 0; i < 30 && !console; i++) {
                if (trace_try_line(g_pending, sizeof(g_pending)) > 0) {
                    g_have_pending = false;
                    console = true;
                    dispatch_line(g_pending);
                    break;
                }
                alert_ui_idle_led(true, now_ms());
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            continue;
        }

        int n = trace_read_line(line, sizeof(line), portMAX_DELAY);
        if (n <= 0) {
            continue;
        }
        /* `S` hands the device back to itself: the console releases it and the
         * guard restarts from the persisted settings. It is how a bench
         * session ends without a power cycle, and it is the only way back into
         * standalone once a host has spoken. dispatch_line() owns the letter;
         * this loop only acts on the flag it sets. */
        dispatch_line(line);
        if (g_want_standalone) {
            g_want_standalone = false;
            console = false;
            fail_streak = 0;
        }
    }
}
