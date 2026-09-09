#include "epaper.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "epaper/DEV_Config.h"
#include "epaper/EPD_1in54_V2.h"
#include "epaper_draw.h"
#include "epaper_screens.h"
#include "trace.h"


/* There is no partial refresh path. Dead bookkeeping that looks live - a
 * partials counter and a base-image flag written in several places and read
 * in none - is how the next reader concludes partial refreshes exist and
 * reasons from a mechanism that is not there. If partial refresh is ever
 * wanted, it arrives with its own state. */

/* The driver and the drawer must agree about the panel. epaper_draw.c sizes
 * its framebuffer from the generated artwork so that it needs no ESP-IDF
 * header; this is the other half of that bargain, and it is a build error
 * rather than a comment because the last time these two disagreed the symptom
 * was a noise band across the top of the panel and clipped text - a 200x200
 * part driven by the 122x250 driver - and it took a photograph of the
 * silkscreen to find. */
_Static_assert(EPD_1IN54_V2_WIDTH == EPAPER_SCREEN_W &&
               EPD_1IN54_V2_HEIGHT == EPAPER_SCREEN_H,
               "the panel driver and the artwork disagree about the panel");

#define EPD_QUEUE_LEN   4
#define EPD_TASK_STACK  4096
#define EPD_TASK_PRIO   2
#define EPD_TASK_CORE   1        /* the app is single-threaded on core 0 */

static QueueHandle_t s_q;
static TaskHandle_t  s_task;
static uint8_t  s_state = TRACE_EPD_UNINIT;
static uint8_t  s_rot = EPD_ROT_270;  /* the shipped default; settings_get()   */
                                      /* overwrites it before the first draw,  */
                                      /* and matching it here means a panel    */
                                      /* drawn before that is not sideways     */
static bool     s_mirror;
static uint32_t s_dropped;
static bool     s_panel_inited;
static volatile int s_inflight;     /* enqueued + drawing, for wait_idle */
static volatile bool s_audio_hot;   /* the frame loop is running - see epaper.h */



static void mark_faulted(void)
{
    s_state = TRACE_EPD_FAULTED;
}

/* Every driver call can trip the patched busy timeout; check after each. */
static bool check_timeout(void)
{
    if (EPD_busy_timeout) {
        EPD_busy_timeout = 0;
        mark_faulted();
        return true;
    }
    return false;
}

static void do_screen(epaper_screen_t scr)
{
    if (s_state == TRACE_EPD_FAULTED) {
        return;
    }
    s_state = TRACE_EPD_BUSY;

    if (!s_panel_inited) {
        if (DEV_Module_Init() != 0) {
            mark_faulted();
            return;
        }
        EPD_1IN54_V2_Init();
        if (check_timeout()) { return; }
        s_panel_inited = true;
    }

    /* THE TWO THAT TALK TO THE CONTROLLER RATHER THAN TO THE FRAMEBUFFER. */
    if (scr == EPAPER_SCREEN_INIT_CLEAR || scr == EPAPER_SCREEN_CLEAR) {
        if (scr == EPAPER_SCREEN_INIT_CLEAR) {
            EPD_1IN54_V2_Init();
            if (check_timeout()) { return; }
        }
        EPD_1IN54_V2_Clear();
        if (check_timeout()) { return; }
        s_state = TRACE_EPD_READY;
        return;
    }

    /* ---------------------------------------------------------------------
     * Always a full refresh, never Display_Partial.
     *
     * Partial refresh only rewrites what differs from the panel's stored base
     * image. If the base and the panel's real geometry disagree even slightly,
     * the untouched region keeps whatever was in RAM - which is exactly the
     * "half the screen is static, half is fine" symptom seen on the bench.
     *
     * Note that the full refresh does not write both RAM planes:
     * EPD_1IN54_V2_Display() writes plane 0x24 only, and Clear() and
     * DisplayPartBaseImage() are the two that write both.
     *
     * The cost is ~2 s per update - a NOMINAL figure from the datasheet that
     * has never been measured on this board, and this project has been wrong
     * about an unmeasured cost by 14x - and it is FREE here regardless: this
     * runs on a task pinned to core 1, fed by a queue, so the 32 ms frame loop
     * never waits for it. Alert latency at the buzzer is unaffected, because
     * the buzzer fires from alert_ui_tick() and not from here.
     * ------------------------------------------------------------------ */
    /* ---- re-init before every write -------------------------------------
     *
     * The symptom without this is a screen that wraps left to right and top
     * to bottom, especially after an alert, and needs a power cycle to
     * recover.
     *
     * EPD_1IN54_V2_Init() is what sets the RAM window (0x44,
     * 0x45) and the address counters (0x4E, 0x4F). It used to run ONCE, and
     * every draw after that went straight to Display(), which writes plane
     * 0x24 and nothing else. So the window and the cursor were established at
     * boot and then trusted forever. A controller left mid-sequence - by the
     * brownout this device demonstrably suffers on its first alert, or by a
     * transfer cut short - keeps a wrong window, and every later image is
     * written at the wrong offset and wraps. Nothing in the firmware could
     * ever put it back, which is why it took a power cycle.
     *
     * Init() is a reset, a SWRESET, a handful of registers and the window and
     * cursor. It carries no Clear and no waveform flush, so against the ~2 s
     * of the refresh that follows it is close to free, and it runs on core 1
     * where the frame loop never waits for it. Paying it every time buys an
     * image that cannot inherit a broken window from anything. */
    EPD_1IN54_V2_Init();
    if (check_timeout()) {
        return;
    }

    epaper_draw_compose(scr, s_rot, s_mirror);
    EPD_1IN54_V2_Display(epaper_draw_fb());

    if (check_timeout()) {
        return;
    }
    s_state = TRACE_EPD_READY;
}

static void epaper_task(void *arg)
{
    (void)arg;
    for (;;) {
        epaper_screen_t scr;
        if (xQueueReceive(s_q, &scr, portMAX_DELAY) == pdTRUE) {
            do_screen(scr);
            if (s_inflight > 0) {
                s_inflight--;
            }
            /* One STAT per completed screen so the host can see the panel
             * state change without polling. */
            trace_stat_t s = {0};
            s.magic = TRACE_MAGIC_STA;
            s.mode = 'e';
            s.epaper_state = s_state;
            s.button_level = 1;
            trace_send_stat(&s);
        }
    }
}

bool epaper_start(void)
{
    if (s_task) {
        return true;
    }
    s_q = xQueueCreate(EPD_QUEUE_LEN, sizeof(epaper_screen_t));
    if (!s_q) {
        return false;
    }
    if (xTaskCreatePinnedToCore(epaper_task, "epd", EPD_TASK_STACK, NULL,
                                EPD_TASK_PRIO, &s_task,
                                EPD_TASK_CORE) != pdPASS) {
        vQueueDelete(s_q);
        s_q = NULL;
        s_task = NULL;
        return false;
    }
    return true;
}

bool epaper_request(epaper_screen_t screen)
{
    if (s_state == TRACE_EPD_FAULTED) {
        return false;
    }
    if (!s_task && !epaper_start()) {
        return false;
    }
    /* Zero wait: this is called from the frame loop and must never block. */
    s_inflight++;
    if (xQueueSend(s_q, &screen, 0) != pdTRUE) {
        /* ---- the newest state wins ------------------------------------
         *
         * The symptom is a screen that looks glitched or half-drawn and comes
         * right on the next button press. That is not a corrupted image, it
         * is a stale one: a full refresh takes about two seconds and this
         * queue is
         * four deep, so the panel can already be EIGHT SECONDS behind; when a
         * burst overflows it, dropping the NEWEST request meant the panel
         * settled on an OLD screen and stayed there until some later event
         * pushed another refresh through. A button press is exactly such an
         * event, which is why pressing one appeared to fix it.
         *
         * A display is not a log. Nobody wants the backlog replayed; they
         * want what is true now. So on a full queue, discard the OLDEST
         * pending screen and enqueue this one. The panel may still lag, but
         * it now always CONVERGES on the current state instead of latching a
         * superseded one.
         *
         * The counter is kept and is now printed on the TEST page: a drop is
         * still worth knowing about, it just no longer leaves the wrong
         * picture on the glass. */
        epaper_screen_t stale;
        if (xQueueReceive(s_q, &stale, 0) == pdTRUE) {
            s_inflight--;               /* the one we just discarded */
            s_dropped++;
            if (xQueueSend(s_q, &screen, 0) == pdTRUE) {
                return true;
            }
        }
        s_inflight--;
        s_dropped++;
        return false;
    }
    return true;
}

void epaper_set_audio_hot(bool hot) { s_audio_hot = hot; }
bool epaper_audio_hot(void)         { return s_audio_hot; }

bool epaper_wait_idle(uint32_t timeout_ms)
{
    if (s_audio_hot) {
        /* Refuse rather than block. See the ANCHOR note in epaper.h: this is
         * the tripwire, and it fires at the bench, not in the field. */
        trace_text("BUG epaper_wait_idle called from the audio path - "
                   "refused\n");
        return false;
    }
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (s_inflight > 0 || s_state == TRACE_EPD_BUSY) {
        if (s_state == TRACE_EPD_FAULTED) {
            return false;
        }
        if (xTaskGetTickCount() > deadline) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return true;
}

void epaper_set_rotation(uint8_t rot) { s_rot = rot & 3; }
uint8_t epaper_get_rotation(void)     { return s_rot; }
void epaper_set_mirror(bool m)        { s_mirror = m; }
bool epaper_get_mirror(void)          { return s_mirror; }
uint8_t epaper_state(void)            { return s_state; }
uint32_t epaper_dropped(void)         { return s_dropped; }
