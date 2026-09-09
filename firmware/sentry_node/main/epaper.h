/*
 * epaper.h - the persistent visual indicator, kept entirely off the hot path.
 *
 * A full panel refresh takes about two seconds. The detector's frame budget is
 * 32 ms. Those two facts are irreconcilable in one thread, so ALL panel work
 * runs in a dedicated FreeRTOS task pinned to CORE 1 (the application is
 * otherwise single-threaded on core 0) and is fed by a short queue.
 *
 * epaper_request() only ENQUEUES. It never blocks, never waits on BUSY, and is
 * safe to call from the frame loop. A full queue drops the request and notes
 * it in STAT rather than stalling anything.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    EPAPER_SCREEN_INIT_CLEAR = 0,   /* init + full clear (ghost removal)     */
    EPAPER_SCREEN_READY,            /* page 1: MAIN, the resting screen      */
    EPAPER_SCREEN_ALERT,            /* warning triangle + "ALERT"            */
    EPAPER_SCREEN_CLEAR,            /* blank the panel                       */
    EPAPER_SCREEN_TEST,             /* calibration pattern - see below       */
    /* ---- added by the standalone field build. APPENDED, never inserted:
     * the numbering is what `U e` and the host tools speak. ---------------- */
    EPAPER_SCREEN_ALERT_WASH,       /* triangle + "ALERT" + "TIER 3 WASH"    */
    EPAPER_SCREEN_OFF,              /* one bold bar + "OFF" - see below      */
    /* The operator's two extra pages, appended and never inserted: the enum
     * numbering is what `U e` and the host tools speak, and inserting a
     * screen would silently renumber every one after it. Both are text - no
     * artwork, no new 5000-byte array - so they cost code and nothing else. */
    EPAPER_SCREEN_STATUS,           /* page 2: what this box IS             */
    EPAPER_SCREEN_RECENT,           /* page 3: what it has HEARD            */
    /* Appended, never inserted. SNOOZE gets its own screen because it is a
     * state an operator must be able to read across a field, not a footnote
     * on the resting one. */
    EPAPER_SCREEN_SNOOZE,
} epaper_screen_t;

/* EPAPER_SCREEN_OFF exists because e-paper KEEPS ITS IMAGE WITH NO POWER. A
 * device unplugged while showing LISTENING goes on claiming to listen from a
 * drawer. The long press draws this screen and waits for it before the device
 * stops, so an unpowered panel says unpowered. There is no way to draw it
 * from a power CUT - a full refresh takes ~2 s and the 3V3 rail collapses in
 * milliseconds - which is precisely why the graceful path exists. */

/* EPAPER_SCREEN_TEST draws a border on a known margin, labelled corner ticks,
 * a centre cross and a 10 px ruler. It exists because a panel that renders
 * partly outside the visible area cannot be diagnosed from a description -
 * photograph this screen and the real geometry is readable off it. */

/* Starts the core-1 task and the queue. Lazy: called on first e-paper command.
 * Does NOT touch the panel - the first screen request does that. */
bool epaper_start(void);

/* Enqueue a screen. Returns false if the queue was full (dropped) or the
 * module is faulted. Never blocks. */
bool epaper_request(epaper_screen_t screen);

/* ---- ORIENTATION -------------------------------------------------------
 * Four quadrants, clockwise, plus an independent mirror. Applied at blit time,
 * so an orientation discovered at the bench is fixed by a typed command rather
 * than a rebuild - and therefore never forces a golden re-proof.
 *
 * These used to be four INVOLUTIONS (identity, mirror-X, 180, mirror-Y),
 * because the original panel was 250x122 and a 90 degree turn could not be
 * represented without cropping. The panel is now a SQUARE 200x200, so a
 * quarter turn is exact. The mirror flag survives separately: sideways and
 * mirrored are different faults. */
#define EPD_ROT_0    0u
#define EPD_ROT_90   1u    /* a quarter turn CLOCKWISE                       */
#define EPD_ROT_180  2u
#define EPD_ROT_270  3u    /* three quarters CW - the shipped default, the   */
                           /* orientation the panel reads upright in the case */

void epaper_set_rotation(uint8_t quadrant);
uint8_t epaper_get_rotation(void);
void epaper_set_mirror(bool mirror);
bool epaper_get_mirror(void);

/* Degrees <-> quadrant, so the operator-facing commands can speak degrees.
 * Anything that is not 0/90/180/270 (or already 0..3) returns 0xFF. */
static inline uint8_t epaper_quadrant_from_arg(int v)
{
    switch (v) {
    case 0:            return EPD_ROT_0;
    case 1: case 90:   return EPD_ROT_90;
    case 2: case 180:  return EPD_ROT_180;
    case 3: case 270:  return EPD_ROT_270;
    default:           return 0xFFu;
    }
}

static inline int epaper_degrees_from_quadrant(uint8_t q)
{
    return (int)(q & 3u) * 90;
}

/* TRACE_EPD_* - uninit / ready / busy / faulted. */
uint8_t epaper_state(void);

/* Number of requests dropped because the queue was full. */
uint32_t epaper_dropped(void);

/* ---- WHAT GOES ON A SCREEN LIVES IN epaper_draw.h -----------------------
 *
 * The content API - the MAIN snapshot, the STATUS and RECENT text rows, the
 * battery bar, the OFF reason and the ALERT note - moved to epaper_draw.h
 * when the drawing was split out of the driver, so that a host test can
 * render every page without an ESP-IDF. This header is now the PANEL: start
 * it, ask it for a screen, ask it how it is, and never wait on it.
 *
 * epaper_set_overlay() and epaper_set_footer() are GONE. They were two loose
 * strings stamped onto the bottom of the LISTENING artwork, written from core
 * 0 and read on core 1 with nothing in between; MAIN is now a page and its
 * content arrives as one snapshot, which is both more to say and safer to
 * say it with. See epaper_set_main().
 * ---------------------------------------------------------------------- */

/* ---- THE ANCHOR: the audio path may NEVER wait on this panel -------------
 * A full refresh is ~2 s. The measured worst-case frame is 30.9 ms of a 32 ms
 * hop. Those two numbers cannot meet, so the rule is structural: panel work
 * runs on a task pinned to CORE 1, fed by a queue, and epaper_request() posts
 * with a ZERO wait and drops on a full queue rather than blocking.
 *
 * epaper_wait_idle() is the one function that violates that, deliberately, for
 * the power-off screen - which runs after the detector has stopped. To keep
 * that exception honest rather than a comment, the frame loop marks itself:
 * while the audio path is hot, wait_idle REFUSES and returns false instead of
 * blocking. A future edit that moves a wait into the hot path therefore fails
 * loudly at the bench instead of dropping audio in the field. */
void epaper_set_audio_hot(bool hot);
bool epaper_audio_hot(void);

/* Blocks until every enqueued screen has finished drawing, or the timeout
 * expires. Returns true if the panel went idle. NEVER call this from the frame
 * loop - a full refresh is ~2 s. It exists for the power-off path, which has
 * stopped the detector already and whose whole purpose is to leave a correct
 * image on an unpowered panel. */
bool epaper_wait_idle(uint32_t timeout_ms);
