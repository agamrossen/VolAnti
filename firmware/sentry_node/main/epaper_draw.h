/*
 * epaper_draw.h - the drawing half of the panel, with no ESP-IDF in it.
 *
 * See epaper_draw.c for why the split exists. In one line: so that a host test
 * can render every screen and LOOK AT IT, on a night when no board is
 * attached, which is most nights.
 *
 * The API is deliberately a SNAPSHOT plus two line buffers rather than a pile
 * of setters. epaper.c draws on core 1 while the frame loop writes on core 0;
 * a page assembled from ten separately-written fields can be internally
 * inconsistent by the time it is drawn - a count from one second beside an age
 * from another - and on a two-second refresh that inconsistency is on the
 * panel for two seconds. One struct, copied into a back buffer with the index
 * flipped after the copy, cannot do that.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "epaper.h"

/* ---- the three pages ---------------------------------------------------- */
#define EPAPER_PAGE_MAIN    0u
#define EPAPER_PAGE_STATUS  1u
#define EPAPER_PAGE_RECENT  2u
#define EPAPER_PAGE_COUNT   3u

/* The text pages are free text, left aligned, small. They are read close up by
 * somebody who has walked to the box, so the constraint is "does it fit", not
 * "is it legible across a field".
 *
 * Fourteen rows, and the number is arithmetic rather than a preference. A
 * scale-1 glyph is OV_GLYPH_H = 7 px tall; at TX_ROW_DY = 10 that leaves a
 * 3 px gap, and rows from TX_ROW0_Y = 42 reach 42 + 13*10 = 172 with the glyph
 * ending at 179, inside the 184 px bottom margin. Fifteen would end at 189 and
 * clip - which does not error, it silently loses the tail. */
#define EPAPER_PAGE_ROWS    14
/* TWENTY-EIGHT, and it is arithmetic rather than a round number: the usable
 * box is 200 - 2*16 = 168 px, a scale-1 glyph advances 6 px and the last one
 * needs no trailing gap, so n*6 - 1 <= 168 gives n = 28. It was 30 for about
 * an hour, which is 179 px - and a row that wide does not error, it CLIPS,
 * losing its tail with nothing anywhere to say so. tests/test_epaper_pages.py
 * caught it before a panel did. */
#define EPAPER_LINE_MAX    28
#define EPAPER_WORD_MAX    12

/* ---------------------------------------------------------------------------
 * The MAIN page, as one value.
 *
 * What an operator standing over the box needs, and nothing else. Build
 * telemetry - the tier list, the microphone count, the thresholds - moved to
 * STATUS, because those answer "is it configured right", which is a question
 * asked once at deployment, and this screen answers "is it working and has it
 * heard anything", which is the question asked every time somebody walks past.
 *
 * Note on time fields. up_min and last_alert_min are in MINUTES but are
 * rendered COARSELY, and the producer is required not to push a new snapshot
 * for a time change more often than once an hour. A last-alert age that
 * redrew as it ticked would be a two-second full-panel refresh every sixty
 * seconds, forever - 1440 a day against the ~30 this device is allowed.
 * ------------------------------------------------------------------------ */
typedef struct {
    char     state_word[EPAPER_WORD_MAX + 1];  /* GUARDING / SNOOZED / ...   */
    char     warn[EPAPER_LINE_MAX + 1];        /* "" when nothing is wrong   */
    char     unit_id[EPAPER_WORD_MAX + 1];     /* this box, "N-F2CF"         */
    char     last_src[EPAPER_WORD_MAX + 1];    /* last peer heard, or ""     */
    uint16_t alerts_local;                     /* since power-up             */
    uint16_t alerts_remote;
    int32_t  last_alert_min;                   /* < 0 = nothing yet          */
    uint32_t up_min;
    /* THE SLICE IS HERE TO BE COMPARED, NOT TO BE DRAWN. battery_draw() reads
     * the value epaper_set_battery() recorded; this copy exists so that a
     * slice change shows up in the memcmp that decides whether the panel is
     * worth redrawing. Without it a battery that dropped a bar would change
     * nothing the coalescer could see, and the bar would be stale until the
     * next hourly beat. */
    int8_t   batt_slice;                       /* -1 = no bar (see above)    */
    uint8_t  batt_charging;
    uint8_t  batt_absent;                      /* draw USB instead of a bar  */
    uint8_t  radio_on;
    /* Ambient, as a word and never a number. Four buckets with hysteresis
     * off the detector's existing adaptive floor, so it changes a handful of
     * times an hour rather than every frame. No new signal processing: this
     * is a quantisation of a number the detector already computes. */
    char     ambient[8];                       /* QUIET / LOW / BUSY / LOUD  */
    /* The ALERT screen's one line under the word: which tier fired, and its
     * f0 or rate. Empty on every other screen. */
    char     alert_line[EPAPER_LINE_MAX + 1];
    uint16_t snooze_s;                         /* SNOOZE screen, whole secs  */
    /* The test-mode tag lives in the struct rather than being read from
     * settings by the drawing code, for the reason batt_slice is here:
     * epaper_set_main() decides whether to redraw by memcmp, so a state the
     * drawing depends on that is not in the struct is a state that changes
     * the glass without the coalescer noticing. */
    uint8_t  test_mode;
} epaper_main_t;

/* Copies m into the back buffer and publishes it. Cheap: one struct copy and
 * an index flip, safe to call from the frame loop. */
void epaper_set_main(const epaper_main_t *m);

/* Bumped by every publish that CHANGES something. The refresh coalescer
 * compares it, so a snapshot identical to the one already shown costs nothing
 * - which is what keeps a once-a-second producer from redrawing the panel
 * once a second. */
uint32_t epaper_main_gen(void);

/* STATUS / RECENT text. row >= EPAPER_PAGE_ROWS is ignored rather than
 * clipped into somebody else's row. */
void epaper_page_clear(uint8_t page);
void epaper_page_line(uint8_t page, uint8_t row, const char *text);

/* As above, but sets the row's font scale too. Scale 2 is double height and
 * holds 14 characters instead of 28; the row's own height is what the page
 * advances by, so a page may mix the two. The frozen alert page uses scale 2
 * for its four tier rows, because it is read from a distance. */
void epaper_page_line_sc(uint8_t page, uint8_t row, const char *text,
                         uint8_t scale);

/* ---- what epaper.c needs ------------------------------------------------ */
uint8_t *epaper_draw_fb(void);
int      epaper_draw_fb_bytes(void);

/* Renders one screen into the framebuffer. Pure: same inputs, same 5000
 * bytes, every time - which is what makes it testable. */
void epaper_draw_compose(epaper_screen_t scr, uint8_t rot, bool mirror);

/* The four legacy strings, kept because the ALERT and OFF screens still use
 * them and because nothing about those screens should change: they are read
 * at a glance from a distance and a redesign would be a regression. */
void epaper_set_battery(int slice, bool charging);
void epaper_set_off_reason(const char *text);
void epaper_set_alert_note(const char *text);

/* IS THIS ALERT SOMEBODY ELSE'S? The small note line carries the peer id, but
 * the first question an operator asks about an alarm with no audible source in
 * front of them is "was that mine", and that has to be answerable across a
 * field at a glance rather than by reading a line of 5x7 text. Sets a BIG
 * word on the alert screen. */
void epaper_set_alert_remote(bool remote);
