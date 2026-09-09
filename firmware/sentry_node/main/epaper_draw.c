/*
 * epaper_draw.c - EVERY PIXEL THIS DEVICE DRAWS, AND NOTHING THAT TALKS TO A
 * PANEL.
 *
 * Why this file exists, and it is the same reason mic_cal.c, power_slice.c,
 * lora_proto.c and detector_t4.c exist: there is NO ESP-IDF in it, so a host
 * test can compile THE SHIPPED C and render every screen into a buffer it can
 * look at. Before the split, the only way to find out what the operator would
 * see was to flash a board and look at it - which on a night with no board
 * attached means the only way was to guess.
 *
 * The division is exact:
 *
 *   epaper_draw.c  the framebuffer, the rotation map, the font, the drawing
 *                  primitives, and the composition of each screen. Pure
 *                  computation over one 5000-byte array.
 *   epaper.c       the queue, the task, the SPI bus, the panel driver, the
 *                  fault latch and the refresh discipline. Everything that
 *                  can fail because of hardware.
 *
 * Nothing about WHAT IS DRAWN moved when the two were separated; the code
 * below is the code that was in epaper.c, and tests/test_epaper_pages.py
 * renders the artwork screens and asserts they still come out as the blit of
 * their generated bitmap through the rotation map.
 */
#include "epaper_draw.h"

#include <stdio.h>
#include <string.h>

#include "epaper_screens.h"

/* Waveshare 1.54 inch, 200 x 200, SQUARE. Identified from a photograph of the
 * board's silkscreen after the panel showed a noise band across the top and
 * clipped text: the build had been driving it with the 2.13 inch V4 driver
 * (122 x 250), so the buffer geometry was simply wrong. Square also means the
 * old landscape->portrait mapping is gone; the blit is identity + rotation. */
/* THE PANEL'S SIZE, TAKEN FROM THE ARTWORK AND NOT FROM THE DRIVER.
 *
 * epaper_screens.h is generated at 200 x 200 and is the only thing here that
 * knows a pixel from a byte, so this file takes its geometry from that and
 * stays free of any ESP-IDF header - which is what lets a host test render
 * these pages. epaper.c carries a _Static_assert that the DRIVER agrees, so a
 * mismatch is a build error rather than the symptom it produced last time:
 * a noise band across the top of the panel and clipped text, from driving a
 * 200x200 part with the 122x250 driver. */
#define EPD_PANEL_W   EPAPER_SCREEN_W           /* 200 */
#define EPD_PANEL_H   EPAPER_SCREEN_H           /* 200 */
#define EPD_ROW_BYTES ((EPD_PANEL_W + 7) / 8)   /* 25   */
#define EPD_FB_BYTES  (EPD_ROW_BYTES * EPD_PANEL_H)

static uint8_t s_fb[EPD_FB_BYTES];

uint8_t *epaper_draw_fb(void)      { return s_fb; }
int      epaper_draw_fb_bytes(void) { return (int)sizeof(s_fb); }
/* The mirror in force for the screen being composed. Set by
 * epaper_draw_compose() before anything draws, read by ov_plot() and blit().
 * It is a parameter rather than module state as far as callers are concerned;
 * this static exists only so the primitives keep the signatures the suite
 * pins. */
static bool s_mirror;

/* ---------------------------------------------------------------------------
 * ORIENTATION: ONE COORDINATE MAP, AND IT IS THE ONLY ONE.
 *
 * The panel is 200x200 - SQUARE - so a 90 degree rotation is exactly
 * representable and loses nothing. That was not true of the 2.13 inch part
 * this code was first written for (250x122), which is why the original
 * offered four INVOLUTIONS (identity, mirror-X, 180, mirror-Y) and a comment
 * saying that offering "90 degrees" would be a lie. On a square panel it is
 * not a lie, and a quarter turn is what the enclosure needs.
 *
 * So rotation is four quadrants, clockwise, plus an independent mirror
 * flag - which keeps the diagnostic the involutions existed for. A panel that
 * comes back mirrored and a panel that comes back sideways are different
 * faults and both are fixable from a command.
 *
 * COST. The map is applied inside the blit that already had to run, one
 * switch per pixel over 40000 pixels, on the e-paper task on core 1. There is
 * NO framebuffer transpose and no second buffer: the loop walks PANEL pixels
 * and pulls the source pixel through the inverse map, which is the same
 * amount of work it did before. This matters because this project has twice
 * paid an order of magnitude for a "just copy it" - the Stage-1a comb tables
 * and the Tier-3 envelope ring - and a 5 KB framebuffer is exactly the size
 * that makes a per-refresh transpose look harmless.
 *
 * There is no partial refresh anywhere in this driver (see do_screen: always
 * Display_Base, never Display_Partial), so there is no partial-window
 * rectangle to transform and no way for a rotated window to corrupt a region.
 * ------------------------------------------------------------------------ */
static inline bool logical_pixel(const uint8_t *img, int lx, int ly)
{
    /* 1 = white, 0 = black, matching the driver's Clear() convention. */
    const int byte = ly * EPAPER_SCREEN_ROW_BYTES + (lx >> 3);
    return (img[byte] >> (7 - (lx & 7))) & 1;
}

/* PANEL -> LOGICAL. Used by the blit, which iterates panel pixels. */
static inline void panel_to_logical(int px, int py, uint8_t rot, bool mirror,
                                    int *lx, int *ly)
{
    switch (rot & 3) {
    case EPD_ROT_90:                        /* artwork turned 90 CW */
        *lx = py;
        *ly = (EPAPER_SCREEN_W - 1) - px;
        break;
    case EPD_ROT_180:
        *lx = (EPAPER_SCREEN_W - 1) - px;
        *ly = (EPAPER_SCREEN_H - 1) - py;
        break;
    case EPD_ROT_270:
        *lx = (EPAPER_SCREEN_H - 1) - py;
        *ly = px;
        break;
    default:
        *lx = px;
        *ly = py;
        break;
    }
    if (mirror) {
        *lx = (EPAPER_SCREEN_W - 1) - *lx;
    }
}

/* LOGICAL -> PANEL, the exact inverse. Used by anything that DRAWS in logical
 * coordinates on top of a blitted screen (the status overlay), so that the
 * overlay turns with the artwork instead of sitting sideways on it. */
static inline void logical_to_panel(int lx, int ly, uint8_t rot, bool mirror,
                                    int *px, int *py)
{
    const int x = mirror ? (EPAPER_SCREEN_W - 1) - lx : lx;
    const int y = ly;
    switch (rot & 3) {
    case EPD_ROT_90:
        *px = (EPAPER_SCREEN_W - 1) - y;
        *py = x;
        break;
    case EPD_ROT_180:
        *px = (EPAPER_SCREEN_W - 1) - x;
        *py = (EPAPER_SCREEN_H - 1) - y;
        break;
    case EPD_ROT_270:
        *px = y;
        *py = (EPAPER_SCREEN_H - 1) - x;
        break;
    default:
        *px = x;
        *py = y;
        break;
    }
}
/* ---------------------------------------------------------------------------
 * TEXT: A 5x7 FONT, INTEGER SCALED, DRAWN IN LOGICAL COORDINATES.
 *
 * Everything here plots through logical_to_panel(), so text turns with the
 * artwork instead of sitting sideways on a rotated screen.
 *
 * THREE PLACEMENTS, and the reason there are three. Centring alone was enough
 * while the panel said "U12 E3" under a picture. A page of facts is a column
 * of labels with a column of values beside it, and centring each line
 * independently produces a ragged mess that is harder to read than the same
 * facts in a sentence. So: ov_text_at() places, ov_text_right() right-aligns a
 * value against a margin, and ov_text() centres - the last implemented as a
 * wrapper over the first, so the ALERT and OFF screens draw exactly the pixels
 * they always did.
 *
 * Silent failure modes, stated because neither announces itself and both are
 * only visible in a photograph of a panel on a hillside:
 *   - a character not in OV_ORDER advances the cursor and draws nothing;
 *   - a string wider than the panel loses its tail to clipping in ov_plot().
 * ov_text_w() exists so callers can budget against the first, and every
 * layout below is budgeted against the second in its comment.
 * ------------------------------------------------------------------------ */
#define OV_GLYPH_W 5
#define OV_GLYPH_H 7
#define OV_SCALE   2
#define OV_ADV     ((OV_GLYPH_W + 1) * OV_SCALE)

/* The footer band the ALERT and OFF screens still draw their one line in.
 * Unchanged: those two screens are read at a glance from a distance and are
 * deliberately not part of the page rework. */
#define OV_FOOT_Y  177
#define OV_FOOT_MAX 26

/* ---- page geometry ------------------------------------------------------
 *
 * The panel is 200 x 200 with a 16 px margin, so the usable box is x,y in
 * [16, 184] - 168 px each way. At scale 2 the advance is 12 px, so a line is
 * at most 14 characters; at scale 1 it is 6 px and 28 characters. Every
 * constant below is chosen against those two numbers and the battery bar's
 * left edge, and the host test renders each page and asserts nothing crosses
 * the margin - because the previous overlay could collide with the battery
 * bar and nothing checked (OV_MAX was 14 characters spanning x 17..182,
 * straight through a bar at x 146..180; it never happened only because the
 * string was six characters long).
 * ---------------------------------------------------------------------- */
#define PG_X0        16                  /* the left margin, everywhere    */
#define PG_X1       183                  /* the LAST usable column         */
#define PG_COLS_1    28                  /* characters per line at scale 1 */
#define PG_COLS_2    14                  /* characters per line at scale 2 */
static const char OV_ORDER[] = "0123456789UEVTMIC+./ "
                               "ABDFGHJKLNOPQRSWXYZ-:~*";
static const uint8_t OV_FONT[][OV_GLYPH_H] = {
    { 0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E },  /* '0' */
    { 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E },  /* '1' */
    { 0x0E, 0x11, 0x01, 0x06, 0x08, 0x10, 0x1F },  /* '2' */
    { 0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E },  /* '3' */
    { 0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02 },  /* '4' */
    { 0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E },  /* '5' */
    { 0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E },  /* '6' */
    { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 },  /* '7' */
    { 0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E },  /* '8' */
    { 0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C },  /* '9' */
    { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E },  /* 'U' */
    { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F },  /* 'E' */
    { 0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04 },  /* 'V' */
    { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 },  /* 'T' */
    { 0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11 },  /* 'M' */
    { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1F },  /* 'I' */
    { 0x0F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0F },  /* 'C' */
    { 0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00 },  /* '+' */
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C },  /* '.' */
    { 0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10 },  /* '/' */
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* ' ' */
    /* ---- appended: the rest of the uppercase alphabet ------------------ */
    { 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 },  /* 'A' */
    { 0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E },  /* 'B' */
    { 0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E },  /* 'D' */
    { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10 },  /* 'F' */
    { 0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F },  /* 'G' */
    { 0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 },  /* 'H' */
    { 0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C },  /* 'J' */
    { 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 },  /* 'K' */
    { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F },  /* 'L' */
    { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 },  /* 'N' */
    { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E },  /* 'O' */
    { 0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10 },  /* 'P' */
    { 0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D },  /* 'Q' */
    { 0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11 },  /* 'R' */
    { 0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E },  /* 'S' */
    { 0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11 },  /* 'W' */
    { 0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11 },  /* 'X' */
    { 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04 },  /* 'Y' */
    { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F },  /* 'Z' */
    { 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00 },  /* '-' */
    /* ':' arrived with the INFO page's hh:mm:ss uptime. The font is
     * uppercase-and-digits only, and the host font test caught the missing
     * colon before a panel could have. */
    { 0x00, 0x00, 0x04, 0x00, 0x00, 0x04, 0x00 },  /* ':' */
    /* '~' marks a tier's last score on the frozen page, where it has no
     * current one. Without the glyph the marker renders as a gap - the same
     * silent failure the colon was caught for. */
    { 0x00, 0x00, 0x0D, 0x16, 0x00, 0x00, 0x00 },  /* '~' */
    /* '*' marks the tier that fired on the frozen page, in place of the word
     * FIRED, which cost five of the fourteen characters a scale-2 row holds.
     * Unlike the tilde, the font test could not have caught this glyph's
     * absence, because the marker lives in a ternary rather than a format
     * string. That gap is closed in the test. */
    { 0x00, 0x15, 0x0E, 0x15, 0x00, 0x00, 0x00 },  /* '*' */
};

/* One row per character, or a string will silently render as its own index
 * into somebody else's glyph. */
_Static_assert(sizeof(OV_ORDER) - 1 ==
               sizeof(OV_FONT) / sizeof(OV_FONT[0]),
               "OV_ORDER and OV_FONT are different lengths");
/* ---- the drawn state ---------------------------------------------------- */

static char s_off_reason[OV_FOOT_MAX + 1];
static char s_alert_note[OV_FOOT_MAX + 1];
static bool s_alert_remote;

static int  s_batt_slice = -1;      /* -1 = no battery on this board       */
static bool s_batt_charging;

/* THE MAIN SNAPSHOT, DOUBLE BUFFERED. One writer (the frame loop, core 0),
 * one reader (the panel task, core 1). The writer fills the slot the reader
 * is not using and then flips the index, so the reader either sees the whole
 * old snapshot or the whole new one and never half of each. */
static epaper_main_t s_main[2];
static volatile uint8_t  s_main_idx;
static volatile uint32_t s_main_gen;

static char s_page[EPAPER_PAGE_COUNT][EPAPER_PAGE_ROWS][EPAPER_LINE_MAX + 1];
/* ---- per-row scale ------------------------------------------------------
 * The frozen alert page is read from a distance, in daylight, possibly
 * through the enclosure window, and scale 1 is too small for that. Scale 2
 * doubles the glyph, so a row holds 14 characters instead of 28 - which is
 * why only the four tier rows get it and the ratios go on one scale-1 line
 * beneath.
 *
 * Default 0 means scale 1, so every existing page is byte-identical. */
static uint8_t s_page_sc[EPAPER_PAGE_COUNT][EPAPER_PAGE_ROWS];

/* LOWERCASE IS FOLDED, NOT DROPPED.
 *
 * The font has no lowercase glyphs and an unknown character advances the
 * cursor and draws nothing - so before this, a runtime string that happened to
 * be lowercase rendered as a row of holes with nothing anywhere to say so.
 * BOARD_NAME is "pcb-rev-a2"; the status page would have shown "--". Folding
 * costs one comparison per character and turns a silent hole into a legible
 * word. */
static int ov_index(char ch)
{
    if (ch >= 'a' && ch <= 'z') {
        ch = (char)(ch - 'a' + 'A');
    }
    for (int i = 0; OV_ORDER[i]; i++) {
        if (OV_ORDER[i] == ch) {
            return i;
        }
    }
    return -1;
}

static void ov_plot(int lx, int ly, uint8_t rot)
{
    if (lx < 0 || lx >= EPAPER_SCREEN_W || ly < 0 || ly >= EPAPER_SCREEN_H) {
        return;
    }
    int px, py;
    logical_to_panel(lx, ly, rot, s_mirror, &px, &py);
    if (px < 0 || px >= EPD_PANEL_W || py < 0 || py >= EPD_PANEL_H) {
        return;
    }
    s_fb[py * EPD_ROW_BYTES + (px >> 3)] &= (uint8_t)~(1u << (7 - (px & 7)));
}

/* Width in pixels of `str` at `scale`. The trailing inter-character gap is
 * not part of the string, hence the - scale. */
static int ov_text_w(const char *str, int scale)
{
    const int n = (int)strlen(str);
    return n ? (n * (OV_GLYPH_W + 1) * scale - scale) : 0;
}

/* THE LARGEST SCALE THAT PROVABLY FITS a width, computed from the same
 * ov_text_w() the renderer uses.
 *
 * Why this exists rather than a chosen number. The previous ALERT screen was
 * a generated bitmap whose word did not fit the panel, and nobody noticed
 * because the size had been estimated by eye from glyph metrics. A scale
 * derived from the measured width cannot overflow: if the string grows, the
 * type shrinks, and tests/test_epaper_pages.py asserts no ink crosses the
 * margin on every screen at every rotation. */
static int ov_fit_scale(const char *str, int max_w)
{
    for (int sc = 8; sc >= 1; sc--) {
        if (ov_text_w(str, sc) <= max_w) {
            return sc;
        }
    }
    return 1;
}


/* THE ONE GLYPH LOOP. Everything else places a cursor and calls this. */
static void ov_text_at(int x, int y, const char *str, int scale, uint8_t rot)
{
    const int adv = (OV_GLYPH_W + 1) * scale;
    for (int i = 0; str[i]; i++) {
        const int gi = ov_index(str[i]);
        if (gi >= 0) {
            for (int gy = 0; gy < OV_GLYPH_H; gy++) {
                const uint8_t row = OV_FONT[gi][gy];
                for (int gx = 0; gx < OV_GLYPH_W; gx++) {
                    if (row & (1u << (OV_GLYPH_W - 1 - gx))) {
                        for (int sy = 0; sy < scale; sy++) {
                            for (int sx = 0; sx < scale; sx++) {
                                ov_plot(x + gx * scale + sx,
                                        y + gy * scale + sy, rot);
                            }
                        }
                    }
                }
            }
        }
        x += adv;
    }
}

/* Right-aligned so its LAST INK COLUMN is x_last. Used for the value column on
 * MAIN, so a one-digit and a three-digit count end in the same place instead
 * of walking across the screen as the night goes on. */
static void ov_text_right(int x_last, int y, const char *str, int scale,
                          uint8_t rot)
{
    int x = x_last + 1 - ov_text_w(str, scale);
    if (x < PG_X0) {
        x = PG_X0;
    }
    ov_text_at(x, y, str, scale, rot);
}

/* CENTRED, and byte-identical to what this function always did: the ALERT and
 * OFF screens' single footer line is drawn by it and must not move. */
static void ov_text(const char *str, int y, int scale, uint8_t rot)
{
    const int n = (int)strlen(str);
    if (n == 0) {
        return;
    }
    const int adv = (OV_GLYPH_W + 1) * scale;
    int x = (EPAPER_SCREEN_W - (n * adv - scale)) / 2;
    if (x < EPAPER_SCREEN_MARGIN) {
        x = EPAPER_SCREEN_MARGIN;
    }
    ov_text_at(x, y, str, scale, rot);
}

/* Centred, at the largest scale that fits the margins. */
static void ov_text_big(const char *str, int y, uint8_t rot)
{
    ov_text(str, y, ov_fit_scale(str, PG_X1 - PG_X0 + 1), rot);
}

/* ---------------------------------------------------------------------------
 * The battery bar, on MAIN's top row.
 *
 * It is an outline rather than a flashing warning because on e-paper there is
 * no such thing as a flash: every change is a two-second full refresh of the
 * whole panel. A flashing "low cell" would be a permanent 50% duty cycle on
 * the slowest peripheral on the board, would drain the battery it was warning
 * about, and would leave the panel mid-refresh most of the time. So low is
 * drawn as an empty outline, plus words on the page - a static image that is
 * unambiguous at a glance and costs one redraw ever.
 *
 * The bar is drawn only when a slice has been SET. Nothing calls
 * epaper_set_battery() on a board with no divider, so on the breadboard the
 * slice stays -1 and no bar appears.
 *
 * GEOMETRY, in LOGICAL coordinates so it turns with the artwork. It moved from
 * the old footer band to the top row when MAIN became a page: it now sits
 * beside the state word, which is where the brief's own sketch puts it and
 * where an operator's eye already is.
 *   body   x 148..179 (32 wide), y 16..29 (14 tall)
 *   nub    x 180..182, y 20..25
 *   bolt   x 139..144, left of the body
 * The state word is budgeted against BATT_X0 below.
 * ------------------------------------------------------------------------ */
#define BATT_X0    148
#define BATT_X1    179
#define BATT_Y0     16
#define BATT_Y1    (BATT_Y0 + 13)
#define BATT_NUB_W 3
#define BATT_CELLS 4

static void ov_hline(int x0, int x1, int y, uint8_t rot)
{
    for (int x = x0; x <= x1; x++) {
        ov_plot(x, y, rot);
    }
}

static void ov_vline(int x, int y0, int y1, uint8_t rot)
{
    for (int y = y0; y <= y1; y++) {
        ov_plot(x, y, rot);
    }
}

static void ov_fill(int x0, int y0, int x1, int y1, uint8_t rot)
{
    for (int y = y0; y <= y1; y++) {
        ov_hline(x0, x1, y, rot);
    }
}

static void battery_draw(uint8_t rot)
{
    if (s_batt_slice < 0) {
        return;                     /* this board has no battery to draw */
    }
    ov_hline(BATT_X0, BATT_X1, BATT_Y0, rot);
    ov_hline(BATT_X0, BATT_X1, BATT_Y1, rot);
    ov_vline(BATT_X0, BATT_Y0, BATT_Y1, rot);
    ov_vline(BATT_X1, BATT_Y0, BATT_Y1, rot);
    /* the terminal nub, so the outline reads as a battery and not as a box */
    ov_fill(BATT_X1 + 1, BATT_Y0 + 4, BATT_X1 + BATT_NUB_W, BATT_Y1 - 4, rot);

    const int inner = (BATT_X1 - BATT_X0) - 3;        /* usable width */
    const int cw = inner / BATT_CELLS;                /* per cell, incl. gap */
    int n = s_batt_slice;
    if (n > BATT_CELLS) {
        n = BATT_CELLS;
    }
    for (int i = 0; i < n; i++) {
        const int x = BATT_X0 + 2 + i * cw;
        ov_fill(x, BATT_Y0 + 2, x + cw - 2, BATT_Y1 - 2, rot);
    }

    /* THE CHARGE GLYPH, beside the bar rather than inside it: a bolt drawn
     * over the cells would be unreadable at 4/4 and invisible at 0/4. Two
     * strokes, left of the body, only while the charger says so - and that
     * polarity is UNVERIFIED until somebody reads the pin both ways with a
     * real cell fitted. See field/BATTERY_DAY_CHECKLIST.md. */
    if (s_batt_charging) {
        const int bx = BATT_X0 - 9, by = BATT_Y0 + 1;
        for (int k = 0; k < 6; k++) {
            ov_plot(bx + 4 - (k >> 1), by + k, rot);
            ov_plot(bx + 5 - (k >> 1), by + k, rot);
        }
        for (int k = 0; k < 6; k++) {
            ov_plot(bx + 3 - (k >> 1), by + 6 + k, rot);
            ov_plot(bx + 4 - (k >> 1), by + 6 + k, rot);
        }
    }
}

/* ---------------------------------------------------------------------------
 * The three pages.
 *
 * MAIN answers "is it working, and has it heard anything" - the question
 * asked every time somebody walks past the box. STATUS answers "is it
 * configured right", which is asked once at deployment. RECENT answers "what
 * did it hear", which is asked the morning after.
 *
 * The tier list, the operating point and the microphone count LEFT MAIN for
 * STATUS deliberately: they are build telemetry, and a resting screen full of
 * telemetry is a screen an operator learns to stop reading. What did NOT move
 * is a fault: a dead microphone or an unarmed tier appears as a WARNING LINE
 * on MAIN, because an operator must see a fault without knowing to look for
 * one.
 *
 * ROW BUDGET, checked by tests/test_epaper_pages.py against the 16 px margin:
 *   MAIN    y 16 state word (scale 2, <= 11 chars to clear BATT_X0 at 148)
 *           y 16 battery bar, x 148..182
 *           y 38 rule
 *           y 48/70/92 three label+value rows (label scale 1, value scale 2)
 *           y 118 the source of the last remote, scale 1, indented
 *           y 136 the warning line, scale 2, centred, only when set
 *           y 158 rule
 *           y 166/176 two footer lines, scale 1
 *   STATUS  y 16 title, y 34 rule, then six rows at scale 1 from y 46, 18 apart
 *   RECENT  the same, so the two read identically
 * ------------------------------------------------------------------------ */
#define PG_TITLE_Y    16
#define PG_RULE1_Y    38
#define PG_ROW0_Y     48
#define PG_ROW_DY     22
#define PG_SRC_Y     118
#define PG_WARN_Y    136
#define PG_RULE2_Y   158
#define PG_FOOT0_Y   166
#define PG_FOOT1_Y   176

/* STATUS and RECENT: a title band and six small rows. */
#define TX_TITLE_Y    16
#define TX_RULE_Y     34
#define TX_ROW0_Y     42
#define TX_ROW_DY     10
/* A scale-2 glyph is 14 px tall; 17 leaves the same 3 px gap scale 1 has. */
#define TX_ROW_DY2    17
#define TX_HINT_Y    176

/* Minutes -> a coarse age an operator reads at a glance. Deliberately loses
 * precision as it grows: "17M" matters, "3H" matters, "2D" matters, and the
 * difference between 61 and 62 minutes does not - and every unit of precision
 * kept here is a two-second full-panel refresh spent on it. */
static void age_str(int32_t minutes, char *dst, int cap)
{
    if (minutes < 0) {
        snprintf(dst, cap, "NONE");
    } else if (minutes < 60) {
        snprintf(dst, cap, "%dM", (int)minutes);
    } else if (minutes < 60 * 48) {
        snprintf(dst, cap, "%dH", (int)(minutes / 60));
    } else {
        snprintf(dst, cap, "%dD", (int)(minutes / (60 * 24)));
    }
}

/* ---------------------------------------------------------------------------
 * HOME - the listening screen, and it is up for hours.
 *
 * One dominant word, the battery, and a two-field footer. The tier list, the
 * thresholds and the uptime live on the info page: they answer "is it
 * configured right", which is asked once at deployment, and this screen
 * answers "is it working", which is asked every time somebody walks past.
 *
 * The word is sized by ov_fit_scale() rather than by a chosen number, so it
 * cannot overflow the margins however long a future state word gets.
 * ------------------------------------------------------------------------ */
#define HOME_WORD_Y      66
#define HOME_RULE_Y     146
#define HOME_FOOT_Y     158

static void page_main_draw(uint8_t rot)
{
    const epaper_main_t *m = &s_main[s_main_idx & 1u];
    char buf[EPAPER_LINE_MAX + 1];

    /* THE BATTERY, top right, unchanged in mapping and thresholds. USB where
     * a cell is deliberately absent, because an empty bar reads as a broken
     * gauge and that is a different fact. */
    if (m->batt_absent) {
        ov_text_at(BATT_X0, PG_TITLE_Y, "USB", 2, rot);
    } else {
        battery_draw(rot);
    }

    /* ---- the test tag --------------------------------------------------
     * Small, top right, immediately left of the battery. It exists so that a
     * unit cannot reach a deployment in test mode unnoticed: test mode is
     * invisible in the detection numbers by design, so the glass is the only
     * place an operator without a laptop can see it.
     *
     * ov_text_width() rather than a hand-counted offset, because "TEST" at
     * scale 1 is four glyphs and the advance is a constant this file already
     * owns; a literal here would silently overlap the bar if either moved. */
    if (m->test_mode) {
        ov_text_at(BATT_X0 - ov_text_w("TEST", 1) - 4, PG_TITLE_Y, "TEST", 1,
                   rot);
    }

    /* LOW BATT beside the bar, in words. On e-paper there is no such thing as
     * a flashing warning - every change is a full refresh - so it is static
     * text or it is nothing. */
    if (m->warn[0]) {
        ov_text_at(PG_X0, PG_TITLE_Y + 4, m->warn, 1, rot);
    }

    /* THE DOMINANT ELEMENT. */
    ov_text_big(m->state_word[0] ? m->state_word : "LISTENING",
                HOME_WORD_Y, rot);

    ov_hline(PG_X0, PG_X1, HOME_RULE_Y, rot);

    /* THE FOOTER, two fields and nothing else: what it is hearing, and how
     * many times it has fired since power-on. */
    ov_text_at(PG_X0, HOME_FOOT_Y, m->ambient[0] ? m->ambient : "----", 2, rot);
    /* ---- local plus remote --------------------------------------------
     * The count an operator reads off this screen is "how many alerts has
     * this position had", and an alert relayed from a peer is one of them.
     * Printing alerts_local alone leaves the number frozen on the receiving
     * unit of a pair. Both halves are already in the struct. */
    snprintf(buf, sizeof(buf), "%u",
             (unsigned)(m->alerts_local + m->alerts_remote));
    ov_text_right(PG_X1, HOME_FOOT_Y, buf, 2, rot);
}

/* ALERT. The word as large as it will go, and one line saying which tier and
 * at what frequency - because afterwards that is the only thing anyone wants
 * to know, and Tier-3 fires on fans as readily as on rotors. */
static void page_alert_draw(uint8_t rot)
{
    const epaper_main_t *m = &s_main[s_main_idx & 1u];
    ov_text_big("ALERT", 62, rot);
    if (m->alert_line[0]) {
        ov_text(m->alert_line, 126, 2, rot);
    }
}

/* SNOOZE. Deliberately NOT an animated countdown: ten seconds of per-second
 * redraw is ten full-panel refreshes to tell somebody something they already
 * know, and the panel would spend the whole snooze mid-refresh. */
static void page_snooze_draw(uint8_t rot)
{
    const epaper_main_t *m = &s_main[s_main_idx & 1u];
    char buf[16];
    ov_text_big("SNOOZE", 62, rot);
    snprintf(buf, sizeof(buf), "%uS", (unsigned)m->snooze_s);
    ov_text(buf, 128, 3, rot);
}


static void page_text_draw(uint8_t page, const char *title, uint8_t rot)
{
    ov_text(title, TX_TITLE_Y, 2, rot);
    ov_hline(PG_X0, PG_X1, TX_RULE_Y, rot);
    /* Y ADVANCES BY THE ROW'S OWN HEIGHT, so a page may mix scales. With
     * every scale at its default this is TX_ROW0_Y + r * TX_ROW_DY exactly as
     * before. */
    int y = TX_ROW0_Y;
    for (int r = 0; r < EPAPER_PAGE_ROWS; r++) {
        const int sc = s_page_sc[page][r] ? s_page_sc[page][r] : 1;
        if (s_page[page][r][0]) {
            ov_text_at(PG_X0, y, s_page[page][r], sc, rot);
        }
        y += (sc >= 2) ? TX_ROW_DY2 : TX_ROW_DY;
    }
    /* One line of grammar, so nobody has to be told twice how to get back.
     *
     * Not on INFO, for two reasons. It would sit on top of the fourteenth row,
     * and it would be a LIE: under button v3 a tap on INFO returns to the base
     * state, it does not turn a page. A hint that describes a gesture the
     * device no longer has is worse than no hint. */
    if (page != EPAPER_PAGE_STATUS) {
        ov_text("TAP TO RETURN", TX_HINT_Y, 1, rot);
    }
}

/* ---- the published API -------------------------------------------------- */

void epaper_set_battery(int slice, bool charging)
{
    s_batt_slice = (slice < 0) ? -1 : (slice > BATT_CELLS ? BATT_CELLS : slice);
    s_batt_charging = charging;
}

void epaper_set_off_reason(const char *text)
{
    if (!text) {
        s_off_reason[0] = 0;
        return;
    }
    strncpy(s_off_reason, text, OV_FOOT_MAX);
    s_off_reason[OV_FOOT_MAX] = 0;
}

void epaper_set_alert_remote(bool remote) { s_alert_remote = remote; }

void epaper_set_alert_note(const char *text)
{
    if (!text) {
        s_alert_note[0] = 0;
        return;
    }
    strncpy(s_alert_note, text, OV_FOOT_MAX);
    s_alert_note[OV_FOOT_MAX] = 0;
}

void epaper_set_main(const epaper_main_t *m)
{
    if (!m) {
        return;
    }
    const uint8_t front = s_main_idx & 1u;
    /* IDENTICAL SNAPSHOT, NO GENERATION BUMP. This is the whole of the
     * refresh discipline's arithmetic: the producer may call this every
     * second, and only a snapshot that would DRAW DIFFERENTLY costs a
     * two-second full refresh. Without it a once-a-second producer would
     * redraw the panel once a second forever. */
    if (memcmp(&s_main[front], m, sizeof(*m)) == 0) {
        return;
    }
    const uint8_t back = (uint8_t)(front ^ 1u);
    s_main[back] = *m;
    s_main_idx = back;              /* publish, after the copy is complete */
    s_main_gen++;
}

uint32_t epaper_main_gen(void) { return s_main_gen; }

void epaper_page_clear(uint8_t page)
{
    if (page >= EPAPER_PAGE_COUNT) {
        return;
    }
    memset(s_page[page], 0, sizeof(s_page[page]));
    memset(s_page_sc[page], 0, sizeof(s_page_sc[page]));
}

/* Sets the row's text AND its scale in one call, so a caller cannot set one
 * and forget the other. scale 1 or 2; anything else is treated as 1. */
void epaper_page_line_sc(uint8_t page, uint8_t row, const char *text,
                         uint8_t scale)
{
    if (page >= EPAPER_PAGE_COUNT || row >= EPAPER_PAGE_ROWS) {
        return;
    }
    epaper_page_line(page, row, text);      /* resets the scale to default */
    s_page_sc[page][row] = (scale >= 2u) ? 2u : 1u;
}

void epaper_page_line(uint8_t page, uint8_t row, const char *text)
{
    if (page >= EPAPER_PAGE_COUNT || row >= EPAPER_PAGE_ROWS) {
        return;
    }
    /* ---- the scale is reset, not inherited -----------------------------
     * The live test page and the frozen page share the STATUS slot, and the
     * frozen page sets rows 1-4 to scale 2. Writing the live page's rows over
     * them while leaving s_page_sc alone brings the legend and the first three
     * tier rows up at double width, running off the right edge, while the rows
     * the frozen page never touched render correctly at scale 1.
     *
     * So the two setters are honest about what they own: this one writes
     * text AT THE DEFAULT SCALE, and epaper_page_line_sc() is the only way to
     * get anything else. A row's scale can no longer outlive the page that
     * asked for it. */
    if (!text) {
        s_page[page][row][0] = 0;
        s_page_sc[page][row] = 0;
        return;
    }
    strncpy(s_page[page][row], text, EPAPER_LINE_MAX);
    s_page[page][row][EPAPER_LINE_MAX] = 0;
    s_page_sc[page][row] = 0;
}

static void blit(const uint8_t *img, uint8_t rot)
{
    memset(s_fb, 0xFF, sizeof(s_fb));       /* white ground */
    for (int py = 0; py < EPD_PANEL_H; py++) {
        for (int px = 0; px < EPD_PANEL_W; px++) {
            int lx, ly;
            panel_to_logical(px, py, rot, s_mirror, &lx, &ly);
            if (lx < 0 || lx >= EPAPER_SCREEN_W ||
                ly < 0 || ly >= EPAPER_SCREEN_H) {
                continue;
            }
            if (!logical_pixel(img, lx, ly)) {
                s_fb[py * EPD_ROW_BYTES + (px >> 3)] &=
                    (uint8_t)~(1u << (7 - (px & 7)));
            }
        }
    }
}

/* A white ground with no artwork on it. The three pages are text, so there is
 * nothing to blit - and before this existed, "draw nothing first" was not
 * expressible: every case blitted a bitmap. */
static void ground(void)
{
    memset(s_fb, 0xFF, sizeof(s_fb));
}

void epaper_draw_compose(epaper_screen_t scr, uint8_t rot, bool mirror)
{
    s_mirror = mirror;
    switch (scr) {
    case EPAPER_SCREEN_READY:
        ground();
        page_main_draw(rot);
        break;

    /* MENU - everything that used to clutter the resting screen, on one page,
     * reached by a double press. It keeps the STATUS screen's enum slot: the
     * numbering is what `U e` and the host tools speak, and renumbering it
     * would silently change what an older runbook asks for. */
    case EPAPER_SCREEN_STATUS:
        ground();
        /* THE SLOT IS THE SAME, THE TITLE IS NOT. In test mode this page
         * carries the four tier rows instead of the box's identity, and a
         * page of live scores titled MENU would be read as the wrong page.
         * The enum slot is deliberately unchanged: the numbering is what
         * `U e` and the host tools speak. */
        page_text_draw(EPAPER_PAGE_STATUS,
                       s_main[s_main_idx & 1u].test_mode ? "TEST" : "MENU",
                       rot);
        break;

    case EPAPER_SCREEN_RECENT:
        ground();
        page_text_draw(EPAPER_PAGE_RECENT, "RECENT", rot);
        break;

    case EPAPER_SCREEN_SNOOZE:
        ground();
        page_snooze_draw(rot);
        break;

    /* ALERT is composed rather than blitted. A generated bitmap has to be
     * sized by eye, and one that does not fit the panel is a hard defect. The
     * word is sized from its own measured width and a host test asserts it
     * stays inside the margin at every rotation. */
    case EPAPER_SCREEN_ALERT:
        ground();
        page_alert_draw(rot);
        /* WHO IT CAME FROM. Empty for a local alert, and "REMOTE N-1A2B" when
         * a peer heard it - the first question an operator asks about an alert
         * with no audible source in front of them. */
        /* AND WHETHER IT WAS OURS AT ALL, in a size that carries.
         *
         * The peer id below is the detail; THIS is the fact. A remote alert
         * means there is nothing in front of this operator to look at, and
         * until they know that they will spend the alert window searching
         * their own sky. Scale 3 is 21 px tall at y=148, ending at 169 with
         * the note at 177 and the margin at 184: it fits between the two
         * things already on this screen and collides with neither, which is
         * what tests/test_epaper_pages.py checks. */
        if (s_alert_remote) {
            ov_text("REMOTE", 148, 3, rot);
        }
        ov_text(s_alert_note, OV_FOOT_Y, 1, rot);
        break;

    case EPAPER_SCREEN_ALERT_WASH:
        blit(epaper_screen_alert_wash, rot);
        break;

    case EPAPER_SCREEN_OFF:
        blit(epaper_screen_off, rot);
        /* THE REASON LINE. "OFF" alone answers "is it listening" and leaves
         * "why did it stop" to whoever finds it, which on this device is two
         * completely different situations: an operator who is about to slide
         * the switch, and a cell that ran out overnight. */
        ov_text(s_off_reason, OV_FOOT_Y, 1, rot);
        break;

    case EPAPER_SCREEN_TEST:
        blit(epaper_screen_test, rot);
        break;

    default:
        break;
    }
}
