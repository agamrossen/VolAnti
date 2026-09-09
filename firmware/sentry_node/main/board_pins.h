/*
 * board_pins.h - the board seam. One include, two targets, one pin map.
 *
 * This header is a dispatcher; the maps themselves live in boards/. Every
 * consumer says `#include "board_pins.h"` and reads PIN_I2S_BCLK, PIN_EPD_CS,
 * PIN_SNOOZE_BTN and the rest without knowing which board it is on.
 *
 * The DevKit build and the PCB build are two targets of one codebase and
 * never two codebases. A fork would mean two detectors, two golden gates and
 * two sets of measurements that drift apart. So the difference between the
 * boards is confined to one directory, and the rule that keeps it there is
 * short enough to check:
 *
 *     A #ifdef on a GPIO number, anywhere outside boards/, is a defect.
 *
 * Feature flags are the mechanism instead. Code that needs hardware the
 * DevKit has not got asks `#if BOARD_HAS_LORA` / `BOARD_HAS_VBAT` /
 * `BOARD_HAS_CHG_STAT` / `BOARD_SLIDE_HARD_CUT`, which is a question about
 * capability and compiles out cleanly when the answer is no. Capability tests
 * are permitted anywhere, drivers and the frame loop alike, but the frame
 * loop is where a seam starts to leak: if the count of `BOARD_*` occurrences
 * in sentry_node.c passes five, extract a `board_caps` accessor rather than
 * adding a sixth.
 *
 * Selecting a target:
 *
 *     idf.py menuconfig   ->  SENTRY-Node board  ->  ESP32-S3-DevKitC-1 ...
 *     idf.py -DSENTRY_BOARD_PCB_A2=y build
 *
 * The default is the DevKit, deliberately: every workflow, runbook, gate and
 * measurement in this repository was taken on that board, and a seam whose
 * arrival silently changed what `idf.py build` produces would invalidate all
 * of them at once.
 *
 * What each target must define is the contract at the bottom of this file,
 * asserted rather than described - a board header that forgets a flag fails
 * the build rather than defaulting to something plausible.
 */
#pragma once

#include <stdint.h>

/* Geometry profile ids. These are the keys in data/geometry_profiles.json,
 * spelled identically on purpose: `I` prints the id, and an operator or a
 * host tool then looks up the actual microphone coordinates in the one place
 * they are written down. Detection is geometry-free - nothing here reaches
 * the detector - so these are documentation and instrument constants. */
#define GEOM_BREADBOARD  "breadboard_plus_v1"
#define GEOM_PCB_A2      "pcb_rev_a2"

#if defined(CONFIG_SENTRY_BOARD_PCB_A2)
#include "boards/board_pcb_a2.h"
#else
#include "boards/board_devkit.h"
#endif

/* ---- the contract every board header owes -------------------------------
 * Checked here rather than trusted, because the failure mode of a missing
 * capability flag is the worst kind: `#if BOARD_HAS_LORA` on an undefined
 * macro is not an error in C, it is a silent zero. A board that has a radio
 * and forgot to say so would simply not build the driver, and the first
 * symptom would be a field device that never transmits. */
#if !defined(BOARD_NAME)
#error "the board header must define BOARD_NAME"
#endif
#if !defined(BOARD_HAS_VBAT) || !defined(BOARD_HAS_CHG_STAT) || \
    !defined(BOARD_HAS_LORA) || !defined(BOARD_SLIDE_HARD_CUT)
#error "the board header must define every BOARD_HAS_* / BOARD_SLIDE_* flag"
#endif
#if !defined(BOARD_GEOMETRY)
#error "the board header must define BOARD_GEOMETRY"
#endif
/* The `U G` pin scan derives its candidate set from these three and nothing
 * else. All three are function-like, so a board that defines none of them
 * fails here with a sentence rather than at the derivation with a cascade of
 * "undeclared identifier". */
#if !defined(BOARD_PIN_ALLOCATED) || !defined(BOARD_PIN_IS_RESERVED) || \
    !defined(BOARD_PIN_KEEP_CLEAR)
#error "the board header must define BOARD_PIN_ALLOCATED / _IS_RESERVED / _KEEP_CLEAR"
#endif

/* A board that claims a radio owes its four pins, and a board that claims a
 * battery owes its divider and its slice table. Stated as one check each so
 * the error message names the missing half. */
#if BOARD_HAS_LORA && (!defined(PIN_LORA_MISO) || !defined(PIN_LORA_CS) || \
                       !defined(PIN_LORA_RST) || !defined(PIN_LORA_DIO0))
#error "BOARD_HAS_LORA is set but the radio's pins are not defined"
#endif
#if BOARD_HAS_VBAT && (!defined(PIN_VBAT_SENSE) || \
                       !defined(BOARD_VBAT_MV_1) || !defined(BOARD_VBAT_MV_4))
#error "BOARD_HAS_VBAT is set but the sense pin or the slice table is missing"
#endif
#if BOARD_HAS_CHG_STAT && !defined(PIN_CHG_STAT)
#error "BOARD_HAS_CHG_STAT is set but PIN_CHG_STAT is not defined"
#endif

/* A board with no radio still has to compile the settings record that holds
 * the radio's frequency - one struct, both targets, so that a settings blob
 * means the same thing everywhere and a future board cannot silently change
 * its layout. Zero is the honest value: there is no channel. */
#if !defined(BOARD_LORA_HZ_DEFAULT)
#define BOARD_LORA_HZ_DEFAULT 0u
#endif

/* ==========================================================================
 * The `U G` scan set, derived from the active board header.
 *
 * The candidate pins are derived rather than listed. A hand-maintained list
 * beside each board's PIN_ defines is a second statement of the same fact and
 * drifts from it: six of the DevKit's free pins are allocated on the PCB (the
 * VBAT divider, the charger STAT line and four LoRa signals), so scanning a
 * stale list there would drive a pull-up onto a live SPI bus and onto the
 * radio's reset.
 *
 * Asserting each entry against BOARD_PIN_ALLOCATED would also catch that, and
 * is the smaller edit, but it leaves the two statements in place and hires a
 * referee between them - and the referee has to be remembered, so an eleventh
 * entry added without its assertion is un-refereed and the build stays green.
 * Deriving leaves one statement, and one statement cannot disagree with
 * itself.
 *
 * The button is the one exemption, and it is here rather than in a board
 * header because the reason is not a board fact: GPIO21 is where the button
 * is supposed to be, and an instrument that cannot return the expected answer
 * cannot tell "wired correctly" from "never looked". Probing it drives
 * nothing, because an input with a pull-up is what it already is in normal
 * service.
 *
 * BOARD_PIN_MAX is a GPIO number outside boards/, which the seam rule
 * otherwise forbids. It is allowed here, and only here, because it is not a
 * board fact: it is the size of the ESP32-S3's GPIO space, the same on every
 * board this firmware will run on, and duplicating it into each header would
 * invite exactly the drift this file exists to stop.
 * ========================================================================== */
#define BOARD_PIN_MAX  48        /* GPIO48 is the highest on the ESP32-S3 */

#define BOARD_PIN_SCANNABLE(p)                                              \
    ((p) >= 0 && (p) <= BOARD_PIN_MAX &&                                    \
     !BOARD_PIN_IS_RESERVED(p) && !BOARD_PIN_KEEP_CLEAR(p) &&               \
     (!BOARD_PIN_ALLOCATED(p) || (p) == PIN_SNOOZE_BTN))

/* The scan set, materialised. A function rather than a macro at the call
 * site, so the frame loop asks one question - "what may I probe on this
 * board" - and the rule that answers it stays here with the board headers.
 * `n` is at most BOARD_PIN_MAX + 1 by construction, so the caller's buffer is
 * sized from the struct and cannot be walked off the end by a board with one
 * more free pin than the last one. */
typedef struct {
    uint8_t pin[BOARD_PIN_MAX + 1];
    int     n;
} board_scan_t;

static inline void board_scan_pins(board_scan_t *out)
{
    out->n = 0;
    for (int p = 0; p <= BOARD_PIN_MAX; p++) {
        if (BOARD_PIN_SCANNABLE(p)) {
            out->pin[out->n++] = (uint8_t)p;
        }
    }
}
