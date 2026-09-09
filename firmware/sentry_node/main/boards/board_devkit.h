/*
 * boards/board_devkit.h - the pin map for the ESP32-S3-DevKitC-1 breadboard
 * build. One file, no second opinion.
 *
 * Target: ESP32-S3-DevKitC-1 N16R8 (16 MB quad flash, 8 MB octal PSRAM).
 *
 * If you are wiring the board, do not read the numbers out of a document.
 * Send `I` to the running firmware: it prints the pin map from these macros,
 * so what you read is what is actually flashed. A document can go stale; this
 * cannot.
 *
 * ---------------------------------------------------------------------------
 * Why these pins. Unavailable on this module, and why:
 *
 *   0, 3, 45, 46   strapping - level at reset selects boot mode / VDD_SPI /
 *                  ROM log verbosity
 *   19, 20         native USB D-/D+ - this is the console and the flash link
 *   26-32          SPI flash. QIO (set in sdkconfig.defaults) uses IO2/IO3,
 *                  which were already bonded here, so QIO costs no extra pins
 *   33-37          octal PSRAM. The N16R8 bonds the PSRAM die to these pins
 *                  even with CONFIG_SPIRAM=n; the module wires them
 *                  regardless, so they are not free
 *   38, 48         onboard addressable RGB LED (48 on DevKitC-1 v1.0, 38 on
 *                  v1.1). Avoid both rather than guess the board revision
 *   39-42          MTCK/MTDO/MTDI/MTMS - usable as GPIO, kept free so an
 *                  external JTAG probe stays possible
 *   43, 44         UART0 TX/RX (silkscreened) - kept free as a fallback
 *                  console if USB-Serial-JTAG ever misbehaves
 *
 * GPIO 4, 5, 6, 7 are free, have no alternate function, and sit adjacent on
 * one header - four wires in a row.
 * ---------------------------------------------------------------------------
 */
#pragma once

/* ---- mic #1 (INMP441) --------------------------------------------------- */
#define PIN_I2S_BCLK   4    /* mic SCK  - bit clock,  master out */
#define PIN_I2S_WS     5    /* mic WS   - word select, master out */
#define PIN_I2S_SD     6    /* mic SD   - data,        mic out / ESP in */

/* Mic #2 shares BCLK and WS with #1 and drives the other half of the frame
 * (its L/R tied to 3V3 instead of GND), so a second mic needs no new clock
 * pins at all. GPIO 7 carries a second data line instead - mics #3/#4 on the
 * same peripheral, or a second mic pair that must stay electrically
 * independent. */
#define PIN_I2S_SD2    7    /* quad build: mics #3/#4 data line.
                             * #3 = LEFT slot (L/R->GND), #4 = RIGHT (L/R->3V3).
                             * Read by I2S1 as a slave off the same GPIO4/GPIO5
                             * clocks that I2S0 drives, so all four microphones
                             * share one clock domain. */

/* ---- alert outputs and operator controls -------------------------------
 * None of these is initialised at boot except the two NPN bases below, which
 * must be driven low immediately: their bases hang off these pins through
 * 1 kOhm, so a high-Z pad at reset is an undefined base drive. Everything
 * else initialises lazily on first command.
 *
 * GPIO 10/11/12 are the native FSPICS0 / FSPID / FSPICLK pins, which is why
 * the e-paper is on SPI2 there rather than on arbitrary GPIO. */
#define PIN_EPD_CS     10   /* e-paper chip select   (SPI2 / FSPI)          */
#define PIN_EPD_DIN    11   /* e-paper MOSI                                  */
#define PIN_EPD_CLK    12   /* e-paper SCLK                                  */
#define PIN_EPD_DC     13   /* e-paper data/command, plain GPIO              */
#define PIN_EPD_RST    14   /* e-paper reset, plain GPIO                     */
#define PIN_EPD_BUSY   15   /* e-paper busy, INPUT, polled with a timeout    */
#define PIN_LED_WS2812 16   /* WS2812 data via 330 Ohm; RMT TX, 800 kHz      */
#define PIN_BUZZER     17   /* NPN base via 1 kOhm. HIGH = ON. LOW at boot.  */
#define PIN_MOTOR      18   /* NPN base via 1 kOhm. HIGH = ON. LOW at boot.  */
#define PIN_SNOOZE_BTN 21   /* button to GND, internal pull-up. PRESSED = LOW */

/* ---- what the mic itself must be strapped to ---------------------------- */
/*   VDD -> 3V3      (INMP441 is 1.8-3.3 V; the usual breakout has a reg)
 *   GND -> GND
 *   L/R -> GND      => the mic drives the LEFT slot, data valid while WS low,
 *                      which is why the firmware selects I2S_STD_SLOT_LEFT.
 *                      Tying L/R to 3V3 instead moves it to the RIGHT slot and
 *                      the firmware reads silence. */
#define MIC_LR_STRAP   "GND (LEFT slot)"

/* BCLK = fs * bits_per_slot * slots = 16000 * 32 * 2 = 1.024 MHz.
 * Std (Philips) mode always clocks two slots even when only one is stored. */
#define I2S_BCLK_HZ    (CFG_FS * 32 * 2)

/* ==========================================================================
 * What this board has, and what it has not.
 *
 * These flags exist so that PCB-only hardware is compiled out here rather
 * than remapped onto whatever pin happens to be free. Remapping is how a
 * driver ends up driving the wrong thing on the wrong board and nobody finds
 * out until the smoke; a zero here means the code does not exist in this
 * image at all.
 *
 * GPIO48 is why that distinction matters. The PCB puts the LoRa module's DIO0
 * on GPIO48, which on a DevKitC-1 v1.0 is the onboard addressable RGB LED (it
 * is GPIO38 on a v1.1, which is why the map above avoids both).
 * BOARD_HAS_LORA 0 therefore does not mean "the radio is absent so pick
 * another pin" - it means the radio driver is not built, is not linked, and
 * cannot touch GPIO48 on this board by any path.
 * ========================================================================== */
#define BOARD_NAME            "devkit-breadboard"
#define BOARD_HAS_VBAT        0   /* no divider, no fuel gauge, USB only     */
#define BOARD_HAS_CHG_STAT    0   /* no charger                              */
#define BOARD_HAS_LORA        0   /* no radio - and never GPIO48 here        */
#define BOARD_SLIDE_HARD_CUT  0   /* no slide switch at all                  */
#define BOARD_GEOMETRY        GEOM_BREADBOARD

/* ---- what `U G` may pull up: derived, not listed -------------------------
 *
 * The pin scan configures a GPIO with the internal pull-up and watches it for
 * six seconds, to answer "which pin is the button really on". That is safe
 * only on a pin nothing else drives, and which pins those are is a property
 * of the board, not of the firmware.
 *
 * The three predicates below are what this board says about itself;
 * board_pins.h derives the scan set from them. A hand-maintained list beside
 * the PIN_ defines would be a second statement of the same fact, and the two
 * would drift: six of this board's free pins are allocated on the PCB - the
 * VBAT divider, the charger STAT line and four LoRa signals - so the same
 * list scanned there would drive a pull-up onto a live SPI bus and the
 * radio's reset line.
 *
 * The derived set on this board is {1, 2, 8, 9, 21, 39, 40, 41, 42, 47}.
 * tests/test_board_seam.py asserts that by host-compiling this header, not by
 * re-reading this sentence.
 */

/* Every pin this firmware drives on this board, enumerated from the PIN_
 * defines above and from nothing else. A new peripheral adds its pin here in
 * the same edit that defines it; the seam test fails when a PIN_ define in
 * this file is missing from this predicate, because a driven pin the scan
 * does not know about is exactly the hazard the derivation exists to remove. */
#define BOARD_PIN_ALLOCATED(p)                                               \
    ((p) == PIN_I2S_BCLK || (p) == PIN_I2S_WS || (p) == PIN_I2S_SD ||        \
     (p) == PIN_I2S_SD2 || (p) == PIN_EPD_CS || (p) == PIN_EPD_DIN ||        \
     (p) == PIN_EPD_CLK || (p) == PIN_EPD_DC || (p) == PIN_EPD_RST ||        \
     (p) == PIN_EPD_BUSY || (p) == PIN_LED_WS2812 || (p) == PIN_BUZZER ||    \
     (p) == PIN_MOTOR || (p) == PIN_SNOOZE_BTN)

/* What the part and the module make unusable - the table at the top of this
 * file, as code. Identical to the PCB header's copy because it is a fact
 * about the ESP32-S3-WROOM-1 N16R8 and not about either board; written out
 * twice rather than shared, on the same grounds the PCB header gives for
 * repeating the pin numbers - the two agreeing is a thing to be checked, and
 * the seam test checks it, rather than an inheritance to be assumed.
 *
 * 22-25 are here and not in the prose table above because they are not pins
 * at all: the ESP32-S3 does not bond them. A scan that walks the GPIO space
 * has to know that; a scan that read a hand-written list never had to. */
#define BOARD_PIN_IS_RESERVED(p)                                             \
    ((p) == 0 || (p) == 3 || (p) == 45 || (p) == 46 ||  /* strapping     */  \
     (p) == 19 || (p) == 20 ||                          /* native USB    */  \
     ((p) >= 22 && (p) <= 25) ||                        /* not bonded    */  \
     ((p) >= 26 && (p) <= 32) ||                        /* SPI flash     */  \
     ((p) >= 33 && (p) <= 37))                          /* octal PSRAM   */

/* Pins this board keeps clear. Usable, unallocated - and still not the pin
 * scan's to touch:
 *
 *   38, 48   the DevKitC-1's onboard addressable RGB (48 on v1.0, 38 on
 *            v1.1). Both avoided rather than guess the board revision, which
 *            is the same reason the map at the top of this file avoids them.
 *   43, 44   U0TXD / U0RXD. Kept free as a fallback console, and on this part
 *            the ROM and second-stage bootloaders print on U0TXD at every
 *            reset - so something is on that pin whatever the firmware does.
 *            That is what separates 43/44 from 39-42, which the scan does
 *            probe: JTAG needs a probe and no probe is attached, while the
 *            console is attached by definition.
 */
#define BOARD_PIN_KEEP_CLEAR(p)                                              \
    ((p) == 38 || (p) == 48 ||                          /* onboard RGB   */  \
     (p) == 43 || (p) == 44)                            /* UART0 console */
