/*
 * boards/board_pcb_a2.h - the pin map for the SENTRY-Node PCB, rev A2.
 *
 * Transcribed from the board specification, which lives with the board design
 * rather than in this repository. This header is therefore a copy of numbers
 * that are authoritative somewhere else - the one place in this firmware
 * where a constant cannot be re-derived from anything in the tree. When a
 * board is in front of you, `I` and a multimeter are the authority, not this
 * file, which is the same rule the DevKit header carries.
 *
 * ---------------------------------------------------------------------------
 * GPIO numbering is identical to the validated DevKitC-1 build, so the
 * fourteen proven pins below are byte-for-byte the DevKit's. They are
 * repeated here rather than included from the DevKit header, because the two
 * boards agreeing today is a fact to be checked - the seam test checks it -
 * and not an inheritance to be assumed: the day a later revision moves one
 * pin, an inheritance would move it on the breadboard too and silently
 * invalidate every bench measurement this project owns.
 *
 * What is genuinely new on this board is at the bottom: the battery sense
 * divider, the charger status line, and the LoRa radio.
 * ---------------------------------------------------------------------------
 */
#pragma once

/* ---- mic #1 (INMP441) --------------------------------------------------- */
#define PIN_I2S_BCLK   4    /* mic SCK  - bit clock,  master out */
#define PIN_I2S_WS     5    /* mic WS   - word select, master out */
#define PIN_I2S_SD     6    /* mic SD   - data,        mic out / ESP in */
#define PIN_I2S_SD2    7    /* mics #3/#4 data line, read by I2S1 as a slave
                             * off the same GPIO4/GPIO5 clocks I2S0 drives. */

/* ---- alert outputs and operator controls -------------------------------- */
#define PIN_EPD_CS     10   /* e-paper chip select   (SPI2 / FSPI)          */
#define PIN_EPD_DIN    11   /* e-paper MOSI - shared with the radio          */
#define PIN_EPD_CLK    12   /* e-paper SCLK - shared with the radio          */
#define PIN_EPD_DC     13   /* e-paper data/command, plain GPIO              */
#define PIN_EPD_RST    14   /* e-paper reset, plain GPIO                     */
#define PIN_EPD_BUSY   15   /* e-paper busy, INPUT, polled with a timeout    */
#define PIN_LED_WS2812 16   /* WS2812 data via 330 Ohm; RMT TX, 800 kHz      */
#define PIN_BUZZER     17   /* NPN base via 1 kOhm. HIGH = ON. LOW at boot.  */
#define PIN_MOTOR      18   /* NPN base via 1 kOhm. HIGH = ON. LOW at boot.  */
#define PIN_SNOOZE_BTN 21   /* button to GND, internal pull-up. PRESSED = LOW */

/* ---- what the mic itself must be strapped to ---------------------------- */
#define MIC_LR_STRAP   "GND (LEFT slot)"
#define I2S_BCLK_HZ    (CFG_FS * 32 * 2)

/* ==========================================================================
 * New on the PCB.
 * ========================================================================== */

/* Battery sense. A 100k/100k divider from VBAT into ADC1_CH0, with a 100 nF
 * cap across the lower leg, so the pin sees VBAT/2 - comfortably inside the
 * ADC's range for a 1S cell at 4.2 V. The divider ratio is a board constant
 * and lives here; power_mon.c multiplies by it and never knows the number. */
#define PIN_VBAT_SENSE        1
#define BOARD_VBAT_DIV_NUM    2      /* VBAT = pin_mV * NUM / DEN            */
#define BOARD_VBAT_DIV_DEN    1

/* Charger status. BQ24074-class power path; STAT is open drain with a 100k
 * pull-up, so it reads low while charging and floats high otherwise.
 *
 * That polarity is what the part family does rather than what this board has
 * been measured to do, and a charge glyph that is lit whenever the battery is
 * not charging is the kind of wrong that survives a whole field season. Read
 * the pin plugged and unplugged at bring-up; if it comes back inverted, this
 * one line changes and nothing else does. */
#define PIN_CHG_STAT          2
#define BOARD_CHG_ACTIVE_LOW  1

/* The radio: Ai-Thinker Ra-01H (SX1276) on SPI2, sharing SCK and MOSI with
 * the e-paper and owning MISO alone - the panel is write-only, so nothing
 * else can drive that line. Its own CS, reset and DIO0.
 *
 * PA_BOOST only. The Ra-01H bonds the SX1276's PA_BOOST output and leaves RFO
 * unconnected, so the PA select bit is not a preference on this module, it is
 * the only setting that transmits at all. */
#define PIN_LORA_MISO         8
#define PIN_LORA_CS           9    /* idle HIGH */
#define PIN_LORA_RST          47
#define PIN_LORA_DIO0         48
#define BOARD_LORA_PA_BOOST   1

/* The deployment frequency is a configuration, not a constant. 868.100 MHz is
 * the UK test channel; the antenna is the band-specific element and the
 * module covers 803-930 MHz. A deployment sets this from the local
 * regulations with `U c lorahz`, which is why it is persisted rather than
 * compiled. This is the value a factory-reset device comes up on. */
#define BOARD_LORA_HZ_DEFAULT  868100000u

/* ---- battery slice thresholds, in millivolts ----------------------------
 * A 1S Li-Po under this device's ~100 mA class load: four slices and a low
 * warning, with hysteresis on every boundary so a battery sitting on a
 * threshold cannot flicker the bar - and on e-paper a flickering bar is not a
 * cosmetic problem, it is a full 2 s panel refresh each way.
 *
 * These are defaults, to be re-anchored at bring-up against a meter. They are
 * here rather than in power_mon.c because the answer is a property of the
 * cell and the load, which are board facts. */
#define BOARD_VBAT_MV_4       3920
#define BOARD_VBAT_MV_3       3780
#define BOARD_VBAT_MV_2       3620
#define BOARD_VBAT_MV_1       3450
#define BOARD_VBAT_MV_HYST      40
/* No cell fitted is not a flat cell. With the battery unpopulated the
 * BQ24074's BAT node is unloaded and drifts - measured passing through
 * 4.106 V, 3.940 V and 0.000 V on one board - and a 0.000 V reading down the
 * sustained-empty path deep-sleeps a device that is running on external power
 * and needs a physical power cycle to come back.
 *
 * A protected 1S pack disconnects around 2.5 V and even a deeply flat cell
 * still presents a voltage at the divider. Below this there is no cell on the
 * other end of the sense line, and the correct behaviour for a device running
 * on external power is to keep guarding: the empty-battery park exists to
 * protect a cell, and there is no cell to protect.
 *
 * It is deliberately well under BOARD_VBAT_MV_EMPTY so the two can never be
 * confused, and well over 0 so a real sense-line fault still reads as absent
 * rather than as a healthy battery. */
#define BOARD_VBAT_MV_ABSENT  2000

/* Below this, sustained, the guard stops and the panel says CHARGE ME. The
 * buck-boost carries the rail well past here, so this is the cell's limit,
 * not the rail's. */
#define BOARD_VBAT_MV_EMPTY   3300

/* ==========================================================================
 * Capability flags.
 * ========================================================================== */
#define BOARD_NAME            "pcb-rev-a2"
#define BOARD_HAS_VBAT        1
#define BOARD_HAS_CHG_STAT    1
#define BOARD_HAS_LORA        1
#define BOARD_SLIDE_HARD_CUT  1   /* the slider gates the regulator EN       */
#define BOARD_GEOMETRY        GEOM_PCB_A2

/* ---- what `U G` may pull up: derived, not listed -------------------------
 *
 * This is where the seam earns its keep. Six of the DevKit's ten free pins
 * are allocated on this board - GPIO1 the VBAT divider, GPIO2 the charger
 * STAT line, GPIO8/9/47 the radio's MISO, CS and reset - so probing a list
 * copied from there would drive a pull-up onto a live SPI bus and onto the
 * radio's reset.
 *
 * board_pins.h derives the scan set from the predicates below plus
 * BOARD_PIN_IS_RESERVED, so what gets probed is a function of what this
 * header allocates and cannot disagree with it.
 *
 * The derived set is {21, 38, 39, 40, 41, 42}. GPIO38 is unallocated on this
 * board and reserved by nothing on this part, so it is a legitimate candidate
 * here even though it is kept clear on the DevKit, where it is the v1.1
 * onboard RGB - a DevKit fact that never applied to this board, which puts
 * DIO0 on GPIO48 instead. If a board shows something on GPIO38, the fix is
 * one line in BOARD_PIN_KEEP_CLEAR below, and `I` plus a multimeter is the
 * authority, exactly as everywhere else in this file.
 */

/* Every pin this firmware drives on this board, enumerated from the PIN_
 * defines above and from nothing else - the fourteen proven ones plus the six
 * this board adds. The seam test fails when a PIN_ define in this file is
 * missing here, because a pin that is driven but unenumerated is exactly the
 * pull-up-onto-a-live-bus this derivation exists to prevent. */
#define BOARD_PIN_ALLOCATED(p)                                              \
    ((p) == PIN_I2S_BCLK || (p) == PIN_I2S_WS || (p) == PIN_I2S_SD ||       \
     (p) == PIN_I2S_SD2 || (p) == PIN_EPD_CS || (p) == PIN_EPD_DIN ||       \
     (p) == PIN_EPD_CLK || (p) == PIN_EPD_DC || (p) == PIN_EPD_RST ||       \
     (p) == PIN_EPD_BUSY || (p) == PIN_LED_WS2812 || (p) == PIN_BUZZER ||   \
     (p) == PIN_MOTOR || (p) == PIN_SNOOZE_BTN ||                           \
     (p) == PIN_VBAT_SENSE || (p) == PIN_CHG_STAT ||                        \
     (p) == PIN_LORA_MISO || (p) == PIN_LORA_CS || (p) == PIN_LORA_RST ||   \
     (p) == PIN_LORA_DIO0)

/* Pins this board keeps clear. Usable, unallocated, and still not the scan's
 * to touch: 43/44 are U0TXD/U0RXD, and on this part the ROM and second-stage
 * bootloaders print on U0TXD at every reset whatever the board does with the
 * pins, so a six-second pull-up there is not a neutral act. Nothing else is
 * listed: the DevKit header also keeps 38 and 48 clear, but that is its
 * onboard RGB LED and is not a fact about this board. */
#define BOARD_PIN_KEEP_CLEAR(p)                                             \
    ((p) == 43 || (p) == 44)                            /* UART0 console  */

/* ==========================================================================
 * The reserved set, asserted rather than described. Every pin this header
 * allocates is checked against it at compile time, so a future revision that
 * moves a signal onto a strapping pin fails the build instead of failing the
 * boot.
 *
 *   0, 3, 45, 46   strapping - level at reset selects boot mode / VDD_SPI /
 *                  ROM log verbosity
 *   19, 20         native USB D-/D+ - the console and the flash link
 *   22-25          not bonded. The ESP32-S3 has no such pins at all; they are
 *                  listed because a scan that walks the GPIO space has to
 *                  know that, where a hand-written list never had to
 *   26-32          SPI flash (QIO uses IO2/IO3, already bonded)
 *   33-37          octal PSRAM, all five. The N16R8 bonds the PSRAM die to
 *                  these pins even with CONFIG_SPIRAM=n - the module wires
 *                  them regardless, so they are not free on any build.
 * ========================================================================== */
#define BOARD_PIN_IS_RESERVED(p)                                            \
    ((p) == 0 || (p) == 3 || (p) == 45 || (p) == 46 ||   /* strapping    */  \
     (p) == 19 || (p) == 20 ||                           /* native USB   */  \
     ((p) >= 22 && (p) <= 25) ||                         /* not bonded   */  \
     ((p) >= 26 && (p) <= 32) ||                         /* SPI flash    */  \
     ((p) >= 33 && (p) <= 37))                           /* octal PSRAM  */

#define BOARD_ASSERT_FREE(p)                                                \
    _Static_assert(!BOARD_PIN_IS_RESERVED(p),                               \
                   #p " is on a reserved pin (strapping / USB / flash / "   \
                   "PSRAM) - see the table in board_pcb_a2.h")

BOARD_ASSERT_FREE(PIN_I2S_BCLK);
BOARD_ASSERT_FREE(PIN_I2S_WS);
BOARD_ASSERT_FREE(PIN_I2S_SD);
BOARD_ASSERT_FREE(PIN_I2S_SD2);
BOARD_ASSERT_FREE(PIN_EPD_CS);
BOARD_ASSERT_FREE(PIN_EPD_DIN);
BOARD_ASSERT_FREE(PIN_EPD_CLK);
BOARD_ASSERT_FREE(PIN_EPD_DC);
BOARD_ASSERT_FREE(PIN_EPD_RST);
BOARD_ASSERT_FREE(PIN_EPD_BUSY);
BOARD_ASSERT_FREE(PIN_LED_WS2812);
BOARD_ASSERT_FREE(PIN_BUZZER);
BOARD_ASSERT_FREE(PIN_MOTOR);
BOARD_ASSERT_FREE(PIN_SNOOZE_BTN);
BOARD_ASSERT_FREE(PIN_VBAT_SENSE);
BOARD_ASSERT_FREE(PIN_CHG_STAT);
BOARD_ASSERT_FREE(PIN_LORA_MISO);
BOARD_ASSERT_FREE(PIN_LORA_CS);
BOARD_ASSERT_FREE(PIN_LORA_RST);
BOARD_ASSERT_FREE(PIN_LORA_DIO0);

/* And no two signals on one pin. Checked pairwise where a collision is
 * plausible: the radio arrived last and had four pins to place. */
_Static_assert(PIN_LORA_CS != PIN_EPD_CS,
               "the radio and the panel cannot share a chip select");
_Static_assert(PIN_LORA_MISO != PIN_EPD_DIN &&
               PIN_LORA_MISO != PIN_EPD_CLK,
               "MISO must be the radio's alone");
_Static_assert(PIN_LORA_DIO0 != PIN_LED_WS2812 &&
               PIN_LORA_RST != PIN_LED_WS2812,
               "the radio must not land on the LED");
