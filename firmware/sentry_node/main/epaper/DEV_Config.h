/*
 * DEV_Config.h - ESP-IDF HAL shim for the vendored Waveshare 2.13" V4 driver.
 *
 * The upstream driver needs exactly four primitives (DEV_Digital_Write,
 * DEV_Digital_Read, DEV_Delay_ms, DEV_SPI_WriteByte) plus four pin macros and
 * the UBYTE/UWORD/UDOUBLE typedefs. Everything else in Waveshare's Raspberry Pi
 * DEV layer (wiringPi, bcm2835, sysfs GPIO, /dev/spidev) is irrelevant here and
 * deliberately not carried over.
 *
 * Pins come from board_pins.h so there is exactly ONE pin map in the project.
 */
#pragma once

#include <stdint.h>

#include "board_pins.h"

typedef uint8_t  UBYTE;
typedef uint16_t UWORD;
typedef uint32_t UDOUBLE;

/* The driver refers to pins by these names. */
#define EPD_RST_PIN   PIN_EPD_RST
#define EPD_DC_PIN    PIN_EPD_DC
#define EPD_CS_PIN    PIN_EPD_CS
#define EPD_BUSY_PIN  PIN_EPD_BUSY

/* Bound for the patched ReadBusy(). A full refresh on this panel is ~2 s and a
 * partial ~0.3 s, so 10 s is "the panel is not answering", not "be patient". */
#define EPD_BUSY_TIMEOUT_MS 10000

/* Set by the patched ReadBusy() when the bound is hit. */
extern volatile int EPD_busy_timeout;

/* ---- the four primitives the driver actually calls ---------------------- */
void DEV_Digital_Write(UWORD pin, UBYTE value);
UBYTE DEV_Digital_Read(UWORD pin);
void DEV_Delay_ms(UDOUBLE ms);
void DEV_SPI_WriteByte(UBYTE value);

/* ---- lifecycle, called by the ESP-IDF wrapper only ---------------------- */
int  DEV_Module_Init(void);      /* GPIO + SPI2 bring-up. 0 = OK. */
void DEV_Module_Exit(void);
