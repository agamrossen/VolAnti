/*
 * DEV_Config.c - the ESP-IDF side of the HAL shim.
 *
 * SPI2_HOST (FSPI) at 10 MHz, mode 0. CS is driven MANUALLY as a plain GPIO
 * rather than handed to the SPI driver, because the vendored driver toggles it
 * around multi-byte sequences itself and expects that ownership.
 */
#include "DEV_Config.h"

#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define EPD_SPI_HOST   SPI2_HOST
#define EPD_SPI_HZ     (10 * 1000 * 1000)

static spi_device_handle_t s_spi;
static int s_ready;
/* Did WE bring SPI2 up? Only the owner may tear it down. */
static int s_owns_bus;

void DEV_Digital_Write(UWORD pin, UBYTE value)
{
    gpio_set_level((gpio_num_t)pin, value ? 1 : 0);
}

UBYTE DEV_Digital_Read(UWORD pin)
{
    return (UBYTE)gpio_get_level((gpio_num_t)pin);
}

void DEV_Delay_ms(UDOUBLE ms)
{
    if (ms == 0) {
        return;
    }
    /* Always at least one tick, so a 1 ms request cannot become a busy spin. */
    TickType_t t = pdMS_TO_TICKS(ms);
    vTaskDelay(t ? t : 1);
}

void DEV_SPI_WriteByte(UBYTE value)
{
    if (!s_ready) {
        return;
    }
    spi_transaction_t t = {
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
    };
    t.tx_data[0] = value;
    /* Polling transmit: this runs on the dedicated e-paper task, never on the
     * frame loop, so blocking here costs nothing the detector can feel. */
    (void)spi_device_polling_transmit(s_spi, &t);
}

int DEV_Module_Init(void)
{
    if (s_ready) {
        return 0;
    }

    gpio_config_t out = {
        .pin_bit_mask = (1ULL << EPD_RST_PIN) | (1ULL << EPD_DC_PIN) |
                        (1ULL << EPD_CS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&out) != ESP_OK) {
        return -1;
    }
    gpio_config_t in = {
        .pin_bit_mask = 1ULL << EPD_BUSY_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&in) != ESP_OK) {
        return -1;
    }
    gpio_set_level(EPD_CS_PIN, 1);

    /* The panel does not own SPI2 on every board.
     *
     * On the PCB the SX1276 shares SCK and MOSI with this panel and owns MISO
     * alone. Both drivers call spi_bus_initialize() on SPI2_HOST, so exactly
     * one of them is second, and the second one gets ESP_ERR_INVALID_STATE.
     * lora_link.c has always tolerated that; this file did not - it returned
     * -1, which epaper.c reads as a dead panel and latches FAULTED.
     *
     * The standalone guard starts the radio BEFORE it draws anything, so on
     * that board the panel is faulted on every armed run while working
     * perfectly from the console, where no radio is started. A board with no
     * radio never shows it at all.
     *
     * MISO IS DECLARED HERE TOO on a board with a radio. The bus belongs to
     * whichever driver wins the race, and its config is what gets applied -
     * so if the panel won while declaring miso_io_num = -1, the radio's MISO
     * would never be routed and the SX1276 would answer nothing. Both configs
     * must describe the same three pins for the race to be harmless. */
    spi_bus_config_t bus = {
        .mosi_io_num = PIN_EPD_DIN,
#if BOARD_HAS_LORA
        .miso_io_num = PIN_LORA_MISO,
#else
        .miso_io_num = -1,
#endif
        .sclk_io_num = PIN_EPD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    const esp_err_t be = spi_bus_initialize(EPD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (be == ESP_OK) {
        s_owns_bus = 1;
    } else if (be != ESP_ERR_INVALID_STATE) {
        return -1;
    }
    spi_device_interface_config_t dev = {
        .clock_speed_hz = EPD_SPI_HZ,
        .mode = 0,
        .spics_io_num = -1,          /* CS driven by the driver, not the bus */
        .queue_size = 4,
    };
    if (spi_bus_add_device(EPD_SPI_HOST, &dev, &s_spi) != ESP_OK) {
        if (s_owns_bus) {
            spi_bus_free(EPD_SPI_HOST);
            s_owns_bus = 0;
        }
        return -1;
    }
    s_ready = 1;
    return 0;
}

void DEV_Module_Exit(void)
{
    if (!s_ready) {
        return;
    }
    spi_bus_remove_device(s_spi);
    /* Only if we brought it up. Freeing a bus the radio is still on would take
     * the peer beacon down with the panel. */
    if (s_owns_bus) {
        spi_bus_free(EPD_SPI_HOST);
        s_owns_bus = 0;
    }
    s_spi = NULL;
    s_ready = 0;
}
