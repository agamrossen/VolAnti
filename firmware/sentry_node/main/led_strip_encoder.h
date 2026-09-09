/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "driver/rmt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Type of led strip encoder configuration
 */
typedef struct {
    uint32_t resolution; /*!< Encoder resolution, in Hz */
    /* LATCH/RESET length in MICROSECONDS. 0 takes LED_RESET_US from
     * ui_config.h, which is the shipped value; the hardcoded 50 us that
     * Espressif's example carried is gone.
     *
     * Why this is a parameter now. 50 us satisfies the original WS2812/WS2812B
     * datasheet, and does NOT satisfy the WS2812B-V5 silicon that has been
     * shipping since ~2022, which wants an order of magnitude more. A part
     * that never latches never lights, and it does it silently: every call in
     * the transmit path still returns ESP_OK, because a WS2812 has no back
     * channel and the driver cannot know. Making it a parameter is what lets
     * the answer be MEASURED on the bench rather than guessed at. */
    uint32_t reset_us;
} led_strip_encoder_config_t;

/**
 * @brief Create RMT encoder for encoding LED strip pixels into RMT symbols
 *
 * @param[in] config Encoder configuration
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM out of memory when creating led strip encoder
 *      - ESP_OK if creating encoder successfully
 */
esp_err_t rmt_new_led_strip_encoder(const led_strip_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

#ifdef __cplusplus
}
#endif
