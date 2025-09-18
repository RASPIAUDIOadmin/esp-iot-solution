/*
 * SPDX-FileCopyrightText: 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_codec_dev.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C configuration */
#define BSP_I2C_NUM          (0)
#define BSP_I2C_SCL          (GPIO_NUM_11)
#define BSP_I2C_SDA          (GPIO_NUM_18)

/* I2S configuration */
#define BSP_I2S_NUM          (0)
#define BSP_I2S_MCLK         (GPIO_NUM_0)
#define BSP_I2S_BCLK         (GPIO_NUM_5)
#define BSP_I2S_LRCK         (GPIO_NUM_16)
#define BSP_I2S_DOUT         (GPIO_NUM_17)
#define BSP_I2S_DIN          (GPIO_NUM_4)

/* Power amplifier control */
#define BSP_POWER_AMP_IO     (GPIO_NUM_46)

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config);

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void);

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void);

esp_err_t bsp_audio_poweramp_enable(bool enable);

esp_err_t bsp_audio_codec_configure_inputs(void);
#ifdef __cplusplus
}
#endif


