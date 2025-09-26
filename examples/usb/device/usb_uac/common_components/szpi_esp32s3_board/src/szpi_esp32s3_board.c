/*
 * SPDX-FileCopyrightText: 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "esp_codec_dev_defaults.h"
#include "audio_codec_ctrl_if.h"
#include "audio_codec_gpio_if.h"
#include "esp_codec_dev_types.h"
#include "es8311_codec.h"
#include "es7210_adc.h"

#include "bsp/szpi_esp32s3_board.h"

static const char *TAG = "bsp_szpi";

#define SZPI_ES7210_ADDR    (0x82)

#define PCA9557_REG_INPUT0   (0x00)
#define PCA9557_REG_OUTPUT0  (0x01)
#define PCA9557_REG_CONFIG0  (0x03)
#define PCA9557_IO_TIMEOUT_TICKS  pdMS_TO_TICKS(100)

static bool pca9557_initialized;
static uint8_t pca9557_output_state;

static esp_err_t ensure_i2c_bus(void);
static esp_err_t ensure_pca9557(void);
static esp_err_t pca9557_set_pa(bool enable);

static bool i2c_initialized;
static const audio_codec_data_if_t *i2s_data_if;
static i2s_chan_handle_t i2s_tx_chan;
static i2s_chan_handle_t i2s_rx_chan;
static bool pa_gpio_configured;

static esp_codec_dev_handle_t speaker_dev_handle;
static esp_codec_dev_handle_t mic_dev_handle;

static const audio_codec_ctrl_if_t *es8311_ctrl_if;
static const audio_codec_ctrl_if_t *es7210_ctrl_if;

static esp_err_t ensure_pa_gpio(void)
{
#if (BSP_POWER_AMP_GPIO_NUM < 0)
    ESP_RETURN_ON_ERROR(ensure_pca9557(), TAG, "init PA expander");
    pa_gpio_configured = true;
    return ESP_OK;
#else
    if (pa_gpio_configured) {
        return ESP_OK;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BSP_POWER_AMP_GPIO_NUM,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "configure PA GPIO");
    ESP_RETURN_ON_ERROR(gpio_set_level(BSP_POWER_AMP_IO, 0), TAG, "default PA low");
    pa_gpio_configured = true;
    return ESP_OK;
#endif
}

static esp_err_t ensure_i2c_bus(void)
{
    if (i2c_initialized) {
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .clk_flags = 0,
    };
    conf.master.clk_speed = 400000;

    ESP_RETURN_ON_ERROR(i2c_param_config(BSP_I2C_NUM, &conf), TAG, "config I2C");
    ESP_RETURN_ON_ERROR(i2c_driver_install(BSP_I2C_NUM, conf.mode, 0, 0, 0), TAG, "install I2C driver");
    i2c_initialized = true;
    return ESP_OK;
}

static inline uint8_t pca9557_addr_7bit(void)
{
    return (BSP_PCA9557_ADDR > 0x7F) ? (BSP_PCA9557_ADDR >> 1) : BSP_PCA9557_ADDR;
}

static esp_err_t pca9557_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t payload[2] = {reg, value};
    return i2c_master_write_to_device(BSP_I2C_NUM, pca9557_addr_7bit(), payload, sizeof(payload), PCA9557_IO_TIMEOUT_TICKS);
}

static esp_err_t pca9557_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_master_write_read_device(BSP_I2C_NUM, pca9557_addr_7bit(), &reg, sizeof(reg), value, 1, PCA9557_IO_TIMEOUT_TICKS);
}

static esp_err_t ensure_pca9557(void)
{
    ESP_RETURN_ON_ERROR(ensure_i2c_bus(), TAG, "init I2C bus for PCA9557");

    if (pca9557_initialized) {
        return ESP_OK;
    }

    uint8_t config0 = 0xFF;
    if (pca9557_read_reg(PCA9557_REG_CONFIG0, &config0) != ESP_OK) {
        config0 = 0xFF;
    }
    config0 &= (uint8_t)~(1U << BSP_PCA9557_PA_BIT);
    ESP_RETURN_ON_ERROR(pca9557_write_reg(PCA9557_REG_CONFIG0, config0), TAG, "set PCA9557 config");

    if (pca9557_read_reg(PCA9557_REG_OUTPUT0, &pca9557_output_state) != ESP_OK) {
        pca9557_output_state = 0;
    }
    pca9557_output_state &= (uint8_t)~(1U << BSP_PCA9557_PA_BIT);
    ESP_RETURN_ON_ERROR(pca9557_write_reg(PCA9557_REG_OUTPUT0, pca9557_output_state), TAG, "default PA off");

    pca9557_initialized = true;
    return ESP_OK;
}

static esp_err_t pca9557_set_pa(bool enable)
{
    ESP_RETURN_ON_ERROR(ensure_pca9557(), TAG, "ensure PCA9557 ready");

    if (enable) {
        pca9557_output_state |= (uint8_t)(1U << BSP_PCA9557_PA_BIT);
    } else {
        pca9557_output_state &= (uint8_t)~(1U << BSP_PCA9557_PA_BIT);
    }
    ESP_RETURN_ON_ERROR(pca9557_write_reg(PCA9557_REG_OUTPUT0, pca9557_output_state), TAG, "update PA state");
    return ESP_OK;
}

esp_err_t bsp_audio_poweramp_enable(bool enable)
{
    ESP_RETURN_ON_ERROR(ensure_pa_gpio(), TAG, "init PA control");

    if (BSP_POWER_AMP_GPIO_NUM < 0) {
        return pca9557_set_pa(enable);
    }

    return gpio_set_level(BSP_POWER_AMP_IO, enable ? 1 : 0);
}

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
{
    ESP_RETURN_ON_ERROR(ensure_pa_gpio(), TAG, "init PA pin");

    if (i2s_data_if) {
        return ESP_OK;
    }

    const i2s_std_config_t default_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = BSP_I2S_MCLK,
            .bclk = BSP_I2S_BCLK,
            .ws = BSP_I2S_LRCK,
            .dout = BSP_I2S_DOUT,
            .din = BSP_I2S_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    const i2s_std_config_t *cfg = i2s_config ? i2s_config : &default_std_cfg;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;

    esp_err_t ret = i2s_new_channel(&chan_cfg, &i2s_tx_chan, &i2s_rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate I2S channels: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_init_std_mode(i2s_tx_chan, cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init TX channel: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    ret = i2s_channel_enable(i2s_tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable TX channel: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = i2s_channel_init_std_mode(i2s_rx_chan, cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init RX channel: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    ret = i2s_channel_enable(i2s_rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RX channel: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    audio_codec_i2s_cfg_t codec_i2s_cfg = {
        .port = BSP_I2S_NUM,
        .tx_handle = i2s_tx_chan,
        .rx_handle = i2s_rx_chan,
    };

    i2s_data_if = audio_codec_new_i2s_data(&codec_i2s_cfg);
    if (i2s_data_if == NULL) {
        ESP_LOGE(TAG, "Failed to create codec I2S interface");
        ret = ESP_FAIL;
        goto cleanup;
    }

    esp_err_t pa_ret = bsp_audio_poweramp_enable(false);
    if (pa_ret != ESP_OK) {
        ret = pa_ret;
        goto cleanup;
    }

    ESP_LOGI(TAG, "I2S configured: MCLK=%d, BCLK=%d, LRCK=%d, DOUT=%d, DIN=%d",
             BSP_I2S_MCLK, BSP_I2S_BCLK, BSP_I2S_LRCK, BSP_I2S_DOUT, BSP_I2S_DIN);

    return ESP_OK;

cleanup:
    if (i2s_tx_chan) {
        i2s_del_channel(i2s_tx_chan);
        i2s_tx_chan = NULL;
    }
    if (i2s_rx_chan) {
        i2s_del_channel(i2s_rx_chan);
        i2s_rx_chan = NULL;
    }
    i2s_data_if = NULL;
    return ret;
}

static esp_codec_dev_handle_t create_speaker_codec(void)
{
    if (speaker_dev_handle) {
        return speaker_dev_handle;
    }

    esp_err_t err = ensure_i2c_bus();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init I2C failed: %s", esp_err_to_name(err));
        return NULL;
    }
    err = bsp_audio_init(NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init I2S failed: %s", esp_err_to_name(err));
        return NULL;
    }
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(gpio_if != NULL, NULL, TAG, "create GPIO IF");

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
    };
    es8311_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(es8311_ctrl_if != NULL, NULL, TAG, "create ES8311 ctrl IF");

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0f,
        .codec_dac_voltage = 3.3f,
    };

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = es8311_ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .digital_mic = false,
        .invert_mclk = false,
        .invert_sclk = false,
        .hw_gain = gain,
    };

    const audio_codec_if_t *codec_if = es8311_codec_new(&es8311_cfg);
    ESP_RETURN_ON_FALSE(codec_if != NULL, NULL, TAG, "create ES8311 codec");

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = codec_if,
        .data_if = i2s_data_if,
    };

    speaker_dev_handle = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(speaker_dev_handle != NULL, NULL, TAG, "create speaker device");

    return speaker_dev_handle;
}

static esp_codec_dev_handle_t create_microphone_codec(void)
{
    if (mic_dev_handle) {
        return mic_dev_handle;
    }

    esp_err_t err = ensure_i2c_bus();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init I2C failed: %s", esp_err_to_name(err));
        return NULL;
    }
    err = bsp_audio_init(NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init I2S failed: %s", esp_err_to_name(err));
        return NULL;
    }
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = SZPI_ES7210_ADDR,
    };
    es7210_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(es7210_ctrl_if != NULL, NULL, TAG, "create ES7210 ctrl IF");

    es7210_codec_cfg_t es7210_cfg = {
        .ctrl_if = es7210_ctrl_if,
        .master_mode = false,
        .mic_selected = ES7120_SEL_MIC1 | ES7120_SEL_MIC3, /* MIC1: user mic, MIC3: speaker reference loopback */
        .mclk_src = ES7210_MCLK_FROM_PAD,
        .mclk_div = 256,
    };

    const audio_codec_if_t *codec_if = es7210_codec_new(&es7210_cfg);
    ESP_RETURN_ON_FALSE(codec_if != NULL, NULL, TAG, "create ES7210 codec");

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = codec_if,
        .data_if = i2s_data_if,
    };

    mic_dev_handle = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(mic_dev_handle != NULL, NULL, TAG, "create microphone device");

    return mic_dev_handle;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    return create_speaker_codec();
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    return create_microphone_codec();
}

esp_err_t bsp_audio_codec_configure_inputs(void)
{
    if (es7210_ctrl_if == NULL || es7210_ctrl_if->is_open == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!es7210_ctrl_if->is_open(es7210_ctrl_if)) {
        return ESP_OK;
    }

    /* Default routing from es7210 driver already enables selected mics. */
    ESP_LOGI(TAG, "ES7210 inputs configured (MIC1 + MIC2)");
    return ESP_OK;
}




