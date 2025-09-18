/*
 * SPDX-FileCopyrightText: 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
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

#include "bsp/raspiaudio_muse_board.h"

static const char *TAG = "bsp_muse_radio";

#define ES8388_ADCPOWER        0x03
#define ES8388_ADCCONTROL1     0x09
#define ES8388_ADCCONTROL2     0x0A
#define ES8388_ADCCONTROL3     0x0B
#define ES8388_ADCCONTROL4     0x0C
#define ES8388_ADCCONTROL5     0x0D
#define ES8388_ADCCONTROL8     0x10
#define ES8388_ADCCONTROL9     0x11

static const audio_codec_ctrl_if_t *codec_ctrl_if;

static esp_err_t es8388_write_reg_u8(const audio_codec_ctrl_if_t *ctrl_if, uint8_t reg, uint8_t value)
{
    if (ctrl_if == NULL || ctrl_if->write_reg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    int ret = ctrl_if->write_reg(ctrl_if, reg, 1, &value, 1);
    if (ret != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "ES8388 write 0x%02X failed (%d)", reg, ret);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t muse_configure_es8388_inputs(void)
{
    if (codec_ctrl_if == NULL || codec_ctrl_if->is_open == NULL || !codec_ctrl_if->is_open(codec_ctrl_if)) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(es8388_write_reg_u8(codec_ctrl_if, ES8388_ADCPOWER, 0xFF), TAG, "power down ADC");
    ESP_RETURN_ON_ERROR(es8388_write_reg_u8(codec_ctrl_if, ES8388_ADCCONTROL1, 0x88), TAG, "set PGA gain");
    ESP_RETURN_ON_ERROR(es8388_write_reg_u8(codec_ctrl_if, ES8388_ADCCONTROL2, 0xFC), TAG, "route differential inputs");
    ESP_RETURN_ON_ERROR(es8388_write_reg_u8(codec_ctrl_if, ES8388_ADCCONTROL3, 0x02), TAG, "adc control3");
    ESP_RETURN_ON_ERROR(es8388_write_reg_u8(codec_ctrl_if, ES8388_ADCCONTROL4, 0x0C), TAG, "adc data format");
    ESP_RETURN_ON_ERROR(es8388_write_reg_u8(codec_ctrl_if, ES8388_ADCCONTROL5, 0x02), TAG, "adc ratio");
    ESP_RETURN_ON_ERROR(es8388_write_reg_u8(codec_ctrl_if, ES8388_ADCCONTROL8, 0x00), TAG, "adc volume L");
    ESP_RETURN_ON_ERROR(es8388_write_reg_u8(codec_ctrl_if, ES8388_ADCCONTROL9, 0x00), TAG, "adc volume R");
    ESP_RETURN_ON_ERROR(es8388_write_reg_u8(codec_ctrl_if, ES8388_ADCPOWER, 0x09), TAG, "power up ADC");

    ESP_LOGI(TAG, "ES8388 ADC routed for differential microphones");
    return ESP_OK;
}

static bool i2c_initialized;

static const audio_codec_data_if_t *i2s_data_if;
static i2s_chan_handle_t i2s_tx_chan;
static i2s_chan_handle_t i2s_rx_chan;

static bool pa_gpio_configured;
static esp_codec_dev_handle_t codec_handle;

static esp_err_t ensure_pa_gpio(void)
{
    if (pa_gpio_configured) {
        return ESP_OK;
    }

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BSP_POWER_AMP_IO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "configure PA pin");
    ESP_RETURN_ON_ERROR(gpio_set_level(BSP_POWER_AMP_IO, 0), TAG, "default PA low");
    pa_gpio_configured = true;
    return ESP_OK;
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

esp_err_t bsp_audio_poweramp_enable(bool enable)
{
    ESP_RETURN_ON_ERROR(ensure_pa_gpio(), TAG, "init PA pin");
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

static esp_codec_dev_handle_t create_codec_handle(void)
{
    if (codec_handle) {
        return codec_handle;
    }

    if (ensure_i2c_bus() != ESP_OK) {
        return NULL;
    }
    if (bsp_audio_init(NULL) != ESP_OK) {
        return NULL;
    }

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    if (gpio_if == NULL) {
        ESP_LOGE(TAG, "Failed to create codec GPIO interface");
        return NULL;
    }

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES8388_CODEC_DEFAULT_ADDR,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (ctrl_if == NULL) {
        ESP_LOGE(TAG, "Failed to create codec control interface");
        return NULL;
    }
    codec_ctrl_if = ctrl_if;
    if (codec_ctrl_if->is_open == NULL || !codec_ctrl_if->is_open(codec_ctrl_if)) {
        ESP_LOGE(TAG, "Codec control interface not opened");
        return NULL;
    }

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0f,
        .codec_dac_voltage = 3.3f,
    };

    es8388_codec_cfg_t codec_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .master_mode = false,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,
        .hw_gain = gain,
    };

    const audio_codec_if_t *codec_if = es8388_codec_new(&codec_cfg);
    if (codec_if == NULL) {
        ESP_LOGE(TAG, "Failed to create ES8388 codec interface");
        return NULL;
    }

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = codec_if,
        .data_if = i2s_data_if,
    };

    codec_handle = esp_codec_dev_new(&dev_cfg);
    if (codec_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create codec device handle");
        return NULL;
    }

    esp_err_t cfg_ret = bsp_audio_codec_configure_inputs();
    if (cfg_ret != ESP_OK && cfg_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "ES8388 input routing update failed: %s", esp_err_to_name(cfg_ret));
    }

    return codec_handle;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    return create_codec_handle();
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    return create_codec_handle();
}

esp_err_t bsp_audio_codec_configure_inputs(void)
{
    esp_err_t ret = muse_configure_es8388_inputs();
    if (ret == ESP_ERR_INVALID_STATE) {
        return ESP_OK;
    }
    return ret;
}





