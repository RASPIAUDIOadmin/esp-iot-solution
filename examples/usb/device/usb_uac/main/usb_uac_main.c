/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "bsp/esp-bsp.h"
#include "bsp_board_extra.h"
#include "usb_device_uac.h"

static const char *TAG = "usb_uac_main";

static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *arg)
{
    size_t bytes_written = 0;
    bsp_extra_i2s_write(buf, len, &bytes_written, 0);
    return ESP_OK;
}

static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    if (bsp_extra_i2s_read(buf, len, bytes_read, 0) != ESP_OK) {
        ESP_LOGE(TAG, "i2s read failed");
    }
    return ESP_OK;
}

static void uac_device_set_mute_cb(uint32_t mute, void *arg)
{
    ESP_LOGI(TAG, "uac_device_set_mute_cb: %"PRIu32"", mute);
    bsp_extra_codec_mute_set(mute);
}

static void play_startup_beep(void)
{
    const uint32_t sample_rate = CONFIG_UAC_SAMPLE_RATE;
    const uint32_t channels = CONFIG_UAC_SPEAKER_CHANNEL_NUM;
    const uint32_t duration_ms = 2000;
    const float frequency_hz = 1000.0f;
    const float amplitude = 0.25f; /* 25% of full scale to avoid clipping */

    if (sample_rate == 0 || channels == 0) {
        return;
    }

    bsp_audio_poweramp_enable(true);
    bsp_extra_codec_mute_set(false);
    bsp_extra_codec_volume_set(90, NULL);
    vTaskDelay(pdMS_TO_TICKS(10));

    const size_t total_frames = ((size_t)sample_rate * duration_ms) / 1000;
    if (total_frames == 0) {
        return;
    }

    const size_t total_samples = total_frames * channels;
    int16_t *buffer = malloc(total_samples * sizeof(int16_t));
    if (!buffer) {
        ESP_LOGW(TAG, "startup beep allocation failed");
        return;
    }

    const float two_pi = 6.283185307179586476925286766559f;
    for (size_t i = 0; i < total_frames; ++i) {
        float sample = sinf(two_pi * frequency_hz * ((float)i / (float)sample_rate));
        int16_t value = (int16_t)(sample * amplitude * (float)INT16_MAX);
        for (uint32_t ch = 0; ch < channels; ++ch) {
            buffer[i * channels + ch] = value;
        }
    }

    const size_t frame_bytes = channels * sizeof(int16_t);
    const size_t total_bytes = total_samples * sizeof(int16_t);
    size_t offset = 0;
    uint8_t *raw = (uint8_t *)buffer;

    while (offset < total_bytes) {
        size_t chunk = total_bytes - offset;
        if (chunk > 4096) {
            size_t frames = 4096 / frame_bytes;
            chunk = frames ? frames * frame_bytes : frame_bytes;
        }

        size_t written = 0;
        if (bsp_extra_i2s_write(raw + offset, chunk, &written, 200) != ESP_OK || written == 0) {
            ESP_LOGW(TAG, "startup beep truncated (written=%zu)", written);
            break;
        }
        offset += written;
    }

    free(buffer);
    ESP_LOGI(TAG, "startup beep played");
}
static void uac_device_set_volume_cb(uint32_t volume, void *arg)
{
    ESP_LOGI(TAG, "uac_device_set_volume_cb: %"PRIu32"", volume);
    bsp_extra_codec_volume_set(volume, NULL);
}

void app_main(void)
{
    esp_log_level_set("usbd_uac", ESP_LOG_DEBUG);
    esp_log_level_set("tusb", ESP_LOG_DEBUG);
    esp_log_level_set("bt_ampl", ESP_LOG_VERBOSE);
    bsp_extra_codec_init();
    bsp_extra_codec_set_fs(CONFIG_UAC_SAMPLE_RATE, 16, CONFIG_UAC_SPEAKER_CHANNEL_NUM);
    play_startup_beep();

    uac_device_config_t config = {
        .output_cb = uac_device_output_cb,
        .input_cb = uac_device_input_cb,
        .set_mute_cb = uac_device_set_mute_cb,
        .set_volume_cb = uac_device_set_volume_cb,
        .cb_ctx = NULL,
    };

    uac_device_init(&config);
}



