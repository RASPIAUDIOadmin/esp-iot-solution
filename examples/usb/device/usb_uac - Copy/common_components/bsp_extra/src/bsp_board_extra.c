/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_check.h"\r\n#include "esp_err.h"

#include "audio_player.h"
#include "bsp/esp-bsp.h"
#include "bsp_board_extra.h"

static const char *TAG = "bsp_extra_board";

#define CODEC_DEFAULT_SAMPLE_RATE          (16000)
#define CODEC_DEFAULT_BIT_WIDTH            (16)
#define CODEC_DEFAULT_ADC_VOLUME           (24.0)
#define CODEC_DEFAULT_CHANNEL              (2)

static esp_codec_dev_handle_t play_dev_handle;
static esp_codec_dev_handle_t record_dev_handle;
static bool codec_handles_shared;

static file_iterator_instance_t *file_iterator;

static esp_err_t codec_close_if_open(esp_codec_dev_handle_t handle)
{
    if (!handle) {
        return ESP_OK;
    }
    esp_err_t err = esp_codec_dev_close(handle);
    return (err == ESP_ERR_INVALID_STATE) ? ESP_OK : err;
}

static esp_err_t codec_open_with_fs(esp_codec_dev_handle_t handle, const esp_codec_dev_sample_info_t *fs)
{
    if (!handle) {
        return ESP_OK;
    }
    esp_err_t err = esp_codec_dev_open(handle, fs);
    return (err == ESP_ERR_INVALID_STATE) ? ESP_OK : err;
}

static esp_err_t ensure_codec_handles(void)
{
    if (play_dev_handle && record_dev_handle) {
        codec_handles_shared = (play_dev_handle == record_dev_handle);
        return ESP_OK;
    }

    play_dev_handle = bsp_audio_codec_speaker_init();
    ESP_RETURN_ON_FALSE(play_dev_handle != NULL, ESP_FAIL, TAG, "speaker handle not initialized");

    record_dev_handle = bsp_audio_codec_microphone_init();
    ESP_RETURN_ON_FALSE(record_dev_handle != NULL, ESP_FAIL, TAG, "microphone handle not initialized");

    codec_handles_shared = (play_dev_handle == record_dev_handle);

    return ESP_OK;
}

esp_err_t bsp_extra_led_init(void)
{
    ESP_LOGW(TAG, "LED control not supported on this board");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t bsp_extra_led_set_rgb(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    (void)index;
    (void)red;
    (void)green;
    (void)blue;
    ESP_LOGW(TAG, "LED control not supported on this board");
    return ESP_ERR_NOT_SUPPORTED;
}

file_iterator_instance_t *bsp_extra_get_file_instance(void)
{
    return file_iterator;
}

static esp_err_t audio_mute_function(AUDIO_PLAYER_MUTE_SETTING setting)
{
    bsp_extra_codec_mute_set(setting == AUDIO_PLAYER_MUTE);

    if (setting == AUDIO_PLAYER_UNMUTE) {
        bsp_extra_codec_volume_set(80, NULL);
    }

    return ESP_OK;
}

static void audio_callback(audio_player_cb_ctx_t *ctx)
{
    switch (ctx->audio_event) {
    case AUDIO_PLAYER_CALLBACK_EVENT_IDLE:
        ESP_LOGD(TAG, "IDLE");
        bsp_audio_poweramp_enable(false);
        break;
    case AUDIO_PLAYER_CALLBACK_EVENT_COMPLETED_PLAYING_NEXT:
    case AUDIO_PLAYER_CALLBACK_EVENT_PLAYING:
        ESP_LOGD(TAG, "PLAYING");
        bsp_audio_poweramp_enable(true);
        break;
    default:
        ESP_LOGD(TAG, "EVENT %d", ctx->audio_event);
        break;
    }
}

esp_err_t bsp_extra_i2s_read(void *audio_buffer, size_t len, size_t *bytes_read, uint32_t timeout_ms)
{
    (void)timeout_ms;
    ESP_RETURN_ON_ERROR(ensure_codec_handles(), TAG, "codec init");
    ESP_RETURN_ON_FALSE(record_dev_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "record handle unavailable");

    esp_err_t ret = esp_codec_dev_read(record_dev_handle, audio_buffer, len);
    if (bytes_read) {
        *bytes_read = (ret == ESP_OK) ? len : 0;
    }
    return ret;
}

esp_err_t bsp_extra_i2s_write(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    (void)timeout_ms;
    ESP_RETURN_ON_ERROR(ensure_codec_handles(), TAG, "codec init");
    ESP_RETURN_ON_FALSE(play_dev_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "playback handle unavailable");

    esp_err_t ret = esp_codec_dev_write(play_dev_handle, audio_buffer, len);
    if (bytes_written) {
        *bytes_written = (ret == ESP_OK) ? len : 0;
    }
    return ret;
}

esp_err_t bsp_extra_codec_set_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    ESP_RETURN_ON_ERROR(ensure_codec_handles(), TAG, "codec init");

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = rate,
        .channel = ch,
        .bits_per_sample = bits_cfg,
    };

    ESP_RETURN_ON_ERROR(codec_close_if_open(play_dev_handle), TAG, "close speaker");
    if (record_dev_handle && !codec_handles_shared) {
        ESP_RETURN_ON_ERROR(codec_close_if_open(record_dev_handle), TAG, "close mic");
    }

    ESP_RETURN_ON_ERROR(codec_open_with_fs(play_dev_handle, &fs), TAG, "open speaker");
    if (record_dev_handle && !codec_handles_shared) {
        ESP_RETURN_ON_ERROR(codec_open_with_fs(record_dev_handle, &fs), TAG, "open mic");
    }

    if (record_dev_handle) {
        ESP_RETURN_ON_ERROR(esp_codec_dev_set_in_gain(record_dev_handle, CODEC_DEFAULT_ADC_VOLUME), TAG, "mic gain");
        esp_err_t route_ret = bsp_audio_codec_configure_inputs();
        if (route_ret != ESP_OK) {
            ESP_LOGW(TAG, "Custom mic routing failed: %s", esp_err_to_name(route_ret));
        }
    }

    return ESP_OK;
}

esp_err_t bsp_extra_codec_volume_set(int volume, int *volume_set)
{
    ESP_RETURN_ON_ERROR(ensure_codec_handles(), TAG, "codec init");
    if (volume_set) {
        *volume_set = volume;
    }
    return esp_codec_dev_set_out_vol(play_dev_handle, volume);
}

esp_err_t bsp_extra_codec_mute_set(bool enable)
{
    ESP_RETURN_ON_ERROR(ensure_codec_handles(), TAG, "codec init");
    return esp_codec_dev_set_out_mute(play_dev_handle, enable);
}

esp_err_t bsp_extra_codec_dev_stop(void)
{
    ESP_RETURN_ON_ERROR(ensure_codec_handles(), TAG, "codec init");

    ESP_RETURN_ON_ERROR(codec_close_if_open(play_dev_handle), TAG, "close speaker");
    if (record_dev_handle && !codec_handles_shared) {
        ESP_RETURN_ON_ERROR(codec_close_if_open(record_dev_handle), TAG, "close mic");
    }
    return ESP_OK;
}

esp_err_t bsp_extra_codec_dev_resume(void)
{
    return bsp_extra_codec_set_fs(CODEC_DEFAULT_SAMPLE_RATE, CODEC_DEFAULT_BIT_WIDTH, CODEC_DEFAULT_CHANNEL);
}

esp_err_t bsp_extra_codec_init()
{
    ESP_RETURN_ON_ERROR(ensure_codec_handles(), TAG, "codec init");
    return bsp_extra_codec_set_fs(CODEC_DEFAULT_SAMPLE_RATE, CODEC_DEFAULT_BIT_WIDTH, CODEC_DEFAULT_CHANNEL);
}

esp_err_t bsp_extra_player_init(char *path)
{
    if (path) {
        file_iterator = file_iterator_new(path);
        ESP_RETURN_ON_FALSE(file_iterator != NULL, ESP_FAIL, TAG, "file iterator create failed");
    }

    audio_player_config_t config = {
        .mute_fn = audio_mute_function,
        .write_fn = bsp_extra_i2s_write,
        .clk_set_fn = bsp_extra_codec_set_fs,
        .priority = 5,
    };
    ESP_RETURN_ON_ERROR(audio_player_new(config), TAG, "audio player new");
    audio_player_callback_register(audio_callback, NULL);

    return ESP_OK;
}



