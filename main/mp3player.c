#include "mp3player.h"
#include "audio_player.h"
#include "usb/usb_host.h"
#include "usb/uac_host.h"
#include "file_iterator.h"
#include <bsp/esp-bsp.h>
#include "esp_log.h"

#define TAG "mp3player"

#define USB_HOST_TASK_PRIORITY 5
#define UAC_TASK_PRIORITY 5
#define USER_TASK_PRIORITY 2
#define DEFAULT_VOLUME 60

static audio_player_t audio_player_type = AUDIO_PLAYER_I2S;
static QueueHandle_t s_event_queue = NULL;
static uac_host_device_handle_t s_audio_player_handle = NULL;
static audio_player_config_t player_config = {};
static FILE *s_fp = NULL;
static void uac_device_callback(uac_host_device_handle_t uac_device_handle, const uac_host_device_event_t event, void *arg);
static file_iterator_instance_t *file_iterator = NULL;



uint8_t get_sys_volume()
{
    return DEFAULT_VOLUME;
}

/**
 * @brief event group
 *
 * APP_EVENT            - General control event
 * UAC_DRIVER_EVENT     - UAC Host Driver event, such as device connection
 * UAC_DEVICE_EVENT     - UAC Host Device event, such as rx/tx completion, device disconnection
 */
typedef enum
{
    APP_EVENT = 0,
    UAC_DRIVER_EVENT,
    UAC_DEVICE_EVENT,
} event_group_t;

typedef struct
{
    event_group_t event_group;
    union
    {
        struct
        {
            uint8_t addr;
            uint8_t iface_num;
            uac_host_driver_event_t event;
            void *arg;
        } driver_evt;
        struct
        {
            uac_host_device_handle_t handle;
            uac_host_driver_event_t event;
            void *arg;
        } device_evt;
    };
} s_event_queue_t;

static esp_err_t _audio_player_mute_fn(AUDIO_PLAYER_MUTE_SETTING setting)
{
    esp_err_t ret = ESP_OK;
    if (audio_player_type == AUDIO_PLAYER_I2S)
    {
        // Volume saved when muting and restored when unmuting. Restoring volume is necessary
        // as es8311_set_voice_mute(true) results in voice volume (REG32) being set to zero.
        uint8_t volume = get_sys_volume();
         bsp_codec_config_t* codec_handle = bsp_get_codec_handle();
        codec_handle->set_mute(setting == AUDIO_PLAYER_MUTE ? true : false);
        // bsp_codec_mute_set(setting == AUDIO_PLAYER_MUTE ? true : false);

        // restore the voice volume upon unmuting
        if (setting == AUDIO_PLAYER_UNMUTE)
        {
            codec_handle->set_volume(volume);
            // bsp_codec_volume_set(volume, NULL);
        }
        ret = ESP_OK;
    }
    else
    {
        if (s_audio_player_handle == NULL)
        {
            return ESP_ERR_INVALID_STATE;
        }
        ESP_LOGI(TAG, "mute setting: %s", setting == 0 ? "mute" : "unmute");

        ret = uac_host_device_set_mute(s_audio_player_handle, (setting == 0 ? true : false));
    }
    return ret;
}

static esp_err_t _audio_player_write_fn(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    if (audio_player_type == AUDIO_PLAYER_I2S)
    {
        bsp_codec_config_t* codec_handle = bsp_get_codec_handle();
        ret = codec_handle->i2s_write(audio_buffer, len, bytes_written, timeout_ms);
    }
    else
    {
        if (s_audio_player_handle == NULL)
        {
            return ESP_ERR_INVALID_STATE;
        }
        *bytes_written = 0;
        esp_err_t ret = uac_host_device_write(s_audio_player_handle, (uint8_t*)audio_buffer, len, timeout_ms);
        if (ret == ESP_OK) {
            *bytes_written = len;
        }
    }

    return ret;
}

static esp_err_t _audio_player_std_clock(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    if (audio_player_type == AUDIO_PLAYER_I2S)
    {
        bsp_codec_config_t* codec_handle = bsp_get_codec_handle();
        codec_handle->i2s_reconfig_clk_fn(rate, bits_cfg, ch);
        // ret = bsp_codec_set_fs(rate, bits_cfg, ch);
    }
    else
    {
        if (s_audio_player_handle == NULL)
        {
            return ESP_ERR_INVALID_STATE;
        }
        // ESP_LOGI(TAG, "Re-config: speaker rate %"PRIu32", bits %"PRIu32", mode %s", rate, bits_cfg, ch == 1 ? "MONO" : (ch == 2 ? "STEREO" : "INVALID"));
        ESP_ERROR_CHECK(uac_host_device_stop(s_audio_player_handle));
        const uac_host_stream_config_t stm_config = {
            .channels = ch,
            .bit_resolution = bits_cfg,
            .sample_freq = rate,
            .flags = 0,
        };
        ret = uac_host_device_start(s_audio_player_handle, &stm_config);
    }
    return ret;
}

static void _audio_player_callback(audio_player_cb_ctx_t *ctx)
{
    ESP_LOGI(TAG, "ctx->audio_event = %d", ctx->audio_event);
    switch (ctx->audio_event)
    {
    case AUDIO_PLAYER_CALLBACK_EVENT_IDLE:
    {
        ESP_LOGI(TAG, "AUDIO_PLAYER_REQUEST_IDLE");
        if (s_audio_player_handle == NULL)
        {
            break;
        }
        ESP_ERROR_CHECK(uac_host_device_suspend(s_audio_player_handle));
        ESP_LOGI(TAG, "Play in loop");
        // s_fp = fopen(SPIFFS_BASE MP3_FILE_NAME, "rb");
        // if (s_fp)
        // {
        //     ESP_LOGI(TAG, "Playing '%s'", MP3_FILE_NAME);
        //     audio_player_play(s_fp);
        // }
        // else
        // {
        //     ESP_LOGE(TAG, "unable to open filename '%s'", MP3_FILE_NAME);
        // }
        break;
    }
    case AUDIO_PLAYER_CALLBACK_EVENT_PLAYING:
        ESP_LOGI(TAG, "AUDIO_PLAYER_REQUEST_PLAY");
        if (s_audio_player_handle == NULL)
        {
            break;
        }
        ESP_ERROR_CHECK(uac_host_device_resume(s_audio_player_handle));
        uac_host_device_set_volume(s_audio_player_handle, get_sys_volume());
        break;
    case AUDIO_PLAYER_CALLBACK_EVENT_PAUSE:
        ESP_LOGI(TAG, "AUDIO_PLAYER_REQUEST_PAUSE");
        break;
    default:
        break;
    }
}

static void uac_device_callback(uac_host_device_handle_t uac_device_handle, const uac_host_device_event_t event, void *arg)
{
    if (event == UAC_HOST_DRIVER_EVENT_DISCONNECTED)
    {
        audio_player_type = AUDIO_PLAYER_I2S;
        // stop audio player first
        s_audio_player_handle = NULL;
        // audio_player_stop();
        ESP_LOGI(TAG, "UAC Device disconnected");
        ESP_ERROR_CHECK(uac_host_device_close(uac_device_handle));
        return;
    }
    // Send uac device event to the event queue
    s_event_queue_t evt_queue = {
        .event_group = UAC_DEVICE_EVENT,
        .device_evt = {
            .handle = uac_device_handle,
            .event = event,
            .arg = arg
        }
    };
    // should not block here
    xQueueSend(s_event_queue, &evt_queue, 0);
}

static void uac_host_lib_callback(uint8_t addr, uint8_t iface_num, const uac_host_driver_event_t event, void *arg)
{
    // Send uac driver event to the event queue
    s_event_queue_t evt_queue = {
        .event_group = UAC_DRIVER_EVENT,
        .driver_evt = {
            .addr = addr,
            .iface_num = iface_num,
            .event = event,
            .arg = arg
        }
    };
    xQueueSend(s_event_queue, &evt_queue, 0);
}

/**
 * @brief Start USB Host install and handle common USB host library events while app pin not low
 *
 * @param[in] arg  Not used
 */
static void usb_lib_task(void *arg)
{
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL2,
    };

    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "USB Host installed");
    xTaskNotifyGive(arg);

    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        // In this example, there is only one client registered
        // So, once we deregister the client, this call must succeed with ESP_OK
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
            break;
        }
    }

    ESP_LOGI(TAG, "USB Host shutdown");
    // Clean up USB Host
    vTaskDelay(10); // Short delay to allow clients clean-up
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(NULL);
}

static void uac_lib_task(void *arg)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uac_host_driver_config_t uac_config = {
        .create_background_task = true,
        .task_priority = UAC_TASK_PRIORITY,
        .stack_size = 4096,
        .core_id = 0,
        .callback = uac_host_lib_callback,
        .callback_arg = NULL};

    ESP_ERROR_CHECK(uac_host_install(&uac_config));
    ESP_LOGI(TAG, "UAC Class Driver installed");
    s_event_queue_t evt_queue = {};
    while (1)
    {
        if (xQueueReceive(s_event_queue, &evt_queue, pdMS_TO_TICKS(100)))
        {
            if (UAC_DRIVER_EVENT == evt_queue.event_group)
            {
                uac_host_driver_event_t event = evt_queue.driver_evt.event;
                uint8_t addr = evt_queue.driver_evt.addr;
                uint8_t iface_num = evt_queue.driver_evt.iface_num;
                switch (event)
                {
                case UAC_HOST_DRIVER_EVENT_TX_CONNECTED:
                {
                    audio_player_type = AUDIO_PLAYER_USB;
                    uac_host_dev_info_t dev_info;
                    uac_host_device_handle_t uac_device_handle = NULL;
                    const uac_host_device_config_t dev_config = {
                        .addr = addr,
                        .iface_num = iface_num,
                        .buffer_size = 16000,
                        .buffer_threshold = 4000,
                        .callback = uac_device_callback,
                        .callback_arg = NULL,
                    };
                    ESP_ERROR_CHECK(uac_host_device_open(&dev_config, &uac_device_handle));
                    ESP_ERROR_CHECK(uac_host_get_device_info(uac_device_handle, &dev_info));
                    ESP_LOGI(TAG, "UAC Device connected: SPK");
                    uac_host_printf_device_param(uac_device_handle);
                    const uac_host_stream_config_t stm_config = {
                        .channels = 2,
                        .bit_resolution = 16,
                        .sample_freq = 48000,
                    };
                    ESP_ERROR_CHECK(uac_host_device_start(uac_device_handle, &stm_config));
                    s_audio_player_handle = uac_device_handle;
                    uac_host_device_set_volume(s_audio_player_handle, get_sys_volume());
                    // s_fp = fopen(SPIFFS_BASE MP3_FILE_NAME, "rb");
                    // s_fp = fopen("/sd/music/CANON.MP3", "rb");
                    // if (s_fp)
                    // {
                    //     ESP_LOGI(TAG, "Playing '%s'", MP3_FILE_NAME);
                    //     audio_player_play(s_fp);
                    // }
                    // else
                    // {
                    //     ESP_LOGE(TAG, "unable to open filename '%s'", MP3_FILE_NAME);
                    // }
                    break;
                }
                case UAC_HOST_DRIVER_EVENT_RX_CONNECTED:
                {
                    // we don't support MIC in this example
                    ESP_LOGI(TAG, "UAC Device connected: MIC");
                    break;
                }
                default:
                    break;
                }
            }
            else if (UAC_DEVICE_EVENT == evt_queue.event_group)
            {
                // uac_host_device_event_t event = evt_queue.device_evt.event;
                // switch (event) {
                // case UAC_HOST_DRIVER_EVENT_DISCONNECTED:
                //     ESP_LOGI(TAG, "UAC Device disconnected");
                //     break;
                // case UAC_HOST_DEVICE_EVENT_RX_DONE:
                //     break;
                // case UAC_HOST_DEVICE_EVENT_TX_DONE:
                //     break;
                // case UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR:
                //     break;
                // default:
                //     break;
                // }
            }
            else if (APP_EVENT == evt_queue.event_group)
            {
                break;
            }
        }
    }

    ESP_LOGI(TAG, "UAC Driver uninstall");
    ESP_ERROR_CHECK(uac_host_uninstall());
}

audio_player_t get_audio_player_type(void)
{
    return audio_player_type;
}

uac_host_device_handle_t get_audio_player_handle(void)
{
    return s_audio_player_handle;
}

void usb_host_init(void)
{
  s_event_queue = xQueueCreate(10, sizeof(s_event_queue_t));
    assert(s_event_queue != NULL);
  
    static TaskHandle_t uac_task_handle = NULL;
    BaseType_t ret = xTaskCreatePinnedToCore(uac_lib_task, "uac_events", 4096, NULL,
                                                USER_TASK_PRIORITY, &uac_task_handle, 1);
    assert(ret == pdTRUE);
    ret = xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096, (void *)uac_task_handle,
                                    USB_HOST_TASK_PRIORITY, NULL, 1);
    assert(ret == pdTRUE);  
}

void player_init(void)
{

    bsp_sdcard_init(CONFIG_BSP_SD_MOUNT_POINT, 25);
    file_iterator = file_iterator_new(CONFIG_BSP_SD_MOUNT_POINT "/music");
    assert(file_iterator != NULL);

    /* Initialize audio player, the default configuration is set to play through the USB headset. */
    player_config.mute_fn = _audio_player_mute_fn;
    player_config.write_fn = _audio_player_write_fn;
    player_config.clk_set_fn = _audio_player_std_clock;
    player_config.priority = 1;

    ESP_ERROR_CHECK(audio_player_new(player_config));

    ESP_ERROR_CHECK(audio_player_callback_register(_audio_player_callback, NULL));



    char filename[128];
    int retval = file_iterator_get_full_path_from_index(file_iterator, 0, filename, sizeof(filename));
    if (retval == 0) {
        ESP_LOGE(TAG, "unable to retrieve filename");
        return;
    }

    FILE *fp = fopen(filename, "rb");
    if (fp) {
        ESP_LOGI(TAG, "Playing '%s'", filename);
        audio_player_play(fp);
    } else {
        ESP_LOGE(TAG, "unable to open index %d, filename '%s'", 0, filename);
    }
}
