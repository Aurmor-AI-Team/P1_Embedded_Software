#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "lsm6dsv.h"
#include "wifi_udp_tx.h"
#include "ble_provision.h"
#include "mock_playback.h"
#include "app_ctrl.h"

static const char *TAG = "main";

// I2C pins for the LSM6DSV80X IMU.
#define PIN_SDA  22
#define PIN_SCL  23

// Sampling, console-print, and UDP send rates.
#define IMU_SAMPLE_HZ     200
#define IMU_PRINT_HZ      10
#define UDP_SEND_HZ       100   // <= IMU_SAMPLE_HZ; decimated from the sample stream

#define IMU_SAMPLE_PERIOD_US (1000000 / IMU_SAMPLE_HZ)
#define IMU_PRINT_PERIOD_MS  (1000 / IMU_PRINT_HZ)
#define UDP_DECIMATE         (IMU_SAMPLE_HZ / UDP_SEND_HZ)

#define IMU_QUEUE_DEPTH ((IMU_SAMPLE_HZ / IMU_PRINT_HZ) * 2)

static QueueHandle_t      s_imu_q       = NULL;
static esp_timer_handle_t s_imu_timer   = NULL;
static volatile uint32_t  s_imu_dropped = 0;

static void imu_timer_cb(void *arg)
{
    lsm6_sample_t s;
    if (lsm6_read_sample(&s) != ESP_OK) return;
    if (xQueueSend(s_imu_q, &s, 0) != pdTRUE) {
        lsm6_sample_t discard;
        xQueueReceive(s_imu_q, &discard, 0);
        xQueueSend(s_imu_q, &s, 0);
        s_imu_dropped++;
    }
}

// ---------------------------------------------------------------------------
// IMU print task
// ---------------------------------------------------------------------------
static void imu_print_task(void *arg)
{
    ESP_LOGI(TAG, "IMU print task started (sample=%d Hz, print=%d Hz, udp=%d Hz, wid=%u)",
             IMU_SAMPLE_HZ, IMU_PRINT_HZ, UDP_SEND_HZ, wifi_udp_get_wearable_id());

    printf("# t_ms      | ax        ay        az       | "
           "hx       hy       hz       | "
           "gx        gy        gz        | temp | peak_h | all_peak | link\n");

    TickType_t next_print = xTaskGetTickCount() + pdMS_TO_TICKS(IMU_PRINT_PERIOD_MS);

    float         window_peak_g      = 0.0f;
    lsm6_sample_t window_peak_sample = {};
    bool          window_has_sample  = false;
    float         all_time_peak_g    = 0.0f;
    uint32_t      udp_decim          = 0;
    bool          last_connected     = false;
    bool          last_verified      = false;

    while (true) {
        TickType_t now = xTaskGetTickCount();
        TickType_t wait_ticks;
        if ((int32_t)(now - next_print) >= 0) {
            wait_ticks = 0;
        } else {
            wait_ticks = next_print - now;
            if (wait_ticks == 0) wait_ticks = 1;
        }

        lsm6_sample_t s;
        if (xQueueReceive(s_imu_q, &s, wait_ticks) == pdTRUE) {
            if (++udp_decim >= UDP_DECIMATE) {
                udp_decim = 0;
                // Live sensor stream pauses while the mock CSV plays, so the
                // Pi never sees the two interleaved.
                if (!mock_playback_is_active()) wifi_udp_send_imu(&s);
            }
            float h_mag = sqrtf(s.hx_g * s.hx_g + s.hy_g * s.hy_g + s.hz_g * s.hz_g);
            if (!window_has_sample || h_mag > window_peak_g) {
                window_peak_g      = h_mag;
                window_peak_sample = s;
                window_has_sample  = true;
            }
            if (h_mag > all_time_peak_g) all_time_peak_g = h_mag;
            continue;
        }

        if ((int32_t)(xTaskGetTickCount() - next_print) < 0) continue;

        next_print += pdMS_TO_TICKS(IMU_PRINT_PERIOD_MS);
        if ((int32_t)(xTaskGetTickCount() - next_print) > pdMS_TO_TICKS(IMU_PRINT_PERIOD_MS)) {
            next_print = xTaskGetTickCount() + pdMS_TO_TICKS(IMU_PRINT_PERIOD_MS);
        }

        // Notify the app if connection or verification state changed.
        bool connected = wifi_udp_is_connected();
        bool verified  = wifi_udp_is_verified();
        if (connected != last_connected || verified != last_verified) {
            last_connected = connected;
            last_verified  = verified;
            ble_provision_push_status();
        }

        if (!window_has_sample) {
            static int empty_logged = 0;
            if (empty_logged++ < 5) {
                ESP_LOGW(TAG, "No IMU samples in last %d ms — timer or I2C stalled?",
                         IMU_PRINT_PERIOD_MS);
            }
            continue;
        }

        const char *link = !wifi_udp_is_connected() ? " ---"
                         : !wifi_udp_has_target()    ? " net"
                         : wifi_udp_is_verified()    ? "  ok"
                                                     : "  tx";

        int64_t now_ms = esp_timer_get_time() / 1000;
        const lsm6_sample_t &p = window_peak_sample;
        printf("%-10lld | %+8.3f %+8.3f %+8.3f | "
               "%+8.2f %+8.2f %+8.2f | "
               "%+9.2f %+9.2f %+9.2f | %5.1f | %5.2fg | %6.2fg  | %s\n",
               now_ms,
               p.ax_g, p.ay_g, p.az_g,
               p.hx_g, p.hy_g, p.hz_g,
               p.gx_dps, p.gy_dps, p.gz_dps,
               p.temp_c,
               window_peak_g, all_time_peak_g, link);

        window_peak_g     = 0.0f;
        window_has_sample = false;

        static uint32_t last_dropped_logged = 0;
        if (s_imu_dropped != last_dropped_logged) {
            ESP_LOGW(TAG, "IMU queue drops: %lu (cumulative)",
                     (unsigned long)s_imu_dropped);
            last_dropped_logged = s_imu_dropped;
        }
    }
}

static void boot_reset_imu(void)
{
    ESP_LOGI(TAG, "Resetting IMU...");
    lsm6_force_i2c_mode(PIN_SDA, PIN_SCL);
    lsm6_init(PIN_SDA, PIN_SCL);
    lsm6_software_reset();
    lsm6_deinit();
}

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(4000));
    ESP_LOGI(TAG, "=== Boot ===");

    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs);

    boot_reset_imu();

    // --- IMU (optional: pairing + mock playback must work on a bare board) ---
    bool imu_ok = false;
    if (lsm6_init(PIN_SDA, PIN_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed — live sensor stream disabled");
    } else {
        uint8_t imu_addr, whoami;
        if (lsm6_read_who_am_i(&imu_addr, &whoami) != ESP_OK) {
            ESP_LOGW(TAG, "IMU not found — live sensor stream disabled; "
                          "mock playback (BOOT short-press) still works");
        } else {
            ESP_LOGI(TAG, "IMU at 0x%02X, WHO_AM_I=0x%02X", imu_addr, whoami);
            if (lsm6_configure_default() == ESP_OK) imu_ok = true;
            else ESP_LOGE(TAG, "IMU configuration failed — live sensor stream disabled");
        }
    }

    // --- Wi-Fi / UDP (auto-connects if previously provisioned) ---
    if (wifi_udp_init() != ESP_OK) { ESP_LOGE(TAG, "Wi-Fi/UDP init failed"); return; }

    // --- Mode state machine (button, LED, BLE lifecycle, mock playback) ---
    // BLE stays off until a 3 s BOOT long-press enters pairing mode.
    if (app_ctrl_init() != ESP_OK) {
        ESP_LOGE(TAG, "app_ctrl init failed");   // not fatal
    }

    // --- IMU sample queue + timer (live stream only; skipped without a sensor) ---
    if (imu_ok) {
        s_imu_q = xQueueCreate(IMU_QUEUE_DEPTH, sizeof(lsm6_sample_t));
        if (s_imu_q == NULL) { ESP_LOGE(TAG, "Failed to create IMU queue"); return; }

        const esp_timer_create_args_t timer_args = {
            .callback              = &imu_timer_cb,
            .arg                   = NULL,
            .dispatch_method       = ESP_TIMER_TASK,
            .name                  = "imu_sample",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&timer_args, &s_imu_timer) != ESP_OK) { ESP_LOGE(TAG, "timer create failed"); return; }
        if (esp_timer_start_periodic(s_imu_timer, IMU_SAMPLE_PERIOD_US) != ESP_OK) { ESP_LOGE(TAG, "timer start failed"); return; }
        ESP_LOGI(TAG, "IMU timer running at %d us period", IMU_SAMPLE_PERIOD_US);

        xTaskCreate(imu_print_task, "imu_print", 4096, NULL, 5, NULL);
    }
    ESP_LOGI(TAG, "Tasks running. app_main exiting.");
}