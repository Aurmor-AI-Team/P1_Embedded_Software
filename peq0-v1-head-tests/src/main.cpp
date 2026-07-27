#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "lsm6dsv.h"
#include "mibs_wire.h"
#include "wifi_udp_tx.h"
#include "ble_provision.h"
#include "mock_playback.h"
#include "app_ctrl.h"

static const char *TAG = "main";

// I2C pins for the LSM6DSV80X IMU.
#define PIN_SDA  22
#define PIN_SCL  23

// Sampling, console-print, and send rates.
#define IMU_SAMPLE_HZ     200
#define IMU_PRINT_HZ      10
#define UDP_SEND_HZ       100   // <= IMU_SAMPLE_HZ; decimated from the sample stream

// BLE cannot carry 100 Hz of telemetry. At a 15 ms connection interval and one
// 49-byte notification per event, ~20 Hz is the realistic ceiling with headroom
// for the alert indications that share the same mbuf pool.
#define BT_SEND_HZ        20

#define IMU_SAMPLE_PERIOD_US (1000000 / IMU_SAMPLE_HZ)
#define IMU_PRINT_PERIOD_MS  (1000 / IMU_PRINT_HZ)
#define UDP_DECIMATE         (IMU_SAMPLE_HZ / UDP_SEND_HZ)
#define BT_DECIMATE          (IMU_SAMPLE_HZ / BT_SEND_HZ)

#define IMU_QUEUE_DEPTH ((IMU_SAMPLE_HZ / IMU_PRINT_HZ) * 2)

// Impact detection. Once |h| crosses the threshold we hold the peak and emit a
// single record, instead of one per sample above threshold — at 200 Hz a 150 ms
// contact would otherwise generate ~30 alerts for one hit.
#define IMPACT_WINDOW_US     150000   // peak-hold / event duration cap
#define IMPACT_REFRACTORY_US 250000   // dead time after an event, debounces ringing

static QueueHandle_t      s_imu_q       = NULL;
static esp_timer_handle_t s_imu_timer   = NULL;
static volatile uint32_t  s_imu_dropped = 0;

mibs_message _mibs_message;


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
// Impact detector
//
// A hit produces exactly one mibs_impact_t carrying the PEAK sample, not the
// first sample over the line — at 200 Hz the first crossing is usually well
// below the real peak.
// ---------------------------------------------------------------------------
typedef struct {
    bool          armed;
    int64_t       start_us;
    int64_t       refractory_until_us;
    float         peak_g;
    lsm6_sample_t peak_sample;
    uint32_t      seq;
} impact_det_t;

static void impact_emit(impact_det_t *d, int64_t now_us)
{
    mibs_impact_t imp;
    memset(&imp, 0, sizeof(imp));
    imp.seq         = ++d->seq;
    imp.t_us        = d->start_us;
    imp.peak_g      = d->peak_g;
    imp.threshold_g = app_ctrl_threshold_g();
    imp.hx_g        = d->peak_sample.hx_g;
    imp.hy_g        = d->peak_sample.hy_g;
    imp.hz_g        = d->peak_sample.hz_g;
    imp.gx_dps      = d->peak_sample.gx_dps;
    imp.gy_dps      = d->peak_sample.gy_dps;
    imp.gz_dps      = d->peak_sample.gz_dps;
    imp.dur_ms      = (uint16_t)((now_us - d->start_us) / 1000);

    app_ctrl_report_impact(&imp);

    // Running totals that ride along in the telemetry frame.
    _mibs_message.impact_count       += 1;
    _mibs_message.impact_accumulator += d->peak_g;

    d->armed = false;
    d->refractory_until_us = now_us + IMPACT_REFRACTORY_US;
}

// Feed one sample. Returns true if an impact record was emitted.
static bool impact_feed(impact_det_t *d, const lsm6_sample_t *s, float h_mag)
{
    const int64_t now_us = esp_timer_get_time();
    const float   thresh = app_ctrl_threshold_g();

    if (d->armed) {
        if (h_mag > d->peak_g) { d->peak_g = h_mag; d->peak_sample = *s; }
        if (h_mag < thresh || (now_us - d->start_us) >= IMPACT_WINDOW_US) {
            impact_emit(d, now_us);
            return true;
        }
        return false;
    }

    if (h_mag > thresh && now_us >= d->refractory_until_us) {
        d->armed       = true;
        d->start_us    = now_us;
        d->peak_g      = h_mag;
        d->peak_sample = *s;
    }
    return false;
}

// ---------------------------------------------------------------------------
// IMU task
// ---------------------------------------------------------------------------
static void imu_print_task(void *arg)
{
    ESP_LOGI(TAG, "IMU task started (sample=%d Hz, print=%d Hz, wifi=%d Hz, bt=%d Hz, wid=%u)",
             IMU_SAMPLE_HZ, IMU_PRINT_HZ, UDP_SEND_HZ, BT_SEND_HZ,
             wifi_udp_get_wearable_id());

    printf("# t_ms      | ax        ay        az       | "
           "hx       hy       hz       | "
           "gx        gy        gz        | temp | peak_h | all_peak | mode\n");

    TickType_t next_print = xTaskGetTickCount() + pdMS_TO_TICKS(IMU_PRINT_PERIOD_MS);

    float         window_peak_g      = 0.0f;
    lsm6_sample_t window_peak_sample = {};
    bool          window_has_sample  = false;
    float         all_time_peak_g    = 0.0f;
    uint32_t      send_decim         = 0;
    bool          last_connected     = false;
    bool          last_verified      = false;
    app_mode_t    last_mode          = app_ctrl_mode();

    impact_det_t  det = {};

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

            float h_mag = sqrtf(s.hx_g * s.hx_g + s.hy_g * s.hy_g + s.hz_g * s.hz_g);
            if (!window_has_sample || h_mag > window_peak_g) {
                window_peak_g      = h_mag;
                window_peak_sample = s;
                window_has_sample  = true;
            }
            if (h_mag > all_time_peak_g) all_time_peak_g = h_mag;

            // Detection runs in EVERY mode, including MOCK and with no radio
            // up. app_ctrl decides whether to send, buffer, or hold. Never
            // gate this on link state.
            impact_feed(&det, &s, h_mag);

            // Live telemetry: only in a LIVE mode, with a transport, mock off.
            const uint32_t decim = (app_ctrl_xport() == APP_XPORT_BT)
                                 ? BT_DECIMATE : UDP_DECIMATE;
            if (++send_decim >= decim) {
                send_decim = 0;
                if (app_ctrl_stream_enabled()) {
                    _mibs_message.impact_threshold = app_ctrl_threshold_g();
                    _mibs_message.all_time_peak_g  = all_time_peak_g;
                    // Real sensor path: no biometric channels.
                    app_ctrl_send_stream(&_mibs_message, window_peak_sample.temp_c,
                                         0.0f, 0.0f, 0.0f, 0.0f);
                }
            }
            continue;
        }

        if ((int32_t)(xTaskGetTickCount() - next_print) < 0) continue;

        next_print += pdMS_TO_TICKS(IMU_PRINT_PERIOD_MS);
        if ((int32_t)(xTaskGetTickCount() - next_print) > pdMS_TO_TICKS(IMU_PRINT_PERIOD_MS)) {
            next_print = xTaskGetTickCount() + pdMS_TO_TICKS(IMU_PRINT_PERIOD_MS);
        }

        // A stalled sample stream must not leave a half-open impact window.
        if (det.armed && (esp_timer_get_time() - det.start_us) >= IMPACT_WINDOW_US) {
            impact_emit(&det, esp_timer_get_time());
        }

        // Notify the app if connection, verification, or mode changed.
        bool connected = wifi_udp_is_connected();
        bool verified  = wifi_udp_is_verified();
        app_mode_t mode = app_ctrl_mode();
        if (connected != last_connected || verified != last_verified || mode != last_mode) {
            last_connected = connected;
            last_verified  = verified;
            last_mode      = mode;
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

        int64_t now_ms = esp_timer_get_time() / 1000;
        const lsm6_sample_t &p = window_peak_sample;
        (void)now_ms; (void)p;
        // printf("%-10lld | %+8.3f %+8.3f %+8.3f | "
        //        "%+8.2f %+8.2f %+8.2f | "
        //        "%+9.2f %+9.2f %+9.2f | %5.1f | %5.2fg | %6.2fg  | %s\n",
        //        now_ms,
        //        p.ax_g, p.ay_g, p.az_g,
        //        p.hx_g, p.hy_g, p.hz_g,
        //        p.gx_dps, p.gy_dps, p.gz_dps,
        //        p.temp_c,
        //        window_peak_g, all_time_peak_g, app_mode_str(mode));

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
                          "mock playback still works");
        } else {
            ESP_LOGI(TAG, "IMU at 0x%02X, WHO_AM_I=0x%02X", imu_addr, whoami);
            if (lsm6_configure_default() == ESP_OK) imu_ok = true;
            else ESP_LOGE(TAG, "IMU configuration failed — live sensor stream disabled");
        }
    }

    // --- Wi-Fi / UDP (auto-connects if previously provisioned) ---
    if (wifi_udp_init() != ESP_OK) { ESP_LOGE(TAG, "Wi-Fi/UDP init failed"); return; }

    // --- Mode state machine (button, LED, BLE lifecycle, alerts, playback) ---
    if (app_ctrl_init() != ESP_OK) {
        ESP_LOGE(TAG, "app_ctrl init failed");   // not fatal
    }

    // Without an IMU there is no impact detection, so say so loudly: the
    // device would sit in an ALERTS mode that can never fire.
    if (!imu_ok) {
        ESP_LOGE(TAG, "NO IMU — impact alerts are INOPERATIVE on this board");
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