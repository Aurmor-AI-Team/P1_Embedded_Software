// ---------------------------------------------------------------------------
// mock_playback.cpp — replay the embedded HEAD mock CSV over UDP, once.
//
// A BOOT short-press (see app_ctrl) starts playback: one CSV row at the CSV
// cadence (HEAD_MOCK_CADENCE_MS), down whichever transport is live — the UDP
// path to the Pi in WIFI mode, or the direct BLE stream when a phone is
// subscribed in PAIRING mode (a solo session). After the last row playback
// stops itself; another press while playing stops it early.
// ---------------------------------------------------------------------------
#include "mock_playback.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "ble_stream.h"
#include "head_mock_data.h"
#include "lsm6dsv.h"
#include "wifi_udp_tx.h"

static const char *TAG = "mock_play";

static esp_timer_handle_t s_timer;
static volatile bool      s_active = false;
static int                s_row = 0;

static void tick_cb(void *arg)
{
    if (!s_active) return;
    if (s_row >= HEAD_MOCK_ROWS) {
        mock_playback_stop();
        ESP_LOGI(TAG, "playback complete (%d rows)", HEAD_MOCK_ROWS);
        return;
    }
    const head_mock_row_t *r = &HEAD_MOCK_DATA[s_row++];
    lsm6_sample_t s = {
        .ax_g = r->ax_g, .ay_g = r->ay_g, .az_g = r->az_g,
        .hx_g = r->hx_g, .hy_g = r->hy_g, .hz_g = r->hz_g,
        .gx_dps = r->gx_dps, .gy_dps = r->gy_dps, .gz_dps = r->gz_dps,
        .temp_c = r->imu_temp_c,
    };
    // The head has no biometrics of its own; the mock carries chest/wrist
    // values so the app's Heart rate / SpO2 / Respiration / HRV tiles fill.
    // Both sinks are no-ops when their transport isn't up, and the two are
    // mutually exclusive in practice (BLE is off in WIFI mode).
    wifi_udp_send_imu_bio(&s, r->hr, r->spo2, r->resp, r->hrv);
    ble_stream_notify_bio(&s, r->hr, r->spo2, r->resp, r->hrv);
}

esp_err_t mock_playback_start(void)
{
    if (s_active) return ESP_OK;
    // Refuse only when there is nowhere to send: either the Pi (WIFI mode) or a
    // subscribed phone on the direct BLE stream (PAIRING mode, solo session).
    const bool udp_up = wifi_udp_is_connected() && wifi_udp_has_target();
    if (!udp_up && !ble_stream_ready()) {
        ESP_LOGW(TAG, "no receiver and no BLE subscriber — playback refused");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_timer == NULL) {
        const esp_timer_create_args_t targs = {
            .callback = tick_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "mock_play",
            .skip_unhandled_events = true,
        };
        esp_err_t err = esp_timer_create(&targs, &s_timer);
        if (err != ESP_OK) return err;
    }
    s_row = 0;
    s_active = true;
    esp_err_t err = esp_timer_start_periodic(s_timer,
                                             (uint64_t)HEAD_MOCK_CADENCE_MS * 1000);
    if (err != ESP_OK) { s_active = false; return err; }
    ESP_LOGI(TAG, "playing %d rows @ %d ms (~%d s, once)",
             HEAD_MOCK_ROWS, HEAD_MOCK_CADENCE_MS,
             HEAD_MOCK_ROWS * HEAD_MOCK_CADENCE_MS / 1000);
    return ESP_OK;
}

void mock_playback_stop(void)
{
    if (!s_active) return;
    s_active = false;
    if (s_timer) esp_timer_stop(s_timer);
    ESP_LOGI(TAG, "playback stopped at row %d/%d", s_row, HEAD_MOCK_ROWS);
}

bool mock_playback_is_active(void)
{
    return s_active;
}
