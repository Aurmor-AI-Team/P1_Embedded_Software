// ---------------------------------------------------------------------------
// mock_playback.cpp — replay the embedded HEAD mock CSV.
//
// Playback runs one CSV row at a time, down whichever transport is live — the
// UDP path to the Pi in WIFI mode, or the direct BLE stream when a phone is
// subscribed in PAIRING mode (a solo session).
//
// Playback LOOPS, and only app_ctrl stops it. It backs the WMODE_MOCK working
// mode, which lasts until the user picks another one — so it has to outlive a
// single ~72 s pass of the CSV. The BOOT short-press reaches it the same way,
// by selecting that mode, so the button and the app can never disagree about
// whether a demo is running.
//
// Timing is PER ROW (head_mock_row_t.dt_ms), not one periodic tick. The capture
// is sampled every 255 ms, but the synthetic head impacts spliced into it are
// 16 ms half-sine pulses sampled every 2 ms — an impact squeezed into a single
// 255 ms row would claim a contact 16x longer than any real one.
// ---------------------------------------------------------------------------
#include "mock_playback.h"

#include <math.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "ble_stream.h"
#include "impact_det.h"
#include "head_mock_data.h"
#include "lsm6dsv.h"
#include "wifi_udp_tx.h"

static const char *TAG = "mock_play";

static esp_timer_handle_t s_timer;
static volatile bool      s_active = false;
static int                s_row = 0;
static uint32_t           s_laps = 0;
// Frames handed to a transport that wasn't up. Counted rather than refused:
// MOCK is a mode, and the user can select it before a phone has subscribed.
static uint32_t           s_sink_misses = 0;

static void tick_cb(void *arg)
{
    if (!s_active) return;
    if (s_row >= HEAD_MOCK_ROWS) {
        // The MOCK mode outlives one pass of the CSV — it runs until the user
        // picks another mode — so wrap rather than stopping. app_ctrl is the
        // only thing that ends playback.
        s_row = 0;
        s_laps++;
        ESP_LOGI(TAG, "playback loop %lu (%lu frame(s) had no transport)",
                 (unsigned long)s_laps, (unsigned long)s_sink_misses);
    }
    const head_mock_row_t *r = &HEAD_MOCK_DATA[s_row++];
    lsm6_sample_t s = {
        .ax_g = r->ax_g, .ay_g = r->ay_g, .az_g = r->az_g,
        .hx_g = r->hx_g, .hy_g = r->hy_g, .hz_g = r->hz_g,
        .gx_dps = r->gx_dps, .gy_dps = r->gy_dps, .gz_dps = r->gz_dps,
        .temp_c = r->imu_temp_c,
    };

#ifdef IMPACT_TEST_HOOK
    // Demo only. The live IMU path (main.cpp) is the ONLY producer in a
    // production build, so fabricated rows can never be recorded as a real
    // athlete's head impacts — which is exactly what an impact record must
    // never be wrong about.
    {
        float h_mag = sqrtf(s.hx_g * s.hx_g + s.hy_g * s.hy_g + s.hz_g * s.hz_g);
        impact_det_feed(&s, h_mag);
        impact_det_service();
    }
#endif

    // The head has no biometrics of its own; the mock carries chest/wrist
    // values so the app's Heart rate / SpO2 / Respiration / HRV tiles fill.
    // Both sinks are no-ops when their transport isn't up, and the two are
    // mutually exclusive in practice (BLE is off in WIFI mode).
    const bool udp_up = wifi_udp_is_connected() && wifi_udp_has_target();
    const bool ble_up = ble_stream_ready();
    wifi_udp_send_imu_bio(&s, r->hr, r->spo2, r->resp, r->hrv);
    ble_stream_notify_bio(&s, r->hr, r->spo2, r->resp, r->hrv);
    if (!udp_up && !ble_up && s_sink_misses++ == 0) {
        ESP_LOGW(TAG, "no receiver and no BLE subscriber — frames are going nowhere");
    }

    // Schedule the next row at ITS own interval.
    if (s_active) {
        esp_timer_start_once(s_timer, (uint64_t)(r->dt_ms ? r->dt_ms : 1) * 1000);
    }
}

esp_err_t mock_playback_start_loop(void)
{
    if (s_active) return ESP_OK;
    // Deliberately does NOT check for a transport. MOCK is a working mode the
    // app can select before a phone has subscribed or before the board has
    // rejoined its receiver; refusing here would mean the mode silently didn't
    // take. Frames that land nowhere are counted (s_sink_misses) instead.
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
    s_laps = 0;
    s_sink_misses = 0;
    s_active = true;
    // One-shot, re-armed per row: rows do not share a cadence any more.
    esp_err_t err = esp_timer_start_once(s_timer, 1000);
    if (err != ESP_OK) { s_active = false; return err; }
    uint32_t total_ms = 0;
    for (int i = 0; i < HEAD_MOCK_ROWS; i++) total_ms += HEAD_MOCK_DATA[i].dt_ms;
    ESP_LOGI(TAG, "playing %d rows (~%lu s, looping)",
             HEAD_MOCK_ROWS, (unsigned long)(total_ms / 1000));
    return ESP_OK;
}

void mock_playback_stop(void)
{
    if (!s_active) return;
    s_active = false;
    if (s_timer) esp_timer_stop(s_timer);
    ESP_LOGI(TAG, "playback stopped at row %d/%d (%lu lap(s))",
             s_row, HEAD_MOCK_ROWS, (unsigned long)s_laps);
}

bool mock_playback_is_active(void)
{
    return s_active;
}
