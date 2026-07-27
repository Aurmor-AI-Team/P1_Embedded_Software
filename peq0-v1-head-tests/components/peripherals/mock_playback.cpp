// ---------------------------------------------------------------------------
// mock_playback.cpp — replay the embedded HEAD mock CSV, once, over whichever
// transport is currently live.
//
// One packet per CSV row at the CSV cadence (HEAD_MOCK_CADENCE_MS = 255 ms,
// so ~4 Hz for ~65 s). Playback used to call wifi_udp_send_imu_bio() directly,
// which made BT_MOCK impossible; it now goes through a sink that app_ctrl
// points at the current transport.
// ---------------------------------------------------------------------------
#include "mock_playback.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "head_mock_data.h"
#include "lsm6dsv.h"
#include "wifi_udp_tx.h"
#include <math.h>

static const char *TAG = "mock_play";

static esp_timer_handle_t s_timer;
static volatile bool      s_active = false;
static int                s_row = 0;
static mibs_message       s_msg;
static mock_sink_t        s_sink = NULL;

// Running totals across the playback run. These were locals initialised to 0
// on every tick, so impact_count was pinned at 1 and impact_accumulator at 1.0
// for the whole run — the app's impact tiles never moved.
static uint32_t s_impact_count = 0;
static float    s_impact_accum = 0.0f;
static float    s_peak_g       = 0.0f;
static uint32_t s_sink_fails   = 0;

#define MOCK_THRESHOLD_G 20.0f

void mock_playback_set_sink(mock_sink_t sink) { s_sink = sink; }

static esp_err_t emit(const mibs_message *m, float temp,
                      float hr, float spo2, float resp, float hrv)
{
    if (s_sink) return s_sink(m, temp, hr, spo2, resp, hrv);
    return wifi_udp_send_imu_bio(m, temp, hr, spo2, resp, hrv);
}

static void tick_cb(void *arg)
{
    if (!s_active) return;
    if (s_row >= HEAD_MOCK_ROWS) {
        mock_playback_stop();
        ESP_LOGI(TAG, "playback complete (%d rows, %lu sink failures)",
                 HEAD_MOCK_ROWS, (unsigned long)s_sink_fails);
        return;
    }
    const head_mock_row_t *r = &HEAD_MOCK_DATA[s_row++];

    float peak = sqrtf(r->hx_g * r->hx_g + r->hy_g * r->hy_g + r->hz_g * r->hz_g);
    if (peak > s_peak_g) s_peak_g = peak;
    if (peak > MOCK_THRESHOLD_G) {
        s_impact_count++;
        s_impact_accum += peak;
    }

    s_msg.impact_count       = s_impact_count;
    s_msg.impact_threshold   = MOCK_THRESHOLD_G;
    s_msg.impact_accumulator = s_impact_accum;
    s_msg.all_time_peak_g    = s_peak_g;

    // The head has no biometrics of its own; the mock carries chest/wrist
    // values so the app's Heart rate / SpO2 / Respiration / HRV tiles fill.
    if (emit(&s_msg, r->imu_temp_c, r->hr, r->spo2, r->resp, r->hrv) != ESP_OK) {
        if (s_sink_fails++ == 0) {
            ESP_LOGW(TAG, "sink rejected frame — no transport up?");
        }
    }
}

esp_err_t mock_playback_start(void)
{
    if (s_active) return ESP_OK;

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
    s_impact_count = 0;
    s_impact_accum = 0.0f;
    s_peak_g       = 0.0f;
    s_sink_fails   = 0;
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