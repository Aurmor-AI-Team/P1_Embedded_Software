// ---------------------------------------------------------------------------
// impact_det.cpp — peak-hold head-impact detector + delivery backlog.
//
// See impact_det.h for why detection lives on the board rather than in the app.
//
// Threading: impact_det_feed()/impact_det_service() run on the IMU task;
// impact_det_inject() runs on the NimBLE host task. The detector state is
// touched only by the IMU task, but the sequence counter, the running totals
// and the backlog are shared — those are the only things under the mutex.
// ---------------------------------------------------------------------------
#include "impact_det.h"

#include "ble_stream.h"
#include "wifi_udp_tx.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "impact_det";

// --- detector state (IMU task only) ----------------------------------------
static bool          s_armed = false;
static int64_t       s_start_us = 0;
static int64_t       s_refractory_until_us = 0;
static float         s_peak_g = 0.0f;
// Set when an event was closed by the window cap rather than by the signal
// falling back under the threshold. Until we actually SEE it drop, we refuse to
// arm again — otherwise a stuck high-g axis (or a board being sat on) re-arms
// every refractory period and emits an impact every 400 ms forever, flooding
// the backlog and inflating the count with one non-event. A genuine second hit
// always dips below threshold in between, so nothing real is suppressed.
static bool          s_awaiting_release = false;

// --- shared state (under s_lock) -------------------------------------------
static SemaphoreHandle_t s_lock = NULL;
static uint32_t      s_seq = 0;
static uint32_t      s_count = 0;
static float         s_sum_g = 0.0f;
static float         s_max_g = 0.0f;

static impact_rec_t  s_backlog[IMPACT_BACKLOG_DEPTH];
static uint8_t       s_backlog_head = 0;
static uint8_t       s_backlog_count = 0;

static inline bool lock(void)
{
    return s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(50)) == pdTRUE;
}

static inline void unlock(void)
{
    if (s_lock) xSemaphoreGive(s_lock);
}

// Caller holds the lock.
static void backlog_push(const impact_rec_t *r)
{
    uint8_t tail = (uint8_t)((s_backlog_head + s_backlog_count) % IMPACT_BACKLOG_DEPTH);
    s_backlog[tail] = *r;
    if (s_backlog_count < IMPACT_BACKLOG_DEPTH) {
        s_backlog_count++;
    } else {
        // Drop the OLDEST. Losing an impact record is the one failure this
        // module exists to prevent, so say so loudly rather than counting it.
        s_backlog_head = (uint8_t)((s_backlog_head + 1) % IMPACT_BACKLOG_DEPTH);
        ESP_LOGE(TAG, "backlog full — DROPPED an impact record");
    }
}

// Whether records may leave the board at all. False in the IDLE and MOCK
// working modes; see impact_det_set_delivery_enabled(). Starts false because a
// board boots into IDLE.
static volatile bool s_deliver = false;

/** Try both transports. Returns true if the record left the board. */
static bool dispatch(const impact_rec_t *r)
{
    // The mode says nothing goes out. Refusing here (rather than earlier) is
    // deliberate: the caller then backlogs the record, so leaving IDLE replays
    // every hit that happened while we were quiet.
    if (!s_deliver) return false;
    if (ble_stream_ready() && ble_stream_send_impact(r) == ESP_OK) return true;
    if (wifi_udp_is_verified() && wifi_udp_send_alert(r) == ESP_OK) return true;
    return false;
}

static void deliver(const impact_rec_t *r)
{
    if (dispatch(r)) {
        ESP_LOGI(TAG, "impact #%lu %.1f g sent", (unsigned long)r->seq, (double)r->peak_g);
        return;
    }
    if (lock()) {
        backlog_push(r);
        ESP_LOGW(TAG, "impact #%lu %.1f g held (%u queued)",
                 (unsigned long)r->seq, (double)r->peak_g, (unsigned)s_backlog_count);
        unlock();
    }
}

/** Build a record from a completed event and hand it to the transports. */
static void emit(float peak_g, int64_t start_us, int64_t now_us)
{
    impact_rec_t r;
    memset(&r, 0, sizeof(r));

    if (!lock()) return;   // never block the IMU task on a stuck lock
    r.seq    = ++s_seq;
    s_count += 1;
    s_sum_g += peak_g;
    if (peak_g > s_max_g) s_max_g = peak_g;
    r.count = s_count;
    r.sum_g = s_sum_g;
    r.max_g = s_max_g;
    unlock();

    r.t_ms        = (uint32_t)(start_us / 1000);
    r.peak_g      = peak_g;
    r.threshold_g = IMPACT_THRESHOLD_G;
    int64_t dur_us = now_us - start_us;
    if (dur_us < 0) dur_us = 0;
    if (dur_us > 65535 * 1000) dur_us = 65535 * 1000;
    r.dur_ms = (uint16_t)(dur_us / 1000);

    deliver(&r);
}

void impact_det_init(void)
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    s_armed = false;
    s_awaiting_release = false;
    s_refractory_until_us = 0;
    s_backlog_head = s_backlog_count = 0;
    ESP_LOGI(TAG, "impact detection armed at %.1f g (window %d ms, refractory %d ms)",
             (double)IMPACT_THRESHOLD_G, IMPACT_WINDOW_US / 1000,
             IMPACT_REFRACTORY_US / 1000);
}

void impact_det_set_delivery_enabled(bool on)
{
    if (on == s_deliver) return;
    s_deliver = on;
    ESP_LOGI(TAG, "delivery %s (%u record(s) held)",
             on ? "enabled" : "disabled", (unsigned)s_backlog_count);
    // Nothing to kick here: impact_det_service() runs at 10 Hz from the IMU task
    // and drains the backlog on its own now that dispatch() will accept.
}

bool impact_det_delivery_enabled(void) { return s_deliver; }

void impact_det_feed(const lsm6_sample_t *s, float h_mag)
{
    if (!s) return;
    const int64_t now_us = esp_timer_get_time();

    if (s_armed) {
        if (h_mag > s_peak_g) s_peak_g = h_mag;
        // The event ends when the resultant falls back under the threshold, or
        // when the window caps it — a sustained overload is one impact, not a
        // record every 150 ms.
        const bool released = (h_mag < IMPACT_THRESHOLD_G);
        if (released || (now_us - s_start_us) >= IMPACT_WINDOW_US) {
            const float   peak  = s_peak_g;
            const int64_t start = s_start_us;
            s_armed = false;
            s_awaiting_release = !released;
            s_refractory_until_us = now_us + IMPACT_REFRACTORY_US;
            emit(peak, start, now_us);
        }
        return;
    }

    if (h_mag < IMPACT_THRESHOLD_G) {
        s_awaiting_release = false;
        return;
    }

    if (!s_awaiting_release && now_us >= s_refractory_until_us) {
        s_armed    = true;
        s_start_us = now_us;
        s_peak_g   = h_mag;
    }
}

void impact_det_service(void)
{
    // A stalled sample stream must not leave a half-open window: the hit
    // happened, and it would otherwise sit unreported until the next sample.
    if (s_armed) {
        const int64_t now_us = esp_timer_get_time();
        if ((now_us - s_start_us) >= IMPACT_WINDOW_US) {
            const float   peak  = s_peak_g;
            const int64_t start = s_start_us;
            s_armed = false;
            // The samples stopped, so we never saw the signal come back down.
            // Do not demand a release we can no longer observe — that would
            // disarm the detector for good if the stream resumes above
            // threshold.
            s_awaiting_release = false;
            s_refractory_until_us = now_us + IMPACT_REFRACTORY_US;
            emit(peak, start, now_us);
        }
    }

    // Drain a few at a time. A full 32-deep flush spread over ~1 s of ticks
    // keeps a reconnect from stalling the sample stream behind 32 notifies.
    for (int i = 0; i < IMPACT_DRAIN_PER_TICK; i++) {
        impact_rec_t r;
        if (!lock()) return;
        if (s_backlog_count == 0) { unlock(); return; }
        r = s_backlog[s_backlog_head];
        unlock();

        if (!dispatch(&r)) return;   // still no transport — keep it, retry next tick

        if (!lock()) return;
        // Re-check: the head can only have moved if something else drained it.
        if (s_backlog_count > 0 && s_backlog[s_backlog_head].seq == r.seq) {
            s_backlog_head = (uint8_t)((s_backlog_head + 1) % IMPACT_BACKLOG_DEPTH);
            s_backlog_count--;
        }
        unlock();
    }
}

void impact_det_inject(float peak_g)
{
    const int64_t now_us = esp_timer_get_time();
    ESP_LOGW(TAG, "INJECTED synthetic impact %.1f g", (double)peak_g);
    emit(peak_g, now_us, now_us + 20000);   // 20 ms, a plausible contact duration
}

uint32_t impact_det_count(void)  { return s_count; }
float    impact_det_peak_g(void) { return s_max_g; }
float    impact_det_sum_g(void)  { return s_sum_g; }
uint8_t  impact_det_backlog(void){ return s_backlog_count; }
