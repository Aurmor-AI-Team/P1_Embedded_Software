#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "dwm3000.h"
#include "lsm6dsv.h"
#include "uwb_ranging.h"
#include "port.h"
extern "C" {
    #include "deca_device_api.h"
}

static const char *TAG = "main";

#define PIN_MOSI 18
#define PIN_MISO 20
#define PIN_SCLK 19
#define PIN_CS   16
#define PIN_RST  17
#define PIN_SDA  22
#define PIN_SCL  23

// Set this per-board: one as INITIATOR, the others as RESPONDER.
// Flash the same firmware to all boards and change MY_UWB_ROLE plus,
// for responders, MY_RESPONDER_SUFFIX ('A'..'I' for up to 9 responders).
#define MY_UWB_ROLE         UWB_ROLE_RESPONDER  // UWB_ROLE_RESPONDER or UWB_ROLE_INITIATOR
#define MY_RESPONDER_SUFFIX 'B'                 // 'A'..'I', ignored for initiator

// Initiator: which responders to range against, in cycle order. List
// only the suffix character; the full address is "W<suffix>". The
// initiator round-robins through this list, one peer per cycle.
//
// Set this to the list of responders you've actually flashed. Unflashed
// suffixes will just time out and be marked stale — harmless but wasteful
// of cycle time.
// Initiator = Head
// A = Chest/Waist
// B = Left Elbow (above elbow)
// C = Right Elbow (above elbow)
// D = Left Wrist
// E = Right Wrist
// F = Left Knee (above knee)
// G = Right Knee (above knee)
// H = Left ankle
// I = Right ankle
static const char s_initiator_peer_list[] = {
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'
};
#define INITIATOR_PEER_COUNT (sizeof(s_initiator_peer_list) / sizeof(s_initiator_peer_list[0]))

// Sampling and reporting rates. With 9 peers, UWB_RANGE_HZ is the
// *total* round-robin rate; per-peer rate is UWB_RANGE_HZ / N_PEERS.
// 30 Hz total / 9 peers = ~3.3 Hz per peer.
#define IMU_SAMPLE_HZ     200
#define IMU_PRINT_HZ      10
#define UWB_RANGE_HZ      30

#define IMU_SAMPLE_PERIOD_US (1000000 / IMU_SAMPLE_HZ)
#define IMU_PRINT_PERIOD_MS  (1000 / IMU_PRINT_HZ)
#define UWB_PERIOD_MS        (1000 / UWB_RANGE_HZ)

// ---------------------------------------------------------------------------
// Per-peer state table (initiator only).
//
// Indexed by position in s_initiator_peer_list, NOT by ASCII value of
// the suffix. So peer slot 0 holds whatever responder is at
// s_initiator_peer_list[0], etc.
//
// Each slot holds the most recent ranging result for that peer, when it
// was captured, and a small failure counter. Stale slots (no update in
// PEER_STALE_MS) are flagged in the snapshot so consumers can ignore
// them.
//
// Protected by a portMUX_TYPE spinlock. Contention is low (the UWB task
// writes one slot per round-robin cycle, the print task reads everything
// once per print interval), and a spinlock never blocks.
// ---------------------------------------------------------------------------
#define PEER_STALE_MS  500   // Slot is stale if no update in this many ms

typedef struct {
    uwb_range_result_t result;       // distance + peer_imu (when .valid)
    int64_t            last_ok_us;   // Wall clock of most recent successful update
    uint32_t           ok_count;     // Cumulative successful cycles for this peer
    uint32_t           miss_count;   // Cumulative consecutive missed cycles
} peer_state_t;

static peer_state_t   s_peer_state[INITIATOR_PEER_COUNT] = {};
static portMUX_TYPE   s_peer_lock = portMUX_INITIALIZER_UNLOCKED;

// Responder-side shared state: this device's last "range from initiator".
// Single slot (one initiator, one responder = one relationship). The
// responder cycle writes this; the print task on the responder reads it.
// Initiator does not use s_self_range.
static uwb_range_result_t s_self_range = {};
static portMUX_TYPE       s_self_lock  = portMUX_INITIALIZER_UNLOCKED;

static inline void peer_state_publish(size_t idx, const uwb_range_result_t *r) {
    if (idx >= INITIATOR_PEER_COUNT) return;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_peer_lock);
    s_peer_state[idx].result     = *r;
    s_peer_state[idx].last_ok_us = now;
    s_peer_state[idx].ok_count++;
    s_peer_state[idx].miss_count = 0;
    portEXIT_CRITICAL(&s_peer_lock);
}

static inline void peer_state_record_miss(size_t idx) {
    if (idx >= INITIATOR_PEER_COUNT) return;
    portENTER_CRITICAL(&s_peer_lock);
    s_peer_state[idx].miss_count++;
    portEXIT_CRITICAL(&s_peer_lock);
}

static inline void peer_state_snapshot(peer_state_t *out) {
    portENTER_CRITICAL(&s_peer_lock);
    for (size_t i = 0; i < INITIATOR_PEER_COUNT; i++) {
        out[i] = s_peer_state[i];
    }
    portEXIT_CRITICAL(&s_peer_lock);
}

// Convenience for responder side — same pattern but a single slot.
static inline void self_range_publish(const uwb_range_result_t *r) {
    portENTER_CRITICAL(&s_self_lock);
    s_self_range = *r;
    portEXIT_CRITICAL(&s_self_lock);
}

static inline void self_range_invalidate(void) {
    portENTER_CRITICAL(&s_self_lock);
    s_self_range.valid = false;
    portEXIT_CRITICAL(&s_self_lock);
}

static inline uwb_range_result_t self_range_snapshot(void) {
    uwb_range_result_t r;
    portENTER_CRITICAL(&s_self_lock);
    r = s_self_range;
    portEXIT_CRITICAL(&s_self_lock);
    return r;
}

// ---------------------------------------------------------------------------
// IMU sampling: esp_timer pushes samples into a queue at exactly IMU_SAMPLE_HZ.
// The print task drains the queue, tracks peak high-g, and prints at 10 Hz.
//
// Queue depth = 2 * sample rate / print rate gives one full print interval
// of headroom in case the print task gets preempted.
// ---------------------------------------------------------------------------
#define IMU_QUEUE_DEPTH ((IMU_SAMPLE_HZ / IMU_PRINT_HZ) * 2)

static QueueHandle_t      s_imu_q       = NULL;
static esp_timer_handle_t s_imu_timer   = NULL;
static volatile uint32_t  s_imu_dropped = 0;

static void imu_timer_cb(void *arg)
{
    // esp_timer with TASK dispatch runs in the timer's own task context,
    // not an ISR — I2C transactions are safe here.
    lsm6_sample_t s;
    if (lsm6_read_sample(&s) != ESP_OK) return;

    // Publish to the UWB module so the responder can embed the freshest
    // sample in its next Report frame. On the initiator this is harmless
    // (the function checks role internally).
    uwb_publish_local_imu(&s);

    // Non-blocking send. If the queue is full, drop the oldest sample to
    // keep newest data flowing rather than backing up the timer.
    if (xQueueSend(s_imu_q, &s, 0) != pdTRUE) {
        lsm6_sample_t discard;
        xQueueReceive(s_imu_q, &discard, 0);
        xQueueSend(s_imu_q, &s, 0);
        s_imu_dropped++;
    }
}

// ---------------------------------------------------------------------------
// IMU print task — drains the queue, tracks windowed peak high-g, prints
// at IMU_PRINT_HZ. Also tracks an all-time peak across the session.
// ---------------------------------------------------------------------------
static void imu_print_task(void *arg)
{
    ESP_LOGI(TAG, "IMU print task started (sample=%d Hz, print=%d Hz)",
             IMU_SAMPLE_HZ, IMU_PRINT_HZ);

    printf("# t_ms      | ax        ay        az       | "
           "hx       hy       hz       | "
           "gx        gy        gz        | temp | peak_h | all_peak | range\n");

    TickType_t next_print = xTaskGetTickCount() + pdMS_TO_TICKS(IMU_PRINT_PERIOD_MS);

    // Windowed peak (resets each print interval)
    float         window_peak_g      = 0.0f;
    lsm6_sample_t window_peak_sample = {};
    bool          window_has_sample  = false;

    // All-time peak (resets only on boot)
    float all_time_peak_g = 0.0f;

    while (true) {
        // Compute time until next print deadline. If we're past it,
        // skip straight to printing. Otherwise, block on the queue for
        // at most that long, so we yield CPU when no samples arrive.
        TickType_t now = xTaskGetTickCount();
        TickType_t wait_ticks;
        if ((int32_t)(now - next_print) >= 0) {
            wait_ticks = 0;
        } else {
            wait_ticks = next_print - now;
            // Guarantee at least 1 tick of yielding when we do block,
            // even if the deadline rounds down to 0 ticks. This is the
            // anti-spin guarantee.
            if (wait_ticks == 0) wait_ticks = 1;
        }

        lsm6_sample_t s;
        if (xQueueReceive(s_imu_q, &s, wait_ticks) == pdTRUE) {
            float h_mag = sqrtf(s.hx_g * s.hx_g +
                                s.hy_g * s.hy_g +
                                s.hz_g * s.hz_g);
            if (!window_has_sample || h_mag > window_peak_g) {
                window_peak_g      = h_mag;
                window_peak_sample = s;
                window_has_sample  = true;
            }
            if (h_mag > all_time_peak_g) {
                all_time_peak_g = h_mag;
            }
            // Loop back; we may still have time before the deadline.
            continue;
        }

        // xQueueReceive returned false. Either the deadline arrived
        // (wait_ticks expired) or no sample showed up. Either way,
        // it's time to evaluate whether to print.
        if ((int32_t)(xTaskGetTickCount() - next_print) < 0) {
            // Spurious wake-up (shouldn't happen, but be defensive).
            continue;
        }

        // Advance the deadline. If we've fallen far behind (e.g. due to
        // a long log stall), snap forward to "now" rather than spamming
        // catch-up prints.
        next_print += pdMS_TO_TICKS(IMU_PRINT_PERIOD_MS);
        if ((int32_t)(xTaskGetTickCount() - next_print) > pdMS_TO_TICKS(IMU_PRINT_PERIOD_MS)) {
            next_print = xTaskGetTickCount() + pdMS_TO_TICKS(IMU_PRINT_PERIOD_MS);
        }

        if (!window_has_sample) {
            // No samples this interval — log once, then move on without
            // burning CPU. The next iteration will block on the queue
            // for the full print interval.
            static int empty_logged = 0;
            if (empty_logged++ < 5) {
                ESP_LOGW(TAG, "No IMU samples in last %d ms — timer or I2C stalled?",
                         IMU_PRINT_PERIOD_MS);
            }
            continue;
        }

        // Build the range column. On the responder, this is just the
        // distance to the (one and only) initiator. On the initiator, we
        // pick a peer to show on the main line and dump a per-peer
        // summary line below (rotating which peer's IMU we detail, so
        // every peer gets shown roughly once per N print intervals).
        char range_str[16];
        if (MY_UWB_ROLE == UWB_ROLE_INITIATOR) {
            // For the main line: show the most recently-updated peer's
            // distance, or "---" if no peer has reported in.
            snprintf(range_str, sizeof(range_str), "   ---");
        } else {
            uwb_range_result_t sr = self_range_snapshot();
            if (sr.valid) snprintf(range_str, sizeof(range_str), "%7.3f", sr.distance_m);
            else          snprintf(range_str, sizeof(range_str), "   ---");
        }

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
               window_peak_g, all_time_peak_g,
               range_str);

        // Initiator: dump compact all-peer distance grid every ~500 ms
        // (5 print intervals), plus rotate through peers' IMU detail one
        // per print iteration so every peer's IMU shows up over time.
        if (MY_UWB_ROLE == UWB_ROLE_INITIATOR) {
            static int peer_grid_counter = 0;
            static size_t peer_rotate_idx = 0;

            peer_state_t snap[INITIATOR_PEER_COUNT];
            peer_state_snapshot(snap);
            int64_t now_us = esp_timer_get_time();

            if (++peer_grid_counter >= IMU_PRINT_HZ / 2) {  // ~2 Hz
                peer_grid_counter = 0;
                // Compact distance grid: one line, all 9 peers.
                printf("# peers:");
                for (size_t i = 0; i < INITIATOR_PEER_COUNT; i++) {
                    char  suffix   = s_initiator_peer_list[i];
                    int64_t age_ms = (now_us - snap[i].last_ok_us) / 1000;
                    bool    fresh  = snap[i].result.valid && age_ms < PEER_STALE_MS;
                    if (fresh) {
                        printf("  W%c=%+5.2f", suffix, snap[i].result.distance_m);
                    } else {
                        printf("  W%c=  ---", suffix);
                    }
                }
                printf("\n");
            }

            // Per-print rotation: show one peer's IMU detail. Skip stale
            // slots silently and try the next one until we find one with
            // fresh IMU or we've walked the whole list.
            for (size_t tries = 0; tries < INITIATOR_PEER_COUNT; tries++) {
                size_t i = peer_rotate_idx;
                peer_rotate_idx = (peer_rotate_idx + 1) % INITIATOR_PEER_COUNT;
                int64_t age_ms = (now_us - snap[i].last_ok_us) / 1000;
                if (snap[i].result.peer_imu_valid &&
                    snap[i].result.valid &&
                    age_ms < PEER_STALE_MS) {
                    const lsm6_sample_t &q = snap[i].result.peer_imu;
                    float h = sqrtf(q.hx_g*q.hx_g + q.hy_g*q.hy_g + q.hz_g*q.hz_g);
                    printf("# W%c: d=%+.3fm  ax=%+.2f ay=%+.2f az=%+.2f  "
                           "h|=%.2fg  gz=%+.1f dps  T=%.1f  "
                           "ok=%lu miss=%lu age=%lldms\n",
                           s_initiator_peer_list[i],
                           snap[i].result.distance_m,
                           q.ax_g, q.ay_g, q.az_g,
                           h,
                           q.gz_dps, q.temp_c,
                           (unsigned long)snap[i].ok_count,
                           (unsigned long)snap[i].miss_count,
                           (long long)age_ms);
                    break;
                }
            }
        }

        // Reset window
        window_peak_g     = 0.0f;
        window_has_sample = false;

        // Periodic drop report
        static uint32_t last_dropped_logged = 0;
        if (s_imu_dropped != last_dropped_logged) {
            ESP_LOGW(TAG, "IMU queue drops: %lu (cumulative)",
                     (unsigned long)s_imu_dropped);
            last_dropped_logged = s_imu_dropped;
        }
    }
}

// ---------------------------------------------------------------------------
// UWB task — performs ranging, updates shared state.
// ---------------------------------------------------------------------------
static void uwb_task(void *arg)
{
    ESP_LOGI(TAG, "UWB task started (role: %s)",
             MY_UWB_ROLE == UWB_ROLE_INITIATOR ? "initiator" : "responder");

    TickType_t next = xTaskGetTickCount();
    size_t     peer_idx = 0;                       // Initiator round-robin index
    int        responder_fails = 0;                // Consecutive misses on the responder

    while (true) {
        uwb_range_result_t r;
        char   peer_suffix;
        size_t this_idx = 0;

        if (MY_UWB_ROLE == UWB_ROLE_INITIATOR) {
            this_idx     = peer_idx;
            peer_suffix  = s_initiator_peer_list[this_idx];
            peer_idx     = (peer_idx + 1) % INITIATOR_PEER_COUNT;
        } else {
            peer_suffix  = 0;  // Ignored by responder cycle
        }

        esp_err_t err = uwb_perform_ranging(peer_suffix, &r);

        if (MY_UWB_ROLE == UWB_ROLE_INITIATOR) {
            if (err == ESP_OK && r.valid) {
                peer_state_publish(this_idx, &r);
            } else {
                peer_state_record_miss(this_idx);
            }
        } else {
            // Responder side: same logic as before, single slot
            if (err == ESP_OK && r.valid) {
                self_range_publish(&r);
                responder_fails = 0;
            } else {
                responder_fails++;
                if (responder_fails == 5) {
                    self_range_invalidate();
                    ESP_LOGW(TAG, "5 consecutive ranging failures, marking invalid");
                }
            }
        }

        vTaskDelayUntil(&next, pdMS_TO_TICKS(UWB_PERIOD_MS));
    }
}

// ---------------------------------------------------------------------------
// Boot-time peripheral reset
// ---------------------------------------------------------------------------
static void boot_reset_peripherals(void)
{
    ESP_LOGI(TAG, "Resetting peripherals...");

    // Force IMU out of any stale I3C state
    lsm6_force_i2c_mode(PIN_SDA, PIN_SCL);

    // Hard-reset DWM3000
    gpio_set_direction((gpio_num_t)PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_direction((gpio_num_t)PIN_RST, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Software-reset IMU
    lsm6_init(PIN_SDA, PIN_SCL);
    lsm6_software_reset();
    lsm6_deinit();
}

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------
extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(4000));
    ESP_LOGI(TAG, "=== Boot ===");

    boot_reset_peripherals();

    // --- UWB ---
    if (uwb_init(MY_UWB_ROLE, MY_RESPONDER_SUFFIX,
                 PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS, PIN_RST) != ESP_OK) {
        ESP_LOGE(TAG, "UWB init failed");
        return;
    }

    // --- IMU ---
    if (lsm6_init(PIN_SDA, PIN_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed");
        return;
    }

    uint8_t imu_addr, whoami;
    if (lsm6_read_who_am_i(&imu_addr, &whoami) != ESP_OK) {
        ESP_LOGE(TAG, "IMU not found");
        return;
    }
    ESP_LOGI(TAG, "IMU at 0x%02X, WHO_AM_I=0x%02X", imu_addr, whoami);

    if (lsm6_configure_default() != ESP_OK) {
        ESP_LOGE(TAG, "IMU configuration failed");
        return;
    }

    // --- IMU sample queue + timer ---
    s_imu_q = xQueueCreate(IMU_QUEUE_DEPTH, sizeof(lsm6_sample_t));
    if (s_imu_q == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU queue");
        return;
    }

    const esp_timer_create_args_t timer_args = {
        .callback              = &imu_timer_cb,
        .arg                   = NULL,
        .dispatch_method       = ESP_TIMER_TASK,
        .name                  = "imu_sample",
        .skip_unhandled_events = true,
    };
    if (esp_timer_create(&timer_args, &s_imu_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create IMU timer");
        return;
    }
    if (esp_timer_start_periodic(s_imu_timer, IMU_SAMPLE_PERIOD_US) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start IMU timer");
        return;
    }
    ESP_LOGI(TAG, "IMU timer running at %d us period", IMU_SAMPLE_PERIOD_US);

    // --- Tasks ---
    xTaskCreate(imu_print_task, "imu_print", 4096, NULL, 5, NULL);
    xTaskCreate(uwb_task,       "uwb",       4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Tasks running. app_main exiting.");
}