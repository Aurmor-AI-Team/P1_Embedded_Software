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

// Set this per-board: one as MAIN, the others as PERIPHERAL.
// Flash the same firmware to all boards and change MY_UWB_ROLE plus,
// for peripherals, MY_PERIPHERAL_SUFFIX ('A'..'I' for up to 9 peripherals).
//
// 3-message DS-TWR: the PERIPHERAL sends the first message (Poll) and the
// last (Final); the MAIN replies (Response) and computes the distance, so
// the result lands on the MAIN node.
#define MY_UWB_ROLE          UWB_ROLE_MAIN  // UWB_ROLE_PERIPHERAL or UWB_ROLE_MAIN
#define MY_PERIPHERAL_SUFFIX 'A'            // 'A'..'I', ignored for main

// Main: which peripherals to range against, in cycle order. List only the
// suffix character; the full address is "W<suffix>". The main round-robins
// through this list, one peer per cycle, waiting for each peripheral's Poll.
//
// TDMA SCHEDULING:
//   The main broadcasts a beacon at the start of each round; each
//   peripheral schedules its Poll at its assigned slot offset from the
//   beacon RX timestamp (slot = suffix - 'A' + 1, so 'A' = slot 1, etc).
//   Slots are non-overlapping by construction — peripherals cannot
//   collide on air as long as they each have a unique suffix.
//
//   Because slots are anchored to the DW3000's hardware RX timestamp of
//   the beacon (the same 499.2 MHz clock DS-TWR cancels), host-clock
//   drift between boards does NOT accumulate. Peripherals re-sync to the
//   main every round.
//
// Assign each peripheral one of suffixes 'A'..'O' (15 max). Unassigned
// slots time out at the main (~10 ms per empty slot) and are marked
// stale in the per-peer table — harmless but reduces useful round rate.
//
// Main = Head
// A = Chest/Waist            I = Right ankle
// B = Left Elbow             J = (spare)
// C = Right Elbow            K = (spare)
// D = Left Wrist             L = (spare)
// E = Right Wrist            M = (spare)
// F = Left Knee              N = (spare)
// G = Right Knee             O = (spare)
// H = Left ankle

// Sampling and reporting rates. The UWB round rate is fixed by TDMA
// geometry (UWB_ROUND_PERIOD_MS in uwb_ranging.h), not by a configurable
// Hz value here. At default 15 ms slots × 16 phases (beacon + 15
// peripherals) + 15 ms guard = 255 ms per round ≈ 3.9 Hz per peripheral.
// If you need more rate, shrink UWB_SLOT_WIDTH_MS in the header (at the
// cost of less jitter tolerance).
#define IMU_SAMPLE_HZ     200
#define IMU_PRINT_HZ      10

#define IMU_SAMPLE_PERIOD_US (1000000 / IMU_SAMPLE_HZ)
#define IMU_PRINT_PERIOD_MS  (1000 / IMU_PRINT_HZ)

// ---------------------------------------------------------------------------
// Per-peer state table (main only).
//
// Indexed by slot index 0..UWB_MAX_PERIPHERALS-1, where index i
// corresponds to peripheral suffix 'A' + i. So index 0 is always 'A',
// index 14 is always 'O'.
//
// Each slot holds the most recent ranging result for that peer, when it
// was captured, and a small failure counter. Stale slots (no update in
// PEER_STALE_MS) are flagged in the snapshot so consumers can ignore them.
//
// Protected by a portMUX_TYPE spinlock. Contention is low (the UWB task
// writes one slot per slot exchange, the print task reads everything
// once per print interval), and a spinlock never blocks.
// ---------------------------------------------------------------------------
#define PEER_STALE_MS  2000   // ~8 rounds at 4 Hz round rate

typedef struct {
    uwb_range_result_t result;       // distance + peer_imu (when .valid)
    int64_t            last_ok_us;   // Wall clock of most recent successful update
    uint32_t           ok_count;     // Cumulative successful cycles for this peer
    uint32_t           miss_count;   // Cumulative consecutive missed cycles
} peer_state_t;

static peer_state_t   s_peer_state[UWB_MAX_PERIPHERALS] = {};
static portMUX_TYPE   s_peer_lock = portMUX_INITIALIZER_UNLOCKED;

// Peripheral-side shared state: this device's link status to the main.
// Single slot (one main, one peripheral = one relationship). The
// peripheral cycle writes this (distance is always 0 — the main computes
// the real distance); the print task on the peripheral reads it as a
// liveness flag. Main does not use s_self_range.
static uwb_range_result_t s_self_range = {};
static portMUX_TYPE       s_self_lock  = portMUX_INITIALIZER_UNLOCKED;

static inline void peer_state_publish(size_t idx, const uwb_range_result_t *r) {
    if (idx >= UWB_MAX_PERIPHERALS) return;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_peer_lock);
    s_peer_state[idx].result     = *r;
    s_peer_state[idx].last_ok_us = now;
    s_peer_state[idx].ok_count++;
    s_peer_state[idx].miss_count = 0;
    portEXIT_CRITICAL(&s_peer_lock);
}

static inline void peer_state_record_miss(size_t idx) {
    if (idx >= UWB_MAX_PERIPHERALS) return;
    portENTER_CRITICAL(&s_peer_lock);
    s_peer_state[idx].miss_count++;
    portEXIT_CRITICAL(&s_peer_lock);
}

static inline void peer_state_snapshot(peer_state_t *out) {
    portENTER_CRITICAL(&s_peer_lock);
    for (size_t i = 0; i < UWB_MAX_PERIPHERALS; i++) {
        out[i] = s_peer_state[i];
    }
    portEXIT_CRITICAL(&s_peer_lock);
}

// Convenience for peripheral side — same pattern but a single slot.
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

/* ============================================================================
 * Observational ranging diagnostics — LOGS ONLY, never touches the radio.
 *
 * Captures the timing signature of failures so we can distinguish cadence
 * desync from chip faults: per-peer success/miss counts, current miss streak,
 * worst streak, time since last success, and the actual loop period (to catch
 * vTaskDelayUntil free-running). Dumps a summary every DIAG_DUMP_PERIOD_MS.
 * Remove once the root cause is found. */
#define DIAG_DUMP_PERIOD_MS  5000

typedef struct {
    uint32_t ok;                 /* total successes */
    uint32_t miss;               /* total misses */
    uint32_t cur_streak;         /* current consecutive misses */
    uint32_t worst_streak;       /* worst consecutive misses seen */
    int64_t  last_ok_us;         /* timestamp of last success (0 = never) */
    int64_t  worst_gap_us;       /* longest observed gap between successes */
} diag_peer_t;

static diag_peer_t s_diag[UWB_MAX_PERIPHERALS];
static int64_t     s_diag_last_dump_us = 0;
static int64_t     s_diag_last_cycle_us = 0;
static int64_t     s_diag_max_period_us = 0;   /* worst loop period since last dump */
static int64_t     s_diag_min_period_us = 0;

/* Call once per slot, right after uwb_perform_round returns on the main.
 * idx = slot index (0..UWB_MAX_PERIPHERALS-1, == suffix - 'A').
 * success = results[idx].valid. */
static void diag_record(size_t idx, bool success)
{
    int64_t now = esp_timer_get_time();

    /* Loop period tracking — detects cadence collapse / free-running. */
    if (s_diag_last_cycle_us != 0) {
        int64_t period = now - s_diag_last_cycle_us;
        if (period > s_diag_max_period_us) s_diag_max_period_us = period;
        if (s_diag_min_period_us == 0 || period < s_diag_min_period_us) {
            s_diag_min_period_us = period;
        }
    }
    s_diag_last_cycle_us = now;

    if (idx >= UWB_MAX_PERIPHERALS) return;
    diag_peer_t *p = &s_diag[idx];

    if (success) {
        if (p->last_ok_us != 0) {
            int64_t gap = now - p->last_ok_us;
            if (gap > p->worst_gap_us) p->worst_gap_us = gap;
        }
        p->last_ok_us = now;
        p->ok++;
        p->cur_streak = 0;
    } else {
        p->miss++;
        p->cur_streak++;
        if (p->cur_streak > p->worst_streak) p->worst_streak = p->cur_streak;
    }

    /* Periodic dump. */
    if (now - s_diag_last_dump_us >= (int64_t)DIAG_DUMP_PERIOD_MS * 1000) {
        s_diag_last_dump_us = now;
        printf("# DIAG period(ms) min=%.1f max=%.1f | per-peer ok/miss streak(cur/worst) ageOK(ms) worstGap(ms)\n",
               s_diag_min_period_us / 1000.0, s_diag_max_period_us / 1000.0);
        for (size_t i = 0; i < UWB_MAX_PERIPHERALS; i++) {
            diag_peer_t *q = &s_diag[i];
            double age_ms = (q->last_ok_us == 0) ? -1.0
                          : (now - q->last_ok_us) / 1000.0;
            printf("#   W%c: ok=%lu miss=%lu streak=%lu/%lu ageOK=%.0f worstGap=%.0f\n",
                   (char)('A' + (i)),
                   (unsigned long)q->ok, (unsigned long)q->miss,
                   (unsigned long)q->cur_streak, (unsigned long)q->worst_streak,
                   age_ms, q->worst_gap_us / 1000.0);
        }
        s_diag_max_period_us = 0;
        s_diag_min_period_us = 0;
    }
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

    // Publish to the UWB module so the peripheral can embed the freshest
    // sample in its next Final frame. On the main this is harmless
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

        // Build the range column. On the MAIN, distances come from the
        // per-peer grid printed below, so the main line just shows "---".
        // On the PERIPHERAL there is no local distance (the main computes
        // it), so we show a link indicator instead: "link" when recent
        // cycles are completing, "---" when they aren't.
        char range_str[16];
        if (MY_UWB_ROLE == UWB_ROLE_MAIN) {
            // For the main line: the per-peer grid below carries distances.
            snprintf(range_str, sizeof(range_str), "   ---");
        } else {
            uwb_range_result_t sr = self_range_snapshot();
            if (sr.valid) snprintf(range_str, sizeof(range_str), "   link");
            else          snprintf(range_str, sizeof(range_str), "   ---");
        }

        int64_t now_ms = esp_timer_get_time() / 1000;
        const lsm6_sample_t &p = window_peak_sample;
        // printf("%-10lld | %+8.3f %+8.3f %+8.3f | "
        //        "%+8.2f %+8.2f %+8.2f | "
        //        "%+9.2f %+9.2f %+9.2f | %5.1f | %5.2fg | %6.2fg  | %s\n",
        //        now_ms,
        //        p.ax_g, p.ay_g, p.az_g,
        //        p.hx_g, p.hy_g, p.hz_g,
        //        p.gx_dps, p.gy_dps, p.gz_dps,
        //        p.temp_c,
        //        window_peak_g, all_time_peak_g,
        //        range_str);

        // Main: dump full 16-row node table every ~500 ms (5 print
        // intervals). One row for the main itself, plus one for each of
        // the 15 peripherals A..O. Empty cells when a peripheral has not
        // reported anything fresh (age > PEER_STALE_MS or never seen).
        // Counters stay visible regardless, so a row with 0 ok / N miss
        // tells you "this peripheral is silent" at a glance.
        //
        // Column widths are sized for realistic worst-case values:
        //   d(m)  ±99.99   accel  ±19.99 g   |h|  0..99.99 g
        //   gz    ±2000 dps  T  -40..125 °C  ok/miss up to 99999
        if (MY_UWB_ROLE == UWB_ROLE_MAIN) {
            static int peer_table_counter = 0;

            if (++peer_table_counter >= IMU_PRINT_HZ / 2) {  // ~2 Hz
                peer_table_counter = 0;

                peer_state_t snap[UWB_MAX_PERIPHERALS];
                peer_state_snapshot(snap);
                int64_t now_us = esp_timer_get_time();

                // Body-position labels, indexed by suffix - 'A'. Keep
                // these short — they sit in a fixed-width column.
                static const char *kPosLabel[UWB_MAX_PERIPHERALS] = {
                    "Chest   ", "L Elbow ", "R Elbow ", "L Wrist ",
                    "R Wrist ", "L Knee  ", "R Knee  ", "L Ankle ",
                    "R Ankle ", "spare J ", "spare K ", "spare L ",
                    "spare M ", "spare N ", "spare O ",
                };

                // Header. Reprint every cycle so the table is self-
                // contained for log scraping. Column widths must match
                // the format strings below or alignment breaks.
                printf("# Node            d(m)     ax     ay     az    |h|       gz     T       ok   miss   age\n");

                // Row 0: main itself. No distance to self. IMU comes from
                // window_peak_sample (the most recent windowed reading
                // already in hand). "ok/miss/age" not meaningful for the
                // main's own UWB — leave blank.
                {
                    const lsm6_sample_t &q = window_peak_sample;
                    float h = sqrtf(q.hx_g*q.hx_g + q.hy_g*q.hy_g + q.hz_g*q.hz_g);
                    printf("# Head  (main)    ---  %+6.2f %+6.2f %+6.2f  %5.2f  %+7.1f %5.1f      ---    ---   ---\n",
                           q.ax_g, q.ay_g, q.az_g, h, q.gz_dps, q.temp_c);
                }

                // Rows 1..15: peripherals A..O.
                for (size_t i = 0; i < UWB_MAX_PERIPHERALS; i++) {
                    char    suffix = (char)('A' + i);
                    int64_t age_ms = snap[i].last_ok_us == 0
                                   ? -1
                                   : (now_us - snap[i].last_ok_us) / 1000;
                    bool fresh = snap[i].result.valid &&
                                 age_ms >= 0 &&
                                 age_ms < PEER_STALE_MS;

                    // Row prefix is fixed-width so the table stays aligned.
                    printf("# W%c (%s)", suffix, kPosLabel[i]);

                    if (fresh) {
                        const lsm6_sample_t &q = snap[i].result.peer_imu;
                        bool imu_fresh = snap[i].result.peer_imu_valid;
                        float h = imu_fresh
                                ? sqrtf(q.hx_g*q.hx_g + q.hy_g*q.hy_g + q.hz_g*q.hz_g)
                                : 0.0f;
                        if (imu_fresh) {
                            printf(" %+6.2f  %+6.2f %+6.2f %+6.2f  %5.2f  %+7.1f %5.1f",
                                   snap[i].result.distance_m,
                                   q.ax_g, q.ay_g, q.az_g, h,
                                   q.gz_dps, q.temp_c);
                        } else {
                            // Distance valid, IMU not yet (peripheral
                            // hadn't published a sample at the moment
                            // the Final was assembled).
                            printf(" %+6.2f     ---    ---    ---    ---      ---   ---",
                                   snap[i].result.distance_m);
                        }
                    } else {
                        // Nothing fresh: blank distance + IMU columns.
                        // Counters and age still printed below to convey
                        // link state independent of freshness.
                        printf("    ---     ---    ---    ---    ---      ---   ---");
                    }

                    // Counters + age. age "---" if never seen.
                    if (age_ms < 0) {
                        printf("    %5lu  %5lu   ---\n",
                               (unsigned long)snap[i].ok_count,
                               (unsigned long)snap[i].miss_count);
                    } else {
                        printf("    %5lu  %5lu  %4lld\n",
                               (unsigned long)snap[i].ok_count,
                               (unsigned long)snap[i].miss_count,
                               (long long)age_ms);
                    }
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
             MY_UWB_ROLE == UWB_ROLE_MAIN ? "main" : "peripheral");

    int peripheral_fails = 0;   // Consecutive misses on the peripheral

    while (true) {
        if (MY_UWB_ROLE == UWB_ROLE_MAIN) {
            // One call = one full TDMA round. Beacon TX, then sweep all
            // 15 slots in order, collecting per-peripheral results.
            // The round itself takes ~UWB_ROUND_PERIOD_MS, so no extra
            // delay is needed — pacing is intrinsic.
            uwb_range_result_t results[UWB_MAX_PERIPHERALS];
            esp_err_t err = uwb_perform_round(results);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "uwb_perform_round failed (err=%d)", err);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            // Publish each slot's outcome to the per-peer state table.
            for (size_t i = 0; i < UWB_MAX_PERIPHERALS; i++) {
                bool success = results[i].valid;
                diag_record(i, success);
                if (success) {
                    peer_state_publish(i, &results[i]);
                } else {
                    peer_state_record_miss(i);
                }
            }
        } else {
            // Peripheral: uwb_perform_round blocks on the beacon, then
            // runs one slot exchange. Beacon RX is the pacing; no
            // vTaskDelay needed. results[0] is the only meaningful slot.
            uwb_range_result_t r;
            esp_err_t err = uwb_perform_round(&r);

            if (err == ESP_OK && r.valid) {
                peripheral_fails = 0;
                self_range_publish(&r);   // distance_m is 0; .valid = linked
            } else {
                peripheral_fails++;
                if (peripheral_fails == 5) {
                    self_range_invalidate();
                    ESP_LOGW(TAG, "5 consecutive ranging failures, marking invalid");
                }
            }
        }
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
    if (uwb_init(MY_UWB_ROLE, MY_PERIPHERAL_SUFFIX,
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