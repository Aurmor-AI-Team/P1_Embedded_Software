#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "lsm6dsv.h"   /* for lsm6_sample_t */
#include "bio_telemetry.h"   // bio_telemetry_t + bio_get_* accessors

#ifdef __cplusplus
extern "C" {
#endif

/* TDMA DS-TWR with main-broadcast beacon synchronisation.
 *
 * One ROUND is:
 *   t=0       Main broadcasts Beacon
 *   t=slot×i  Peripheral i sends Poll  (scheduled off beacon RX timestamp)
 *             Main sends Response
 *             Peripheral i sends Final (carries 3 timestamps + IMU)
 *             Main computes distance + extracts IMU for slot i
 *   ... for i = 1..UWB_MAX_PERIPHERALS
 *
 * Why the beacon: each peripheral schedules its Poll using
 * dwt_setdelayedtrxtime referenced to the DW3000's hardware RX timestamp
 * of the beacon — the same 499.2 MHz clock that DS-TWR already cancels.
 * Host-clock drift between boards becomes irrelevant; peripherals re-sync
 * to the main every round. */

#define UWB_MAX_PERIPHERALS  15

/* Slot timing. ONE slot must fit: Response delay (6 ms) + Final delay
 * (6 ms) + frame air time (~1 ms) + guard (2 ms) = ~15 ms. Decrease only
 * after observing low "late/cancelled" rates in real data. Increase if
 * you see Poll-listen misses caused by neighboring slots bleeding in. */
#define UWB_SLOT_WIDTH_MS      15
#define UWB_ROUND_GUARD_MS     15   /* trailing pad after last slot      */

/* Total round duration. With 15 peripherals + beacon + guard:
 *   beacon(slot 0) + 15 slots + guard = 16 × 15 + 15 = 255 ms ≈ 3.9 Hz   */
#define UWB_ROUND_PERIOD_MS    \
    ((UWB_MAX_PERIPHERALS + 1) * UWB_SLOT_WIDTH_MS + UWB_ROUND_GUARD_MS)

/* Roles for the 3-message DS-TWR exchange under TDMA scheduling.
 *
 *   MAIN        broadcasts beacon, then ranges each peripheral slot in
 *               turn. The result for each peripheral lands here.
 *   PERIPHERAL  receives beacon, waits its assigned slot offset, then
 *               sends Poll and Final. Carries its IMU in the Final. */
typedef enum {
    UWB_ROLE_MAIN,
    UWB_ROLE_PERIPHERAL,
} uwb_role_t;

typedef struct {
    /* Distance in meters after applying g_uwb_distance_offset_m. Valid
     * only when .valid is true. Negative values can occur very close in
     * if the calibration offset is too large; that's expected. */
    float   distance_m;
    int64_t timestamp_us;
    bool    valid;

    /* Peer (peripheral) IMU snapshot, carried inside the Final frame.
     * Only meaningful on the main side, after a successful range cycle.
     * peer_imu_valid is false on the peripheral. */
    bio_telemetry_t peer_bio;
    bool    peer_bio_valid;
} uwb_range_result_t;

/* Initialise UWB.
 *
 * For UWB_ROLE_PERIPHERAL, the peripheral's own address suffix is
 *   'A'..'O' for boards WA..WO  (15 peripherals max).
 * The suffix doubles as the slot index: 'A' = slot 1, 'B' = slot 2, ...,
 * 'O' = slot 15. Slot 0 is the beacon (main TX only). Pass 0 to use the
 * default 'A'. Main ignores peripheral_addr_suffix. */
esp_err_t uwb_init(uwb_role_t role,
                   char peripheral_addr_suffix,
                   int mosi, int miso, int sclk, int cs, int rst);

/* On MAIN:
 *   Performs one full TDMA ROUND. Broadcasts the beacon, then sweeps the
 *   15 peripheral slots; for each slot, listens for Poll, sends Response,
 *   receives Final, computes distance. Fills results[0..UWB_MAX_PERIPHERALS-1]
 *   so results[i] is the outcome for the peripheral whose suffix = 'A'+i.
 *   Empty / non-responding slots leave results[i].valid = false.
 *   Call this in a tight loop with no extra delay — the round itself
 *   provides the pacing.
 *
 * On PERIPHERAL:
 *   Performs one cycle. Waits for the beacon (blocks up to ~UWB_ROUND_PERIOD_MS),
 *   then schedules its Poll at its assigned slot offset, completes the
 *   exchange. Writes results[0] only (the peripheral knows only its own
 *   cycle outcome). results[1..] are untouched. No distance is computed
 *   here (it lives on main); results[0].valid means "Final sent OK".
 *   Call in a tight loop — the beacon wait IS the pacing.
 *
 * `results` must be an array of at least UWB_MAX_PERIPHERALS entries on
 * MAIN and at least 1 entry on PERIPHERAL.
 *
 * Returns ESP_OK unless a driver/SPI fault occurs. Individual slot
 * outcomes live in results[i].valid. */
esp_err_t uwb_perform_round(uwb_range_result_t *results);

#include "bio_telemetry.h"     /* replaces the lsm6dsv.h include for peer data */

typedef struct {
    float   distance_m;
    int64_t timestamp_us;
    bool    valid;

    bio_telemetry_t peer_bio;        /* was: lsm6_sample_t peer_imu;      */
    bool            peer_bio_valid;  /* was: bool          peer_imu_valid;*/
} uwb_range_result_t;

/* Option B: per-sensor setters. Each takes s_bio_lock, writes ONLY its own
 * fields, ORs in ONLY its present-mask bit, and marks the struct valid. Safe
 * to call from independent sensor tasks at independent rates. */
void uwb_bio_set_imu(const float accel_g[3], const float gyro_dps[3],
                     const float highg_g[3], float temp_c);
void uwb_bio_set_ppg(uint8_t hr_bpm, uint8_t spo2_pct, uint8_t quality);
void uwb_bio_set_resp(uint8_t rate_bpm);
void uwb_bio_set_eeg(const float band_power[BIO_EEG_BANDS]);  /* relative 0..1 */
void uwb_bio_set_ecg(uint8_t hr_bpm, uint16_t rmssd_ms, uint8_t flags);
void uwb_bio_set_emg(const float rms[BIO_EMG_CHANNELS]);      /* normalized   */

/* Calibration offset subtracted from raw distance (meters). Leave at 0
 * until you've run a known-distance calibration. */
extern float g_uwb_distance_offset_m;

#ifdef __cplusplus
}
#endif