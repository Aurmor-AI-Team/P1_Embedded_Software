#pragma once

#include "esp_err.h"
#include "lsm6dsv.h"   // lsm6_sample_t

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// impact_det — head-impact detection on the board.
//
// Detection MUST happen here, not in the app. The BLE sample stream runs at
// ~10 Hz on the wire (STREAM_PERIOD_MS) and packs ax_g as i16 x1000, which
// clamps at 32.767 g. A real head impact is a few milliseconds wide and can
// exceed 50 g: sampled at stream rate it is aliased away entirely, and even if
// it landed on a sample it would saturate the field. So we watch the high-g
// channel at the full IMU rate (200 Hz, +/-80 g) and emit one discrete record
// per hit.
//
// A hit produces exactly ONE record carrying the PEAK sample, not the first
// sample over the line — at 200 Hz the first crossing is usually well below the
// real peak. Without the peak-hold a 150 ms contact would generate ~30 records
// for one impact.
//
// Records are delivered opportunistically: straight out over BLE if a phone is
// subscribed, else over UDP to the receiver (acked, retransmitted), else held
// in a RAM backlog until a transport comes back. Detection itself never depends
// on link state — a board with no radio at all still counts hits.
// ---------------------------------------------------------------------------

// The one threshold. Mirrored in rpi-receiver/ble-sender/protocol.py
// (IMPACT_THRESHOLD_G) and the app's features/ble-stream/protocol.ts. It also
// travels on every record, so nothing downstream should hard-code it for
// display — read it off the wire.
#define IMPACT_THRESHOLD_G      20.0f

#define IMPACT_WINDOW_US        150000   // peak-hold / event duration cap
#define IMPACT_REFRACTORY_US    250000   // dead time after a hit, debounces ringing
#define IMPACT_BACKLOG_DEPTH    32
#define IMPACT_DRAIN_PER_TICK   4        // cap the flush burst per service() call

// One detected impact. Passed by pointer to the transports, which each
// serialise it into their own wire format.
typedef struct {
    uint32_t seq;          // monotonic per boot, starts at 1 (0 is reserved)
    uint32_t t_ms;         // esp_timer ms at the START of the event
    float    peak_g;       // |h| at the peak sample
    float    threshold_g;  // IMPACT_THRESHOLD_G, on the wire so nobody hard-codes it
    uint16_t dur_ms;

    // Running totals THIS BOOT. Diagnostics only — they reset on reboot, never
    // at a session boundary, so the app must NOT display them. They exist so a
    // receiver can tell "3 impacts happened" from "3 impacts arrived".
    uint32_t count;
    float    sum_g;
    float    max_g;
} impact_rec_t;

void impact_det_init(void);

// Feed one sample at the full IMU rate. `h_mag` is the high-g resultant, which
// the caller has already computed. Never gate this on link state or playback.
void impact_det_feed(const lsm6_sample_t *s, float h_mag);

// Periodic housekeeping (~10 Hz): drains the backlog a few records at a time
// and closes a half-open event window if the sample stream stalled.
void impact_det_service(void);

// Synthesise an impact. The mock CSV peaks near 1 g, so this is the only way to
// exercise the pipeline without hitting real hardware.
void impact_det_inject(float peak_g);

uint32_t impact_det_count(void);
float    impact_det_peak_g(void);
float    impact_det_sum_g(void);
uint8_t  impact_det_backlog(void);

#ifdef __cplusplus
}
#endif
