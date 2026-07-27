#pragma once

#include "esp_err.h"
#include "lsm6dsv.h"   // mibs_message
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Where playback frames go. Signature is deliberately identical to
// wifi_udp_send_imu_bio() and ble_provision_send_stream(), so app_ctrl can
// install its transport router directly and BT_MOCK works off-network.
typedef esp_err_t (*mock_sink_t)(const mibs_message *m, float temp,
                                 float hr, float spo2, float resp, float hrv);

// Install the sink. If never called, playback falls back to
// wifi_udp_send_imu_bio() — i.e. the old WiFi-only behaviour.
void mock_playback_set_sink(mock_sink_t sink);

// Play the embedded HEAD mock CSV once at the CSV cadence, then stop
// automatically. No-op if already playing.
//
// NOTE: this no longer requires WiFi. Playback is refused only if the sink
// rejects every frame, which is reported at the end.
esp_err_t mock_playback_start(void);

// Stop playback early. Idempotent.
void mock_playback_stop(void);

bool mock_playback_is_active(void);

#ifdef __cplusplus
}
#endif