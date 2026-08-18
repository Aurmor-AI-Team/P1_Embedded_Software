#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Play the embedded HEAD mock CSV once over UDP (52-byte IMU packets at the
// CSV cadence), then stop automatically. No-op if already playing.
// Requires WiFi to be connected and a UDP target provisioned.
esp_err_t mock_playback_start(void);

// Stop playback early. Idempotent.
void mock_playback_stop(void);

bool mock_playback_is_active(void);

#ifdef __cplusplus
}
#endif
