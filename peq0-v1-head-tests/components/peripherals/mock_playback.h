#pragma once

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Play the embedded HEAD mock CSV on a LOOP, down whichever transport is live
// (UDP to the Pi, and/or the direct BLE stream). Wraps back to row 0 at the end
// rather than stopping: this backs the MOCK working mode, which lasts until the
// user picks another one, so the demo has to outlive one ~72 s pass of the CSV.
// No-op if already playing.
//
// Does NOT require WiFi, and does not refuse when nothing is listening. A
// transport that isn't up simply swallows the frames, which are counted and
// logged — MOCK is a mode the user can select before a link exists, and
// refusing would mean the mode silently didn't take.
//
// Call it through app_ctrl (apply_wmode), never directly: the working mode and
// whether playback is running must not be able to disagree.
esp_err_t mock_playback_start_loop(void);

// Stop playback early. Idempotent.
void mock_playback_stop(void);

bool mock_playback_is_active(void);

#ifdef __cplusplus
}
#endif
