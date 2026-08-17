#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Device mode, driven by the BOOT button, WiFi events, and Pi FORGET packets.
//
// BLE is the RESTING state: an unprovisioned board advertises continuously so
// the app can reach it (to pair with it, to stream from it directly in a solo
// session, or to push it onto the receiver's WiFi for a group session) without
// anyone touching the board. WiFi is entered only for the duration of a group
// session, and the app pulls the board back out of it when the session ends.
//
// The BOOT long-press always means "come back towards my phone", and never more
// than that. What it does depends on where the board is, but each is a single
// intent — it takes a SECOND press to make a board claimable:
//
//   on WiFi        -> leave the network and advertise again
//   advertising    -> open the enrolment window (claim me)
//
// Returning to Bluetooth is safe to offer on a button because boards are
// claimed: a board that is merely advertising still refuses credential writes
// and stream subscribes to anyone who cannot answer its challenge (ble_auth.h).
//
//   IDLE    — radios quiet. Only reached when BLE fails to start; a long-press
//             retries. Not part of the normal lifecycle any more.
//   PAIRING — BLE advertising: the provisioning service accepts WiFi
//             credentials, and the stream service serves live IMU samples.
//   WIFI    — on the Pi's network, BLE off. Short-press toggles mock playback;
//             a FORGET from the Pi (app-driven) returns it to PAIRING, as do
//             the orphan timeout below and a long-press.
//
// PAIRING has an ORPHANED sub-state: the receiver stopped answering for 90 s, so
// we came back on BLE (the app's only remaining channel to us) while KEEPING the
// credentials and hunting for that network in the background. It rejoins by
// itself if the receiver returns — no button press, no re-provisioning. The LED
// double-flashes so it's tellable from a never-paired board.
typedef enum {
    APP_MODE_IDLE,
    APP_MODE_PAIRING,
    APP_MODE_WIFI,
} app_mode_t;

// Wire up the button, forget callback, IP events, and the mode task.
// Call after wifi_udp_init() (needs the default event loop + NVS restore).
esp_err_t app_ctrl_init(void);

app_mode_t app_ctrl_mode(void);

#ifdef __cplusplus
}
#endif
