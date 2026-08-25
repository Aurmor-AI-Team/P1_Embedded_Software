#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

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

// ---------------------------------------------------------------------------
// The WORKING MODE — what the user picked in the app.
//
// Orthogonal to app_mode_t above, and deliberately so. app_mode_t is which RADIO
// we are on; this is what we DO with it. The same four modes behave identically
// over BLE (a solo session streaming straight off the board) and over the
// receiver's WiFi (a group session), which is the whole point: the user picks a
// mode, not a transport, and walking between the two changes nothing.
//
//   IDLE   — the default, and truly quiet: no telemetry, no impact records.
//            Detection still runs and impact_det holds what it finds, so
//            leaving IDLE replays the hits that happened while we were silent.
//   LIVE   — real sensors: decimated telemetry (100 Hz UDP / 10 Hz BLE) plus
//            impact records as they happen.
//   ALERTS — the same detector, telemetry off. Nothing is on the wire until
//            something crosses IMPACT_THRESHOLD_G, so the session screen simply
//            updates less often. This is the low-power match mode.
//   MOCK   — the embedded CSV on a loop; the live sensor stream is suppressed
//            so the receiver never sees replay and real samples interleaved.
//            Impacts DO go out: the CSV has them spliced in so a demo shows the
//            impact pipeline working, which is most of the point of a demo.
//
// Not persisted: every board boots into IDLE. A wearable that came back from a
// battery swap already streaming into an empty room is worse than one waiting
// to be told what to do.
// ---------------------------------------------------------------------------
typedef enum {
    WMODE_IDLE   = 0,
    WMODE_LIVE   = 1,
    WMODE_ALERTS = 2,
    WMODE_MOCK   = 3,
} wearable_mode_t;

// Wire up the button, forget callback, IP events, and the mode task.
// Call after wifi_udp_init() (needs the default event loop + NVS restore).
esp_err_t app_ctrl_init(void);

app_mode_t app_ctrl_mode(void);

wearable_mode_t app_ctrl_wearable_mode(void);

// Request a working mode. Safe to call from any task (including the NimBLE host
// task and the UDP rx task): it only posts to the control queue.
void app_ctrl_set_wearable_mode(wearable_mode_t m);

// "idle" | "live" | "alerts" | "mock". The strings are the wire grammar shared
// with the app and the Pi — see ble_stream_control_access() and
// rpi-receiver/ble-sender/ble_sender.py control_write().
const char *app_wmode_str(wearable_mode_t m);
bool        app_wmode_from_str(const char *s, wearable_mode_t *out);

// True only in WMODE_LIVE, with mock playback stopped. The IMU task calls this
// per sample to decide whether to put telemetry on the wire.
bool app_ctrl_stream_enabled(void);

#ifdef __cplusplus
}
#endif
