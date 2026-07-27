#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "lsm6dsv.h"    // mibs_message
#include "mibs_wire.h"  // mibs_impact_t

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Device mode = (transport x data policy), plus two standalone states.
//
//   IDLE     — unprovisioned, radios quiet. Long-press -> PAIRING.
//   PAIRING  — BLE advertising, waiting for the app to write WiFi credentials.
//
// Everything else composes a *transport* with a *data policy*:
//
//   transport  follows link state, not the user. WiFi wins when associated
//              with a target set; a connected+subscribed BLE client is the
//              fallback; neither means alerts are buffered in RAM.
//   policy     is set by the BOOT short-press or by the app over the BLE
//              control characteristic, and survives transport changes.
//
//              ALERTS — only >threshold impact records go out (low power)
//              LIVE   — decimated telemetry stream + impact records
//              MOCK   — CSV playback; live telemetry is suppressed
//
// So a short-press in WIFI_ALERTS moves to WIFI_LIVE, and walking out of WiFi
// range moves that to BT_LIVE with no user action.
// ---------------------------------------------------------------------------
typedef enum {
    APP_MODE_IDLE,
    APP_MODE_PAIRING,
    APP_MODE_BT_MOCK,
    APP_MODE_WIFI_MOCK,
    APP_MODE_BT_ALERTS,
    APP_MODE_BT_LIVE,
    APP_MODE_WIFI_ALERTS,
    APP_MODE_WIFI_LIVE
} app_mode_t;

typedef enum {
    APP_XPORT_NONE,
    APP_XPORT_BT,
    APP_XPORT_WIFI,
} app_xport_t;

typedef enum {
    APP_POLICY_ALERTS,
    APP_POLICY_LIVE,
    APP_POLICY_MOCK,
} app_policy_t;

#define APP_IMPACT_THRESHOLD_G_DEFAULT 20.0f
#define APP_IMPACT_THRESHOLD_G_MIN      5.0f
#define APP_IMPACT_THRESHOLD_G_MAX     80.0f

// Wire up the button, forget/link callbacks, IP events, and the mode task.
// Call after wifi_udp_init() (needs the default event loop + NVS restore).
esp_err_t app_ctrl_init(void);

app_mode_t   app_ctrl_mode(void);
app_xport_t  app_ctrl_xport(void);
app_policy_t app_ctrl_policy(void);
const char  *app_mode_str(app_mode_t m);

// Impact threshold in g, persisted in NVS, settable from the app over BLE.
float app_ctrl_threshold_g(void);
void  app_ctrl_set_threshold_g(float g);

void  app_ctrl_set_policy(app_policy_t p);

// --- called from the IMU task and from mock_playback -------------------------

// True only in a LIVE policy, with a live transport, and mock playback stopped.
bool app_ctrl_stream_enabled(void);

// Route one telemetry frame over whichever transport is current. Signature
// matches mock_sink_t so it can be installed directly as the playback sink.
esp_err_t app_ctrl_send_stream(const mibs_message *m, float temp,
                               float hr, float spo2, float resp, float hrv);

// Report a detected impact. Non-blocking: posts to the control queue, which
// dispatches it (or buffers it if no transport is up) and flashes the LED.
void app_ctrl_report_impact(const mibs_impact_t *imp);

#ifdef __cplusplus
}
#endif