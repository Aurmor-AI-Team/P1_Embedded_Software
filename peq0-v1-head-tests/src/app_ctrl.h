#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Device mode, driven by the BOOT button, WiFi events, and Pi FORGET packets:
//   IDLE    — unprovisioned, radios quiet. Long-press -> PAIRING.
//   PAIRING — BLE advertising, waiting for the app to write WiFi credentials.
//   WIFI    — on the Pi's network, BLE off. Short-press toggles mock playback;
//             long-press forgets the WiFi and returns to PAIRING;
//             a FORGET from the Pi (app unpair) returns to IDLE.
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
