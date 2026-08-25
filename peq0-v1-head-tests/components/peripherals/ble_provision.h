#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Invoked (from the NimBLE host task) once the app has written SSID, password,
// and the target endpoint. ip is a dotted-quad string, port a UDP port.
typedef void (*ble_provision_cb_t)(const char *ssid, const char *password,
                                   const char *ip, uint16_t port);

// Fills buf with a short human-readable status string for the status
// characteristic (e.g. "up ip=192.168.1.42 pi=7 ok").
typedef void (*ble_status_getter_t)(char *buf, size_t n);

// Size of the status buffer. Was 64 until the working mode joined the string,
// which pushed a fully-populated status ("up ip=… wid=… unverified pi=… mode=…")
// past it — and snprintf truncates silently, so the app would have read a status
// with the mode sheared off the end. Reads are ATT long reads and fine at any
// length; a NOTIFY is still capped at MTU-3.
#define BLE_STATUS_MAX 96

typedef struct {
    ble_provision_cb_t  on_provision;        // SSID+pass+target committed
    void (*on_wearable_id)(uint16_t id);     // wearable ID written (may be NULL)
    void (*on_expected_pi_id)(uint32_t id);  // expected Pi ID written (may be NULL)
    ble_status_getter_t status_getter;       // supplies status string
} ble_provision_cfg_t;

// Bring up NimBLE and advertise the provisioning GATT server (as
// "aurmor-mibs-XXXX", suffix from the MAC). Call on demand (BOOT long-press),
// not at boot. Safe to call again after ble_provision_stop().
esp_err_t ble_provision_start(const ble_provision_cfg_t *cfg);

// Tear BLE down completely (stop advertising, drop any connection, stop and
// deinit the NimBLE host + controller). Idempotent.
esp_err_t ble_provision_stop(void);

bool ble_provision_is_active(void);

// Push the current status to a subscribed client (call when state changes).
// No-op while BLE is stopped.
void ble_provision_push_status(void);

#ifdef __cplusplus
}
#endif
