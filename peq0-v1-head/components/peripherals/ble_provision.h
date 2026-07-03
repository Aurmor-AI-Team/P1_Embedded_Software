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

typedef struct {
    ble_provision_cb_t  on_provision;        // SSID+pass+target committed
    void (*on_wearable_id)(uint16_t id);     // wearable ID written (may be NULL)
    void (*on_expected_pi_id)(uint32_t id);  // expected Pi ID written (may be NULL)
    ble_status_getter_t status_getter;       // supplies status string
} ble_provision_cfg_t;

// Start the BLE GATT server used to provision Wi-Fi, UDP endpoint, and IDs.
esp_err_t ble_provision_init(const ble_provision_cfg_t *cfg);

// Push the current status to a subscribed client (call when state changes).
void ble_provision_push_status(void);

#ifdef __cplusplus
}
#endif
