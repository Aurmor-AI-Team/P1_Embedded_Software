#pragma once

#include "esp_err.h"
#include "lsm6dsv.h"    // mibs_message
#include "mibs_wire.h"  // mibs_impact_t
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Invoked (from the NimBLE host task) once the app has written SSID, password,
// and the target endpoint. ip is a dotted-quad string, port a UDP port.
typedef void (*ble_provision_cb_t)(const char *ssid, const char *password,
                                   const char *ip, uint16_t port);

// Fills buf with a short human-readable status string for the status
// characteristic. NOTE: buf is BLE_STATUS_MAX bytes, and the *notify* path is
// additionally capped by the negotiated ATT MTU minus 3.
typedef void (*ble_status_getter_t)(char *buf, size_t n);

// Link up/down = a client connected AND subscribed to telemetry, or dropped.
// Runs on the NimBLE host task — post an event, do nothing heavy.
typedef void (*ble_link_cb_t)(bool up);

// App wrote the control characteristic. policy is an app_policy_t; a
// threshold_g of 0 means "leave the threshold alone".
typedef void (*ble_control_cb_t)(uint8_t policy, float threshold_g);

typedef struct {
    ble_provision_cb_t  on_provision;        // SSID+pass+target committed
    void (*on_wearable_id)(uint16_t id);     // wearable ID written (may be NULL)
    void (*on_expected_pi_id)(uint32_t id);  // expected Pi ID written (may be NULL)
    ble_status_getter_t status_getter;       // supplies status string
    ble_link_cb_t       on_link;             // data-link up/down (may be NULL)
    ble_control_cb_t    on_control;          // mode/threshold write (may be NULL)
} ble_provision_cfg_t;

// Status strings outgrew the old 64-byte buffer once mode/threshold/backlog
// were added. Reads are fine at any length (ATT long reads); notifies are
// truncated to MTU-3, which is why the MTU exchange matters.
#define BLE_STATUS_MAX 160

// Bring up NimBLE and advertise the GATT server (as "aurmor-esp32-XXXX",
// suffix from the MAC). Call on demand (BOOT long-press), not at boot.
// Safe to call again after ble_provision_stop().
esp_err_t ble_provision_start(const ble_provision_cfg_t *cfg);

// Tear BLE down completely (stop advertising, drop any connection, stop and
// deinit the NimBLE host + controller). Idempotent.
//
// NOTE: this is now called ONLY on unpair. The BT_* modes need the link to
// survive provisioning, so app_ctrl no longer stops BLE on GOT_IP.
esp_err_t ble_provision_stop(void);

bool ble_provision_is_active(void);     // NimBLE host running
bool ble_provision_is_connected(void);  // connected AND subscribed = usable link

// Push the current status to a subscribed client (call when state changes).
// No-op while BLE is stopped or nobody is subscribed.
void ble_provision_push_status(void);

// --- data path ---------------------------------------------------------------

// Stamped into every outgoing telemetry frame so the receiver can tell live
// data from CSV playback. Takes an app_mode_t.
void ble_provision_set_mode(uint8_t mode);

// Telemetry notification. Signature matches mock_sink_t. Returns an error
// (never blocks) if there is no link, nobody is subscribed, the MTU is too
// small, or the mbuf pool is exhausted — the caller decides what to do.
esp_err_t ble_provision_send_stream(const mibs_message *m, float temp,
                                    float hr, float spo2, float resp, float hrv);

// Alert INDICATION — acknowledged at the ATT layer, unlike telemetry. Only one
// indication may be outstanding at a time; returns ESP_ERR_INVALID_STATE while
// the previous one is unconfirmed so the caller can buffer and retry.
esp_err_t ble_provision_send_alert(const mibs_impact_t *imp);

#ifdef __cplusplus
}
#endif