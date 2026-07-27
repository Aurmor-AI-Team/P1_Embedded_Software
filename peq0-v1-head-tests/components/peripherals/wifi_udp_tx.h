#pragma once

#include "esp_err.h"
#include "lsm6dsv.h"    // lsm6_sample_t, mibs_message
#include "mibs_wire.h"  // mibs_impact_t
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Bring up Wi-Fi in station mode (does not connect yet), open + bind the UDP
// socket, and start the receive/handshake task. Auto-connects if a profile
// was previously saved to NVS.
esp_err_t wifi_udp_init(void);

// (Re)connect to an access point. Credentials are persisted to NVS.
esp_err_t wifi_udp_connect(const char *ssid, const char *password);

// True when WiFi credentials are stored (provisioned, or restored from NVS).
bool wifi_udp_has_creds(void);

// "Unpair": erase all persisted provisioning (credentials, target, IDs),
// disconnect from WiFi, and stop the auto-reconnect loop.
esp_err_t wifi_udp_forget(void);

// Invoked from the UDP receive task when the Pi sends a valid FORGET packet.
// The callback must only post an event (it runs on the rx task).
void wifi_udp_set_forget_cb(void (*cb)(void));

// Invoked when the station associates/disassociates, so the controller can
// fail transport over to BLE. Fires on WIFI_EVENT_STA_DISCONNECTED, which is
// seconds earlier than IP_EVENT_STA_LOST_IP — that gap is dropped alerts.
// Runs on the system event task: post an event, do nothing heavy.
void wifi_udp_set_link_cb(void (*cb)(bool up));

// Set the UDP destination (dotted-quad IP + port). Persisted to NVS.
esp_err_t wifi_udp_set_target(const char *ip, uint16_t port);

// Identity stamped into every outgoing packet. Persisted to NVS. If never
// set, defaults to the low 16 bits of the Wi-Fi MAC.
esp_err_t wifi_udp_set_wearable_id(uint16_t id);
uint16_t  wifi_udp_get_wearable_id(void);

// If non-zero, the link is only "verified" when the Pi that answers reports
// this exact Pi ID. Zero (default) accepts any Pi. Persisted to NVS.
esp_err_t wifi_udp_set_expected_pi_id(uint32_t pi_id);

// Connection state.
bool wifi_udp_is_connected(void);   // associated + got DHCP IP
bool wifi_udp_has_target(void);     // destination provisioned
bool wifi_udp_is_verified(void);    // a matching WELCOME arrived recently
uint32_t wifi_udp_get_pi_id(void);  // Pi ID from the last WELCOME (0 = none)

// Copy this device's current IP ("192.168.x.y" or "0.0.0.0") into buf.
void wifi_udp_get_ip(char *buf, size_t n);

// Stamped into every outgoing IMU packet so the Pi can tell a live stream from
// CSV mock playback. Takes an app_mode_t.
void wifi_udp_set_mode(uint8_t mode);

// Send one telemetry frame. Returns ESP_ERR_INVALID_STATE when there is no
// link or no target — callers need to distinguish "sent" from "dropped", so
// this no longer reports success in that case.
esp_err_t wifi_udp_send_imu(const mibs_message *m);

// As wifi_udp_send_imu, plus 4 mock biometric values (heart rate, SpO2,
// respiration, HRV) the app renders. Signature matches mock_sink_t.
esp_err_t wifi_udp_send_imu_bio(const mibs_message *m, float temp,
                                float hr, float spo2, float resp, float hrv);

// Send an impact alert RELIABLY: queued, transmitted immediately, and
// retransmitted from the rx task until the Pi returns a matching ALERT_ACK or
// the retry budget is exhausted. Returns ESP_OK once accepted for delivery,
// ESP_ERR_INVALID_STATE with no link, ESP_ERR_NO_MEM if the pending queue is
// full — in the failure cases the caller must buffer it itself.
esp_err_t wifi_udp_send_alert(const mibs_impact_t *imp);

// Number of alerts sent but not yet acknowledged.
uint8_t wifi_udp_alerts_pending(void);

#ifdef __cplusplus
}
#endif