#pragma once

#include "esp_err.h"
#include "impact_det.h"   // impact_rec_t
#include "lsm6dsv.h"      // lsm6_sample_t
#include <stdint.h>
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

// How hard we chase the network when we are not on it.
//
// A wearable that has been provisioned retries FOREVER — it must re-join its own
// AP after a power cycle, and the receiver's AP can reject re-auth for minutes
// while a stale association ages out. But "forever" also means a board whose
// receiver is genuinely gone never comes back on Bluetooth, which is the only
// channel the app has left. So the retry has three gears:
//
//   FOREGROUND — the normal provisioned state: reconnect immediately, every
//                time. BLE is off, so the radio is ours alone.
//   BACKGROUND — orphaned: we kept the credentials and still want the network
//                back, but BLE is now advertising so the app can reach us.
//                Retry occasionally instead of continuously, to keep the shared
//                2.4 GHz front end mostly free for BLE.
//   PAUSED     — a phone is streaming from us over BLE right now. Do not touch
//                the WiFi radio at all until it stops; a live session must not
//                be degraded by association attempts for a receiver that is not
//                there.
//
// Has no effect once the credentials are erased (wifi_udp_forget).
typedef enum {
    WIFI_RADIO_FOREGROUND,
    WIFI_RADIO_BACKGROUND,
    WIFI_RADIO_PAUSED,
} wifi_radio_policy_t;

void wifi_udp_set_radio_policy(wifi_radio_policy_t policy);

// Invoked from the UDP receive task when the link has been dead for
// WIFI_ORPHAN_TIMEOUT_MS despite holding credentials — i.e. the receiver is
// gone, not merely slow. Fires once per outage; rearmed when the link recovers.
// The callback must only post an event (it runs on the rx task).
void wifi_udp_set_orphan_cb(void (*cb)(void));

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

// Send one IMU sample as a UDP datagram to the target. No-op (ESP_OK) if not
// connected or no target set. Safe to call from the IMU print task.
esp_err_t wifi_udp_send_imu(const lsm6_sample_t *s);

// As wifi_udp_send_imu, plus 4 mock biometric values (heart rate, SpO2,
// respiration, HRV) the app renders. Used by the CSV mock playback; the real
// sensor path sends 0 for all four via wifi_udp_send_imu.
esp_err_t wifi_udp_send_imu_bio(const lsm6_sample_t *s,
                                float hr, float spo2, float resp, float hrv);

// Send one impact RELIABLY: parked in the pending table, transmitted
// immediately, and retransmitted every ALERT_RETRY_MS until the Pi acks it.
// Unlike the IMU path an impact is sparse and individually meaningful, so it
// must not be a fire-and-forget datagram.
//
// Returns ESP_OK once the record is queued (whether or not the first datagram
// succeeded — the retry sweep covers it), ESP_ERR_INVALID_STATE with no link,
// and ESP_ERR_NO_MEM when the pending table is full, which is the caller's cue
// to hold the record in its own backlog rather than lose it.
esp_err_t wifi_udp_send_alert(const impact_rec_t *r);

// Alerts sent but not yet acknowledged.
uint8_t wifi_udp_alerts_pending(void);

#ifdef __cplusplus
}
#endif
