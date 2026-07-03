#pragma once

#include "esp_err.h"
#include "lsm6dsv.h"   // lsm6_sample_t
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

#ifdef __cplusplus
}
#endif
