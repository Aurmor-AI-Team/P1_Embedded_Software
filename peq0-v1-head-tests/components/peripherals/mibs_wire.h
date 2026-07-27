#pragma once
// ---------------------------------------------------------------------------
// mibs_wire.h — on-air/on-wire message formats, shared by BOTH transports.
//
// wifi_udp_tx and ble_provision emit byte-identical payloads so the Pi and the
// phone app can share one parser. Previously these structs lived inside
// wifi_udp_tx.cpp; they moved here when BLE became a data path.
//
// All messages share a 4-byte header:
//   uint8  msg_type
//   uint8  version    (MIBS_MSG_VERSION)
//   uint16 wearable_id
//
// VERSION 2 CHANGES (coordinate with the Pi and the app before flashing):
//   - IMU packet gained a trailing `mode` byte so the receiver can tell a live
//     stream from CSV mock playback. Appended at the end, so a v1 parser
//     reading fixed offsets still gets the same prefix.
//   - Added MSG_ALERT (wearable -> Pi) and MSG_ALERT_ACK (Pi -> wearable).
//     Alerts are retransmitted until acked; the Pi MUST reply to every alert
//     or the wearable will keep resending it.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include "lsm6dsv.h"   // mibs_message

#ifdef __cplusplus
extern "C" {
#endif

#define MIBS_MSG_VERSION    2

#define MIBS_MSG_IMU        1
#define MIBS_MSG_HELLO      2
#define MIBS_MSG_WELCOME    3
#define MIBS_MSG_FORGET     4
#define MIBS_MSG_ALERT      5   // wearable -> Pi/app
#define MIBS_MSG_ALERT_ACK  6   // Pi -> wearable, keyed on alert seq

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;
    uint8_t  version;
    uint16_t wearable_id;
} mibs_hdr_t;

// 49 bytes. Byte-for-byte identical to the v1 layout plus a trailing `mode`.
typedef struct __attribute__((packed)) {
    mibs_hdr_t hdr;
    uint32_t   seq;
    uint32_t   t_ms;
    uint32_t   impact_count;
    float      impact_threshold;
    float      impact_accumulator;
    float      all_time_peak_g;
    float      temp_c;
    float      hr, spo2, resp, hrv;   // 0 from the real IMU; mock fills these
    uint8_t    mode;                  // app_mode_t — NEW in v2
} mibs_imu_pkt_t;

// 48 bytes — fits one BLE notification at MTU >= 51.
typedef struct __attribute__((packed)) {
    mibs_hdr_t hdr;
    uint32_t   seq;          // alert sequence, monotonic, ack key
    uint32_t   t_ms;
    float      peak_g;
    float      threshold_g;
    float      hx_g, hy_g, hz_g;
    float      gx_dps, gy_dps, gz_dps;
    uint16_t   dur_ms;
    uint8_t    mode;         // app_mode_t at detection time
    uint8_t    xport;        // app_xport_t used to send (0 = was buffered)
} mibs_alert_pkt_t;

typedef struct __attribute__((packed)) {
    mibs_hdr_t hdr;
    uint32_t   seq;          // echoes the alert being acknowledged
} mibs_alert_ack_pkt_t;

typedef struct __attribute__((packed)) {
    mibs_hdr_t hdr;
    uint32_t   nonce;
} mibs_hello_pkt_t;

typedef struct __attribute__((packed)) {
    mibs_hdr_t hdr;
    uint32_t   nonce;
    uint32_t   pi_id;
} mibs_welcome_pkt_t;

typedef struct __attribute__((packed)) {
    mibs_hdr_t hdr;
    uint32_t   pi_id;
} mibs_forget_pkt_t;

// ---------------------------------------------------------------------------
// Host-side impact record. Passed by pointer between app_ctrl and the
// transports; each transport serialises it into mibs_alert_pkt_t.
// ---------------------------------------------------------------------------
typedef struct {
    uint32_t seq;
    int64_t  t_us;
    float    peak_g;
    float    threshold_g;
    float    hx_g, hy_g, hz_g;
    float    gx_dps, gy_dps, gz_dps;
    uint16_t dur_ms;
    uint8_t  mode;
    uint8_t  xport;
} mibs_impact_t;

#ifdef __cplusplus
}
#endif