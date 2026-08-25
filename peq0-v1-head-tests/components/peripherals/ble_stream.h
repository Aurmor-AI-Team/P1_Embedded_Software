#pragma once

#include "esp_err.h"
#include "impact_det.h"   // impact_rec_t
#include "lsm6dsv.h"      // lsm6_sample_t

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// ble_stream — live IMU streaming straight to the phone, speaking the SAME
// binary-v1 GATT contract as the Pi receiver.
//
// A solo session has no receiver in the loop: the app connects to this board
// and reads samples directly. Rather than invent a second wire format, we serve
// the receiver's exact contract (service 5a8e0000-…, Data notify, length-
// prefixed binary-v1 records) so the app's existing decoder, frame recorder and
// live-stats pipeline work against a wearable with no changes.
//
// MUST stay in sync with rpi-receiver/ble-sender/protocol.py and the app's
// features/ble-stream/protocol.ts.
//
// The GATT service itself is registered by ble_provision.cpp (it owns the
// NimBLE lifecycle and the single GATT table); this module owns the encoding,
// the subscription state and the notify cadence.
// ---------------------------------------------------------------------------

// Forward-declared so callers don't have to pull in the NimBLE host headers.
struct ble_gatt_access_ctxt;

// Reset all stream state. Called by ble_provision_start() before the host runs.
// `wid` is the board's wearable id — it becomes the single node label the app
// sees (4 uppercase hex, matching the Pi's f"{wid:04X}" and the serial suffix).
void ble_stream_reset(uint16_t wid);

// GATT access callback for the Data characteristic (notify-only; a plain read
// returns nothing). Wired into the service table in ble_provision.cpp.
int ble_stream_data_access(uint16_t conn, uint16_t attr,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);

// GATT access callback for the Meta characteristic (read -> the descriptor).
int ble_stream_meta_access(uint16_t conn, uint16_t attr,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);

// GATT access callback for the Control characteristic. Accepts the receiver's
// start/stop/restart grammar and ignores it — a wearable streams whenever a
// client is subscribed — so the app can talk to either peer identically.
//
// It DOES act on "mode <idle|live|alerts|mock> [wid]", which is how a solo
// session sets the working mode: the phone is connected straight to this board,
// so there is no receiver to relay through. The optional trailing wid lets the
// app send one identically-shaped string to either peer (the Pi needs it to
// pick a board; we ignore one that isn't ours).
//
// MUST stay in sync with the same grammar in
// rpi-receiver/ble-sender/ble_sender.py (control_write) and the app's
// features/esp32-provisioning/wearableMode.ts.
int ble_stream_control_access(uint16_t conn, uint16_t attr,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

// Called when a client writes "mode <arg>", with everything after the verb.
// Parsing the NAME is the callback's job, not this module's: the name<->enum
// table belongs to app_ctrl, and peripherals must not include src/. Return
// false for an unrecognised mode and the write is rejected, so a typo in the
// app surfaces instead of silently doing nothing.
//
// Runs on the NimBLE host task: parse, post an event, return. Nothing heavy.
void ble_stream_set_mode_cb(bool (*cb)(const char *arg));

// Called from the GAP event handler when a client subscribes to / unsubscribes
// from the Data characteristic. `data_handle` is that characteristic's value
// handle, which NimBLE only fills in once the GATT server starts — passing it
// here avoids any dependency on when that happens. On subscribe this sends
// MSG_META immediately: the app cannot decode a single sample without the
// decode tables.
void ble_stream_on_subscribe(uint16_t conn_handle, uint16_t data_handle,
                             bool enabled);

// Called from the GAP event handler on disconnect.
void ble_stream_on_disconnect(void);

// Called once a connection answers the auth challenge. If it had already
// subscribed, its Meta was suppressed at the time — this re-sends it so the app
// can decode the samples that are about to start. Lets a client subscribe and
// authenticate in either order.
void ble_stream_on_auth(uint16_t conn_handle);

// Queue the decode tables to go out again ahead of the next frame.
//
// Call this whenever a working mode that STREAMS begins (see app_ctrl.h). Meta
// used to be a subscribe-time affair only, which was safe when the board
// streamed continuously: the first sample followed milliseconds later, so any
// gap was invisible. With IDLE as the resting mode, minutes can pass between
// subscribing and the first frame — long enough for the app to have missed,
// dropped, or torn down and rebuilt the listener that was waiting for it. A
// client that resumes without the tables silently discards every sample, which
// looks exactly like a wearable that stopped sending.
//
// Cheap enough not to think about: ~640 bytes, once per mode change.
void ble_stream_request_meta(void);

// True once a client is connected, subscribed, AND allowed to read from us.
// Re-evaluated on every send, so authenticating after subscribing just works.
bool ble_stream_ready(void);

// Notified when a phone starts / stops streaming from us. Lets app_ctrl keep the
// WiFi radio out of the way of a live solo session (see wifi_radio_policy_t).
// The callback runs on the NimBLE host task and must only post an event.
void ble_stream_set_subscriber_cb(void (*cb)(bool streaming));

// Offer one IMU sample to the stream. No-op unless a client is subscribed, and
// internally rate-limited to STREAM_PERIOD_MS, so this is safe to call at the
// full IMU sample rate from the print task.
//
// Mirrors the UDP pair in wifi_udp_tx.h: the plain form is the real-sensor path
// (no biometrics, so those channels go out as zero) and the _bio form carries
// the mock CSV's values.
void ble_stream_notify(const lsm6_sample_t *s);
void ble_stream_notify_bio(const lsm6_sample_t *s,
                           float hr, float spo2, float resp, float hrv);

// Send one detected impact as a MSG_IMPACT record. Unlike the sample path this
// is NOT rate-limited — an impact goes out the moment it is detected — and it
// carries its own fixed field layout, so it stays decodable by a client that
// has not (re-)read Meta.
//
// Returns ESP_ERR_INVALID_STATE when no authenticated subscriber is listening,
// which is the caller's cue to try the other transport or buffer the record.
esp_err_t ble_stream_send_impact(const impact_rec_t *r);

#ifdef __cplusplus
}
#endif
