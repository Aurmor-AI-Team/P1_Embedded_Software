#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// ble_auth — who is allowed to reconfigure or read from this wearable.
//
// The board advertises whenever it is not in a group session, so without this
// anyone within Bluetooth range could write WiFi credentials (moving the board
// onto their own receiver) or subscribe to the live IMU stream. Neither the
// board's GATT server nor the link itself offers any protection: there is no
// bonding and no encryption.
//
// So the board keeps a secret in NVS and requires a connection to prove it
// knows it before any privileged operation:
//
//   read  AuthNonce  -> 16 fresh random bytes (hardware RNG), hex
//   write AuthResp   <- HMAC-SHA256(secret, nonce), hex
//
// The secret itself never goes back over the air after enrolment, so a passive
// listener gains nothing by watching a normal session.
//
// ENROLMENT is gated by physical possession instead, because a phone that has
// never met this board has no secret to prove: holding BOOT for 3 s opens a
// short window during which the secret may be read out. That is deliberately
// also the lost-phone recovery path — the board hands the SAME secret to
// whoever is standing in front of it, so a replacement phone (or a second
// family member's phone) enrols by pressing the button, while someone merely in
// radio range cannot. An attacker who can hold the button could just pocket the
// wearable instead, so this costs nothing in practice.
//
// An un-enrolled board (no secret yet) leaves everything open: there is nothing
// to protect until someone claims it, and the app enrols during pairing.
//
// MUST stay in sync with the app's features/esp32-provisioning/{protocol,auth}.ts.
// ---------------------------------------------------------------------------

#define AUTH_SECRET_BYTES  16
#define AUTH_NONCE_BYTES   16
#define AUTH_HMAC_BYTES    32

// Load the secret from NVS (if any). Call once at startup.
void ble_auth_init(void);

// True once this board has been claimed. While false every operation is
// permitted — see the header comment.
bool ble_auth_is_enrolled(void);

// Open the enrolment window for `ms` (BOOT long-press). While open,
// ble_auth_read_secret() succeeds; the secret is generated on first use.
void ble_auth_open_window(uint32_t ms);

// True while the enrolment window is open — drives the LED so the user can see
// the board is claimable right now.
bool ble_auth_window_open(void);

// Copy the enrolment secret as lowercase hex into `out` (needs
// AUTH_SECRET_BYTES*2+1 bytes). Fails unless the window is open. Generates and
// persists the secret on first call.
esp_err_t ble_auth_read_secret(char *out, size_t n);

// Produce a fresh challenge as lowercase hex into `out` (needs
// AUTH_NONCE_BYTES*2+1 bytes). Each read yields a new nonce, so a captured
// response cannot be replayed.
esp_err_t ble_auth_make_nonce(char *out, size_t n);

// Verify `hex_response` == HMAC-SHA256(secret, last nonce issued to this
// connection). On success the connection is marked authenticated until it drops.
bool ble_auth_verify(uint16_t conn_handle, const char *hex_response);

// True when `conn_handle` may perform privileged operations: it authenticated,
// or the board has never been enrolled.
bool ble_auth_conn_allowed(uint16_t conn_handle);

// Drop any authentication tied to `conn_handle` (call on disconnect).
void ble_auth_on_disconnect(uint16_t conn_handle);

#ifdef __cplusplus
}
#endif
