// ---------------------------------------------------------------------------
// ble_auth.cpp — enrolment secret + challenge/response. See ble_auth.h for the
// threat model and why enrolment is gated by the button rather than by a key.
// ---------------------------------------------------------------------------
#include "ble_auth.h"

#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "mbedtls/md.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "ble_auth";

// Same namespace the provisioning values live in (wifi_udp_tx.cpp).
#define NVS_NS   "prov"
#define NVS_KEY  "authsec"

static uint8_t  s_secret[AUTH_SECRET_BYTES];
static bool     s_have_secret = false;

static int64_t  s_window_until_us = 0;

// The challenge is per-connection: two phones connecting at once must not be
// able to answer each other's nonce. One entry is enough — the board accepts a
// single connection at a time (NimBLE is configured for one).
static uint16_t s_nonce_conn = 0xFFFF;
static uint8_t  s_nonce[AUTH_NONCE_BYTES];
static bool     s_have_nonce = false;

static uint16_t s_authed_conn = 0xFFFF;

// --- helpers ---------------------------------------------------------------

static void to_hex(const uint8_t *in, size_t n, char *out)
{
    static const char *digits = "0123456789abcdef";
    for (size_t i = 0; i < n; i++) {
        out[i * 2]     = digits[in[i] >> 4];
        out[i * 2 + 1] = digits[in[i] & 0x0F];
    }
    out[n * 2] = '\0';
}

/** Parse exactly `n` bytes of hex. Returns false on any malformed input. */
static bool from_hex(const char *in, uint8_t *out, size_t n)
{
    if (!in || strlen(in) != n * 2) return false;
    for (size_t i = 0; i < n * 2; i++) {
        const char c = in[i];
        uint8_t v;
        if      (c >= '0' && c <= '9') v = (uint8_t)(c - '0');
        else if (c >= 'a' && c <= 'f') v = (uint8_t)(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') v = (uint8_t)(c - 'A' + 10);
        else return false;
        if (i & 1) out[i / 2] |= v;
        else       out[i / 2]  = (uint8_t)(v << 4);
    }
    return true;
}

/** Compare without an early exit, so timing can't leak how much matched. */
static bool const_time_eq(const uint8_t *a, const uint8_t *b, size_t n)
{
    uint8_t diff = 0;
    for (size_t i = 0; i < n; i++) diff |= (uint8_t)(a[i] ^ b[i]);
    return diff == 0;
}

static void persist_secret(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed — secret not persisted");
        return;
    }
    nvs_set_blob(h, NVS_KEY, s_secret, sizeof(s_secret));
    nvs_commit(h);
    nvs_close(h);
}

// --- public ---------------------------------------------------------------

void ble_auth_init(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        size_t len = sizeof(s_secret);
        if (nvs_get_blob(h, NVS_KEY, s_secret, &len) == ESP_OK && len == sizeof(s_secret)) {
            s_have_secret = true;
        }
        nvs_close(h);
    }
    ESP_LOGI(TAG, "%s", s_have_secret ? "enrolled (secret loaded)"
                                      : "not enrolled — open until claimed");
}

bool ble_auth_is_enrolled(void) { return s_have_secret; }

void ble_auth_open_window(uint32_t ms)
{
    s_window_until_us = esp_timer_get_time() + (int64_t)ms * 1000;
    ESP_LOGI(TAG, "enrolment window open for %lu s", (unsigned long)(ms / 1000));
}

void ble_auth_close_window(void)
{
    s_window_until_us = 0;
}

bool ble_auth_window_open(void)
{
    return esp_timer_get_time() < s_window_until_us;
}

esp_err_t ble_auth_read_secret(char *out, size_t n)
{
    if (!out || n < AUTH_SECRET_BYTES * 2 + 1) return ESP_ERR_INVALID_ARG;

    // The window guards RE-claiming, not the first claim. An unclaimed board
    // accepts every privileged write anyway (ble_auth_conn_allowed returns true
    // while there is no secret), so demanding a button press before handing out
    // a secret it would otherwise let anyone bypass entirely protects nothing —
    // it just makes a factory-fresh board impossible to pair without knowing to
    // hold BOOT, even though it is sitting there advertising.
    if (s_have_secret && !ble_auth_window_open()) {
        ESP_LOGW(TAG, "enrol refused — already claimed; hold BOOT 3 s to re-claim");
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_have_secret) {
        // First claim: mint one from the hardware RNG and keep it for the life
        // of the board, so later enrolments (a second phone, a replacement
        // phone) get the same secret and don't evict anyone.
        esp_fill_random(s_secret, sizeof(s_secret));
        s_have_secret = true;
        persist_secret();
        ESP_LOGI(TAG, "enrolled — secret generated");
    } else {
        ESP_LOGI(TAG, "re-enrolled from the open window");
    }
    // Claimed — so stop advertising claimability. The press meant "this phone
    // may have me", not "anyone may have me for the next two minutes", and a
    // window left open is a 2 Hz LED still asking to be claimed by a board that
    // already has been. A second phone gets a second press.
    ble_auth_close_window();
    to_hex(s_secret, sizeof(s_secret), out);
    return ESP_OK;
}

esp_err_t ble_auth_make_nonce(char *out, size_t n)
{
    if (!out || n < AUTH_NONCE_BYTES * 2 + 1) return ESP_ERR_INVALID_ARG;
    esp_fill_random(s_nonce, sizeof(s_nonce));
    s_have_nonce = true;
    to_hex(s_nonce, sizeof(s_nonce), out);
    return ESP_OK;
}

bool ble_auth_verify(uint16_t conn_handle, const char *hex_response)
{
    if (!s_have_secret || !s_have_nonce) return false;

    uint8_t given[AUTH_HMAC_BYTES];
    if (!from_hex(hex_response, given, sizeof(given))) {
        ESP_LOGW(TAG, "auth response malformed");
        return false;
    }

    uint8_t expect[AUTH_HMAC_BYTES];
    const mbedtls_md_info_t *md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!md) return false;
    if (mbedtls_md_hmac(md, s_secret, sizeof(s_secret),
                        s_nonce, sizeof(s_nonce), expect) != 0) {
        ESP_LOGE(TAG, "hmac failed");
        return false;
    }

    // One shot per nonce: burn it whether or not the answer was right, so a
    // wrong guess can't be followed by another against the same challenge.
    s_have_nonce = false;

    if (!const_time_eq(given, expect, sizeof(expect))) {
        ESP_LOGW(TAG, "auth rejected");
        return false;
    }
    s_authed_conn = conn_handle;
    s_nonce_conn  = conn_handle;
    ESP_LOGI(TAG, "connection %u authenticated", conn_handle);
    return true;
}

bool ble_auth_conn_allowed(uint16_t conn_handle)
{
    if (!s_have_secret) return true;   // nobody has claimed this board yet
    return conn_handle != 0xFFFF && conn_handle == s_authed_conn;
}

void ble_auth_on_disconnect(uint16_t conn_handle)
{
    if (s_authed_conn == conn_handle) s_authed_conn = 0xFFFF;
    if (s_nonce_conn == conn_handle) {
        s_nonce_conn = 0xFFFF;
        s_have_nonce = false;
    }
}
