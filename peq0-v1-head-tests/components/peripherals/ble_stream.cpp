// ---------------------------------------------------------------------------
// ble_stream.cpp — binary-v1 IMU stream over BLE, byte-compatible with the Pi.
//
// Wire format (mirrors rpi-receiver/ble-sender/protocol.py; the app's decoder
// lives in aurmor-sports-mobile/features/ble-stream/protocol.ts):
//
//   record  = msg_type:u8 | length:u16 LE | payload[length]
//             then split into <= chunk_size notifications.
//   MSG_META(3)   payload = the JSON descriptor, UTF-8. Carries the decode
//                 tables; the app cannot decode a sample before it arrives, so
//                 it is sent the moment a client subscribes.
//   MSG_SAMPLE(1) payload = node_idx:u8 | t_ms:u32 | fields in layout order,
//                 each packed per its (type, scale) from field_specs.
//
// We publish the same field set the Pi does for a live wearable
// (LIVE_IMU_FIELDS + LIVE_BIO_FIELDS): 10 IMU values plus the 4 biometric
// channels. The bio channels are zero here for exactly the same reason they are
// zero over UDP — a real head sensor has no biometrics — but they stay in the
// layout so the app's `deriveLiveStats` sees an identical shape either way.
// ---------------------------------------------------------------------------
#include "ble_stream.h"

#include "ble_auth.h"
#include "impact_det.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_stream";

// Match the Pi's live cadence (ble_sender.py --live-period-ms default) rather
// than the IMU's own rate: the app renders ~4 Hz and a faster stream only costs
// airtime and battery.
#define STREAM_PERIOD_MS  100

// Safe notification payload after MTU negotiation (notification <= MTU - 3).
// Same default the Pi uses; clamped to the real MTU at send time.
#define CHUNK_SIZE        180

#define MSG_SAMPLE  1
#define MSG_META    3
// One discrete head impact. Fixed 21-byte layout, deliberately NOT described in
// the Meta field_specs: an impact must stay decodable by a client that
// reconnected and has not re-read Meta yet, and keeping it out of the layout is
// what lets the pinned Meta/sample fixtures stay byte-identical.
#define MSG_IMPACT  4
#define IMPACT_PAYLOAD_LEN 21

// Application-error ATT code for "refused". Mirrors the definition and the long
// rationale in ble_provision.cpp — never BLE_ATT_ERR_INSUFFICIENT_AUTHEN, which
// makes the central try to bond against a server with no security manager and
// drops the link.
#define ATT_ERR_NOT_AUTHORIZED 0x80

// 1 = NDJSON, 2 = binary-v1. Must equal protocol.py's SCHEMA_VERSION.
#define SCHEMA_VERSION  2

static uint16_t s_data_handle = 0;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile bool s_subscribed = false;
static int64_t  s_last_send_us = 0;
static volatile bool s_meta_pending = false;
static void (*s_subscriber_cb)(bool streaming) = NULL;
static bool (*s_mode_cb)(const char *arg) = NULL;

// send_record() is called from TWO tasks: the NimBLE host task (Meta, on
// subscribe) and the IMU task (samples and impacts). It emits one record as N
// chunked notifications, so two interleaved calls splice their chunks together
// and permanently desync the app's BinaryFrameAssembler — it reads a length,
// then consumes the wrong bytes for it. Serialise the whole record.
static SemaphoreHandle_t s_tx_lock = NULL;

// The node label the app sees: the wid as 4 uppercase hex digits, matching the
// Pi's f"{wid:04X}" and the board's own serial suffix. Attribution therefore
// works out identically whether a sample arrived via the receiver or directly.
static char s_node[5] = "0000";

// The descriptor is fixed for a single-node board, so it is built once at reset
// and reused. ~640 bytes; sized with headroom.
static char s_meta[832];
static size_t s_meta_len = 0;

// ---------------------------------------------------------------------------
// binary-v1 packing
// ---------------------------------------------------------------------------

static inline size_t put_u8(uint8_t *out, size_t off, uint8_t v)
{
    out[off] = v;
    return off + 1;
}

static inline size_t put_u16(uint8_t *out, size_t off, uint16_t v)
{
    out[off]     = (uint8_t)(v & 0xFF);
    out[off + 1] = (uint8_t)(v >> 8);
    return off + 2;
}

static inline size_t put_u32(uint8_t *out, size_t off, uint32_t v)
{
    out[off]     = (uint8_t)(v & 0xFF);
    out[off + 1] = (uint8_t)((v >> 8) & 0xFF);
    out[off + 2] = (uint8_t)((v >> 16) & 0xFF);
    out[off + 3] = (uint8_t)((v >> 24) & 0xFF);
    return off + 4;
}

/** Pack a float as int16 at `scale`, clamped — the i16 half of _pack_value(). */
static inline size_t put_scaled_i16(uint8_t *out, size_t off, float v, float scale)
{
    float n = v * scale;
    n = (n < -32768.0f) ? -32768.0f : (n > 32767.0f) ? 32767.0f : n;
    int32_t r = (int32_t)(n < 0 ? n - 0.5f : n + 0.5f);   // round-half-away, as Python's round()
    return put_u16(out, off, (uint16_t)(int16_t)r);
}

/** Pack a float as uint16 at `scale`, clamped — the u16 half of _pack_value(). */
static inline size_t put_scaled_u16(uint8_t *out, size_t off, float v, float scale)
{
    float n = v * scale;
    n = (n < 0.0f) ? 0.0f : (n > 65535.0f) ? 65535.0f : n;
    return put_u16(out, off, (uint16_t)(n + 0.5f));
}

/** Pack a float as uint32 at `scale`, clamped. Double maths: a float cannot
 *  represent the top of the u32 range to 0.01 g, which is what sum_g needs. */
static inline size_t put_scaled_u32(uint8_t *out, size_t off, float v, float scale)
{
    double n = (double)v * (double)scale;
    n = (n < 0.0) ? 0.0 : (n > 4294967295.0) ? 4294967295.0 : n;
    return put_u32(out, off, (uint32_t)(n + 0.5));
}

/** Build the JSON descriptor once. Only the node label varies per board. */
static void build_meta(void)
{
    s_meta_len = (size_t)snprintf(
        s_meta, sizeof(s_meta),
        "{\"exercise\":\"live-ble\","
        "\"period_ms\":%d,"
        "\"fps\":%.2f,"
        "\"frames\":0,"
        "\"nodes\":[\"%s\"],"
        "\"chunk_size\":%d,"
        "\"framing\":\"binary-v1\","
        "\"schema\":%d,"
        "\"field_specs\":{"
        "\"ax_g\":[\"i16\",1000],\"ay_g\":[\"i16\",1000],\"az_g\":[\"i16\",1000],"
        "\"gx_dps\":[\"i16\",10],\"gy_dps\":[\"i16\",10],\"gz_dps\":[\"i16\",10],"
        "\"hx_g\":[\"i16\",1000],\"hy_g\":[\"i16\",1000],\"hz_g\":[\"i16\",1000],"
        "\"imu_temp_c\":[\"i16\",100],"
        "\"ecg_hr_bpm\":[\"u16\",1],\"ppg_spo2_pct\":[\"u16\",100],"
        "\"resp_rate_bpm\":[\"u16\",1],\"ecg_rmssd_ms\":[\"u16\",1]},"
        "\"layouts\":[[\"ax_g\",\"ay_g\",\"az_g\","
        "\"gx_dps\",\"gy_dps\",\"gz_dps\","
        "\"hx_g\",\"hy_g\",\"hz_g\",\"imu_temp_c\","
        "\"ecg_hr_bpm\",\"ppg_spo2_pct\",\"resp_rate_bpm\",\"ecg_rmssd_ms\"]],"
        "\"node_layout\":[0]}",
        STREAM_PERIOD_MS, 1000.0 / STREAM_PERIOD_MS, s_node, CHUNK_SIZE,
        SCHEMA_VERSION);

    if (s_meta_len >= sizeof(s_meta)) {
        // Truncated: the app would fail to parse it and never decode a sample.
        ESP_LOGE(TAG, "meta descriptor truncated (%u >= %u)",
                 (unsigned)s_meta_len, (unsigned)sizeof(s_meta));
        s_meta_len = 0;
    }
}

// ---------------------------------------------------------------------------
// sending
// ---------------------------------------------------------------------------

/** Frame `payload` as a binary-v1 record and notify it in <=chunk_size pieces.
 *  Returns true only if the WHOLE record went out — callers that owe the app
 *  something (Meta) must know when they still owe it. */
static bool send_record(uint8_t msg_type, const uint8_t *payload, size_t len)
{
    // Single gate for everything that leaves this module, Meta included — an
    // unauthenticated peer must not learn the board's node id or field layout
    // either, and suppressing Meta here is what makes the re-send in
    // ble_stream_on_auth() the one place that decides it is finally allowed.
    if (!ble_stream_ready()) return false;

    // Notifications must fit the negotiated ATT MTU (minus the 3-byte opcode +
    // handle). Never assume the 247 we ask for — iOS lands around 185.
    uint16_t mtu = ble_att_mtu(s_conn_handle);
    size_t chunk = CHUNK_SIZE;
    if (mtu > 3 && (size_t)(mtu - 3) < chunk) chunk = (size_t)(mtu - 3);

    uint8_t header[3];
    header[0] = msg_type;
    header[1] = (uint8_t)(len & 0xFF);
    header[2] = (uint8_t)((len >> 8) & 0xFF);

    // Hold the lock across the WHOLE record — see s_tx_lock. Short timeout and
    // drop rather than block: this runs on the IMU task, which must not stall
    // behind a busy host task (a stalled IMU task overruns the sample queue and
    // can lose an impact's true peak).
    if (s_tx_lock && xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "tx busy — dropped a type-%u record", (unsigned)msg_type);
        return false;
    }

    // The record is header + payload as one byte stream, chunked without regard
    // for the boundary — the app's BinaryFrameAssembler reassembles by length.
    uint8_t buf[CHUNK_SIZE];
    size_t sent = 0;
    const size_t total = sizeof(header) + len;
    while (sent < total) {
        size_t n = total - sent;
        if (n > chunk) n = chunk;
        for (size_t i = 0; i < n; i++) {
            size_t pos = sent + i;
            buf[i] = pos < sizeof(header) ? header[pos] : payload[pos - sizeof(header)];
        }
        struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, n);
        // Bailing out mid-record leaves a TRUNCATED record on the wire, and the
        // assembler then mis-frames everything after it. We cannot un-send what
        // already went, so the recovery is to make the app resynchronise: it
        // discards bytes until a record parses, and a fresh Meta re-anchors it.
        if (!om || ble_gatts_notify_custom(s_conn_handle, s_data_handle, om) != 0) {
            if (sent > 0) {
                ESP_LOGE(TAG, "truncated type-%u record at %u/%u — queueing Meta resync",
                         (unsigned)msg_type, (unsigned)sent, (unsigned)total);
                s_meta_pending = true;
            }
            break;
        }
        sent += n;
    }

    if (s_tx_lock) xSemaphoreGive(s_tx_lock);
    return sent == total;
}

/** Returns true if the decode tables actually reached the app. */
static bool send_meta(void)
{
    if (s_meta_len == 0) {
        ESP_LOGE(TAG, "no meta to send — the app can decode nothing");
        return false;
    }
    const bool ok = send_record(MSG_META, (const uint8_t *)s_meta, s_meta_len);
    // Worth a line every time. Meta is the difference between a stream the app
    // renders and one it silently discards, and when it goes missing there is
    // otherwise nothing on either side that says so.
    if (ok) ESP_LOGI(TAG, "meta sent (%u bytes)", (unsigned)s_meta_len);
    else    ESP_LOGW(TAG, "meta NOT sent (link not ready?) — still owed");
    return ok;
}

// ---------------------------------------------------------------------------
// public API
// ---------------------------------------------------------------------------

void ble_stream_reset(uint16_t wid)
{
    if (!s_tx_lock) s_tx_lock = xSemaphoreCreateMutex();
    s_conn_handle  = BLE_HS_CONN_HANDLE_NONE;
    s_subscribed   = false;
    s_last_send_us = 0;
    s_meta_pending = false;
    snprintf(s_node, sizeof(s_node), "%04X", wid);
    build_meta();
}

bool ble_stream_ready(void)
{
    // Authentication is re-checked HERE, on every send, rather than latched when
    // the client subscribed. A client is free to enable notifications before it
    // answers the challenge — ours does not, but GATT gives no ordering
    // guarantee and stacks pipeline these — and deciding once at subscribe time
    // meant such a client got silence forever with nothing to recover it.
    // Evaluating it live makes the order irrelevant: samples simply start
    // flowing the moment the connection becomes authenticated.
    return s_subscribed && s_conn_handle != BLE_HS_CONN_HANDLE_NONE
           && s_data_handle != 0
           && ble_auth_conn_allowed(s_conn_handle);
}

void ble_stream_on_auth(uint16_t conn_handle)
{
    if (!s_subscribed || conn_handle != s_conn_handle) return;
    // Subscribed before authenticating, so the Meta that normally accompanies a
    // subscribe was suppressed. The app drops every sample it has no decode
    // tables for, so it has to be re-sent — but NOT from here: this runs inside
    // the GATT write callback for the auth response, and notifying before that
    // write has been answered means re-entering the ATT layer. Flag it and let
    // the next sample carry it, on the task that normally sends.
    ESP_LOGI(TAG, "authenticated after subscribing — Meta queued");
    s_meta_pending = true;
    s_last_send_us = 0;
}

void ble_stream_set_subscriber_cb(void (*cb)(bool streaming)) { s_subscriber_cb = cb; }

void ble_stream_request_meta(void)
{
    // Deliberately queued rather than sent: this is called from app_ctrl's task
    // on a mode change, and the send belongs on the task that normally sends.
    s_meta_pending = true;
    s_last_send_us = 0;   // don't make the first frame of the new mode wait
}

void ble_stream_on_subscribe(uint16_t conn_handle, uint16_t data_handle, bool enabled)
{
    const bool was = s_subscribed;
    s_data_handle = data_handle;
    s_conn_handle = enabled ? conn_handle : BLE_HS_CONN_HANDLE_NONE;
    s_subscribed  = enabled;
    if (was != enabled && s_subscriber_cb) s_subscriber_cb(enabled);
    if (!enabled) return;

    ESP_LOGI(TAG, "client subscribed; streaming node \"%s\" at %d ms", s_node,
             STREAM_PERIOD_MS);
    // Meta first, always: the app holds every sample it can't decode. If this
    // connection hasn't authenticated yet, or the tx lock is busy, send_meta()
    // reports the failure and we stay in debt — the next send settles it.
    //
    // The debt MUST outlive a failed attempt. Clearing the flag before trying
    // (as this did) meant a Meta dropped here was forgotten, and the app then
    // discarded every sample for the life of the connection with nothing in the
    // log to say why.
    s_meta_pending = !send_meta();
    s_last_send_us = 0;   // let the next sample through immediately
}

void ble_stream_on_disconnect(void)
{
    const bool was = s_subscribed;
    s_conn_handle  = BLE_HS_CONN_HANDLE_NONE;
    s_subscribed   = false;
    s_meta_pending = false;
    // A dropped link ends the session just as surely as an unsubscribe, and is
    // the far more common way one ends. Without this the WiFi radio would stay
    // paused for a phone that walked away.
    if (was && s_subscriber_cb) s_subscriber_cb(false);
}

void ble_stream_notify(const lsm6_sample_t *s)
{
    // Real sensor path: no biometric channels (same as wifi_udp_send_imu).
    ble_stream_notify_bio(s, 0.0f, 0.0f, 0.0f, 0.0f);
}

void ble_stream_notify_bio(const lsm6_sample_t *s,
                           float hr, float spo2, float resp, float hrv)
{
    if (!s || !ble_stream_ready()) return;

    // Decode tables owed from an authenticate-after-subscribe (ble_stream_on_auth).
    if (s_meta_pending) {
        s_meta_pending = false;
        send_meta();
    }

    int64_t now_us = esp_timer_get_time();
    if (s_last_send_us != 0 && (now_us - s_last_send_us) < (STREAM_PERIOD_MS * 1000)) {
        return;
    }
    s_last_send_us = now_us;

    // node_idx:u8 | t_ms:u32 | 10 IMU i16 | 4 bio u16  == 33 bytes.
    uint8_t payload[33];
    size_t off = 0;
    off = put_u8(payload, off, 0);                                  // single node
    off = put_u32(payload, off, (uint32_t)(now_us / 1000));         // t_s * 1000

    off = put_scaled_i16(payload, off, s->ax_g, 1000.0f);
    off = put_scaled_i16(payload, off, s->ay_g, 1000.0f);
    off = put_scaled_i16(payload, off, s->az_g, 1000.0f);
    off = put_scaled_i16(payload, off, s->gx_dps, 10.0f);
    off = put_scaled_i16(payload, off, s->gy_dps, 10.0f);
    off = put_scaled_i16(payload, off, s->gz_dps, 10.0f);
    off = put_scaled_i16(payload, off, s->hx_g, 1000.0f);
    off = put_scaled_i16(payload, off, s->hy_g, 1000.0f);
    off = put_scaled_i16(payload, off, s->hz_g, 1000.0f);
    off = put_scaled_i16(payload, off, s->temp_c, 100.0f);          // -> imu_temp_c

    // Biometrics: zero from a real sensor, filled by the mock. Scales must
    // match _LIVE_BIO_SPECS in protocol.py.
    off = put_scaled_u16(payload, off, hr,   1.0f);     // ecg_hr_bpm
    off = put_scaled_u16(payload, off, spo2, 100.0f);   // ppg_spo2_pct
    off = put_scaled_u16(payload, off, resp, 1.0f);     // resp_rate_bpm
    off = put_scaled_u16(payload, off, hrv,  1.0f);     // ecg_rmssd_ms

    send_record(MSG_SAMPLE, payload, off);
}

esp_err_t ble_stream_send_impact(const impact_rec_t *r)
{
    if (!r || !ble_stream_ready()) return ESP_ERR_INVALID_STATE;

    // Same debt the sample path settles: an impact that arrives before the
    // decode tables is undecodable and the app drops it on the floor.
    if (s_meta_pending) {
        s_meta_pending = false;
        send_meta();
    }

    // node_idx:u8 | seq:u16 | t_ms:u32 | peak,thresh:u16 | dur:u16 |
    // count:u16 | max:u16 | sum:u32  == 21 bytes.
    uint8_t payload[IMPACT_PAYLOAD_LEN];
    size_t off = 0;
    off = put_u8(payload, off, 0);                                   // single node
    off = put_u16(payload, off, (uint16_t)(r->seq & 0xFFFF));
    off = put_u32(payload, off, r->t_ms);
    off = put_scaled_u16(payload, off, r->peak_g, 100.0f);
    off = put_scaled_u16(payload, off, r->threshold_g, 100.0f);
    off = put_u16(payload, off, r->dur_ms);
    off = put_u16(payload, off, (uint16_t)(r->count > 65535 ? 65535 : r->count));
    off = put_scaled_u16(payload, off, r->max_g, 100.0f);
    off = put_scaled_u32(payload, off, r->sum_g, 100.0f);

    // Deliberately does NOT touch s_last_send_us: the STREAM_PERIOD_MS limiter
    // lives in notify_bio, not in send_record, so an impact goes out the moment
    // it is detected without stalling or being stalled by the sample cadence.
    send_record(MSG_IMPACT, payload, off);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// GATT access callbacks (the service table lives in ble_provision.cpp)
// ---------------------------------------------------------------------------

int ble_stream_data_access(uint16_t conn, uint16_t attr,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn; (void)attr; (void)arg;
    // Notify-only. A read returns an empty value rather than an error so a
    // generic GATT client poking at it doesn't see a failure.
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) return 0;
    return BLE_ATT_ERR_UNLIKELY;
}

int ble_stream_meta_access(uint16_t conn, uint16_t attr,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn; (void)attr; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    // Served for parity with the receiver; the app reads the descriptor off the
    // Data stream instead (it exceeds the 512-byte attribute limit there, and
    // this one is close enough to it that we don't rely on this path either).
    if (s_meta_len == 0) return 0;
    int rc = os_mbuf_append(ctxt->om, s_meta, s_meta_len);
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

int ble_stream_control_access(uint16_t conn, uint16_t attr,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn; (void)attr; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;

    // The receiver's grammar is start|stop|restart|forget [wid]. A wearable
    // streams whenever a client is subscribed and has nothing to forget over
    // this link, so we accept and ignore — the point is that the app can speak
    // to either peer without special-casing.
    char buf[32] = {0};
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len > sizeof(buf) - 1) len = sizeof(buf) - 1;
    uint16_t olen = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, len, &olen) != 0) return 0;
    buf[olen] = '\0';

#ifdef IMPACT_TEST_HOOK
    // Bring-up only. The mock CSV peaks near 1 g on the high-g channel, so
    // there is otherwise no way to see the impact pipeline work without hitting
    // real hardware. Unlike the rest of this grammar it has an effect, so it is
    // the one control word that needs the auth gate.
    if (strncmp(buf, "impact-test", 11) == 0) {
        // NOT BLE_ATT_ERR_INSUFFICIENT_AUTHEN — that tells the central to start
        // bonding, which this server cannot do, and the phone drops the link.
        if (!ble_auth_conn_allowed(conn)) return ATT_ERR_NOT_AUTHORIZED;
        float g = (float)atof(buf + 11);        // "impact-test 35" -> 35 g
        if (g < IMPACT_THRESHOLD_G) g = 35.0f;  // bare "impact-test"
        impact_det_inject(g);
        return 0;
    }
#endif

    // "mode <idle|live|alerts|mock> [wid]" — the solo-session path for setting
    // the working mode. A group session sends the same words to the RECEIVER,
    // which relays them to the boards as MSG_MODE datagrams (wifi_udp_tx.cpp),
    // so the two transports share one grammar.
    if (strncmp(buf, "mode ", 5) == 0) {
        // NOT BLE_ATT_ERR_INSUFFICIENT_AUTHEN — that tells the central this
        // attribute needs an encrypted link, so it starts bonding, which this
        // server cannot do, and the phone drops the link entirely.
        if (!ble_auth_conn_allowed(conn)) return ATT_ERR_NOT_AUTHORIZED;
        if (!s_mode_cb || !s_mode_cb(buf + 5)) {
            ESP_LOGW(TAG, "control: unknown mode in \"%s\"", buf);
            return BLE_ATT_ERR_UNLIKELY;
        }
        ESP_LOGI(TAG, "control: \"%s\"", buf);
        return 0;
    }

    // The receiver's grammar is start|stop|restart|forget [wid]. A wearable
    // streams whenever a client is subscribed and has nothing to forget over
    // this link, so we accept and ignore.
    ESP_LOGI(TAG, "control: \"%s\" (ignored)", buf);
    return 0;
}

void ble_stream_set_mode_cb(bool (*cb)(const char *arg))
{
    s_mode_cb = cb;
}
