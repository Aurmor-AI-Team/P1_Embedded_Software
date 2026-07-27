// ---------------------------------------------------------------------------
// ble_provision.cpp — NimBLE GATT server: Wi-Fi/UDP/ID provisioning AND, since
// the BT_* modes landed, a live data link.
//
// The app connects and writes:
//   SSID        (...def4, WRITE)   UTF-8, <= 32 bytes
//   Password    (...def5, WRITE)   UTF-8, <= 64 bytes
//   Target      (...def6, WRITE)   "A.B.C.D:port"  -> triggers on_provision()
//   WearableID  (...def8, WRITE)   decimal string  -> on_wearable_id()
//   ExpPiID     (...def9, WRITE)   decimal string  -> on_expected_pi_id()
//   Control     (...defc, WRITE)   packed {u8 policy; float threshold_g}  NEW
// and reads/subscribes:
//   Status      (...def7, READ + NOTIFY)
//   Telemetry   (...defa, NOTIFY)     mibs_imu_pkt_t                      NEW
//   Alert       (...defb, INDICATE)   mibs_alert_pkt_t                    NEW
//
// Write SSID and Password first, then Target last: the Target write is the
// commit that fires the provisioning callback.
//
// BLE is no longer torn down after provisioning — it is the fallback transport
// when WiFi drops. That means BLE and WiFi now run concurrently; see
// PERIPHERAL_CHANGES.md for the coexistence sdkconfig requirements.
// ---------------------------------------------------------------------------
#include "ble_provision.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_prov";

#define DEVICE_NAME_PREFIX "aurmor-esp32-"
static char s_dev_name[24] = {0};

static ble_provision_cfg_t s_cfg = {};
static uint8_t             s_own_addr_type = 0;

static uint16_t s_conn_handle       = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_status_val_handle = 0;
static uint16_t s_telem_val_handle  = 0;
static uint16_t s_alert_val_handle  = 0;

static volatile bool s_status_notify = false;
static volatile bool s_telem_notify  = false;
static volatile bool s_alert_ind     = false;
static volatile bool s_active        = false;  // gates advertising + notifies

// Only one indication may be outstanding at a time (ATT rule). Cleared by
// BLE_GAP_EVENT_NOTIFY_TX when the peer confirms or the attempt fails.
static volatile bool     s_ind_busy    = false;
static volatile int64_t  s_ind_sent_us = 0;
#define IND_CONFIRM_TIMEOUT_US (5 * 1000 * 1000)

static volatile uint16_t s_mtu  = BLE_ATT_MTU_DFLT;   // 23 until exchanged
static volatile uint8_t  s_mode = 0;
static uint32_t          s_telem_seq = 0;
static uint16_t          s_wearable_id = 0;   // mirrors wifi_udp's, set on write

static char s_ssid[33] = {0};
static char s_pass[65] = {0};

// BLE_UUID128_INIT takes bytes least-significant-first (reversed from text).
#define UUID128(last) BLE_UUID128_INIT( \
    (last), 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, \
    0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12)

static const ble_uuid128_t s_svc_uuid     = UUID128(0xf0);
static const ble_uuid128_t s_ssid_uuid    = UUID128(0xf4);
static const ble_uuid128_t s_pass_uuid    = UUID128(0xf5);
static const ble_uuid128_t s_target_uuid  = UUID128(0xf6);
static const ble_uuid128_t s_status_uuid  = UUID128(0xf7);
static const ble_uuid128_t s_wear_id_uuid = UUID128(0xf8);
static const ble_uuid128_t s_pi_id_uuid   = UUID128(0xf9);
static const ble_uuid128_t s_telem_uuid   = UUID128(0xfa);   // NEW
static const ble_uuid128_t s_alert_uuid   = UUID128(0xfb);   // NEW
static const ble_uuid128_t s_ctrl_uuid    = UUID128(0xfc);   // NEW

static void start_advertising(void);

// A usable data link needs a connection AND a subscriber. "Connected but not
// subscribed" is not a transport — sending to it returns an ATT error.
static bool link_usable(void)
{
    return s_active && s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_telem_notify;
}

static void report_link(bool up)
{
    static bool last = false;
    if (up == last) return;
    last = up;
    if (s_cfg.on_link) s_cfg.on_link(up);
}

static int read_str(struct ble_gatt_access_ctxt *ctxt, char *out, size_t cap)
{
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len > cap - 1) len = cap - 1;
    uint16_t olen = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, out, len, &olen);
    if (rc != 0) return BLE_ATT_ERR_UNLIKELY;
    out[olen] = '\0';
    return 0;
}

static int ssid_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    int rc = read_str(ctxt, s_ssid, sizeof(s_ssid));
    if (rc == 0) ESP_LOGI(TAG, "SSID staged (%d chars)", (int)strlen(s_ssid));
    return rc;
}

static int pass_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    int rc = read_str(ctxt, s_pass, sizeof(s_pass));
    if (rc == 0) ESP_LOGI(TAG, "password staged (%d chars)", (int)strlen(s_pass));
    return rc;
}

static int target_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;

    char buf[32];
    int rc = read_str(ctxt, buf, sizeof(buf));
    if (rc != 0) return rc;

    char *colon = strrchr(buf, ':');
    if (!colon) {
        ESP_LOGW(TAG, "target must be \"ip:port\": %s", buf);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    *colon = '\0';
    const char *ip = buf;
    uint16_t    port = (uint16_t)atoi(colon + 1);

    ESP_LOGI(TAG, "provision commit: ssid=\"%s\" target=%s:%u", s_ssid, ip, port);
    if (s_cfg.on_provision) s_cfg.on_provision(s_ssid, s_pass, ip, port);

    // Credentials are no longer needed in RAM once handed off.
    memset(s_pass, 0, sizeof(s_pass));
    return 0;
}

static int wear_id_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    char buf[12];
    int rc = read_str(ctxt, buf, sizeof(buf));
    if (rc != 0) return rc;
    uint16_t id = (uint16_t)strtoul(buf, NULL, 10);
    s_wearable_id = id;
    if (s_cfg.on_wearable_id) s_cfg.on_wearable_id(id);
    return 0;
}

static int pi_id_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    char buf[12];
    int rc = read_str(ctxt, buf, sizeof(buf));
    if (rc != 0) return rc;
    uint32_t id = (uint32_t)strtoul(buf, NULL, 10);
    if (s_cfg.on_expected_pi_id) s_cfg.on_expected_pi_id(id);
    return 0;
}

static int status_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    char buf[BLE_STATUS_MAX] = "unprovisioned";
    if (s_cfg.status_getter) s_cfg.status_getter(buf, sizeof(buf));
    int rc = os_mbuf_append(ctxt->om, buf, strlen(buf));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// NEW: mode/threshold control. Packed {uint8 policy; float threshold_g}.
typedef struct __attribute__((packed)) {
    uint8_t policy;
    float   threshold_g;
} ctrl_write_t;

static int ctrl_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;

    ctrl_write_t w = {};
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len != sizeof(w)) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint16_t olen = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, &w, sizeof(w), &olen) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    ESP_LOGI(TAG, "control write: policy=%u threshold=%.1f",
             w.policy, (double)w.threshold_g);
    if (s_cfg.on_control) s_cfg.on_control(w.policy, w.threshold_g);
    return 0;
}

// Notify/indicate-only characteristics still need an access_cb for reads; we
// simply reject reads, the value only ever arrives via notification.
static int noread_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            { .uuid = &s_ssid_uuid.u,    .access_cb = ssid_access,
              .flags = BLE_GATT_CHR_F_WRITE },
            { .uuid = &s_pass_uuid.u,    .access_cb = pass_access,
              .flags = BLE_GATT_CHR_F_WRITE },
            { .uuid = &s_target_uuid.u,  .access_cb = target_access,
              .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP },
            { .uuid = &s_wear_id_uuid.u, .access_cb = wear_id_access,
              .flags = BLE_GATT_CHR_F_WRITE },
            { .uuid = &s_pi_id_uuid.u,   .access_cb = pi_id_access,
              .flags = BLE_GATT_CHR_F_WRITE },
            { .uuid = &s_status_uuid.u,  .access_cb = status_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
              .val_handle = &s_status_val_handle },
            { .uuid = &s_telem_uuid.u,   .access_cb = noread_access,
              .flags = BLE_GATT_CHR_F_NOTIFY,
              .val_handle = &s_telem_val_handle },
            { .uuid = &s_alert_uuid.u,   .access_cb = noread_access,
              .flags = BLE_GATT_CHR_F_INDICATE,
              .val_handle = &s_alert_val_handle },
            { .uuid = &s_ctrl_uuid.u,    .access_cb = ctrl_access,
              .flags = BLE_GATT_CHR_F_WRITE },
            { 0 }
        },
    },
    { 0 }
};

// Ask for a connection interval matching the traffic we are about to send.
// 15 ms sustains ~20 Hz telemetry; 120 ms is plenty for an idle alerts link
// and materially cheaper on battery.
static void request_conn_params(bool fast)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;
    struct ble_gap_upd_params p = {};
    p.itvl_min            = fast ? 12 : 96;    // units of 1.25 ms -> 15 / 120 ms
    p.itvl_max            = fast ? 24 : 128;   //                    30 / 160 ms
    p.latency             = 0;
    p.supervision_timeout = 400;               // units of 10 ms -> 4 s
    int rc = ble_gap_update_params(s_conn_handle, &p);
    if (rc != 0) ESP_LOGW(TAG, "conn param update rc=%d", rc);
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_mtu         = ble_att_mtu(s_conn_handle);
            s_ind_busy    = false;
            ESP_LOGI(TAG, "connected (handle=%d, mtu=%u)", s_conn_handle, s_mtu);
            // Most phones initiate this, but not all — ask anyway. A 23-byte
            // MTU cannot carry a telemetry or alert packet.
            ble_gattc_exchange_mtu(s_conn_handle, NULL, NULL);
        } else if (s_active) {
            start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected; reason=%d", event->disconnect.reason);
        s_conn_handle   = BLE_HS_CONN_HANDLE_NONE;
        s_status_notify = false;
        s_telem_notify  = false;
        s_alert_ind     = false;
        s_ind_busy      = false;
        s_mtu           = BLE_ATT_MTU_DFLT;
        report_link(false);
        if (s_active) start_advertising();
        return 0;

    case BLE_GAP_EVENT_MTU:
        s_mtu = event->mtu.value;
        ESP_LOGI(TAG, "MTU now %u (telemetry needs >= %u)",
                 s_mtu, (unsigned)(sizeof(mibs_imu_pkt_t) + 3));
        if (s_mtu < sizeof(mibs_imu_pkt_t) + 3) {
            ESP_LOGW(TAG, "MTU too small for telemetry — BT stream will be refused");
        }
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_status_val_handle) {
            s_status_notify = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_telem_val_handle) {
            s_telem_notify = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "telemetry subscription: %d", (int)s_telem_notify);
            report_link(link_usable());
        } else if (event->subscribe.attr_handle == s_alert_val_handle) {
            s_alert_ind = event->subscribe.cur_indicate;
            ESP_LOGI(TAG, "alert subscription: %d", (int)s_alert_ind);
        }
        return 0;

    case BLE_GAP_EVENT_NOTIFY_TX:
        // For indications this fires again with BLE_HS_EDONE once the peer
        // confirms. Anything else means the attempt is over, successfully or
        // not, so the slot is free either way.
        if (event->notify_tx.indication &&
            event->notify_tx.attr_handle == s_alert_val_handle) {
            if (event->notify_tx.status == BLE_HS_EDONE) {
                ESP_LOGD(TAG, "alert confirmed");
                s_ind_busy = false;
            } else if (event->notify_tx.status != 0) {
                ESP_LOGW(TAG, "alert indication failed rc=%d", event->notify_tx.status);
                s_ind_busy = false;
            }
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        if (s_active) start_advertising();
        return 0;

    default:
        return 0;
    }
}

static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params = {};
    struct ble_hs_adv_fields  fields     = {};
    struct ble_hs_adv_fields  rsp        = {};

    // Legacy ADV payloads are 31 bytes. Flags (3) + the full name (2+17)
    // fit; the 128-bit service UUID (2+16) must ride in the scan response.
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name             = (uint8_t *)s_dev_name;
    fields.name_len         = strlen(s_dev_name);
    fields.name_is_complete = 1;

    rsp.uuids128             = (ble_uuid128_t *)&s_svc_uuid;
    rsp.num_uuids128         = 1;
    rsp.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) { ESP_LOGE(TAG, "adv_set_fields rc=%d", rc); return; }
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) { ESP_LOGE(TAG, "adv_rsp_set_fields rc=%d", rc); return; }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event, NULL);
    if (rc != 0) { ESP_LOGE(TAG, "adv_start rc=%d", rc); return; }
    ESP_LOGI(TAG, "advertising as \"%s\"", s_dev_name);
}

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) { ESP_LOGE(TAG, "infer addr rc=%d", rc); return; }
    start_advertising();
}

static void on_reset(int reason) { ESP_LOGW(TAG, "nimble reset; reason=%d", reason); }

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_provision_push_status(void)
{
    if (!s_active || s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_status_notify
            || !s_cfg.status_getter) {
        return;
    }
    char buf[BLE_STATUS_MAX];
    s_cfg.status_getter(buf, sizeof(buf));

    // A notification cannot exceed MTU-3; longer strings are silently cut by
    // the stack. Truncate here so at least the cut is deliberate and logged.
    size_t len = strlen(buf);
    size_t cap = (s_mtu > 3) ? (size_t)(s_mtu - 3) : 20;
    if (len > cap) {
        ESP_LOGW(TAG, "status truncated %u->%u (MTU=%u) — client should read, not notify",
                 (unsigned)len, (unsigned)cap, s_mtu);
        len = cap;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
    if (om) ble_gatts_notify_custom(s_conn_handle, s_status_val_handle, om);
}

bool ble_provision_is_active(void)    { return s_active; }
bool ble_provision_is_connected(void) { return link_usable(); }

void ble_provision_set_mode(uint8_t mode)
{
    if (mode == s_mode) return;
    s_mode = mode;
    // APP_MODE_BT_LIVE(5) and APP_MODE_BT_MOCK(2) push traffic; alerts idle.
    request_conn_params(mode == 5 || mode == 2);
}

esp_err_t ble_provision_send_stream(const mibs_message *m, float temp,
                                    float hr, float spo2, float resp, float hrv)
{
    if (!m) return ESP_ERR_INVALID_ARG;
    if (!link_usable()) return ESP_ERR_INVALID_STATE;
    if (s_mtu < sizeof(mibs_imu_pkt_t) + 3) return ESP_ERR_INVALID_SIZE;

    mibs_imu_pkt_t pkt = {};
    pkt.hdr = { MIBS_MSG_IMU, MIBS_MSG_VERSION, s_wearable_id };
    pkt.seq                = s_telem_seq++;
    pkt.t_ms               = (uint32_t)(esp_timer_get_time() / 1000);
    pkt.impact_count       = m->impact_count;
    pkt.impact_threshold   = m->impact_threshold;
    pkt.impact_accumulator = m->impact_accumulator;
    pkt.all_time_peak_g    = m->all_time_peak_g;
    pkt.temp_c             = temp;
    pkt.hr = hr; pkt.spo2 = spo2; pkt.resp = resp; pkt.hrv = hrv;
    pkt.mode               = s_mode;

    // Telemetry is disposable: if the mbuf pool is dry, drop this frame rather
    // than block the IMU task or starve the alert path of buffers.
    struct os_mbuf *om = ble_hs_mbuf_from_flat(&pkt, sizeof(pkt));
    if (!om) return ESP_ERR_NO_MEM;

    int rc = ble_gatts_notify_custom(s_conn_handle, s_telem_val_handle, om);
    return rc == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t ble_provision_send_alert(const mibs_impact_t *imp)
{
    if (!imp) return ESP_ERR_INVALID_ARG;
    if (!s_active || s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_alert_ind) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_mtu < sizeof(mibs_alert_pkt_t) + 3) return ESP_ERR_INVALID_SIZE;

    // A peer that never confirms must not wedge the alert path forever.
    if (s_ind_busy && (esp_timer_get_time() - s_ind_sent_us) > IND_CONFIRM_TIMEOUT_US) {
        ESP_LOGW(TAG, "indication confirm timed out — freeing slot");
        s_ind_busy = false;
    }
    if (s_ind_busy) return ESP_ERR_INVALID_STATE;   // caller buffers and retries

    mibs_alert_pkt_t pkt = {};
    pkt.hdr = { MIBS_MSG_ALERT, MIBS_MSG_VERSION, s_wearable_id };
    pkt.seq         = imp->seq;
    pkt.t_ms        = (uint32_t)(imp->t_us / 1000);
    pkt.peak_g      = imp->peak_g;
    pkt.threshold_g = imp->threshold_g;
    pkt.hx_g = imp->hx_g; pkt.hy_g = imp->hy_g; pkt.hz_g = imp->hz_g;
    pkt.gx_dps = imp->gx_dps; pkt.gy_dps = imp->gy_dps; pkt.gz_dps = imp->gz_dps;
    pkt.dur_ms  = imp->dur_ms;
    pkt.mode    = imp->mode;
    pkt.xport   = imp->xport;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(&pkt, sizeof(pkt));
    if (!om) return ESP_ERR_NO_MEM;

    s_ind_busy    = true;
    s_ind_sent_us = esp_timer_get_time();
    int rc = ble_gatts_indicate_custom(s_conn_handle, s_alert_val_handle, om);
    if (rc != 0) {
        s_ind_busy = false;
        ESP_LOGW(TAG, "indicate rc=%d", rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_provision_start(const ble_provision_cfg_t *cfg)
{
    if (s_active) return ESP_OK;
    if (cfg) s_cfg = *cfg;

    if (s_dev_name[0] == '\0') {
        uint8_t mac[6] = {0};
        esp_efuse_mac_get_default(mac);
        snprintf(s_dev_name, sizeof(s_dev_name), DEVICE_NAME_PREFIX "%02X%02X",
                 mac[4], mac[5]);
        if (s_wearable_id == 0) s_wearable_id = (uint16_t)((mac[4] << 8) | mac[5]);
    }

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "nimble_port_init: %d", err); return err; }

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Telemetry and alert packets do not fit in the 23-byte default.
    ble_att_set_preferred_mtu(247);

    // GATT registration does not survive nimble_port_deinit(): redo it on
    // every start.
    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "count_cfg rc=%d", rc); return ESP_FAIL; }
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "add_svcs rc=%d", rc); return ESP_FAIL; }
    rc = ble_svc_gap_device_name_set(s_dev_name);
    if (rc != 0) { ESP_LOGE(TAG, "name_set rc=%d", rc); return ESP_FAIL; }

    s_active = true;   // before host start: on_sync advertises immediately
    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "BLE started (\"%s\")", s_dev_name);
    return ESP_OK;
}

esp_err_t ble_provision_stop(void)
{
    if (!s_active) return ESP_OK;
    s_active = false;   // gates re-advertise on the disconnect we cause below

    ble_gap_adv_stop();
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }

    // Canonical IDF teardown: stop makes nimble_port_run() return (host_task
    // then calls nimble_port_freertos_deinit), deinit releases host+controller.
    int rc = nimble_port_stop();
    if (rc == 0) {
        nimble_port_deinit();
    } else {
        ESP_LOGE(TAG, "nimble_port_stop rc=%d", rc);
    }

    s_conn_handle   = BLE_HS_CONN_HANDLE_NONE;
    s_status_notify = false;
    s_telem_notify  = false;
    s_alert_ind     = false;
    s_ind_busy      = false;
    s_mtu           = BLE_ATT_MTU_DFLT;
    report_link(false);
    ESP_LOGI(TAG, "BLE stopped");
    return rc == 0 ? ESP_OK : ESP_FAIL;
}