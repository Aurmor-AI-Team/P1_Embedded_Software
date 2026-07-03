// ---------------------------------------------------------------------------
// ble_provision.cpp — NimBLE GATT server for Wi-Fi + UDP-endpoint + ID setup.
//
// The app connects and writes:
//   SSID        (...def4, WRITE)   UTF-8, <= 32 bytes
//   Password    (...def5, WRITE)   UTF-8, <= 64 bytes
//   Target      (...def6, WRITE)   "A.B.C.D:port"  -> triggers on_provision()
//   WearableID  (...def8, WRITE)   decimal string  -> on_wearable_id()
//   ExpPiID     (...def9, WRITE)   decimal string  -> on_expected_pi_id()
// and reads/subscribes:
//   Status      (...def7, READ + NOTIFY)  e.g. "up ip=192.168.1.42 pi=7 ok"
//
// Write SSID and Password first, then Target last: the Target write is the
// commit that fires the provisioning callback. WearableID / ExpPiID apply
// immediately on write, independent of the commit.
// ---------------------------------------------------------------------------
#include "ble_provision.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_prov";

#define DEVICE_NAME "XIAO-IMU"

static ble_provision_cfg_t s_cfg = {};
static uint8_t             s_own_addr_type = 0;

static uint16_t s_conn_handle       = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_status_val_handle = 0;
static volatile bool s_status_notify = false;

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

static void start_advertising(void);

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
    return 0;
}

static int wear_id_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    char buf[12];
    int rc = read_str(ctxt, buf, sizeof(buf));
    if (rc != 0) return rc;
    uint16_t id = (uint16_t)strtoul(buf, NULL, 10);
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
    char buf[64] = "unprovisioned";
    if (s_cfg.status_getter) s_cfg.status_getter(buf, sizeof(buf));
    int rc = os_mbuf_append(ctxt->om, buf, strlen(buf));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
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
            { 0 }
        },
    },
    { 0 }
};

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "connected (handle=%d)", s_conn_handle);
        } else {
            start_advertising();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected; reason=%d", event->disconnect.reason);
        s_conn_handle   = BLE_HS_CONN_HANDLE_NONE;
        s_status_notify = false;
        start_advertising();
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_status_val_handle) {
            s_status_notify = event->subscribe.cur_notify;
        }
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        return 0;
    default:
        return 0;
    }
}

static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params = {};
    struct ble_hs_adv_fields  fields     = {};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name             = (uint8_t *)DEVICE_NAME;
    fields.name_len         = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;
    fields.uuids128             = (ble_uuid128_t *)&s_svc_uuid;
    fields.num_uuids128         = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) { ESP_LOGE(TAG, "adv_set_fields rc=%d", rc); return; }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event, NULL);
    if (rc != 0) { ESP_LOGE(TAG, "adv_start rc=%d", rc); return; }
    ESP_LOGI(TAG, "advertising as \"%s\"", DEVICE_NAME);
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
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_status_notify || !s_cfg.status_getter) {
        return;
    }
    char buf[64];
    s_cfg.status_getter(buf, sizeof(buf));
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, strlen(buf));
    if (om) ble_gatts_notify_custom(s_conn_handle, s_status_val_handle, om);
}

esp_err_t ble_provision_init(const ble_provision_cfg_t *cfg)
{
    if (cfg) s_cfg = *cfg;

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "nimble_port_init: %d", err); return err; }

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "count_cfg rc=%d", rc); return ESP_FAIL; }
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "add_svcs rc=%d", rc); return ESP_FAIL; }
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) { ESP_LOGE(TAG, "name_set rc=%d", rc); return ESP_FAIL; }

    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "BLE provisioning service started");
    return ESP_OK;
}
