// ---------------------------------------------------------------------------
// ble_imu.cpp — NimBLE GATT server that streams LSM6DSV samples to a phone app.
//
// ESP32-C6 is BLE-only (no Bluetooth Classic), so this is a BLE GATT
// peripheral. It exposes one custom service with one NOTIFY characteristic.
// The app connects, enables notifications, and receives a packed 44-byte
// binary packet per sample.
//
// Wire format (little-endian, matches ESP32 native byte order):
//   offset  type     field
//   0       uint32   t_ms          milliseconds since boot
//   4       float    ax, ay, az    accelerometer, g
//   16      float    gx, gy, gz    gyroscope, deg/s
//   28      float    hx, hy, hz    high-g accelerometer, g
//   40      float    temp_c        temperature, deg C
//   44      (end)
//
// Requires (sdkconfig / sdkconfig.defaults):
//   CONFIG_BT_ENABLED=y
//   CONFIG_BT_NIMBLE_ENABLED=y
//   CONFIG_BT_CONTROLLER_ENABLED=y   (default on C6)
// and ESP-IDF 5.1+ (needed for ESP32-C6).
// ---------------------------------------------------------------------------
#include "ble_imu.h"

#include <string.h>
#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_imu";

#define DEVICE_NAME "XIAO-IMU"

// 44-byte packed packet sent on every notification.
typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    float    ax, ay, az;
    float    gx, gy, gz;
    float    hx, hy, hz;
    float    temp_c;
} imu_packet_t;

// --- Custom 128-bit UUIDs -------------------------------------------------
// Service:        12345678-1234-5678-1234-56789abcdef0
// IMU characteristic: ...def1
// BLE_UUID128_INIT takes bytes least-significant-first (reversed from the
// textual representation above).
static const ble_uuid128_t s_svc_uuid = BLE_UUID128_INIT(
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t s_imu_chr_uuid = BLE_UUID128_INIT(
    0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

// --- Connection / subscription state --------------------------------------
static uint16_t s_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_imu_val_handle = 0;      // filled in at service registration
static volatile bool s_notify_enabled = false;
static uint8_t  s_own_addr_type = 0;

// Most recent sample, returned on a plain READ of the characteristic.
static imu_packet_t s_last_packet = {};

static void start_advertising(void);

// --- GATT access callback (handles READ of the latest sample) -------------
static int imu_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        int rc = os_mbuf_append(ctxt->om, &s_last_packet, sizeof(s_last_packet));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// --- GATT service table ---------------------------------------------------
static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = &s_imu_chr_uuid.u,
                .access_cb  = imu_chr_access,
                // READ for one-shot polling, NOTIFY for streaming. NimBLE
                // adds the CCCD automatically for NOTIFY characteristics.
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_imu_val_handle,
            },
            { 0 }  // end of characteristics
        },
    },
    { 0 }  // end of services
};

// --- GAP event handler ----------------------------------------------------
static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "connected (handle=%d)", s_conn_handle);
        } else {
            ESP_LOGW(TAG, "connect failed; status=%d", event->connect.status);
            start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected; reason=%d", event->disconnect.reason);
        s_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
        s_notify_enabled = false;
        start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_imu_val_handle) {
            s_notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "notifications %s",
                     s_notify_enabled ? "ON" : "OFF");
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU update; mtu=%d", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        return 0;

    default:
        return 0;
    }
}

// --- Advertising ----------------------------------------------------------
static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params = {};
    struct ble_hs_adv_fields  fields     = {};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name             = (uint8_t *)DEVICE_NAME;
    fields.name_len         = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;
    fields.uuids128 = (ble_uuid128_t *)&s_svc_uuid;
    fields.num_uuids128      = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed; rc=%d", rc);
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;   // connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;   // general discoverable

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start failed; rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "advertising as \"%s\"", DEVICE_NAME);
}

// --- NimBLE host callbacks ------------------------------------------------
static void on_sync(void)
{
    // Pick an address type (public if available, else random).
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "infer addr type failed; rc=%d", rc);
        return;
    }
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "nimble reset; reason=%d", reason);
}

static void host_task(void *param)
{
    nimble_port_run();             // returns only on nimble_port_stop()
    nimble_port_freertos_deinit();
}

// --- Public API -----------------------------------------------------------
esp_err_t ble_imu_init(void)
{
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", err);
        return err;
    }

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

    // Ask for a larger MTU so the 44-byte packet fits in one notification.
    // The phone still has to agree during MTU exchange.
    ble_att_set_preferred_mtu(247);

    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "BLE IMU service started");
    return ESP_OK;
}

bool ble_imu_ready(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_notify_enabled;
}

esp_err_t ble_imu_notify(const lsm6_sample_t *s)
{
    if (!s) return ESP_ERR_INVALID_ARG;

    // Always cache for READ access, even with no subscriber.
    imu_packet_t pkt = {
        .t_ms  = (uint32_t)(esp_timer_get_time() / 1000),
        .ax = s->ax_g, .ay = s->ay_g, .az = s->az_g,
        .gx = s->gx_dps, .gy = s->gy_dps, .gz = s->gz_dps,
        .hx = s->hx_g, .hy = s->hy_g, .hz = s->hz_g,
        .temp_c = s->temp_c,
    };
    s_last_packet = pkt;

    if (!ble_imu_ready()) return ESP_OK;   // nobody listening

    struct os_mbuf *om = ble_hs_mbuf_from_flat(&pkt, sizeof(pkt));
    if (!om) return ESP_ERR_NO_MEM;

    // ble_gatts_notify_custom consumes the mbuf. (Older IDF: the same call
    // is named ble_gattc_notify_custom — swap the name if your IDF predates
    // the rename.)
    int rc = ble_gatts_notify_custom(s_conn_handle, s_imu_val_handle, om);
    if (rc != 0) {
        // Transient buffer-full errors are expected if the link can't keep
        // up with the notify rate; the sample is simply dropped.
        return ESP_FAIL;
    }
    return ESP_OK;
}
