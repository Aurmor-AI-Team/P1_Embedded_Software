// ---------------------------------------------------------------------------
// wifi_udp_tx.cpp — Wi-Fi station + UDP transmit/handshake path.
//
// Joins a normal Wi-Fi network and sends telemetry as UDP datagrams to a
// provisioned IP:port. Each packet carries a wearable ID so the receiver can
// tell multiple sensors apart. A periodic HELLO/WELCOME handshake confirms the
// link is live and learns the Pi's ID. Credentials, target, wearable ID, and
// expected Pi ID persist in NVS.
//
// Wire formats now live in mibs_wire.h and are shared with the BLE transport,
// so the Pi and the app parse one format regardless of how a packet arrived.
//
// Telemetry is fire-and-forget, which is right for a 100 Hz stream. Impact
// alerts are NOT: they are retransmitted until the Pi acks them by sequence
// number. A dropped alert is the failure this product exists to prevent.
// ---------------------------------------------------------------------------
#include "wifi_udp_tx.h"

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

static const char *TAG = "wifi_udp";

#define NVS_NS            "prov"
#define WIFI_MAX_RETRY    8
#define LOCAL_UDP_PORT    5006     // wearable binds here; Pi replies here
#define HELLO_PERIOD_MS   2000     // send a HELLO this often
#define VERIFY_TIMEOUT_MS 6000     // "verified" if WELCOME seen within this window

// Alert retransmission. The rx task wakes at least every 500 ms (socket
// timeout), so that is the natural retry granularity.
#define ALERT_PENDING_MAX   8
#define ALERT_RETRY_MS      600
#define ALERT_MAX_TRIES     6

static volatile bool s_connected = false;
static char          s_ip_str[16] = "0.0.0.0";
static int           s_sock       = -1;
static struct sockaddr_in s_dest  = {};
static volatile bool s_has_target = false;
static portMUX_TYPE  s_lock       = portMUX_INITIALIZER_UNLOCKED;
static uint32_t      s_seq        = 0;
static int           s_retry      = 0;

static uint16_t      s_wearable_id     = 0;
static uint16_t      s_default_wid     = 0;   // MAC-derived, restored on forget
static uint32_t      s_expected_pi_id  = 0;
static volatile uint32_t s_pi_id       = 0;
static volatile int64_t  s_last_welcome_us = 0;
static uint32_t      s_hello_nonce     = 0;
static volatile bool s_has_creds       = false;
static volatile uint8_t s_mode         = 0;
static void (*s_forget_cb)(void)       = NULL;
static void (*s_link_cb)(bool)         = NULL;

// Alerts sent but not yet acknowledged.
typedef struct {
    mibs_alert_pkt_t pkt;
    int64_t          last_tx_us;
    uint8_t          tries;
    bool             used;
} pending_alert_t;

static pending_alert_t s_pending[ALERT_PENDING_MAX];
static portMUX_TYPE    s_pend_lock = portMUX_INITIALIZER_UNLOCKED;

// --- NVS helpers ----------------------------------------------------------
static void nvs_save_str(const char *key, const char *val)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, key, val); nvs_commit(h); nvs_close(h);
    }
}
static void nvs_save_u16(const char *key, uint16_t val)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u16(h, key, val); nvs_commit(h); nvs_close(h);
    }
}
static void nvs_save_u32(const char *key, uint32_t val)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, key, val); nvs_commit(h); nvs_close(h);
    }
}

// --- Wi-Fi events ---------------------------------------------------------
static void wifi_evt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        bool was = s_connected;
        s_connected = false;
        strcpy(s_ip_str, "0.0.0.0");
        // Tell the controller immediately — this fires seconds before
        // IP_EVENT_STA_LOST_IP, and every one of those seconds is alerts that
        // would otherwise be handed to a dead socket.
        if (was && s_link_cb) s_link_cb(false);

        // Only reconnect to a network we're provisioned for; a "forget" clears
        // s_has_creds first, so this also stops us re-joining after unpair.
        if (!s_has_creds) return;
        s_retry++;
        // Keep trying indefinitely — a wearable must auto-rejoin its own AP.
        // In particular, after an ungraceful power-off the receiver's AP can
        // hold a stale ("ghost") association for our MAC and reject re-auth
        // (reason 2) until it ages the ghost out.
        esp_wifi_connect();
        if (s_retry <= WIFI_MAX_RETRY || s_retry % 10 == 0) {
            ESP_LOGW(TAG, "disconnected (reason=%d), reconnecting (attempt %d)",
                     e ? e->reason : -1, s_retry);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        esp_ip4addr_ntoa(&e->ip_info.ip, s_ip_str, sizeof(s_ip_str));
        s_connected = true;
        s_retry = 0;
        ESP_LOGI(TAG, "connected, ip=%s", s_ip_str);
        if (s_link_cb) s_link_cb(true);
    }
}

// --- low-level send -------------------------------------------------------
static bool get_dest(struct sockaddr_in *out)
{
    bool has;
    portENTER_CRITICAL(&s_lock);
    has = s_has_target;
    if (has) *out = s_dest;
    portEXIT_CRITICAL(&s_lock);
    return has;
}

static int raw_send(const void *p, size_t n)
{
    struct sockaddr_in dest;
    if (!s_connected || !get_dest(&dest)) return -1;
    return sendto(s_sock, p, n, 0, (struct sockaddr *)&dest, sizeof(dest));
}

// --- Handshake ------------------------------------------------------------
static void send_hello(void)
{
    s_hello_nonce = esp_random();
    mibs_hello_pkt_t h = {};
    h.hdr   = { MIBS_MSG_HELLO, MIBS_MSG_VERSION, s_wearable_id };
    h.nonce = s_hello_nonce;
    raw_send(&h, sizeof(h));
}

static void handle_welcome(const mibs_welcome_pkt_t *w)
{
    uint32_t pid = w->pi_id;
    if (s_expected_pi_id != 0 && s_expected_pi_id != pid) {
        ESP_LOGW(TAG, "WELCOME from unexpected pi_id=%lu (want %lu)",
                 (unsigned long)pid, (unsigned long)s_expected_pi_id);
        return;
    }
    portENTER_CRITICAL(&s_lock);
    s_pi_id = pid;
    s_last_welcome_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "link verified with pi_id=%lu", (unsigned long)pid);
}

static void handle_alert_ack(const mibs_alert_ack_pkt_t *a)
{
    portENTER_CRITICAL(&s_pend_lock);
    for (int i = 0; i < ALERT_PENDING_MAX; i++) {
        if (s_pending[i].used && s_pending[i].pkt.seq == a->seq) {
            s_pending[i].used = false;
            portEXIT_CRITICAL(&s_pend_lock);
            ESP_LOGI(TAG, "alert %lu acked", (unsigned long)a->seq);
            return;
        }
    }
    portEXIT_CRITICAL(&s_pend_lock);
}

static void retry_pending_alerts(void)
{
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < ALERT_PENDING_MAX; i++) {
        mibs_alert_pkt_t copy;
        bool go = false;

        portENTER_CRITICAL(&s_pend_lock);
        if (s_pending[i].used &&
            (now - s_pending[i].last_tx_us) >= (int64_t)ALERT_RETRY_MS * 1000) {
            if (s_pending[i].tries >= ALERT_MAX_TRIES) {
                s_pending[i].used = false;   // give up; app_ctrl already logged it
            } else {
                s_pending[i].tries++;
                s_pending[i].last_tx_us = now;
                copy = s_pending[i].pkt;
                go = true;
            }
        }
        portEXIT_CRITICAL(&s_pend_lock);

        if (go) {
            ESP_LOGW(TAG, "retransmitting alert %lu", (unsigned long)copy.seq);
            raw_send(&copy, sizeof(copy));
        }
    }
}

static void udp_rx_task(void *arg)
{
    struct timeval tv = { .tv_sec = 0, .tv_usec = 500000 };  // 500 ms
    setsockopt(s_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int64_t last_hello = 0;
    uint8_t buf[64];

    while (true) {
        if (!s_connected || !s_has_target) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        int64_t now = esp_timer_get_time();
        if (now - last_hello >= (int64_t)HELLO_PERIOD_MS * 1000) {
            send_hello();
            last_hello = now;
        }
        retry_pending_alerts();

        struct sockaddr_in src;
        socklen_t slen = sizeof(src);
        int n = recvfrom(s_sock, buf, sizeof(buf), 0,
                         (struct sockaddr *)&src, &slen);
        if (n < (int)sizeof(mibs_hdr_t)) continue;   // timeout or runt

        mibs_hdr_t *h = (mibs_hdr_t *)buf;
        if (h->msg_type == MIBS_MSG_WELCOME && n >= (int)sizeof(mibs_welcome_pkt_t)) {
            handle_welcome((mibs_welcome_pkt_t *)buf);

        } else if (h->msg_type == MIBS_MSG_ALERT_ACK &&
                   n >= (int)sizeof(mibs_alert_ack_pkt_t)) {
            handle_alert_ack((mibs_alert_ack_pkt_t *)buf);

        } else if (h->msg_type == MIBS_MSG_FORGET && n >= (int)sizeof(mibs_forget_pkt_t)) {
            mibs_forget_pkt_t *f = (mibs_forget_pkt_t *)buf;
            bool wid_ok = (f->hdr.wearable_id == 0 ||
                           f->hdr.wearable_id == s_wearable_id);
            bool pi_ok  = (s_pi_id == 0 || f->pi_id == s_pi_id);
            if (f->hdr.version == MIBS_MSG_VERSION && wid_ok && pi_ok) {
                ESP_LOGW(TAG, "FORGET from pi_id=%lu — unpairing",
                         (unsigned long)f->pi_id);
                if (s_forget_cb) s_forget_cb();   // must only post an event
            }
        }
    }
}

// --- Init -----------------------------------------------------------------
esp_err_t wifi_udp_init(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK) return err;

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "wifi_init: %d", err); return err; }

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_evt, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_evt, NULL, NULL);

    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    // app_ctrl overrides this per mode (MAX_MODEM in ALERTS, NONE in LIVE).
    esp_wifi_set_ps(WIFI_PS_NONE);
    err = esp_wifi_start();
    if (err != ESP_OK) { ESP_LOGE(TAG, "wifi_start: %d", err); return err; }

    // Default wearable ID from the low 16 bits of the MAC.
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    s_wearable_id = (uint16_t)((mac[4] << 8) | mac[5]);
    s_default_wid = s_wearable_id;

    // UDP socket, bound to a fixed local port so the Pi's WELCOME can return.
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) { ESP_LOGE(TAG, "socket() failed"); return ESP_FAIL; }
    struct sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(LOCAL_UDP_PORT);
    if (bind(s_sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
        ESP_LOGE(TAG, "bind(%d) failed", LOCAL_UDP_PORT);
        return ESP_FAIL;
    }

    // Restore saved provisioning, if any.
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        char ssid[33] = {0}, pass[65] = {0}, ip[16] = {0};
        size_t l;
        l = sizeof(ssid); bool has_ssid = (nvs_get_str(h, "ssid", ssid, &l) == ESP_OK);
        l = sizeof(pass); nvs_get_str(h, "pass", pass, &l);
        l = sizeof(ip);   bool has_ip = (nvs_get_str(h, "ip", ip, &l) == ESP_OK);
        uint16_t port = 0; nvs_get_u16(h, "port", &port);
        uint16_t wid = 0;  if (nvs_get_u16(h, "wid", &wid) == ESP_OK) s_wearable_id = wid;
        nvs_get_u32(h, "epid", &s_expected_pi_id);
        nvs_close(h);

        if (has_ip && port) wifi_udp_set_target(ip, port);
        if (has_ssid) { ESP_LOGI(TAG, "restoring saved AP \"%s\"", ssid); wifi_udp_connect(ssid, pass); }
    }

    xTaskCreate(udp_rx_task, "udp_rx", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Wi-Fi/UDP ready (wearable_id=%u, local_port=%d, imu_pkt=%u B)",
             s_wearable_id, LOCAL_UDP_PORT, (unsigned)sizeof(mibs_imu_pkt_t));
    return ESP_OK;
}

esp_err_t wifi_udp_connect(const char *ssid, const char *password)
{
    if (!ssid) return ESP_ERR_INVALID_ARG;

    wifi_config_t wc = {};
    strlcpy((char *)wc.sta.ssid, ssid, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, password ? password : "", sizeof(wc.sta.password));
    wc.sta.threshold.authmode = (password && password[0]) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    // The Pi's AP is hidden; scan every channel for it instead of relying on
    // broadcast probe responses.
    wc.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wc);
    if (err != ESP_OK) { ESP_LOGE(TAG, "set_config: %d", err); return err; }

    s_retry = 0;
    esp_wifi_disconnect();
    err = esp_wifi_connect();
    if (err != ESP_OK) { ESP_LOGE(TAG, "connect: %d", err); return err; }

    nvs_save_str("ssid", ssid);
    nvs_save_str("pass", password ? password : "");
    s_has_creds = true;
    ESP_LOGI(TAG, "connecting to \"%s\"", ssid);
    return ESP_OK;
}

bool wifi_udp_has_creds(void) { return s_has_creds; }

esp_err_t wifi_udp_forget(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    portENTER_CRITICAL(&s_lock);
    s_has_target = false;
    s_last_welcome_us = 0;
    portEXIT_CRITICAL(&s_lock);

    portENTER_CRITICAL(&s_pend_lock);
    memset(s_pending, 0, sizeof(s_pending));   // alerts belong to the old Pi
    portEXIT_CRITICAL(&s_pend_lock);

    s_has_creds = false;   // stops the disconnect handler from reconnecting
    s_expected_pi_id = 0;
    s_pi_id = 0;
    s_wearable_id = s_default_wid;
    s_retry = 0;
    esp_wifi_disconnect();
    ESP_LOGW(TAG, "provisioning erased; WiFi disconnected");
    return ESP_OK;
}

void wifi_udp_set_forget_cb(void (*cb)(void)) { s_forget_cb = cb; }
void wifi_udp_set_link_cb(void (*cb)(bool))   { s_link_cb = cb; }
void wifi_udp_set_mode(uint8_t mode)          { s_mode = mode; }

esp_err_t wifi_udp_set_target(const char *ip, uint16_t port)
{
    if (!ip || port == 0) return ESP_ERR_INVALID_ARG;

    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(port);
    if (inet_pton(AF_INET, ip, &dest.sin_addr) != 1) {
        ESP_LOGE(TAG, "bad target IP: %s", ip);
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    s_dest       = dest;
    s_has_target = true;
    s_last_welcome_us = 0;   // force re-verification against the new target
    portEXIT_CRITICAL(&s_lock);

    nvs_save_str("ip", ip);
    nvs_save_u16("port", port);
    ESP_LOGI(TAG, "target set: %s:%u", ip, port);
    return ESP_OK;
}

esp_err_t wifi_udp_set_wearable_id(uint16_t id)
{
    s_wearable_id = id;
    nvs_save_u16("wid", id);
    ESP_LOGI(TAG, "wearable_id = %u", id);
    return ESP_OK;
}

uint16_t wifi_udp_get_wearable_id(void) { return s_wearable_id; }

esp_err_t wifi_udp_set_expected_pi_id(uint32_t pi_id)
{
    s_expected_pi_id = pi_id;
    nvs_save_u32("epid", pi_id);
    portENTER_CRITICAL(&s_lock);
    s_last_welcome_us = 0;   // re-verify against the new expectation
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "expected_pi_id = %lu", (unsigned long)pi_id);
    return ESP_OK;
}

bool wifi_udp_is_connected(void) { return s_connected; }
bool wifi_udp_has_target(void)   { return s_has_target; }

bool wifi_udp_is_verified(void)
{
    int64_t lw;
    portENTER_CRITICAL(&s_lock);
    lw = s_last_welcome_us;
    portEXIT_CRITICAL(&s_lock);
    if (lw == 0) return false;
    return (esp_timer_get_time() - lw) < (int64_t)VERIFY_TIMEOUT_MS * 1000;
}

uint32_t wifi_udp_get_pi_id(void) { return s_pi_id; }

void wifi_udp_get_ip(char *buf, size_t n) { strlcpy(buf, s_ip_str, n); }

esp_err_t wifi_udp_send_imu_bio(const mibs_message *m, float temp,
                                float hr, float spo2, float resp, float hrv)
{
    if (!m) return ESP_ERR_INVALID_ARG;

    struct sockaddr_in dest;
    if (!s_connected || !get_dest(&dest)) return ESP_ERR_INVALID_STATE;

    // Three tasks can reach this now (IMU, mock playback timer, app_ctrl), so
    // the sequence counter is no longer single-writer.
    uint32_t seq;
    portENTER_CRITICAL(&s_lock);
    seq = s_seq++;
    portEXIT_CRITICAL(&s_lock);

    mibs_imu_pkt_t pkt = {};
    pkt.hdr = { MIBS_MSG_IMU, MIBS_MSG_VERSION, s_wearable_id };
    pkt.seq                = seq;
    pkt.t_ms               = (uint32_t)(esp_timer_get_time() / 1000);
    pkt.impact_count       = m->impact_count;
    pkt.impact_threshold   = m->impact_threshold;
    pkt.impact_accumulator = m->impact_accumulator;
    pkt.all_time_peak_g    = m->all_time_peak_g;
    pkt.temp_c             = temp;
    pkt.hr = hr; pkt.spo2 = spo2; pkt.resp = resp; pkt.hrv = hrv;
    pkt.mode               = s_mode;

    int n = sendto(s_sock, &pkt, sizeof(pkt), 0,
                   (struct sockaddr *)&dest, sizeof(dest));
    return (n == (int)sizeof(pkt)) ? ESP_OK : ESP_FAIL;
}

esp_err_t wifi_udp_send_imu(const mibs_message *m)
{
    // Real sensor path: no biometric channels.
    return wifi_udp_send_imu_bio(m, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

esp_err_t wifi_udp_send_alert(const mibs_impact_t *imp)
{
    if (!imp) return ESP_ERR_INVALID_ARG;

    struct sockaddr_in dest;
    if (!s_connected || !get_dest(&dest)) return ESP_ERR_INVALID_STATE;

    mibs_alert_pkt_t pkt = {};
    pkt.hdr = { MIBS_MSG_ALERT, MIBS_MSG_VERSION, s_wearable_id };
    pkt.seq         = imp->seq;
    pkt.t_ms        = (uint32_t)(imp->t_us / 1000);
    pkt.peak_g      = imp->peak_g;
    pkt.threshold_g = imp->threshold_g;
    pkt.hx_g = imp->hx_g; pkt.hy_g = imp->hy_g; pkt.hz_g = imp->hz_g;
    pkt.gx_dps = imp->gx_dps; pkt.gy_dps = imp->gy_dps; pkt.gz_dps = imp->gz_dps;
    pkt.dur_ms = imp->dur_ms;
    pkt.mode   = imp->mode;
    pkt.xport  = imp->xport;

    // Park it for retransmission before the first send, so an ack that races
    // back cannot arrive before there is anything to match it against.
    int slot = -1;
    portENTER_CRITICAL(&s_pend_lock);
    for (int i = 0; i < ALERT_PENDING_MAX; i++) {
        if (!s_pending[i].used) { slot = i; break; }
    }
    if (slot >= 0) {
        s_pending[slot].pkt        = pkt;
        s_pending[slot].last_tx_us = esp_timer_get_time();
        s_pending[slot].tries      = 1;
        s_pending[slot].used       = true;
    }
    portEXIT_CRITICAL(&s_pend_lock);

    if (slot < 0) {
        ESP_LOGE(TAG, "alert pending queue full — caller must buffer");
        return ESP_ERR_NO_MEM;
    }

    int n = sendto(s_sock, &pkt, sizeof(pkt), 0,
                   (struct sockaddr *)&dest, sizeof(dest));
    if (n != (int)sizeof(pkt)) {
        ESP_LOGW(TAG, "alert %lu first send failed — will retry",
                 (unsigned long)pkt.seq);
    }
    return ESP_OK;   // accepted for reliable delivery
}

uint8_t wifi_udp_alerts_pending(void)
{
    uint8_t n = 0;
    portENTER_CRITICAL(&s_pend_lock);
    for (int i = 0; i < ALERT_PENDING_MAX; i++) if (s_pending[i].used) n++;
    portEXIT_CRITICAL(&s_pend_lock);
    return n;
}