// ---------------------------------------------------------------------------
// wifi_udp_tx.cpp — Wi-Fi station + UDP transmit/handshake path for IMU data.
//
// Joins a normal Wi-Fi network and sends IMU packets as UDP datagrams to a
// provisioned IP:port. Each packet carries a wearable ID so the receiver can
// tell multiple sensors apart. A periodic HELLO/WELCOME handshake confirms the
// link is live and learns the Pi's ID (round-trip connection test).
// Credentials, target, wearable ID, and expected Pi ID persist in NVS.
//
// All messages share a 4-byte header:
//   uint8  msg_type   (1=IMU, 2=HELLO, 3=WELCOME, 4=FORGET)
//   uint8  version    (MSG_VERSION)
//   uint16 wearable_id
//
// IMU packet (68 bytes) = header + :
//   uint32 seq, uint32 t_ms, float ax,ay,az, gx,gy,gz, hx,hy,hz, temp_c,
//   float hr, spo2, resp, hrv   (the 4 bio values are 0 from the real sensor,
//   filled by the mock playback from chest/wrist reference data)
// HELLO (8 bytes, wearable->Pi)   = header + uint32 nonce
// WELCOME (12 bytes, Pi->wearable)= header + uint32 nonce + uint32 pi_id
// FORGET (8 bytes, Pi->wearable)  = header + uint32 pi_id
//   "unpair": erase stored credentials and drop off the WiFi. Accepted only
//   when the wearable_id targets us (or is 0) and pi_id matches the last
//   WELCOME (or we have not verified a Pi yet).
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
// How long the link may be dead before we call the receiver gone and hand the
// board back to Bluetooth. Long enough to ride out a Pi reboot, a walk out of
// range, and the ghost-association retry storm after an ungraceful power-off
// (~35 association attempts); short enough that a user isn't left waiting.
#define ORPHAN_TIMEOUT_MS   90000
// Reconnect interval once orphaned. Sparse on purpose: BLE is advertising by
// then and the two radios share one front end.
#define BACKGROUND_RETRY_MS 30000

// Re-provisioning has to wait for the station to actually go idle before the
// driver will accept a new config; the disconnect that gets it there is async.
// ~2 s of headroom, well inside the app's 30 s join timeout, and skipped
// entirely on a board that wasn't already retrying.
#define SET_CONFIG_RETRIES   20
#define SET_CONFIG_RETRY_MS  100

// Version stays 1. We only ADD message types; the IMU packet layout is
// untouched. udp_source.py drops any packet whose version != 1, so bumping this
// would strand every board already in the field against an un-upgraded Pi and
// vice versa.
#define MSG_VERSION  1
#define MSG_IMU      1
#define MSG_HELLO    2
#define MSG_WELCOME  3
#define MSG_FORGET   4
#define MSG_ALERT      5   // wearable -> Pi, one head impact. Acked.
#define MSG_ALERT_ACK  6   // Pi -> wearable, echoes the alert seq

// Impacts are sparse and individually meaningful, so unlike IMU samples they
// are retransmitted until acknowledged. The Pi MUST ack every alert — including
// duplicates — or a board will keep resending one forever.
#define ALERT_PENDING_MAX  8
// Retry cadence is chosen against udp_rx_task's 500 ms SO_RCVTIMEO: the loop
// wakes at least that often, so 600 ms fires exactly once per wake.
#define ALERT_RETRY_MS     600
#define ALERT_MAX_TRIES    6    // ~3.6 s of cover before we give up and log

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;
    uint8_t  version;
    uint16_t wearable_id;
} msg_header_t;

typedef struct __attribute__((packed)) {
    msg_header_t hdr;
    uint32_t seq;
    uint32_t t_ms;
    float    ax, ay, az;
    float    gx, gy, gz;
    float    hx, hy, hz;
    float    temp_c;
    // Mock biometrics (0 from the real IMU): heart rate, SpO2, respiration, HRV.
    float    hr, spo2, resp, hrv;
} imu_packet_t;

typedef struct __attribute__((packed)) {
    msg_header_t hdr;
    uint32_t nonce;
} hello_packet_t;

typedef struct __attribute__((packed)) {
    msg_header_t hdr;
    uint32_t nonce;
    uint32_t pi_id;
} welcome_packet_t;

typedef struct __attribute__((packed)) {
    msg_header_t hdr;
    uint32_t pi_id;
} forget_packet_t;

// 34 bytes. Floats rather than scaled ints, matching imu_packet_t's style on
// this link — the Pi rescales when it re-encodes for the phone.
typedef struct __attribute__((packed)) {
    msg_header_t hdr;
    uint32_t seq;          // ack key, monotonic per boot
    uint32_t t_ms;
    float    peak_g;
    float    threshold_g;
    float    sum_g;
    float    max_g;
    uint32_t count;
    uint16_t dur_ms;
} alert_packet_t;

// 8 bytes — comfortably inside udp_rx_task's 64-byte receive buffer.
typedef struct __attribute__((packed)) {
    msg_header_t hdr;
    uint32_t seq;          // echoes the alert being acknowledged
} alert_ack_packet_t;

_Static_assert(sizeof(alert_packet_t) == 34, "alert_packet_t must match udp_source.py ALERT");
_Static_assert(sizeof(alert_ack_packet_t) == 8, "alert_ack_packet_t must match udp_source.py ALERT_ACK");

static volatile bool s_connected = false;
static char          s_ip_str[16] = "0.0.0.0";
static int           s_sock       = -1;
static struct sockaddr_in s_dest  = {};
static volatile bool s_has_target = false;
static portMUX_TYPE  s_lock       = portMUX_INITIALIZER_UNLOCKED;
static uint32_t      s_seq        = 0;   // touched only by the TX task
static int           s_retry      = 0;

// Alerts awaiting an ack. Guarded by s_lock; the sendto itself always happens
// OUTSIDE the critical section.
typedef struct {
    alert_packet_t pkt;
    int64_t        last_tx_us;
    uint8_t        tries;
    bool           used;
} pending_alert_t;
static pending_alert_t s_pending[ALERT_PENDING_MAX];

static uint16_t      s_wearable_id     = 0;
static uint16_t      s_default_wid     = 0;   // MAC-derived, restored on forget
static uint32_t      s_expected_pi_id  = 0;
static volatile uint32_t s_pi_id       = 0;
static volatile int64_t  s_last_welcome_us = 0;
static uint32_t      s_hello_nonce     = 0;
static volatile bool s_has_creds       = false;
static void (*s_forget_cb)(void)      = NULL;
static void (*s_orphan_cb)(void)      = NULL;

// Orphan watchdog. s_last_link_ok_us is the last moment we were demonstrably
// talking to our receiver; the watchdog in the rx task compares against it.
// Seeded at connect time so a board that has never yet been verified still gets
// the full grace period instead of tripping instantly.
static volatile int64_t s_last_link_ok_us = 0;
static volatile bool    s_orphan_reported = false;   // cb already fired for this outage
static volatile wifi_radio_policy_t s_policy = WIFI_RADIO_FOREGROUND;
// True only while wifi_udp_connect() swaps the station config. Gates the
// disconnect handler so our own teardown can't re-arm a connect mid-swap.
static volatile bool s_reconfiguring = false;
// When BACKGROUND, the disconnect handler stops reconnecting inline and the rx
// task retries on this schedule instead.
static volatile int64_t s_next_retry_us = 0;

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
        s_connected = false;
        strcpy(s_ip_str, "0.0.0.0");
        // Only reconnect to a network we're provisioned for; a "forget" clears
        // s_has_creds first, so this also stops us re-joining after unpair.
        if (!s_has_creds) return;
        // Our own disconnect, on the way to installing new credentials —
        // reconnecting here would restart the old config and starve set_config.
        if (s_reconfiguring) return;
        s_retry++;
        // Keep trying indefinitely — a wearable must auto-rejoin its own AP.
        // In particular, after an ungraceful power-off the receiver's AP can
        // hold a stale ("ghost") association for our MAC and reject re-auth
        // (reason 2, "previous auth no longer valid") until it ages the ghost
        // out. Giving up here is what left the board stuck after a power cycle.
        // esp_wifi_connect() re-attempts (~2.4 s each), so this self-throttles.
        //
        // ...but only at FOREGROUND. Once we've been handed back to Bluetooth
        // the retry moves onto the rx task's slow schedule (BACKGROUND), or
        // stops entirely while a phone is streaming from us (PAUSED) — see
        // wifi_radio_policy_t.
        if (s_policy == WIFI_RADIO_FOREGROUND) {
            esp_wifi_connect();
        } else if (s_policy == WIFI_RADIO_BACKGROUND) {
            s_next_retry_us = esp_timer_get_time() + (int64_t)BACKGROUND_RETRY_MS * 1000;
        }
        // reason 2 stale-assoc/auth-expire, 201 NO_AP_FOUND (AP down/out of
        // range), 15/204 wrong password, 205 generic. Log the first burst, then
        // occasionally, so a long outage doesn't spam the console.
        if (s_retry <= WIFI_MAX_RETRY || s_retry % 10 == 0) {
            ESP_LOGW(TAG, "disconnected (reason=%d), reconnecting (attempt %d)",
                     e ? e->reason : -1, s_retry);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        esp_ip4addr_ntoa(&e->ip_info.ip, s_ip_str, sizeof(s_ip_str));
        s_connected = true;
        s_retry = 0;
        // Back on the network: rearm the orphan watchdog. Deliberately NOT
        // clearing s_orphan_reported here — app_ctrl owns that transition, because
        // coming back only matters once it has decided to leave Bluetooth.
        s_last_link_ok_us = esp_timer_get_time();
        ESP_LOGI(TAG, "connected, ip=%s", s_ip_str);
    }
}

// --- Handshake ------------------------------------------------------------
static void send_hello(void)
{
    struct sockaddr_in dest;
    bool has;
    portENTER_CRITICAL(&s_lock);
    has = s_has_target;
    if (has) dest = s_dest;
    portEXIT_CRITICAL(&s_lock);
    if (!has) return;

    s_hello_nonce = esp_random();
    hello_packet_t h = {
        .hdr = { MSG_HELLO, MSG_VERSION, s_wearable_id },
        .nonce = s_hello_nonce,
    };
    sendto(s_sock, &h, sizeof(h), 0, (struct sockaddr *)&dest, sizeof(dest));
}

static void handle_welcome(const welcome_packet_t *w)
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

/**
 * Decide whether our receiver is still there, and drive the slow reconnect once
 * we've concluded it isn't.
 *
 * "Link OK" means associated AND recently verified — both halves matter. An
 * associated board whose Pi has died still has an IP and would otherwise look
 * healthy forever while streaming into a void; conversely a board mid-DHCP is
 * fine and must not be declared orphaned. Runs on the rx task, ~every 200-500 ms.
 */
static void link_watchdog(void)
{
    if (!s_has_creds) return;   // unprovisioned: nothing to be orphaned from

    const int64_t now = esp_timer_get_time();

    if (s_connected && wifi_udp_is_verified()) {
        s_last_link_ok_us = now;
        return;
    }

    // BACKGROUND: the disconnect handler no longer reconnects inline, so the
    // sparse retry happens here.
    if (s_policy == WIFI_RADIO_BACKGROUND && !s_connected &&
            s_next_retry_us != 0 && now >= s_next_retry_us) {
        s_next_retry_us = now + (int64_t)BACKGROUND_RETRY_MS * 1000;
        ESP_LOGI(TAG, "background reconnect attempt");
        esp_wifi_connect();
    }

    if (s_orphan_reported) return;     // already reported this outage
    if (now - s_last_link_ok_us < (int64_t)ORPHAN_TIMEOUT_MS * 1000) return;

    s_orphan_reported = true;
    ESP_LOGW(TAG, "no receiver for %d s — handing back to Bluetooth",
             ORPHAN_TIMEOUT_MS / 1000);
    if (s_orphan_cb) s_orphan_cb();   // must only post an event
}

// ---------------------------------------------------------------------------
// Reliable alerts
// ---------------------------------------------------------------------------

/** Clear a pending alert whose ack came back. Runs on the rx task. */
static void handle_alert_ack(const alert_ack_packet_t *a)
{
    bool found = false;
    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < ALERT_PENDING_MAX; i++) {
        if (s_pending[i].used && s_pending[i].pkt.seq == a->seq) {
            s_pending[i].used = false;
            found = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_lock);
    if (found) ESP_LOGI(TAG, "alert #%lu acked", (unsigned long)a->seq);
}

/** Resend anything unacked that is due. Runs on the rx task, every pass. */
static void retry_pending_alerts(void)
{
    const int64_t now = esp_timer_get_time();

    for (int i = 0; i < ALERT_PENDING_MAX; i++) {
        alert_packet_t pkt;
        struct sockaddr_in dest;
        bool due = false, give_up = false;

        // Copy under the lock; never sendto() inside a critical section.
        portENTER_CRITICAL(&s_lock);
        if (s_pending[i].used && s_has_target &&
                (now - s_pending[i].last_tx_us) >= (int64_t)ALERT_RETRY_MS * 1000) {
            if (s_pending[i].tries >= ALERT_MAX_TRIES) {
                s_pending[i].used = false;
                give_up = true;
                pkt = s_pending[i].pkt;
            } else {
                s_pending[i].tries++;
                s_pending[i].last_tx_us = now;
                pkt  = s_pending[i].pkt;
                dest = s_dest;
                due  = true;
            }
        }
        portEXIT_CRITICAL(&s_lock);

        if (give_up) {
            ESP_LOGE(TAG, "alert #%lu (%.1f g) UNACKED after %d tries — giving up",
                     (unsigned long)pkt.seq, (double)pkt.peak_g, ALERT_MAX_TRIES);
        } else if (due) {
            sendto(s_sock, &pkt, sizeof(pkt), 0,
                   (struct sockaddr *)&dest, sizeof(dest));
        }
    }
}

esp_err_t wifi_udp_send_alert(const impact_rec_t *r)
{
    if (!r) return ESP_ERR_INVALID_ARG;
    if (!s_connected) return ESP_ERR_INVALID_STATE;

    struct sockaddr_in dest;
    bool has;
    portENTER_CRITICAL(&s_lock);
    has = s_has_target;
    if (has) dest = s_dest;
    portEXIT_CRITICAL(&s_lock);
    if (!has) return ESP_ERR_INVALID_STATE;

    alert_packet_t pkt = {
        .hdr         = { MSG_ALERT, MSG_VERSION, s_wearable_id },
        .seq         = r->seq,
        .t_ms        = r->t_ms,
        .peak_g      = r->peak_g,
        .threshold_g = r->threshold_g,
        .sum_g       = r->sum_g,
        .max_g       = r->max_g,
        .count       = r->count,
        .dur_ms      = r->dur_ms,
    };

    // Park it BEFORE the first send: an ack can race back before sendto()
    // returns, and it needs a slot to match against or we'd retransmit an
    // alert the Pi already has.
    int slot = -1;
    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < ALERT_PENDING_MAX; i++) {
        if (!s_pending[i].used) {
            s_pending[i].used       = true;
            s_pending[i].tries      = 1;
            s_pending[i].last_tx_us = esp_timer_get_time();
            s_pending[i].pkt        = pkt;
            slot = i;
            break;
        }
    }
    portEXIT_CRITICAL(&s_lock);

    // Table full: refuse, so the caller backlogs it instead of us dropping it.
    if (slot < 0) return ESP_ERR_NO_MEM;

    int n = sendto(s_sock, &pkt, sizeof(pkt), 0,
                   (struct sockaddr *)&dest, sizeof(dest));
    // A failed first send is not an error the caller should act on — the record
    // is parked and the retry sweep will carry it.
    if (n != (int)sizeof(pkt)) {
        ESP_LOGW(TAG, "alert #%lu first send failed; queued for retry",
                 (unsigned long)pkt.seq);
    }
    return ESP_OK;
}

uint8_t wifi_udp_alerts_pending(void)
{
    uint8_t n = 0;
    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < ALERT_PENDING_MAX; i++) if (s_pending[i].used) n++;
    portEXIT_CRITICAL(&s_lock);
    return n;
}

static void udp_rx_task(void *arg)
{
    struct timeval tv = { .tv_sec = 0, .tv_usec = 500000 };  // 500 ms
    setsockopt(s_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int64_t last_hello = 0;
    uint8_t buf[64];

    while (true) {
        // Runs on EVERY pass, including the not-connected one below — being
        // off the network is precisely the case it exists to catch.
        link_watchdog();
        // Harmless while the link is down (there is no target to send to), and
        // this is the only periodic wake the alert table has.
        retry_pending_alerts();

        if (!s_connected || !s_has_target) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        int64_t now = esp_timer_get_time();
        if (now - last_hello >= (int64_t)HELLO_PERIOD_MS * 1000) {
            send_hello();
            last_hello = now;
        }

        struct sockaddr_in src;
        socklen_t slen = sizeof(src);
        int n = recvfrom(s_sock, buf, sizeof(buf), 0,
                         (struct sockaddr *)&src, &slen);
        if (n < (int)sizeof(msg_header_t)) continue;   // timeout or runt

        msg_header_t *h = (msg_header_t *)buf;
        if (h->msg_type == MSG_WELCOME && n >= (int)sizeof(welcome_packet_t)) {
            handle_welcome((welcome_packet_t *)buf);
        } else if (h->msg_type == MSG_ALERT_ACK && n >= (int)sizeof(alert_ack_packet_t)) {
            alert_ack_packet_t *a = (alert_ack_packet_t *)buf;
            if (a->hdr.version == MSG_VERSION &&
                    (a->hdr.wearable_id == 0 || a->hdr.wearable_id == s_wearable_id)) {
                handle_alert_ack(a);
            }
        } else if (h->msg_type == MSG_FORGET && n >= (int)sizeof(forget_packet_t)) {
            forget_packet_t *f = (forget_packet_t *)buf;
            bool wid_ok = (f->hdr.wearable_id == 0 ||
                           f->hdr.wearable_id == s_wearable_id);
            bool pi_ok  = (s_pi_id == 0 || f->pi_id == s_pi_id);
            if (f->hdr.version == MSG_VERSION && wid_ok && pi_ok) {
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

    ESP_LOGI(TAG, "Wi-Fi/UDP ready (wearable_id=%u, local_port=%d)",
             s_wearable_id, LOCAL_UDP_PORT);
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

    // Stop the station BEFORE touching its config.
    //
    // esp_wifi_set_config() is refused outright while a connect attempt is in
    // flight — "sta is connecting, cannot set config", ESP_ERR_WIFI_CONN. And
    // the board that most needs re-provisioning is exactly the one that is
    // always in that state: a wrong password leaves it retrying forever (the
    // handler below reconnects on every failure), so every attempt to hand it
    // new credentials was rejected, this function returned early, and NOTHING
    // was updated — not the running config, not even NVS. The board then kept
    // failing on the old password while the app reported a join timeout,
    // pointing at the network rather than at the write that never landed.
    //
    // s_reconfiguring suppresses the auto-reconnect our own disconnect would
    // otherwise trigger, which would put the station straight back into
    // "connecting" with the OLD config and re-lose the race.
    s_reconfiguring = true;
    esp_wifi_disconnect();

    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < SET_CONFIG_RETRIES; attempt++) {
        err = esp_wifi_set_config(WIFI_IF_STA, &wc);
        if (err == ESP_OK) break;
        // The disconnect is asynchronous; give it a moment to land. Costs
        // nothing on an idle board — the first call succeeds there.
        vTaskDelay(pdMS_TO_TICKS(SET_CONFIG_RETRY_MS));
    }
    s_reconfiguring = false;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_config: %d (station would not go idle)", err);
        return err;
    }

    s_retry = 0;
    // An explicit "join this network" — from provisioning, or from the NVS
    // restore at boot — always chases it at full speed, whatever gear a previous
    // outage left us in. Without this, provisioning a board that is currently
    // orphaned would inherit the 30 s backoff (or PAUSED) and appear to hang.
    s_policy = WIFI_RADIO_FOREGROUND;
    // Give a fresh association the full grace period before the watchdog can
    // call it orphaned — at this point we have never been verified.
    s_last_link_ok_us = esp_timer_get_time();
    s_orphan_reported = false;
    s_next_retry_us   = 0;
    err = esp_wifi_connect();  // already disconnected above
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
    // Pending alerts belong to the receiver we just left. Retransmitting them
    // to whoever inherits that address would attribute one athlete's impacts
    // to another session.
    memset(s_pending, 0, sizeof(s_pending));
    portEXIT_CRITICAL(&s_lock);
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
void wifi_udp_set_orphan_cb(void (*cb)(void)) { s_orphan_cb = cb; }

void wifi_udp_set_radio_policy(wifi_radio_policy_t policy)
{
    if (s_policy == policy) return;
    s_policy = policy;
    ESP_LOGI(TAG, "radio policy: %s",
             policy == WIFI_RADIO_FOREGROUND ? "foreground" :
             policy == WIFI_RADIO_BACKGROUND ? "background" : "paused");

    switch (policy) {
    case WIFI_RADIO_FOREGROUND:
        // Chase the network again, starting now, and give the watchdog a fresh
        // window so re-entering the provisioned state can't instantly re-trip it.
        s_next_retry_us   = 0;
        s_last_link_ok_us = esp_timer_get_time();
        s_orphan_reported        = false;
        if (s_has_creds && !s_connected) esp_wifi_connect();
        break;
    case WIFI_RADIO_BACKGROUND:
        // First sparse attempt one interval out; the rx task takes it from here.
        s_next_retry_us = esp_timer_get_time() + (int64_t)BACKGROUND_RETRY_MS * 1000;
        break;
    case WIFI_RADIO_PAUSED:
        // Stop cold: a BLE session is streaming and must own the front end.
        s_next_retry_us = 0;
        esp_wifi_disconnect();
        break;
    }
}

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

esp_err_t wifi_udp_send_imu_bio(const lsm6_sample_t *s,
                                float hr, float spo2, float resp, float hrv)
{
    if (!s) return ESP_ERR_INVALID_ARG;
    if (!s_connected) return ESP_OK;

    struct sockaddr_in dest;
    bool has;
    portENTER_CRITICAL(&s_lock);
    has = s_has_target;
    if (has) dest = s_dest;
    portEXIT_CRITICAL(&s_lock);
    if (!has) return ESP_OK;

    imu_packet_t pkt = {
        .hdr   = { MSG_IMU, MSG_VERSION, s_wearable_id },
        .seq   = s_seq++,
        .t_ms  = (uint32_t)(esp_timer_get_time() / 1000),
        .ax = s->ax_g,   .ay = s->ay_g,   .az = s->az_g,
        .gx = s->gx_dps, .gy = s->gy_dps, .gz = s->gz_dps,
        .hx = s->hx_g,   .hy = s->hy_g,   .hz = s->hz_g,
        .temp_c = s->temp_c,
        .hr = hr, .spo2 = spo2, .resp = resp, .hrv = hrv,
    };

    int n = sendto(s_sock, &pkt, sizeof(pkt), 0,
                   (struct sockaddr *)&dest, sizeof(dest));
    return (n == (int)sizeof(pkt)) ? ESP_OK : ESP_FAIL;
}

esp_err_t wifi_udp_send_imu(const lsm6_sample_t *s)
{
    // Real sensor path: no biometric channels.
    return wifi_udp_send_imu_bio(s, 0.0f, 0.0f, 0.0f, 0.0f);
}
