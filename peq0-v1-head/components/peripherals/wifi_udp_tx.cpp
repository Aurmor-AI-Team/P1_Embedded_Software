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
//   uint8  msg_type   (1=IMU, 2=HELLO, 3=WELCOME)
//   uint8  version    (MSG_VERSION)
//   uint16 wearable_id
//
// IMU packet (52 bytes) = header + :
//   uint32 seq, uint32 t_ms, float ax,ay,az, gx,gy,gz, hx,hy,hz, temp_c
// HELLO (8 bytes, wearable->Pi)   = header + uint32 nonce
// WELCOME (12 bytes, Pi->wearable)= header + uint32 nonce + uint32 pi_id
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

#define MSG_VERSION  1
#define MSG_IMU      1
#define MSG_HELLO    2
#define MSG_WELCOME  3

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

static volatile bool s_connected = false;
static char          s_ip_str[16] = "0.0.0.0";
static int           s_sock       = -1;
static struct sockaddr_in s_dest  = {};
static volatile bool s_has_target = false;
static portMUX_TYPE  s_lock       = portMUX_INITIALIZER_UNLOCKED;
static uint32_t      s_seq        = 0;   // touched only by the TX task
static int           s_retry      = 0;

static uint16_t      s_wearable_id     = 0;
static uint32_t      s_expected_pi_id  = 0;
static volatile uint32_t s_pi_id       = 0;
static volatile int64_t  s_last_welcome_us = 0;
static uint32_t      s_hello_nonce     = 0;

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
        s_connected = false;
        strcpy(s_ip_str, "0.0.0.0");
        if (s_retry < WIFI_MAX_RETRY) {
            s_retry++;
            esp_wifi_connect();
            ESP_LOGW(TAG, "disconnected, retry %d/%d", s_retry, WIFI_MAX_RETRY);
        } else {
            ESP_LOGE(TAG, "giving up reconnect (re-provision to retry)");
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        esp_ip4addr_ntoa(&e->ip_info.ip, s_ip_str, sizeof(s_ip_str));
        s_connected = true;
        s_retry = 0;
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

        struct sockaddr_in src;
        socklen_t slen = sizeof(src);
        int n = recvfrom(s_sock, buf, sizeof(buf), 0,
                         (struct sockaddr *)&src, &slen);
        if (n < (int)sizeof(msg_header_t)) continue;   // timeout or runt

        msg_header_t *h = (msg_header_t *)buf;
        if (h->msg_type == MSG_WELCOME && n >= (int)sizeof(welcome_packet_t)) {
            handle_welcome((welcome_packet_t *)buf);
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

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wc);
    if (err != ESP_OK) { ESP_LOGE(TAG, "set_config: %d", err); return err; }

    s_retry = 0;
    esp_wifi_disconnect();
    err = esp_wifi_connect();
    if (err != ESP_OK) { ESP_LOGE(TAG, "connect: %d", err); return err; }

    nvs_save_str("ssid", ssid);
    nvs_save_str("pass", password ? password : "");
    ESP_LOGI(TAG, "connecting to \"%s\"", ssid);
    return ESP_OK;
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

esp_err_t wifi_udp_send_imu(const lsm6_sample_t *s)
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
    };

    int n = sendto(s_sock, &pkt, sizeof(pkt), 0,
                   (struct sockaddr *)&dest, sizeof(dest));
    return (n == (int)sizeof(pkt)) ? ESP_OK : ESP_FAIL;
}
