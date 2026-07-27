// ---------------------------------------------------------------------------
// app_ctrl.cpp — device mode state machine.
//
// Modes compose a transport (BT / WiFi) with a data policy (ALERTS / LIVE /
// MOCK), plus the standalone IDLE and PAIRING states. Transport follows link
// state automatically; policy follows the user or the app.
//
// Every input (button, GOT_IP, WiFi down, BLE link up/down, Pi FORGET, impact
// detection, app control write) is posted as an event to one queue and handled
// sequentially in one task, so BLE/WiFi lifecycle calls never race.
//
// The user LED (GPIO15, active-low) runs an 8-slot x 250 ms pattern per mode,
// with an override burst when an impact fires.
// ---------------------------------------------------------------------------
#include "app_ctrl.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"

#include "ble_provision.h"
#include "button.h"
#include "mock_playback.h"
#include "wifi_udp_tx.h"

static const char *TAG = "app_ctrl";

#define PIN_BOOT_BUTTON  9    // XIAO ESP32-C6 BOOT, active-low
#define PIN_USER_LED     15   // XIAO ESP32-C6 yellow user LED, active-low

// Keeps the "up ip=..." status notify ahead of the mode change so the app sees
// the provisioning result before anything else moves.
#define BLE_STOP_DELAY_MS 2000

// Alerts held when no transport is up, or when a transport refused them.
// Oldest is dropped first — the newest hit is the one that matters.
#define ALERT_BACKLOG_DEPTH 24
#define BACKLOG_RETRY_MS    2000

#define NVS_NS       "appctrl"
#define NVS_K_POLICY "policy"
#define NVS_K_THRESH "thresh"

// --- events -----------------------------------------------------------------
typedef enum {
    EVT_BTN_SHORT,
    EVT_BTN_LONG,
    EVT_GOT_IP,
    EVT_WIFI_DOWN,
    EVT_BLE_UP,
    EVT_BLE_DOWN,
    EVT_FORGET_RX,
    EVT_IMPACT,
    EVT_CONTROL,
    EVT_TICK,          // periodic: retry the backlog, re-evaluate transport
} app_event_id_t;

typedef struct {
    app_event_id_t id;
    union {
        mibs_impact_t impact;
        struct { uint8_t policy; float threshold_g; } ctrl;
    } u;
} app_event_t;

typedef enum { ST_IDLE, ST_PAIRING, ST_RUN } app_state_t;

static QueueHandle_t s_events;
static app_state_t   s_state  = ST_IDLE;
static app_policy_t  s_policy = APP_POLICY_ALERTS;

// Read from other tasks without a lock, so word-sized and volatile.
static volatile app_mode_t  s_mode        = APP_MODE_IDLE;
static volatile app_xport_t s_xport       = APP_XPORT_NONE;
static volatile bool        s_streaming   = false;
static volatile float       s_threshold_g = APP_IMPACT_THRESHOLD_G_DEFAULT;

static mibs_impact_t s_backlog[ALERT_BACKLOG_DEPTH];
static uint8_t       s_backlog_head, s_backlog_count;

static volatile int64_t s_alert_flash_until_us = 0;

// ---------------------------------------------------------------------------
// naming / composition
// ---------------------------------------------------------------------------
const char *app_mode_str(app_mode_t m)
{
    switch (m) {
    case APP_MODE_IDLE:        return "IDLE";
    case APP_MODE_PAIRING:     return "PAIRING";
    case APP_MODE_BT_MOCK:     return "BT_MOCK";
    case APP_MODE_WIFI_MOCK:   return "WIFI_MOCK";
    case APP_MODE_BT_ALERTS:   return "BT_ALERTS";
    case APP_MODE_BT_LIVE:     return "BT_LIVE";
    case APP_MODE_WIFI_ALERTS: return "WIFI_ALERTS";
    case APP_MODE_WIFI_LIVE:   return "WIFI_LIVE";
    }
    return "?";
}

static const char *policy_str(app_policy_t p)
{
    return p == APP_POLICY_ALERTS ? "ALERTS" : p == APP_POLICY_LIVE ? "LIVE" : "MOCK";
}

// WiFi wins when usable; a connected+subscribed BLE client is the fallback.
static app_xport_t select_xport(void)
{
    if (wifi_udp_is_connected() && wifi_udp_has_target()) return APP_XPORT_WIFI;
    if (ble_provision_is_connected())                     return APP_XPORT_BT;
    return APP_XPORT_NONE;
}

// A transport-less device still has a mode — it keeps detecting and buffering.
// Report it under the BT_* names, since BLE is what comes back first.
static app_mode_t compose(app_xport_t x, app_policy_t p)
{
    const bool wifi = (x == APP_XPORT_WIFI);
    switch (p) {
    case APP_POLICY_MOCK: return wifi ? APP_MODE_WIFI_MOCK : APP_MODE_BT_MOCK;
    case APP_POLICY_LIVE: return wifi ? APP_MODE_WIFI_LIVE : APP_MODE_BT_LIVE;
    default:              return wifi ? APP_MODE_WIFI_ALERTS : APP_MODE_BT_ALERTS;
    }
}

// ---------------------------------------------------------------------------
// NVS-persisted settings
// ---------------------------------------------------------------------------
static void settings_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;

    uint8_t p = APP_POLICY_ALERTS;
    if (nvs_get_u8(h, NVS_K_POLICY, &p) == ESP_OK && p <= APP_POLICY_MOCK) {
        // Never boot into MOCK. It suppresses the live telemetry path, and a
        // wearable that silently stops reporting because it rebooted in demo
        // mode is the one failure this product cannot ship with.
        s_policy = (p == APP_POLICY_MOCK) ? APP_POLICY_ALERTS : (app_policy_t)p;
    }

    uint32_t raw = 0;
    if (nvs_get_u32(h, NVS_K_THRESH, &raw) == ESP_OK) {
        float g;
        memcpy(&g, &raw, sizeof(g));
        if (g >= APP_IMPACT_THRESHOLD_G_MIN && g <= APP_IMPACT_THRESHOLD_G_MAX) {
            s_threshold_g = g;
        }
    }
    nvs_close(h);
}

static void settings_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, NVS_K_POLICY, (uint8_t)s_policy);
    uint32_t raw;
    float g = s_threshold_g;
    memcpy(&raw, &g, sizeof(raw));
    nvs_set_u32(h, NVS_K_THRESH, raw);
    nvs_commit(h);
    nvs_close(h);
}

// ---------------------------------------------------------------------------
// event producers (each only posts to the queue)
// ---------------------------------------------------------------------------
static void post_simple(app_event_id_t id)
{
    if (!s_events) return;
    app_event_t e = {};
    e.id = id;
    xQueueSend(s_events, &e, 0);
}

static void button_cb(button_event_t evt)
{
    post_simple(evt == BUTTON_EVT_LONG ? EVT_BTN_LONG : EVT_BTN_SHORT);
}

static void forget_cb(void) { post_simple(EVT_FORGET_RX); }

static void wifi_link_cb(bool up) { post_simple(up ? EVT_GOT_IP : EVT_WIFI_DOWN); }

static void ble_link_cb(bool up)  { post_simple(up ? EVT_BLE_UP : EVT_BLE_DOWN); }

static void ip_event_cb(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base != IP_EVENT) return;
    if (id == IP_EVENT_STA_GOT_IP)  post_simple(EVT_GOT_IP);
    if (id == IP_EVENT_STA_LOST_IP) post_simple(EVT_WIFI_DOWN);
}

// App wrote the BLE control characteristic (runs on the NimBLE host task).
static void ble_control_cb(uint8_t policy, float threshold_g)
{
    if (!s_events) return;
    app_event_t e = {};
    e.id = EVT_CONTROL;
    e.u.ctrl.policy      = policy;
    e.u.ctrl.threshold_g = threshold_g;
    xQueueSend(s_events, &e, 0);
}

void app_ctrl_report_impact(const mibs_impact_t *imp)
{
    if (!s_events || !imp) return;
    app_event_t e = {};
    e.id = EVT_IMPACT;
    e.u.impact = *imp;
    e.u.impact.mode = (uint8_t)s_mode;
    // The one event worth blocking briefly for rather than dropping.
    xQueueSend(s_events, &e, pdMS_TO_TICKS(10));
}

void app_ctrl_set_policy(app_policy_t p)
{
    ble_control_cb((uint8_t)p, 0.0f);      // 0 = leave threshold alone
}

void app_ctrl_set_threshold_g(float g)
{
    ble_control_cb((uint8_t)s_policy, g);
}

// ---------------------------------------------------------------------------
// LED — 8 slots x 250 ms = 2 s cycle, bit 0 = slot 0. Indexed by app_mode_t.
// ---------------------------------------------------------------------------
static const uint8_t k_led_pattern[] = {
    0x00,   // IDLE        off
    0xAA,   // PAIRING     2 Hz blink
    0x03,   // BT_MOCK     500 ms on, 1.5 s off
    0x0F,   // WIFI_MOCK   1 s on, 1 s off
    0x01,   // BT_ALERTS   single 250 ms blip (armed, low power)
    0x05,   // BT_LIVE     double blip
    0xFE,   // WIFI_ALERTS solid with a 250 ms wink
    0xFF,   // WIFI_LIVE   solid
};
static_assert(sizeof(k_led_pattern) == APP_MODE_WIFI_LIVE + 1,
              "LED pattern table out of sync with app_mode_t");

#define ALERT_FLASH_MS 900

static void led_set(bool on)
{
    gpio_set_level((gpio_num_t)PIN_USER_LED, on ? 0 : 1);   // active-low
}

static void led_tick_cb(void *arg)
{
    static uint32_t tick = 0;
    tick++;

    // Impact override: rapid burst, visible in any mode.
    if (esp_timer_get_time() < s_alert_flash_until_us) {
        led_set(tick & 1);
        return;
    }

    app_mode_t m = s_mode;
    uint8_t pattern = (m <= APP_MODE_WIFI_LIVE) ? k_led_pattern[m] : 0x00;
    led_set((pattern >> (tick & 7)) & 1);
}

static void led_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_USER_LED,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    led_set(false);

    static esp_timer_handle_t led_timer;
    const esp_timer_create_args_t targs = {
        .callback = led_tick_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "led_tick",
        .skip_unhandled_events = true,
    };
    if (esp_timer_create(&targs, &led_timer) == ESP_OK) {
        esp_timer_start_periodic(led_timer, 250 * 1000);
    }
}

// Wakes the control task periodically so the alert backlog drains and
// transport changes that produced no event still get noticed.
static void housekeep_tick_cb(void *arg) { post_simple(EVT_TICK); }

static void housekeep_init(void)
{
    static esp_timer_handle_t t;
    const esp_timer_create_args_t targs = {
        .callback = housekeep_tick_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "app_tick",
        .skip_unhandled_events = true,
    };
    if (esp_timer_create(&targs, &t) == ESP_OK) {
        esp_timer_start_periodic(t, BACKLOG_RETRY_MS * 1000);
    }
}

// ---------------------------------------------------------------------------
// transport dispatch
// ---------------------------------------------------------------------------
bool app_ctrl_stream_enabled(void) { return s_streaming; }

esp_err_t app_ctrl_send_stream(const mibs_message *m, float temp,
                               float hr, float spo2, float resp, float hrv)
{
    switch (s_xport) {
    case APP_XPORT_WIFI: return wifi_udp_send_imu_bio(m, temp, hr, spo2, resp, hrv);
    case APP_XPORT_BT:   return ble_provision_send_stream(m, temp, hr, spo2, resp, hrv);
    default:             return ESP_ERR_INVALID_STATE;
    }
}

static void backlog_push(const mibs_impact_t *imp)
{
    uint8_t tail = (uint8_t)((s_backlog_head + s_backlog_count) % ALERT_BACKLOG_DEPTH);
    s_backlog[tail] = *imp;
    if (s_backlog_count < ALERT_BACKLOG_DEPTH) {
        s_backlog_count++;
    } else {
        s_backlog_head = (uint8_t)((s_backlog_head + 1) % ALERT_BACKLOG_DEPTH);
        ESP_LOGE(TAG, "alert backlog full — DROPPED an impact record");
    }
}

static esp_err_t send_alert(const mibs_impact_t *imp)
{
    mibs_impact_t out = *imp;
    out.xport = (uint8_t)s_xport;
    switch (s_xport) {
    case APP_XPORT_WIFI: return wifi_udp_send_alert(&out);
    case APP_XPORT_BT:   return ble_provision_send_alert(&out);
    default:             return ESP_ERR_INVALID_STATE;
    }
}

static void backlog_flush(void)
{
    while (s_backlog_count > 0) {
        if (send_alert(&s_backlog[s_backlog_head]) != ESP_OK) return;  // retry later
        s_backlog_head = (uint8_t)((s_backlog_head + 1) % ALERT_BACKLOG_DEPTH);
        s_backlog_count--;
    }
}

// ---------------------------------------------------------------------------
// BLE callbacks (run on the NimBLE host task)
// ---------------------------------------------------------------------------
static void on_provision(const char *ssid, const char *password,
                         const char *ip, uint16_t port)
{
    wifi_udp_set_target(ip, port);
    wifi_udp_connect(ssid, password);
}

static void on_wearable_id(uint16_t id)     { wifi_udp_set_wearable_id(id); }
static void on_expected_pi_id(uint32_t id)  { wifi_udp_set_expected_pi_id(id); }

static void status_getter(char *buf, size_t n)
{
    char ip[16];
    wifi_udp_get_ip(ip, sizeof(ip));
    snprintf(buf, n, "%s ip=%s wid=%u %s pi=%lu mode=%s thr=%.1f pend=%u/%u",
             wifi_udp_is_connected() ? "up" : "down",
             ip,
             wifi_udp_get_wearable_id(),
             !wifi_udp_has_target() ? "notgt"
               : wifi_udp_is_verified() ? "verified" : "unverified",
             (unsigned long)wifi_udp_get_pi_id(),
             app_mode_str(s_mode),
             (double)s_threshold_g,
             (unsigned)s_backlog_count,
             (unsigned)wifi_udp_alerts_pending());
}

static const ble_provision_cfg_t s_ble_cfg = {
    .on_provision      = on_provision,
    .on_wearable_id    = on_wearable_id,
    .on_expected_pi_id = on_expected_pi_id,
    .status_getter     = status_getter,
    .on_link           = ble_link_cb,
    .on_control        = ble_control_cb,
};

// ---------------------------------------------------------------------------
// mode application — call after anything that could change link state,
// policy, or playback state. Control task only.
// ---------------------------------------------------------------------------
static void apply_mode(void)
{
    if (s_state != ST_RUN) {
        s_streaming = false;
        s_xport = APP_XPORT_NONE;
        s_mode  = (s_state == ST_PAIRING) ? APP_MODE_PAIRING : APP_MODE_IDLE;
        wifi_udp_set_mode((uint8_t)s_mode);
        ble_provision_set_mode((uint8_t)s_mode);
        return;
    }

    app_xport_t x = select_xport();
    app_mode_t  m = compose(x, s_policy);

    // MOCK owns the wire: the live stream stays off so the Pi never sees CSV
    // rows and real samples interleaved.
    s_streaming = (s_policy == APP_POLICY_LIVE) &&
                  (x != APP_XPORT_NONE) &&
                  !mock_playback_is_active();

    bool xport_changed = (x != s_xport);
    s_xport = x;

    if (m != s_mode) {
        ESP_LOGI(TAG, "mode: %s -> %s (policy=%s, thresh=%.1f g)",
                 app_mode_str(s_mode), app_mode_str(m),
                 policy_str(s_policy), (double)s_threshold_g);
        s_mode = m;
        wifi_udp_set_mode((uint8_t)m);
        ble_provision_set_mode((uint8_t)m);
        ble_provision_push_status();
    }

    if (xport_changed && x != APP_XPORT_NONE) backlog_flush();

    // Radio power: an idle alerts link can sleep between beacons, live cannot.
    // apply_mode() runs on every housekeeping tick, so only touch this on a
    // real change — esp_wifi_set_ps() renegotiates with the AP.
    if (x == APP_XPORT_WIFI) {
        static wifi_ps_type_t last_ps = WIFI_PS_NONE;
        wifi_ps_type_t want = (s_policy == APP_POLICY_ALERTS) ? WIFI_PS_MAX_MODEM
                                                              : WIFI_PS_NONE;
        if (want != last_ps) { esp_wifi_set_ps(want); last_ps = want; }
    }
}

static void set_policy(app_policy_t p)
{
    if (p > APP_POLICY_MOCK || p == s_policy) return;

    if (s_policy == APP_POLICY_MOCK) mock_playback_stop();
    s_policy = p;
    apply_mode();                        // sink must point at the right
    if (s_policy == APP_POLICY_MOCK) {   // transport before playback starts
        mock_playback_start();
        apply_mode();                    // playback now active -> stream off
    }
    settings_save();
}

// Short-press cycles ALERTS -> LIVE -> MOCK -> ALERTS.
static void cycle_policy(void)
{
    set_policy(s_policy == APP_POLICY_ALERTS ? APP_POLICY_LIVE
             : s_policy == APP_POLICY_LIVE   ? APP_POLICY_MOCK
                                             : APP_POLICY_ALERTS);
}

// ---------------------------------------------------------------------------
// state machine
// ---------------------------------------------------------------------------
static void enter_pairing(void)
{
    if (ble_provision_start(&s_ble_cfg) == ESP_OK) {
        s_state = ST_PAIRING;
        apply_mode();
        ESP_LOGI(TAG, "mode: PAIRING (BLE advertising)");
    } else {
        ESP_LOGE(TAG, "BLE start failed — staying in %s", app_mode_str(s_mode));
    }
}

static void enter_idle(const char *why)
{
    mock_playback_stop();
    s_policy = APP_POLICY_ALERTS;
    s_state  = ST_IDLE;
    s_backlog_head = s_backlog_count = 0;
    apply_mode();
    ESP_LOGI(TAG, "mode: IDLE (%s)", why);
}

static void ctrl_task(void *arg)
{
    app_event_t e;
    while (true) {
        if (xQueueReceive(s_events, &e, portMAX_DELAY) != pdTRUE) continue;

        // --- handled the same way in every state ----------------------------
        if (e.id == EVT_IMPACT) {
            s_alert_flash_until_us = esp_timer_get_time() + ALERT_FLASH_MS * 1000;

            if (s_state != ST_RUN || s_policy == APP_POLICY_MOCK) {
                // Not sendable right now — but a real hit during a demo is
                // still a real hit, so hold it rather than discard it.
                backlog_push(&e.u.impact);
                ESP_LOGW(TAG, "impact #%lu %.1f g held (%u queued)",
                         (unsigned long)e.u.impact.seq, (double)e.u.impact.peak_g,
                         (unsigned)s_backlog_count);
            } else if (send_alert(&e.u.impact) != ESP_OK) {
                backlog_push(&e.u.impact);
                ESP_LOGW(TAG, "impact #%lu %.1f g buffered (%u queued)",
                         (unsigned long)e.u.impact.seq, (double)e.u.impact.peak_g,
                         (unsigned)s_backlog_count);
            } else {
                ESP_LOGI(TAG, "impact #%lu %.1f g sent via %s",
                         (unsigned long)e.u.impact.seq, (double)e.u.impact.peak_g,
                         s_xport == APP_XPORT_WIFI ? "wifi" : "bt");
            }
            ble_provision_push_status();
            continue;
        }

        if (e.id == EVT_CONTROL) {
            if (e.u.ctrl.threshold_g >= APP_IMPACT_THRESHOLD_G_MIN &&
                e.u.ctrl.threshold_g <= APP_IMPACT_THRESHOLD_G_MAX) {
                s_threshold_g = e.u.ctrl.threshold_g;
                ESP_LOGI(TAG, "impact threshold -> %.1f g", (double)s_threshold_g);
            }
            if (s_state == ST_RUN) set_policy((app_policy_t)e.u.ctrl.policy);
            else                   settings_save();
            continue;
        }

        if (e.id == EVT_FORGET_RX) {
            // Unpaired from the app via the Pi: drop creds, stop the radios.
            wifi_udp_forget();
            ble_provision_stop();
            enter_idle("unpaired");
            continue;
        }

        if (e.id == EVT_TICK) {
            if (s_state == ST_RUN) {
                apply_mode();
                if (s_xport != APP_XPORT_NONE && s_policy != APP_POLICY_MOCK) {
                    backlog_flush();
                }
                // Playback runs itself to completion; when it ends, drop back
                // to ALERTS so the device does not sit in a demo mode.
                if (s_policy == APP_POLICY_MOCK && !mock_playback_is_active()) {
                    ESP_LOGI(TAG, "playback finished — returning to ALERTS");
                    set_policy(APP_POLICY_ALERTS);
                }
            }
            continue;
        }

        // --- state-specific ---------------------------------------------------
        switch (s_state) {

        case ST_IDLE:
            if (e.id == EVT_BTN_LONG) {
                enter_pairing();
            } else if (e.id == EVT_BTN_SHORT) {
                ESP_LOGI(TAG, "not provisioned — hold BOOT 3 s to pair");
            }
            break;

        case ST_PAIRING:
            if (e.id == EVT_BLE_UP) {
                // A connected, subscribed phone is already a usable transport:
                // the device can run BT_ALERTS before WiFi credentials arrive.
                s_state = ST_RUN;
                apply_mode();
                ESP_LOGI(TAG, "BLE link up — running on BT transport");
            } else if (e.id == EVT_GOT_IP) {
                // Let the app read the "up ..." status before anything moves.
                ble_provision_push_status();
                vTaskDelay(pdMS_TO_TICKS(BLE_STOP_DELAY_MS));
                // BLE is deliberately NOT stopped: the BT_* modes need it as
                // the fallback transport when WiFi drops.
                s_state = ST_RUN;
                apply_mode();
            }
            // BTN_LONG while already advertising: nothing to restart.
            break;

        case ST_RUN:
            switch (e.id) {
            case EVT_BTN_SHORT:
                cycle_policy();
                break;

            case EVT_BTN_LONG:
                // Manual re-pair: forget this network, advertise again.
                mock_playback_stop();
                s_policy = APP_POLICY_ALERTS;
                wifi_udp_forget();
                enter_pairing();
                break;

            case EVT_GOT_IP:
            case EVT_WIFI_DOWN:
            case EVT_BLE_UP:
                apply_mode();          // transport failover, both directions
                break;

            case EVT_BLE_DOWN:
                apply_mode();
                if (s_xport == APP_XPORT_NONE) {
                    ESP_LOGW(TAG, "no transport — impacts will be buffered");
                }
                break;

            default:
                break;
            }
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
esp_err_t app_ctrl_init(void)
{
    s_events = xQueueCreate(16, sizeof(app_event_t));
    if (!s_events) return ESP_ERR_NO_MEM;

    settings_load();
    led_init();

    esp_err_t err = button_init(PIN_BOOT_BUTTON, button_cb);
    if (err != ESP_OK) { ESP_LOGE(TAG, "button_init: %d", err); return err; }

    wifi_udp_set_forget_cb(forget_cb);
    wifi_udp_set_link_cb(wifi_link_cb);

    // Mock frames go out over whichever transport is live, same as real ones.
    mock_playback_set_sink(app_ctrl_send_stream);

    // wifi_udp registers IP_EVENT_STA_GOT_IP for itself; this instance also
    // catches LOST_IP, which nothing else was listening for.
    err = esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                              &ip_event_cb, NULL, NULL);
    if (err != ESP_OK) { ESP_LOGE(TAG, "ip handler: %d", err); return err; }

    // wifi_udp_init() already auto-connected if NVS held credentials.
    s_state = wifi_udp_has_creds() ? ST_RUN : ST_IDLE;
    apply_mode();
    housekeep_init();

    ESP_LOGI(TAG, "boot mode: %s (policy=%s, thresh=%.1f g)",
             app_mode_str(s_mode), policy_str(s_policy), (double)s_threshold_g);

    if (xTaskCreate(ctrl_task, "app_ctrl", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    // A provisioned device should be reachable over BLE too, so the app can
    // change mode and so BT is available the moment WiFi drops.
    if (s_state == ST_RUN) ble_provision_start(&s_ble_cfg);

    return ESP_OK;
}

app_mode_t   app_ctrl_mode(void)        { return s_mode; }
app_xport_t  app_ctrl_xport(void)       { return s_xport; }
app_policy_t app_ctrl_policy(void)      { return s_policy; }
float        app_ctrl_threshold_g(void) { return s_threshold_g; }