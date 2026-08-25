// ---------------------------------------------------------------------------
// app_ctrl.cpp — device mode state machine (IDLE / PAIRING / WIFI).
//
// Every input (button presses, GOT_IP, Pi FORGET) is posted as an event to a
// queue and handled sequentially in one task, so BLE/WiFi lifecycle calls
// never race each other. The user LED (GPIO15, active-low) shows READINESS, not
// which radio is in use: solid = ready (advertising on Bluetooth OR on the
// receiver's WiFi), off = idle/dead. Everything else is an exception — 2 Hz =
// claimable now, 1 Hz = mock playback, double flash = lost its receiver.
// ---------------------------------------------------------------------------
#include "app_ctrl.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include <string.h>

#include "ble_auth.h"
#include "ble_provision.h"
#include "ble_stream.h"
#include "button.h"
#include "impact_det.h"
#include "mock_playback.h"
#include "wifi_udp_tx.h"

static const char *TAG = "app_ctrl";

#define PIN_BOOT_BUTTON  9    // XIAO ESP32-C6 BOOT, active-low
#define PIN_USER_LED     15   // XIAO ESP32-C6 yellow user LED, active-low

// Keeps the "up ip=..." status notify ahead of the BLE teardown so the app
// sees the provisioning result before the link drops.
#define BLE_STOP_DELAY_MS 2000

// How long a BOOT long-press leaves this board claimable. Long enough to pair
// unhurriedly, short enough that a board left advertising isn't claimable by
// whoever walks past an hour later. This window is the ONLY way to obtain the
// enrolment secret, so it is also the lost-phone recovery path (ble_auth.h).
#define ENROL_WINDOW_MS 120000

typedef enum {
    EVT_BTN_SHORT,
    EVT_BTN_LONG,
    EVT_GOT_IP,
    EVT_FORGET_RX,
    EVT_WIFI_ORPHANED,   // receiver gone: hand the board back to Bluetooth
    EVT_STREAM_ON,       // a phone subscribed to the BLE IMU stream
    EVT_STREAM_OFF,      // ...and stopped
    EVT_SET_WMODE,       // the app picked a working mode (over BLE or via the Pi)
} app_event_id_t;

// The queue carries a payload now, because a mode change has to say WHICH mode.
typedef struct {
    app_event_id_t id;
    uint8_t        wmode;   // EVT_SET_WMODE only
} app_event_t;

static QueueHandle_t s_events;
static volatile app_mode_t s_mode = APP_MODE_IDLE;
// The working mode the user picked. Volatile: the IMU task reads it per sample.
// IDLE at boot, never restored from NVS — see app_ctrl.h.
static volatile wearable_mode_t s_wmode = WMODE_IDLE;
// True while we are advertising over BLE but STILL HOLD WiFi credentials: the
// receiver went away and we handed the board back to Bluetooth so the app can
// reach it, without forgetting the network we belong to. Distinguishes this
// from a genuinely unprovisioned board, which is also APP_MODE_PAIRING.
static volatile bool s_orphaned = false;

// --- BLE provisioning callbacks (run on the NimBLE host task) --------------
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
    // `mode=` is how the app confirms a mode write actually took — the Control
    // characteristic write itself only tells it the bytes arrived.
    snprintf(buf, n, "%s ip=%s wid=%u %s pi=%lu mode=%s",
             wifi_udp_is_connected() ? "up" : "down",
             ip,
             wifi_udp_get_wearable_id(),
             !wifi_udp_has_target() ? "notgt"
               : wifi_udp_is_verified() ? "verified" : "unverified",
             (unsigned long)wifi_udp_get_pi_id(),
             app_wmode_str(s_wmode));
}

static const ble_provision_cfg_t s_ble_cfg = {
    .on_provision      = on_provision,
    .on_wearable_id    = on_wearable_id,
    .on_expected_pi_id = on_expected_pi_id,
    .status_getter     = status_getter,
};

// --- event producers (each only posts to the queue) -------------------------
static void post_event(app_event_id_t id)
{
    app_event_t e = { id, 0 };
    if (s_events) xQueueSend(s_events, &e, 0);
}

static void post_wmode(uint8_t m)
{
    app_event_t e = { EVT_SET_WMODE, m };
    if (s_events) xQueueSend(s_events, &e, 0);
}

static void button_cb(button_event_t evt)
{
    post_event(evt == BUTTON_EVT_LONG ? EVT_BTN_LONG : EVT_BTN_SHORT);
}

// Both control paths land here: the BLE Control characteristic on the stream
// service (solo — the phone is talking to this board) and a MSG_MODE datagram
// relayed by the receiver (group — the phone is talking to the Pi). Each runs
// on someone else's task, so both only post.
//
// BLE carries the mode as a NAME and UDP as a byte, which is not an oversight:
// the BLE control characteristic is a human-readable text grammar shared with
// the receiver, while the datagram is a fixed binary struct.
static bool ble_mode_cb(const char *arg)
{
    wearable_mode_t m;
    if (!app_wmode_from_str(arg, &m)) return false;
    post_wmode((uint8_t)m);
    return true;
}

static void udp_mode_cb(uint8_t m) { post_wmode(m); }

static void forget_cb(void)
{
    post_event(EVT_FORGET_RX);
}

static void orphan_cb(void)
{
    post_event(EVT_WIFI_ORPHANED);
}

static void stream_subscriber_cb(bool streaming)
{
    post_event(streaming ? EVT_STREAM_ON : EVT_STREAM_OFF);
}

static void ip_event_cb(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) post_event(EVT_GOT_IP);
}

// --- LED ---------------------------------------------------------------------
static void led_set(bool on)
{
    gpio_set_level((gpio_num_t)PIN_USER_LED, on ? 0 : 1);   // active-low
}

static void led_tick_cb(void *arg)
{
    static uint32_t tick = 0;
    tick++;
    switch (s_mode) {
    case APP_MODE_IDLE:    led_set(false); break;
    case APP_MODE_PAIRING:
        // SOLID means "ready", on either radio: idle-on-Bluetooth looks exactly
        // like on-the-receiver's-WiFi, because to the user they are the same
        // thing — the wearable is powered up and usable. Which radio it happens
        // to be using is the app's business, not something to read off an LED.
        //
        // Every other pattern is therefore an EXCEPTION worth noticing:
        //   2 Hz  — enrolment window open, "claim me now" (closes on claim,
        //           or after 2 min if nobody does)
        //   1 Hz  — mock playback running
        //   double flash — orphaned: lost its receiver, back on Bluetooth and
        //                  still hunting, which is tellable from never-paired.
        led_set(ble_auth_window_open()    ? (tick & 1)                    // 2 Hz
                : mock_playback_is_active() ? ((tick >> 1) & 1)           // 1 Hz
                : s_orphaned              ? ((tick & 7) == 0 || (tick & 7) == 2)
                                          : true);                       // solid
        break;
    case APP_MODE_WIFI:
        led_set(mock_playback_is_active() ? ((tick >> 1) & 1)     // 1 Hz
                                          : true);                // solid
        break;
    }
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

// --- working mode -------------------------------------------------------------
const char *app_wmode_str(wearable_mode_t m)
{
    switch (m) {
    case WMODE_LIVE:   return "live";
    case WMODE_ALERTS: return "alerts";
    case WMODE_MOCK:   return "mock";
    default:           return "idle";
    }
}

bool app_wmode_from_str(const char *s, wearable_mode_t *out)
{
    if (!s || !out) return false;
    while (*s == ' ') s++;
    for (uint8_t m = WMODE_IDLE; m <= WMODE_MOCK; m++) {
        const char *name = app_wmode_str((wearable_mode_t)m);
        size_t len = strlen(name);
        // Prefix match, then insist the next char ends the word — so "mode live
        // 3" parses and "mode livewire" does not.
        if (strncmp(s, name, len) == 0 && (s[len] == '\0' || s[len] == ' ')) {
            *out = (wearable_mode_t)m;
            return true;
        }
    }
    return false;
}

// The ONLY writer of s_wmode. Runs on the ctrl task, so it is free to call into
// the radios; every producer posts EVT_SET_WMODE instead of calling this.
static void apply_wmode(wearable_mode_t m)
{
    if (m == s_wmode) return;   // idempotent: a repeated write from the app is free

    // Mock playback must be stopped on EVERY transition out of MOCK. A demo
    // that survives into the next mode keeps feeding the receiver, and the next
    // short-press stops the STALE playback instead of starting a new one — so
    // the user presses the button, sees nothing, and only gets data on the
    // second press.
    if (s_wmode == WMODE_MOCK) mock_playback_stop();

    ESP_LOGI(TAG, "working mode: %s -> %s", app_wmode_str(s_wmode), app_wmode_str(m));
    s_wmode = m;

    // Only IDLE is silent. MOCK very much is not: the CSV has head impacts
    // spliced into it (gen_head_mock.py) precisely so a demo exercises the
    // impact pipeline, and a demo that shows telemetry but never an impact
    // demonstrates the least interesting half of this product. In a production
    // build the detector is fed by the REAL sensor throughout playback anyway,
    // and a real hit during a demo is still a real hit.
    //
    // While IDLE holds them, impact_det keeps detecting and backlogs, so
    // leaving IDLE replays what happened during the silence.
    impact_det_set_delivery_enabled(m != WMODE_IDLE);
    wifi_udp_set_reported_mode((uint8_t)m);   // rides out on the next HELLO

    // Re-anchor the app's decoder before the first frame of a mode that sends.
    // A board can sit in IDLE for minutes after the phone subscribed, and a
    // client that no longer holds the decode tables drops every sample in
    // silence — indistinguishable, on screen, from a wearable sending nothing.
    if (m != WMODE_IDLE) ble_stream_request_meta();

    if (m == WMODE_MOCK) mock_playback_start_loop();

    ble_provision_push_status();   // the app reads the new mode back from here
}

// --- state machine ------------------------------------------------------------
static void enter_pairing(void)
{
    if (ble_provision_start(&s_ble_cfg) == ESP_OK) {
        s_mode = APP_MODE_PAIRING;
        ESP_LOGI(TAG, "mode: PAIRING (BLE advertising)");
    } else {
        ESP_LOGE(TAG, "BLE start failed — staying in %s",
                 s_mode == APP_MODE_WIFI ? "WIFI" : "IDLE");
    }
}

static void ctrl_task(void *arg)
{
    app_event_t e;
    while (true) {
        if (xQueueReceive(s_events, &e, portMAX_DELAY) != pdTRUE) continue;

        // Handled before the state switch: the working mode is orthogonal to
        // the radio state, so the app can set it whether we are advertising on
        // Bluetooth or sitting on the receiver's WiFi.
        if (e.id == EVT_SET_WMODE) {
            if (e.wmode <= WMODE_MOCK) apply_wmode((wearable_mode_t)e.wmode);
            else ESP_LOGW(TAG, "ignoring unknown working mode %u", e.wmode);
            continue;
        }

        const app_event_id_t evt = e.id;

        switch (s_mode) {
        case APP_MODE_IDLE:
            if (evt == EVT_BTN_LONG) {
                ble_auth_open_window(ENROL_WINDOW_MS);
                enter_pairing();
            } else if (evt == EVT_BTN_SHORT) {
                ESP_LOGI(TAG, "not provisioned — hold BOOT 3 s to pair");
            }
            break;

        case APP_MODE_PAIRING:
            if (evt == EVT_BTN_SHORT) {
                // Same demo as in WIFI mode, down the BLE stream instead: lets
                // a solo session be shown off with no receiver present. Routed
                // through the working mode so the button and the app can never
                // disagree about whether a demo is running.
                apply_wmode(s_wmode == WMODE_MOCK ? WMODE_IDLE : WMODE_MOCK);
            } else if (evt == EVT_STREAM_ON) {
                // A phone is streaming from us. If we're orphaned we still have
                // credentials and are quietly hunting for our receiver — stop,
                // so the association attempts can't degrade a live session.
                if (s_orphaned) wifi_udp_set_radio_policy(WIFI_RADIO_PAUSED);
            } else if (evt == EVT_STREAM_OFF) {
                if (!s_orphaned) break;
                // If the receiver came back DURING that session we deliberately
                // ignored its GOT_IP (below) — and GOT_IP will not fire again,
                // since we never left the network. Complete the transition now
                // that the session is over, or resume hunting if it didn't.
                if (wifi_udp_is_connected()) {
                    post_event(EVT_GOT_IP);
                } else {
                    wifi_udp_set_radio_policy(WIFI_RADIO_BACKGROUND);
                }
            } else if (evt == EVT_GOT_IP) {
                // Our receiver came back on its own. Don't yank the radio out
                // from under a phone that is streaming right now — stay on BLE
                // and keep the association; EVT_STREAM_OFF re-posts this once
                // the session ends.
                if (ble_stream_ready()) {
                    ESP_LOGI(TAG, "network back, but a BLE session is live — staying on BLE");
                    break;
                }
                // A demo started over BLE must not outlive the mode it belongs
                // to. Without this it keeps running into the group session, now
                // feeding the Pi over UDP — and the next short-press STOPS that
                // stale playback instead of starting a new one, so the user
                // presses the button, sees nothing happen, and only gets data
                // on the second press. Back to IDLE: the app re-picks a working
                // mode once the group session is up.
                apply_wmode(WMODE_IDLE);
                // Let the app read/receive the "up ..." status before BLE dies.
                ble_provision_push_status();
                vTaskDelay(pdMS_TO_TICKS(BLE_STOP_DELAY_MS));
                ble_provision_stop();
                s_orphaned = false;
                wifi_udp_set_radio_policy(WIFI_RADIO_FOREGROUND);
                s_mode = APP_MODE_WIFI;
                ESP_LOGI(TAG, "mode: WIFI (provisioned, BLE off)");
            } else if (evt == EVT_BTN_LONG) {
                // We are already advertising, so the press isn't about the
                // radio — it is the user proving they physically hold this
                // board, which is what opens the (short) window in which a new
                // phone may read the enrolment secret. That is how a
                // replacement phone recovers, and why someone merely in radio
                // range cannot claim the board.
                ble_auth_open_window(ENROL_WINDOW_MS);
            }
            break;

        case APP_MODE_WIFI:
            if (evt == EVT_BTN_SHORT) {
                apply_wmode(s_wmode == WMODE_MOCK ? WMODE_IDLE : WMODE_MOCK);
            } else if (evt == EVT_BTN_LONG) {
                // Manual escape back to Bluetooth. This is safe to offer now
                // that boards are claimed: returning to BLE no longer means
                // "anyone nearby may take me over" — a claimed board still
                // refuses credential writes and stream subscribes, and handing
                // out its secret needs the separate enrolment window. So the
                // press expresses exactly one intent: leave this network.
                //
                // Deliberately does NOT open that window. Someone who only
                // wants to reset WiFi shouldn't also be making the board
                // claimable; pressing again once it is advertising does that.
                //
                // Forgets rather than merely disconnecting — otherwise the
                // background retry would drag it straight back onto the network
                // the user just asked it to leave.
                //
                // Back to IDLE as well: this press ends the board's involvement
                // with whoever was driving it, so it should go quiet until
                // someone picks a mode again.
                apply_wmode(WMODE_IDLE);
                wifi_udp_forget();
                s_orphaned = false;
                s_mode = APP_MODE_IDLE;   // off the network; enter_pairing() promotes
                enter_pairing();
                ESP_LOGI(TAG, "long-press — left the network, advertising again");
            } else if (evt == EVT_FORGET_RX) {
                // App-driven release from the Pi's WiFi — either an unpair, or
                // the app moving this board back to BLE (end of a group
                // session, or a solo session that needs to stream from it
                // directly). Advertise again rather than going dark: BLE is the
                // only channel back to a board that is off the WiFi, so going
                // IDLE here would strand it until someone held BOOT.
                //
                // This is the end of a group session in practice, so go quiet
                // too — the app sets a working mode again when the next one
                // starts, and a board left looping a demo is a support call.
                apply_wmode(WMODE_IDLE);
                wifi_udp_forget();
                s_orphaned = false;       // deliberate release, not a lost receiver
                s_mode = APP_MODE_IDLE;   // off the network; enter_pairing() promotes
                enter_pairing();
            } else if (evt == EVT_WIFI_ORPHANED) {
                // The receiver has been unreachable for ORPHAN_TIMEOUT_MS. Come
                // back on Bluetooth so the app has a channel to us again — but
                // KEEP the credentials and keep hunting in the background, so a
                // receiver that merely rebooted is rejoined without the user
                // doing anything. (Contrast EVT_FORGET_RX, which is deliberate.)
                //
                // The working mode is deliberately KEPT here. Nobody asked for
                // anything — a wearable that walks out of receiver range should
                // come back doing what it was told to do, and impacts detected
                // in the meantime are held by impact_det and delivered when a
                // transport returns. Only a deliberate release goes to IDLE.
                s_orphaned = true;
                wifi_udp_set_radio_policy(WIFI_RADIO_BACKGROUND);
                s_mode = APP_MODE_IDLE;   // enter_pairing() promotes on success
                enter_pairing();
                ESP_LOGW(TAG, "receiver lost — advertising on BLE, still seeking WiFi");
            }
            break;
        }
    }
}

esp_err_t app_ctrl_init(void)
{
    s_events = xQueueCreate(8, sizeof(app_event_t));
    if (!s_events) return ESP_ERR_NO_MEM;

    led_init();
    ble_auth_init();   // load the enrolment secret before BLE can be started

    esp_err_t err = button_init(PIN_BOOT_BUTTON, button_cb);
    if (err != ESP_OK) { ESP_LOGE(TAG, "button_init: %d", err); return err; }

    wifi_udp_set_forget_cb(forget_cb);
    wifi_udp_set_orphan_cb(orphan_cb);
    ble_stream_set_subscriber_cb(stream_subscriber_cb);

    // The two ways the app can set a working mode: straight over BLE when it is
    // connected to this board (solo), or relayed as a UDP datagram by the
    // receiver when it isn't (group). Same enum, same handler.
    ble_stream_set_mode_cb(ble_mode_cb);
    wifi_udp_set_mode_cb(udp_mode_cb);
    // IDLE at boot means nothing on the wire — including impacts.
    impact_det_set_delivery_enabled(false);

    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              &ip_event_cb, NULL, NULL);
    if (err != ESP_OK) { ESP_LOGE(TAG, "ip handler: %d", err); return err; }

    // wifi_udp_init() already auto-connected if NVS held credentials.
    //
    // With credentials we're on the Pi's network and BLE stays off (the two
    // radios share one antenna and coexistence is poor). Without them, start
    // advertising immediately instead of waiting for a BOOT long-press: BLE is
    // the resting state, and a board that has already been paired must come
    // back reachable after a power cycle without the user touching it.
    s_mode = APP_MODE_IDLE;
    if (wifi_udp_has_creds()) {
        s_mode = APP_MODE_WIFI;
        ESP_LOGI(TAG, "boot mode: WIFI (provisioned, BLE off)");
    } else {
        enter_pairing();   // logs PAIRING, or leaves us IDLE if BLE won't start
    }

    if (xTaskCreate(ctrl_task, "app_ctrl", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

app_mode_t app_ctrl_mode(void) { return s_mode; }

wearable_mode_t app_ctrl_wearable_mode(void) { return s_wmode; }

void app_ctrl_set_wearable_mode(wearable_mode_t m) { post_wmode((uint8_t)m); }

bool app_ctrl_stream_enabled(void)
{
    // MOCK owns the wire while it plays: the live sensor stream stays off so
    // the receiver never sees CSV rows and real samples interleaved.
    return s_wmode == WMODE_LIVE && !mock_playback_is_active();
}
