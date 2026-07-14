// ---------------------------------------------------------------------------
// app_ctrl.cpp — device mode state machine (IDLE / PAIRING / WIFI).
//
// Every input (button presses, GOT_IP, Pi FORGET) is posted as an event to a
// queue and handled sequentially in one task, so BLE/WiFi lifecycle calls
// never race each other. The user LED (GPIO15, active-low) mirrors the mode:
// off = idle, fast blink = pairing, solid = on WiFi, slow blink = playback.
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

#include "ble_provision.h"
#include "button.h"
#include "mock_playback.h"
#include "wifi_udp_tx.h"

static const char *TAG = "app_ctrl";

#define PIN_BOOT_BUTTON  9    // XIAO ESP32-C6 BOOT, active-low
#define PIN_USER_LED     15   // XIAO ESP32-C6 yellow user LED, active-low

// Keeps the "up ip=..." status notify ahead of the BLE teardown so the app
// sees the provisioning result before the link drops.
#define BLE_STOP_DELAY_MS 2000

typedef enum {
    EVT_BTN_SHORT,
    EVT_BTN_LONG,
    EVT_GOT_IP,
    EVT_FORGET_RX,
} app_event_t;

static QueueHandle_t s_events;
static volatile app_mode_t s_mode = APP_MODE_IDLE;

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
    snprintf(buf, n, "%s ip=%s wid=%u %s pi=%lu",
             wifi_udp_is_connected() ? "up" : "down",
             ip,
             wifi_udp_get_wearable_id(),
             !wifi_udp_has_target() ? "notgt"
               : wifi_udp_is_verified() ? "verified" : "unverified",
             (unsigned long)wifi_udp_get_pi_id());
}

static const ble_provision_cfg_t s_ble_cfg = {
    .on_provision      = on_provision,
    .on_wearable_id    = on_wearable_id,
    .on_expected_pi_id = on_expected_pi_id,
    .status_getter     = status_getter,
};

// --- event producers (each only posts to the queue) -------------------------
static void post_event(app_event_t evt)
{
    if (s_events) xQueueSend(s_events, &evt, 0);
}

static void button_cb(button_event_t evt)
{
    post_event(evt == BUTTON_EVT_LONG ? EVT_BTN_LONG : EVT_BTN_SHORT);
}

static void forget_cb(void)
{
    post_event(EVT_FORGET_RX);
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
    case APP_MODE_PAIRING: led_set(tick & 1); break;              // 2 Hz
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
    app_event_t evt;
    while (true) {
        if (xQueueReceive(s_events, &evt, portMAX_DELAY) != pdTRUE) continue;

        switch (s_mode) {
        case APP_MODE_IDLE:
            if (evt == EVT_BTN_LONG) {
                enter_pairing();
            } else if (evt == EVT_BTN_SHORT) {
                ESP_LOGI(TAG, "not provisioned — hold BOOT 3 s to pair");
            }
            break;

        case APP_MODE_PAIRING:
            if (evt == EVT_GOT_IP) {
                // Let the app read/receive the "up ..." status before BLE dies.
                ble_provision_push_status();
                vTaskDelay(pdMS_TO_TICKS(BLE_STOP_DELAY_MS));
                ble_provision_stop();
                s_mode = APP_MODE_WIFI;
                ESP_LOGI(TAG, "mode: WIFI (provisioned, BLE off)");
            }
            // BTN_LONG while already advertising: nothing to restart.
            break;

        case APP_MODE_WIFI:
            if (evt == EVT_BTN_SHORT) {
                if (mock_playback_is_active()) mock_playback_stop();
                else mock_playback_start();
            } else if (evt == EVT_BTN_LONG) {
                // Manual re-pair: forget this network, advertise again.
                mock_playback_stop();
                wifi_udp_forget();
                enter_pairing();
            } else if (evt == EVT_FORGET_RX) {
                // Unpaired from the app via the Pi.
                mock_playback_stop();
                wifi_udp_forget();
                s_mode = APP_MODE_IDLE;
                ESP_LOGI(TAG, "mode: IDLE (unpaired)");
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

    esp_err_t err = button_init(PIN_BOOT_BUTTON, button_cb);
    if (err != ESP_OK) { ESP_LOGE(TAG, "button_init: %d", err); return err; }

    wifi_udp_set_forget_cb(forget_cb);

    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              &ip_event_cb, NULL, NULL);
    if (err != ESP_OK) { ESP_LOGE(TAG, "ip handler: %d", err); return err; }

    // wifi_udp_init() already auto-connected if NVS held credentials.
    s_mode = wifi_udp_has_creds() ? APP_MODE_WIFI : APP_MODE_IDLE;
    ESP_LOGI(TAG, "boot mode: %s (BLE %s)",
             s_mode == APP_MODE_WIFI ? "WIFI" : "IDLE", "off");

    if (xTaskCreate(ctrl_task, "app_ctrl", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

app_mode_t app_ctrl_mode(void) { return s_mode; }
