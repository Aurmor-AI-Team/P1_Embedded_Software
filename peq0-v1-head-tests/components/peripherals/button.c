// ---------------------------------------------------------------------------
// button.c — polled active-low button with short/long press detection.
//
// A 50 ms esp_timer poll debounces by construction (a bounce shorter than one
// tick is never seen twice). Holding for BUTTON_LONG_TICKS fires
// BUTTON_EVT_LONG exactly once while still held; releasing earlier fires
// BUTTON_EVT_SHORT on release.
// ---------------------------------------------------------------------------
#include "button.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "button";

#define BUTTON_POLL_MS     50
#define BUTTON_LONG_TICKS  (3000 / BUTTON_POLL_MS)   // 3 s hold

static int                s_gpio = -1;
static button_event_cb_t  s_cb;
static esp_timer_handle_t s_timer;
static int                s_held_ticks;
static bool               s_long_fired;

static void poll_cb(void *arg)
{
    bool pressed = (gpio_get_level(s_gpio) == 0);
    if (pressed) {
        s_held_ticks++;
        if (s_held_ticks >= BUTTON_LONG_TICKS && !s_long_fired) {
            s_long_fired = true;
            if (s_cb) s_cb(BUTTON_EVT_LONG);
        }
        return;
    }
    if (s_held_ticks > 0 && !s_long_fired && s_cb) {
        s_cb(BUTTON_EVT_SHORT);
    }
    s_held_ticks = 0;
    s_long_fired = false;
}

esp_err_t button_init(int gpio, button_event_cb_t cb)
{
    if (gpio < 0 || !cb) return ESP_ERR_INVALID_ARG;
    s_gpio = gpio;
    s_cb = cb;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        // BOOT (GPIO9) is a strapping pin with a board pull-up; leave both
        // internal pulls off so we never fight the strap at reset.
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) return err;

    const esp_timer_create_args_t targs = {
        .callback = poll_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "button_poll",
        .skip_unhandled_events = true,
    };
    err = esp_timer_create(&targs, &s_timer);
    if (err != ESP_OK) return err;
    err = esp_timer_start_periodic(s_timer, BUTTON_POLL_MS * 1000);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "polling GPIO%d every %d ms (long press = %d ms)",
             gpio, BUTTON_POLL_MS, BUTTON_POLL_MS * BUTTON_LONG_TICKS);
    return ESP_OK;
}
