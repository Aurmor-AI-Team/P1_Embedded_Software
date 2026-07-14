#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Short press = released before the long-press threshold (3 s).
// Long press  = fired once while still held, at the threshold.
typedef enum {
    BUTTON_EVT_SHORT,
    BUTTON_EVT_LONG,
} button_event_t;

// Runs in the esp_timer task — do nothing heavy here; post to a queue.
typedef void (*button_event_cb_t)(button_event_t evt);

// Poll an active-low button (e.g. the XIAO ESP32-C6 BOOT button on GPIO9).
// GPIO9 is a strapping pin with an external pull-up; no internal pull is set.
esp_err_t button_init(int gpio, button_event_cb_t cb);

#ifdef __cplusplus
}
#endif
