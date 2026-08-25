#pragma once
// Host-test shim: logging is noise in a unit test, so it goes nowhere.
// Arguments are still evaluated-checked by the compiler via the unused fn.
#include <stdio.h>
#define ESP_LOGE(tag, fmt, ...) ((void)0)
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#define ESP_LOGD(tag, fmt, ...) ((void)0)
