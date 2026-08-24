#pragma once
// Host-test shim. The mutex is a no-op: the unit test is single-threaded, and
// what is under test is the detector's state machine, not FreeRTOS.
#include <stdint.h>
typedef int BaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE  1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
