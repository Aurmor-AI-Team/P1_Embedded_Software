#pragma once
#include "freertos/FreeRTOS.h"
typedef void *SemaphoreHandle_t;
// A non-null sentinel: impact_det.cpp treats NULL as "no lock available" and
// would then skip the guarded sections entirely, which is not what we test.
#define TEST_FAKE_SEMAPHORE ((SemaphoreHandle_t)1)
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) { return TEST_FAKE_SEMAPHORE; }
static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t t) { (void)s; (void)t; return pdTRUE; }
static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t s) { (void)s; return pdTRUE; }
