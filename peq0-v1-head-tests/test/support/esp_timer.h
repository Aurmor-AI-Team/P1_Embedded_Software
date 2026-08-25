#pragma once
// Host-test shim with a CONTROLLABLE clock. The detector is all about timing
// (150 ms window, 250 ms refractory), so the test drives time explicitly
// rather than sleeping.
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
int64_t esp_timer_get_time(void);
void    test_clock_set_us(int64_t us);
void    test_clock_advance_us(int64_t us);
#ifdef __cplusplus
}
#endif
