/*! ----------------------------------------------------------------------------
 * @file    port.h
 * @brief   ESP-IDF / XIAO ESP32-C6 port for DW3000 driver.
 *
 * Replaces Nordic nRF52840-DK reference port. Only declares the platform
 * abstractions the Qorvo driver and example code expect to find.
 */

#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_err.h"

/* DW3000 hardware pin assignments on XIAO ESP32-C6.
 * Match the values used in dwm3000_init().
 */
#define DW3000_CS_PIN     16
#define DW3000_RST_PIN    17
#define DW3000_IRQ_PIN     4
#define DW3000_MOSI_PIN   18
#define DW3000_MISO_PIN   20
#define DW3000_SCLK_PIN   19

/* SPI clock rates. Slow rate is used by the driver during init/OTP/PLL lock;
 * fast rate is used during normal operation. The driver toggles between the
 * two via setslowrate/setfastrate callbacks. */
#define DW3000_SPI_FREQ_SLOW  (2  * 1000 * 1000)   /* 2 MHz */
#define DW3000_SPI_FREQ_FAST  (16 * 1000 * 1000)   /* 16 MHz */

/* DW IC IRQ handler type (used by example apps; we stub it for Phase 3). */
typedef void (*port_dwic_isr_t)(void);

/* Platform setup — called once from dwm3000_init before dwt_probe. */
esp_err_t port_init_dw3000(void);
void      port_deinit_dw3000(void);

/* Hard reset (RST pin pulse). Equivalent to Nordic's reset_DWIC. */
void reset_DWIC(void);

/* Sleep / delay wrappers used by deca_sleep.c and various driver sites. */
void Sleep(uint32_t ms);

/* Wakeup-from-deepsleep stub. Called by the driver only when the chip is in
 * deepsleep, which is never in Phase 3. No-op for now. */
void wakeup_device_with_io(void);

/* IRQ infrastructure — stubs for Phase 3, real implementations land in
 * Phase 4 when we wire up interrupt-driven RX. */
void     port_set_dwic_isr(port_dwic_isr_t dwic_isr);
void     port_DisableEXT_IRQ(void);
void     port_EnableEXT_IRQ(void);
uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);
void     process_deca_irq(void);
void port_set_dw_ic_spi_fastrate(void);
esp_err_t port_enable_dw3000_irq(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */