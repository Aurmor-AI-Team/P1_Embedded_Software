// #pragma once
// #include <stdint.h>
// #include "esp_err.h"

// #ifdef __cplusplus
// extern "C" {
// #endif

// #define DW3000_DEV_ID_EXPECTED_HI  0xDECA
// #define DW3000_DEV_ID_EXPECTED     0xDECA0302u

// /* Initialize SPI bus + hard reset + Qorvo driver probe + dwt_initialise. */
// esp_err_t dwm3000_init(int mosi, int miso, int sclk, int cs, int rst);
// esp_err_t dwm3000_deinit(void);

// /* Phase-1 sanity reads. dwm3000_read_devid uses our own minimal SPI path
//  * to confirm wiring before handing control to the Qorvo driver. */
// esp_err_t dwm3000_read_devid(uint32_t *out_devid);
// esp_err_t dwm3000_wait_ready(int timeout_ms);
// /* Receiver/transceiver recovery without a full re-init. Re-runs the Qorvo
//  * driver's RX/PGF calibration (via dwt_configure) and forces a fresh on-chip
//  * temperature read for PLL cal, which clears thermally-induced RX degradation
//  * and most latched RX faults. Pass the same dwt_config_t used at init.
//  *
//  * Returns ESP_OK on success. On failure the caller should escalate to
//  * dwm3000_hard_recover(). */
// esp_err_t dwm3000_reconfigure_recover(const void *uwb_config,
//                                       const void *tx_config,
//                                       uint16_t tx_ant_dly, uint16_t rx_ant_dly);

// /* Full recovery: hard reset pin pulse + probe + initialise + (caller must
//  * reconfigure after). Use only when reconfigure_recover keeps failing. */
// esp_err_t dwm3000_hard_recover(void);

// /* Read the DW3000 die temperature in °C (for thermal-drift diagnostics). */
// float dwm3000_read_temp_c(void);

// /* Hard-reset (RST pin pulse). Wraps reset_DWIC for backward compatibility
//  * with code that already calls these names. */
// void dwm3000_hard_reset(void);
// void dwm3000_reset_pin_only(int rst);

// #ifdef __cplusplus
// }
// #endif