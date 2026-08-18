/*! ----------------------------------------------------------------------------
 * @file    deca_spi.h
 * @brief   SPI access function declarations.
 *
 * The signatures below are dictated by Qorvo's driver — `dwt_spi_s` calls
 * these by name through a function-pointer struct (see deca_probe_interface.c).
 * Do not change parameter types or order.
 */

#ifndef _DECA_SPI_H_
#define _DECA_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DECA_MAX_SPI_HEADER_LENGTH (3)

/* SPI rate switching, called by the driver. */
void port_set_dw_ic_spi_slowrate(void);
void port_set_dw_ic_spi_fastrate(void);

/* Plain write: header + body, no CRC. */
int32_t writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
                   uint16_t bodyLength,   const uint8_t *bodyBuffer);

/* Read: send header, capture body. */
int32_t readfromspi(uint16_t headerLength, uint8_t *headerBuffer,
                    uint16_t readLength,   uint8_t *readBuffer);

/* CRC variant. We don't enable CRC mode in Phase 3, so this stubs out. */
int32_t writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer,
                          uint16_t bodyLength,   const uint8_t *bodyBuffer,
                          uint8_t crc8);

#ifdef __cplusplus
}
#endif

#endif /* _DECA_SPI_H_ */