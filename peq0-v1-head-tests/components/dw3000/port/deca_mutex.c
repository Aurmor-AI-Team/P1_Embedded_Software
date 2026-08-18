/*! ----------------------------------------------------------------------------
 * @file    deca_mutex.c
 * @brief   IRQ-disable mutex stubs.
 *
 * The driver calls decamutexon/off around critical sections to prevent
 * the chip's IRQ from firing during sensitive register sequences. In
 * Phase 3 we don't have IRQs wired up, so these are no-ops. When IRQ-
 * driven RX is added in Phase 4, replace with proper enable/disable.
 */

#include "port.h"
#include "deca_device_api.h"

decaIrqStatus_t decamutexon(void)
{
    return 0;
}

void decamutexoff(decaIrqStatus_t s)
{
    (void)s;
}