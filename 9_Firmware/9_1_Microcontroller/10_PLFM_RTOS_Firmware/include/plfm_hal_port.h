#ifndef PLFM_HAL_PORT_H
#define PLFM_HAL_PORT_H

#include <stdint.h>

/*
 * Board adaptation layer for phase-2 integration.
 * Implement these in the target firmware (main.cpp/BSP) to bind RTOS modules
 * to concrete STM32 peripherals.
 */
uint32_t plfm_hal_port_get_time_us(void);
int plfm_hal_port_start_acq_dma(int16_t *pingpong_iq, uint32_t sample_count);
int plfm_hal_port_start_pri_timer(uint32_t pri_us);
int plfm_hal_port_stream_tx(const uint8_t *data, uint16_t len);
void plfm_hal_port_watchdog_kick(void);

#endif /* PLFM_HAL_PORT_H */
