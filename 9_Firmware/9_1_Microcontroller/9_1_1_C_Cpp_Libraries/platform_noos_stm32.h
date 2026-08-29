/* platform_noos_stm32.h */
#ifndef PLATFORM_NOOS_STM32_H
#define PLATFORM_NOOS_STM32_H

#include "stm32f7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdint.h>

/* Export the HAL SPI handle that will be used by no-OS SPI wrapper */
extern SPI_HandleTypeDef hspi4;

/* Simple wrapper prototypes for no-OS */
int32_t platform_spi_init(void **desc, uint32_t max_speed_hz, uint8_t mode);
int32_t platform_spi_write_and_read(void *desc, uint8_t *data, uint16_t len);
int32_t platform_spi_remove(void *desc);

int32_t platform_gpio_init(void *gpio_desc, uint8_t port_pin, bool is_output);
int32_t platform_gpio_direction_output(void *gpio_desc, uint8_t port_pin, uint8_t value);
int32_t platform_gpio_set_value(void *gpio_desc, uint8_t port_pin, uint8_t value);
int32_t platform_gpio_remove(void *gpio_desc);

void platform_delay_ms(uint32_t ms);

#endif
