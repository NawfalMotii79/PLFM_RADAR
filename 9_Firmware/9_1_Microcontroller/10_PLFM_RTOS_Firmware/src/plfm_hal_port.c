#include "plfm_hal_port.h"

__attribute__((weak)) uint32_t plfm_hal_port_get_time_us(void)
{
    return 0U;
}

__attribute__((weak)) int plfm_hal_port_start_acq_dma(int16_t *pingpong_iq, uint32_t sample_count)
{
    (void)pingpong_iq;
    (void)sample_count;
    return -1;
}

__attribute__((weak)) int plfm_hal_port_start_pri_timer(uint32_t pri_us)
{
    (void)pri_us;
    return -1;
}

__attribute__((weak)) int plfm_hal_port_stream_tx(const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
    return -1;
}

__attribute__((weak)) void plfm_hal_port_watchdog_kick(void)
{
}
