#ifndef PLFM_TYPES_H
#define PLFM_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PLFM_MAX_SAMPLES_PER_PULSE   (2048U)
#define PLFM_MAX_PULSES_PER_FRAME    (128U)
#define PLFM_ADC_PINGPONG_SAMPLES    (4096U)

typedef enum
{
    PLFM_OK = 0,
    PLFM_ERR_INVALID_ARG = -1,
    PLFM_ERR_CRC = -2,
    PLFM_ERR_NVM = -3,
    PLFM_ERR_BUSY = -4
} plfm_status_t;

typedef enum
{
    PLFM_WINDOW_HANN = 0,
    PLFM_WINDOW_HAMMING,
    PLFM_WINDOW_BLACKMAN
} plfm_window_t;

typedef struct
{
    float i;
    float q;
} plfm_cplx_f32_t;

typedef struct
{
    uint32_t timestamp_us;
    uint16_t sample_count;
    uint16_t pulse_idx;
    const int16_t *iq_interleaved;
} plfm_adc_block_t;

#endif /* PLFM_TYPES_H */
