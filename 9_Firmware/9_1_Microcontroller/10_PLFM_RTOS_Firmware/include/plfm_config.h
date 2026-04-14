#ifndef PLFM_CONFIG_H
#define PLFM_CONFIG_H

#include "plfm_types.h"

#define PLFM_CONFIG_MAGIC     (0x504C464DU) /* "PLFM" */
#define PLFM_CONFIG_VERSION   (1U)

typedef struct
{
    uint32_t center_freq_hz;
    uint32_t bandwidth_hz;
    uint32_t sweep_time_us;
    uint32_t sample_rate_hz;
    uint16_t pulses_per_cpi;
    uint16_t pri_us;
    plfm_window_t window;
    uint8_t enable_range_doppler;
    uint8_t reserved[3];
} plfm_chirp_cfg_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t payload_size;
    plfm_chirp_cfg_t chirp;
    float dc_i;
    float dc_q;
    float iq_gain;
    float iq_phase_deg;
    uint32_t crc32;
} plfm_config_blob_t;

plfm_status_t plfm_config_default(plfm_config_blob_t *cfg);
plfm_status_t plfm_config_load(plfm_config_blob_t *cfg);
plfm_status_t plfm_config_save(const plfm_config_blob_t *cfg);
plfm_status_t plfm_config_validate(const plfm_config_blob_t *cfg);

#endif /* PLFM_CONFIG_H */
