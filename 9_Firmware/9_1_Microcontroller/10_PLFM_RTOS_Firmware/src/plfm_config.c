#include "plfm_config.h"

#include <string.h>

/*
 * NOTE: Hook these to platform flash/EEPROM routines.
 * Kept local-static so they can be replaced cleanly per board.
 */
static int nvm_read_blob(plfm_config_blob_t *cfg)
{
    (void)cfg;
    return -1;
}

static int nvm_write_blob(const plfm_config_blob_t *cfg)
{
    (void)cfg;
    return 0;
}

static uint32_t fnv1a32(const uint8_t *data, size_t len)
{
    size_t i;
    uint32_t hash = 2166136261UL;
    for (i = 0U; i < len; i++) {
        hash ^= data[i];
        hash *= 16777619UL;
    }
    return hash;
}

plfm_status_t plfm_config_default(plfm_config_blob_t *cfg)
{
    if (cfg == NULL) {
        return PLFM_ERR_INVALID_ARG;
    }

    (void)memset(cfg, 0, sizeof(*cfg));
    cfg->magic = PLFM_CONFIG_MAGIC;
    cfg->version = PLFM_CONFIG_VERSION;
    cfg->payload_size = (uint16_t)(sizeof(*cfg) - sizeof(cfg->crc32));

    cfg->chirp.center_freq_hz = 10500000000UL;
    cfg->chirp.bandwidth_hz = 20000000UL;
    cfg->chirp.sweep_time_us = 30U;
    cfg->chirp.sample_rate_hz = 5000000UL;
    cfg->chirp.pulses_per_cpi = 32U;
    cfg->chirp.pri_us = 167U;
    cfg->chirp.window = PLFM_WINDOW_HANN;
    cfg->chirp.enable_range_doppler = 1U;

    cfg->dc_i = 0.0f;
    cfg->dc_q = 0.0f;
    cfg->iq_gain = 1.0f;
    cfg->iq_phase_deg = 0.0f;

    cfg->crc32 = fnv1a32((const uint8_t *)cfg, sizeof(*cfg) - sizeof(cfg->crc32));
    return PLFM_OK;
}

plfm_status_t plfm_config_validate(const plfm_config_blob_t *cfg)
{
    uint32_t crc_calc;

    if (cfg == NULL) {
        return PLFM_ERR_INVALID_ARG;
    }
    if ((cfg->magic != PLFM_CONFIG_MAGIC) || (cfg->version != PLFM_CONFIG_VERSION)) {
        return PLFM_ERR_NVM;
    }
    if ((cfg->chirp.sample_rate_hz == 0UL) || (cfg->chirp.sweep_time_us == 0UL) || (cfg->chirp.pri_us == 0U)) {
        return PLFM_ERR_INVALID_ARG;
    }
    crc_calc = fnv1a32((const uint8_t *)cfg, sizeof(*cfg) - sizeof(cfg->crc32));
    if (crc_calc != cfg->crc32) {
        return PLFM_ERR_CRC;
    }
    return PLFM_OK;
}

plfm_status_t plfm_config_load(plfm_config_blob_t *cfg)
{
    if (cfg == NULL) {
        return PLFM_ERR_INVALID_ARG;
    }
    if (nvm_read_blob(cfg) != 0) {
        (void)plfm_config_default(cfg);
        return PLFM_ERR_NVM;
    }
    return plfm_config_validate(cfg);
}

plfm_status_t plfm_config_save(const plfm_config_blob_t *cfg)
{
    plfm_config_blob_t copy;

    if (cfg == NULL) {
        return PLFM_ERR_INVALID_ARG;
    }
    copy = *cfg;
    copy.crc32 = fnv1a32((const uint8_t *)&copy, sizeof(copy) - sizeof(copy.crc32));

    if (nvm_write_blob(&copy) != 0) {
        return PLFM_ERR_NVM;
    }
    return PLFM_OK;
}
