#ifndef PLFM_TASKS_H
#define PLFM_TASKS_H

#include "plfm_config.h"
#include "plfm_types.h"

typedef struct
{
    uint32_t uptime_ms;
    uint16_t queue_highwater_adc;
    uint16_t queue_highwater_comm;
    uint8_t post_ok;
    uint8_t wdg_resets;
    uint8_t adc_overrun;
    uint8_t reserved;
} plfm_diag_t;

plfm_status_t plfm_tasks_start(const plfm_config_blob_t *boot_cfg);
void plfm_tasks_notify_adc_half_complete(uint32_t timestamp_us);
void plfm_tasks_notify_adc_full_complete(uint32_t timestamp_us);
const plfm_diag_t *plfm_get_diag(void);

#endif /* PLFM_TASKS_H */
