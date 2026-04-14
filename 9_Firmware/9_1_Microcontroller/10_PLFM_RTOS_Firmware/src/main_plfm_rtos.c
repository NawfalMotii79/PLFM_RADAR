#include "plfm_config.h"
#include "plfm_dsp.h"
#include "plfm_tasks.h"

#include "FreeRTOS.h"
#include "task.h"

/*
 * Platform hooks (to be implemented in board support package):
 * - clock/peripheral init
 * - ADC+DMA ping-pong start
 * - timer-synchronized chirp trigger
 * - watchdog kick
 */
static void platform_init(void)
{
}

static void post_run_or_die(void)
{
    /* Add SRAM/flash CRC, ADC shorted-input check, rail checks, etc. */
}

int main(void)
{
    plfm_config_blob_t cfg;
    plfm_dsp_cfg_t dsp_cfg;
    plfm_status_t st;

    platform_init();
    post_run_or_die();

    st = plfm_config_load(&cfg);
    if (st != PLFM_OK) {
        (void)plfm_config_default(&cfg);
        (void)plfm_config_save(&cfg);
    }

    dsp_cfg.fft_size = 1024U;
    dsp_cfg.window = cfg.chirp.window;
    dsp_cfg.matched_filter_i = NULL;
    dsp_cfg.matched_filter_q = NULL;
    (void)plfm_dsp_init(&dsp_cfg);

    (void)plfm_tasks_start(&cfg);

    vTaskStartScheduler();
    for (;;) {
    }
}
