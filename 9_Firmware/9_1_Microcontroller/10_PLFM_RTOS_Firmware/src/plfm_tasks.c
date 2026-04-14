#include "plfm_tasks.h"
#include "plfm_dsp.h"
#include "plfm_hal_port.h"
#include "plfm_protocol.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include <string.h>

#define PLFM_ADC_TASK_STACK_WORDS     (768U)
#define PLFM_DSP_TASK_STACK_WORDS     (1024U)
#define PLFM_COMM_TASK_STACK_WORDS    (768U)
#define PLFM_MON_TASK_STACK_WORDS     (512U)

#define PLFM_ADC_QUEUE_DEPTH          (8U)
#define PLFM_COMM_QUEUE_DEPTH         (8U)

static QueueHandle_t g_adc_queue;
static QueueHandle_t g_comm_queue;
static SemaphoreHandle_t g_cfg_mutex;
static plfm_diag_t g_diag;
static plfm_config_blob_t g_cfg;
static uint32_t g_seq;
static uint8_t g_hw_acq_ready;

/* Ping-pong DMA buffers for deterministic continuous acquisition. */
static int16_t g_adc_pingpong[PLFM_ADC_PINGPONG_SAMPLES];
static int16_t g_sim_pingpong[PLFM_ADC_PINGPONG_SAMPLES];

static void task_acq(void *ctx);
static void task_dsp(void *ctx);
static void task_comm(void *ctx);
static void task_monitor(void *ctx);

plfm_status_t plfm_tasks_start(const plfm_config_blob_t *boot_cfg)
{
    if (boot_cfg == NULL) {
        return PLFM_ERR_INVALID_ARG;
    }
    g_cfg = *boot_cfg;
    (void)memset(&g_diag, 0, sizeof(g_diag));
    g_diag.post_ok = 1U;

    g_adc_queue = xQueueCreate(PLFM_ADC_QUEUE_DEPTH, sizeof(plfm_adc_block_t));
    g_comm_queue = xQueueCreate(PLFM_COMM_QUEUE_DEPTH, sizeof(plfm_adc_block_t));
    g_cfg_mutex = xSemaphoreCreateMutex();
    if ((g_adc_queue == NULL) || (g_comm_queue == NULL) || (g_cfg_mutex == NULL)) {
        return PLFM_ERR_BUSY;
    }
    g_hw_acq_ready = (plfm_hal_port_start_acq_dma(g_adc_pingpong, PLFM_ADC_PINGPONG_SAMPLES) == 0) ? 1U : 0U;
    (void)plfm_hal_port_start_pri_timer(g_cfg.chirp.pri_us);

    (void)xTaskCreate(task_acq, "acq", PLFM_ADC_TASK_STACK_WORDS, NULL,
                      (UBaseType_t)(configMAX_PRIORITIES - 1U), NULL);
    (void)xTaskCreate(task_dsp, "dsp", PLFM_DSP_TASK_STACK_WORDS, NULL,
                      (UBaseType_t)(configMAX_PRIORITIES - 2U), NULL);
    (void)xTaskCreate(task_comm, "comm", PLFM_COMM_TASK_STACK_WORDS, NULL,
                      (UBaseType_t)(configMAX_PRIORITIES - 3U), NULL);
    (void)xTaskCreate(task_monitor, "mon", PLFM_MON_TASK_STACK_WORDS, NULL,
                      (UBaseType_t)(configMAX_PRIORITIES - 4U), NULL);

    return PLFM_OK;
}

void plfm_tasks_notify_adc_half_complete(uint32_t timestamp_us)
{
    BaseType_t hpw = pdFALSE;
    plfm_adc_block_t block;
    block.timestamp_us = timestamp_us;
    block.sample_count = (uint16_t)(PLFM_ADC_PINGPONG_SAMPLES / 2U);
    block.pulse_idx = 0U;
    block.iq_interleaved = &g_adc_pingpong[0];
    (void)xQueueSendFromISR(g_adc_queue, &block, &hpw);
    portYIELD_FROM_ISR(hpw);
}

void plfm_tasks_notify_adc_full_complete(uint32_t timestamp_us)
{
    BaseType_t hpw = pdFALSE;
    plfm_adc_block_t block;
    block.timestamp_us = timestamp_us;
    block.sample_count = (uint16_t)(PLFM_ADC_PINGPONG_SAMPLES / 2U);
    block.pulse_idx = 1U;
    block.iq_interleaved = &g_adc_pingpong[PLFM_ADC_PINGPONG_SAMPLES / 2U];
    (void)xQueueSendFromISR(g_adc_queue, &block, &hpw);
    portYIELD_FROM_ISR(hpw);
}

const plfm_diag_t *plfm_get_diag(void)
{
    return &g_diag;
}

static void task_acq(void *ctx)
{
    plfm_adc_block_t block;
    uint16_t k;
    (void)ctx;
    for (;;) {
        if (g_hw_acq_ready == 0U) {
            /* Simulation mode fallback when ADC-DMA is not bound yet. */
            for (k = 0U; k < (PLFM_ADC_PINGPONG_SAMPLES / 2U); k++) {
                g_sim_pingpong[2U * k] = (int16_t)((k * 13U) & 0x0FFFU);
                g_sim_pingpong[(2U * k) + 1U] = (int16_t)((k * 7U) & 0x0FFFU);
            }
            block.timestamp_us = plfm_hal_port_get_time_us();
            block.sample_count = (uint16_t)(PLFM_ADC_PINGPONG_SAMPLES / 2U);
            block.pulse_idx = 0U;
            block.iq_interleaved = g_sim_pingpong;
            (void)xQueueSend(g_adc_queue, &block, 0U);
        }
        vTaskDelay(pdMS_TO_TICKS(5U));
    }
}

static void task_dsp(void *ctx)
{
    plfm_adc_block_t block;
    float range_mag[PLFM_MAX_SAMPLES_PER_PULSE / 2U];
    plfm_range_profile_meta_t meta;
    (void)ctx;

    for (;;) {
        if (xQueueReceive(g_adc_queue, &block, portMAX_DELAY) == pdTRUE) {
            (void)plfm_dsp_range_profile(block.iq_interleaved,
                                         block.sample_count,
                                         range_mag,
                                         (uint16_t)(PLFM_MAX_SAMPLES_PER_PULSE / 2U),
                                         &meta);
            (void)xQueueSend(g_comm_queue, &block, 0U);
            g_diag.queue_highwater_adc = (uint16_t)uxQueueMessagesWaiting(g_adc_queue);
        }
    }
}

static void task_comm(void *ctx)
{
    plfm_adc_block_t block;
    uint8_t tx[PLFM_PROTO_MAX_PAYLOAD + 16U];
    uint8_t payload[16];
    size_t tx_len;
    (void)ctx;
    for (;;) {
        if (xQueueReceive(g_comm_queue, &block, portMAX_DELAY) == pdTRUE) {
            /* Pack and stream compact metadata frame over UART/USB/SPI. */
            payload[0] = (uint8_t)(block.pulse_idx & 0xFFU);
            payload[1] = (uint8_t)(block.sample_count & 0xFFU);
            payload[2] = (uint8_t)((block.sample_count >> 8U) & 0xFFU);
            payload[3] = (uint8_t)(block.timestamp_us & 0xFFU);
            payload[4] = (uint8_t)((block.timestamp_us >> 8U) & 0xFFU);
            payload[5] = (uint8_t)((block.timestamp_us >> 16U) & 0xFFU);
            payload[6] = (uint8_t)((block.timestamp_us >> 24U) & 0xFFU);
            payload[7] = 0U;

            tx_len = sizeof(tx);
            if (plfm_frame_encode(PLFM_CMD_RANGE_PROFILE, g_seq++, payload, 8U, tx, &tx_len) == PLFM_OK) {
                (void)plfm_hal_port_stream_tx(tx, (uint16_t)tx_len);
            }
            g_diag.queue_highwater_comm = (uint16_t)uxQueueMessagesWaiting(g_comm_queue);
        }
    }
}

static void task_monitor(void *ctx)
{
    (void)ctx;
    for (;;) {
        plfm_hal_port_watchdog_kick();
        g_diag.uptime_ms += 1000U;
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}
