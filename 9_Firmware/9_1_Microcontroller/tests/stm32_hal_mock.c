/*******************************************************************************
 * stm32_hal_mock.c -- Spy/recording implementation of STM32 HAL stubs
 ******************************************************************************/
#include "stm32_hal_mock.h"
#include "ad_driver_mock.h"
#include <stdio.h>
#include <stdlib.h>

/* ========================= GPIO port instances ==================== */
GPIO_TypeDef gpio_a = { .id = 0xA };
GPIO_TypeDef gpio_b = { .id = 0xB };
GPIO_TypeDef gpio_c = { .id = 0xC };
GPIO_TypeDef gpio_d = { .id = 0xD };
GPIO_TypeDef gpio_e = { .id = 0xE };
GPIO_TypeDef gpio_f = { .id = 0xF };
GPIO_TypeDef gpio_g = { .id = 0x6 };  /* 0x6 for GPIOG -- avoids overlap */

/* ========================= Peripheral instances =================== */
SPI_HandleTypeDef  hspi1 = { .id = 1 };
SPI_HandleTypeDef  hspi4 = { .id = 4 };
I2C_HandleTypeDef  hi2c1 = { .id = 1 };
I2C_HandleTypeDef  hi2c2 = { .id = 2 };
UART_HandleTypeDef huart3 = { .id = 3 };
UART_HandleTypeDef huart5 = { .id = 5 };  /* GPS UART */
ADC_HandleTypeDef  hadc3 = { .id = 3 };
TIM_HandleTypeDef  htim3 = { .id = 3 };

/* ========================= Spy log ================================ */
SpyRecord spy_log[SPY_MAX_RECORDS];
int       spy_count = 0;

/* ========================= Mock tick (forward decl for spy_reset) == */
uint32_t mock_tick = 0;

/* ========================= Printf control ========================= */
int mock_printf_enabled = 0;

/* ========================= Mock UART TX capture =================== */
uint8_t  mock_uart_tx_buf[MOCK_UART_TX_BUF_SIZE];
uint16_t mock_uart_tx_len = 0;

/* ========================= Mock UART RX buffer ==================== */
#define MOCK_UART_RX_SLOTS 8

static struct {
    uint32_t uart_id;
    uint8_t  buf[MOCK_UART_RX_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
} mock_uart_rx[MOCK_UART_RX_SLOTS];

void mock_uart_tx_clear(void)
{
    mock_uart_tx_len = 0;
    memset(mock_uart_tx_buf, 0, sizeof(mock_uart_tx_buf));
}

/* ========================= Mock GPIO read ========================= */
#define GPIO_READ_TABLE_SIZE 32
static struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    GPIO_PinState val;
} gpio_read_table[GPIO_READ_TABLE_SIZE];

void spy_reset(void)
{
    spy_count = 0;
    memset(spy_log, 0, sizeof(spy_log));
    mock_tick = 0;
    mock_printf_enabled = 0;
    memset(gpio_read_table, 0, sizeof(gpio_read_table));
    memset(mock_uart_rx, 0, sizeof(mock_uart_rx));
    mock_uart_tx_len = 0;
    memset(mock_uart_tx_buf, 0, sizeof(mock_uart_tx_buf));
}

const SpyRecord *spy_get(int index)
{
    if (index < 0 || index >= spy_count) return NULL;
    return &spy_log[index];
}

int spy_count_type(SpyCallType type)
{
    int count = 0;
    for (int i = 0; i < spy_count; i++) {
        if (spy_log[i].type == type) count++;
    }
    return count;
}

int spy_find_nth(SpyCallType type, int n)
{
    int found = 0;
    for (int i = 0; i < spy_count; i++) {
        if (spy_log[i].type == type) {
            if (found == n) return i;
            found++;
        }
    }
    return -1;
}

static void spy_push(SpyRecord rec)
{
    if (spy_count < SPY_MAX_RECORDS) {
        spy_log[spy_count++] = rec;
    }
}

/* ========================= Mock tick API ========================== */

void mock_set_tick(uint32_t tick)   { mock_tick = tick; }
void mock_advance_tick(uint32_t d)  { mock_tick += d; }

/* ========================= Mock GPIO read API ===================== */

void mock_gpio_set_read(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState val)
{
    for (int i = 0; i < GPIO_READ_TABLE_SIZE; i++) {
        if (gpio_read_table[i].port == port && gpio_read_table[i].pin == pin) {
            gpio_read_table[i].val = val;
            return;
        }
        if (gpio_read_table[i].port == NULL) {
            gpio_read_table[i].port = port;
            gpio_read_table[i].pin  = pin;
            gpio_read_table[i].val  = val;
            return;
        }
    }
}

/* ========================= HAL Implementations ==================== */

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    spy_push((SpyRecord){
        .type  = SPY_GPIO_WRITE,
        .port  = GPIOx,
        .pin   = GPIO_Pin,
        .value = (uint32_t)PinState,
        .extra = NULL
    });
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_PinState result = GPIO_PIN_RESET;
    for (int i = 0; i < GPIO_READ_TABLE_SIZE; i++) {
        if (gpio_read_table[i].port == GPIOx && gpio_read_table[i].pin == GPIO_Pin) {
            result = gpio_read_table[i].val;
            break;
        }
    }
    spy_push((SpyRecord){
        .type  = SPY_GPIO_READ,
        .port  = GPIOx,
        .pin   = GPIO_Pin,
        .value = (uint32_t)result,
        .extra = NULL
    });
    return result;
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    spy_push((SpyRecord){
        .type  = SPY_GPIO_TOGGLE,
        .port  = GPIOx,
        .pin   = GPIO_Pin,
        .value = 0,
        .extra = NULL
    });
}

uint32_t HAL_GetTick(void)
{
    spy_push((SpyRecord){
        .type  = SPY_HAL_GET_TICK,
        .port  = NULL,
        .pin   = 0,
        .value = mock_tick,
        .extra = NULL
    });
    return mock_tick;
}

void HAL_Delay(uint32_t Delay)
{
    spy_push((SpyRecord){
        .type  = SPY_HAL_DELAY,
        .port  = NULL,
        .pin   = 0,
        .value = Delay,
        .extra = NULL
    });
    mock_tick += Delay;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData,
                                     uint16_t Size, uint32_t Timeout)
{
    spy_push((SpyRecord){
        .type  = SPY_UART_TX,
        .port  = NULL,
        .pin   = Size,
        .value = Timeout,
        .extra = huart
    });
    /* Capture TX data for test inspection */
    for (uint16_t i = 0; i < Size && mock_uart_tx_len < MOCK_UART_TX_BUF_SIZE; i++) {
        mock_uart_tx_buf[mock_uart_tx_len++] = pData[i];
    }
    return HAL_OK;
}

/* ========================= Mock UART RX helpers ====================== */

/* find_rx_slot, mock_uart_rx_load, etc. use the mock_uart_rx declared above */

static int find_rx_slot(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return -1;
    /* Find existing slot */
    for (int i = 0; i < MOCK_UART_RX_SLOTS; i++) {
        if (mock_uart_rx[i].uart_id == huart->id && mock_uart_rx[i].head != mock_uart_rx[i].tail) {
            return i;
        }
        if (mock_uart_rx[i].uart_id == huart->id) {
            return i;
        }
    }
    /* Find empty slot */
    for (int i = 0; i < MOCK_UART_RX_SLOTS; i++) {
        if (mock_uart_rx[i].uart_id == 0) {
            mock_uart_rx[i].uart_id = huart->id;
            return i;
        }
    }
    return -1;
}

void mock_uart_rx_load(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len)
{
    int slot = find_rx_slot(huart);
    if (slot < 0) return;
    mock_uart_rx[slot].uart_id = huart->id;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (mock_uart_rx[slot].head + 1) % MOCK_UART_RX_BUF_SIZE;
        if (next == mock_uart_rx[slot].tail) break;  /* Buffer full */
        mock_uart_rx[slot].buf[mock_uart_rx[slot].head] = data[i];
        mock_uart_rx[slot].head = next;
    }
}

void mock_uart_rx_clear(UART_HandleTypeDef *huart)
{
    int slot = find_rx_slot(huart);
    if (slot < 0) return;
    mock_uart_rx[slot].head = 0;
    mock_uart_rx[slot].tail = 0;
}

HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData,
                                    uint16_t Size, uint32_t Timeout)
{
    (void)Timeout;
    int slot = find_rx_slot(huart);
    if (slot < 0) return HAL_TIMEOUT;

    for (uint16_t i = 0; i < Size; i++) {
        if (mock_uart_rx[slot].head == mock_uart_rx[slot].tail) {
            return HAL_TIMEOUT;  /* No more data */
        }
        pData[i] = mock_uart_rx[slot].buf[mock_uart_rx[slot].tail];
        mock_uart_rx[slot].tail = (mock_uart_rx[slot].tail + 1) % MOCK_UART_RX_BUF_SIZE;
    }

    spy_push((SpyRecord){
        .type  = SPY_UART_RX,
        .port  = NULL,
        .pin   = Size,
        .value = Timeout,
        .extra = huart
    });

    return HAL_OK;
}

/* ========================= no_os delay stubs ====================== */

void no_os_udelay(uint32_t usecs)
{
    spy_push((SpyRecord){
        .type  = SPY_NO_OS_UDELAY,
        .port  = NULL,
        .pin   = 0,
        .value = usecs,
        .extra = NULL
    });
}

void no_os_mdelay(uint32_t msecs)
{
    spy_push((SpyRecord){
        .type  = SPY_HAL_DELAY,
        .port  = NULL,
        .pin   = 0,
        .value = msecs,
        .extra = NULL
    });
    mock_tick += msecs;
}

/* ========================= ADS7830 stub =========================== */

uint8_t ADS7830_Measure_SingleEnded(ADC_HandleTypeDef *hadc, uint8_t channel)
{
    spy_push((SpyRecord){
        .type  = SPY_ADS7830_MEASURE,
        .port  = NULL,
        .pin   = channel,
        .value = 100,  /* stub: always return 100 (~64.7 C) */
        .extra = hadc
    });
    return 100;
}

/* ========================= TIM PWM stubs ========================== */

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    spy_push((SpyRecord){
        .type  = SPY_TIM_PWM_START,
        .port  = NULL,
        .pin   = (uint16_t)Channel,
        .value = htim->id,
        .extra = htim
    });
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    spy_push((SpyRecord){
        .type  = SPY_TIM_PWM_STOP,
        .port  = NULL,
        .pin   = (uint16_t)Channel,
        .value = htim->id,
        .extra = htim
    });
    return HAL_OK;
}

void mock_tim_set_compare(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t Compare)
{
    spy_push((SpyRecord){
        .type  = SPY_TIM_SET_COMPARE,
        .port  = NULL,
        .pin   = (uint16_t)Channel,
        .value = Compare,
        .extra = htim
    });
}

/* ========================= SPI stubs ============================== */

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
    spy_push((SpyRecord){
        .type  = SPY_SPI_TRANSMIT_RECEIVE,
        .port  = NULL,
        .pin   = Size,
        .value = Timeout,
        .extra = hspi
    });
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    spy_push((SpyRecord){
        .type  = SPY_SPI_TRANSMIT,
        .port  = NULL,
        .pin   = Size,
        .value = Timeout,
        .extra = hspi
    });
    return HAL_OK;
}

/* Stub for platform_noos_stm32.c GPIO functions */
void hal_set_gpio_by_index(uint8_t idx, uint8_t value) {
    (void)idx; (void)value;
}

/* ========================= Mock stm32_spi_ops ===================== */

/* Stub SPI platform ops -- real adf4382a_manager.c references &stm32_spi_ops.
 * In tests, adf4382_init() is mocked so no_os_spi_init() is never called.
 * We provide a non-NULL struct so tests can assert platform_ops != NULL. */
static int mock_spi_init_stub(void) { return 0; }

const struct no_os_spi_platform_ops stm32_spi_ops = {
    .init = mock_spi_init_stub,
};
