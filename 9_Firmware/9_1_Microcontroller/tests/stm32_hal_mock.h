/*******************************************************************************
 * stm32_hal_mock.h -- Minimal STM32 HAL stubs for host-side unit testing
 *
 * Provides:
 *   - Stub GPIO_TypeDef, SPI/I2C/UART/TIM handle types
 *   - GPIO pin defines (GPIO_PIN_0..GPIO_PIN_15)
 *   - GPIO port stub addresses (GPIOA..GPIOG)
 *   - HAL function declarations (spy-layer implemented in stm32_hal_mock.c)
 *   - Pin defines from BOTH main.h and adf4382a_manager.h (to test conflict)
 *   - Misc types/constants needed by the real source files under test
 *
 * This file intentionally does NOT include the real stm32f7xx_hal.h.
 * It replaces it entirely for macOS compilation.
 ******************************************************************************/
#ifndef STM32_HAL_MOCK_H
#define STM32_HAL_MOCK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================= Basic Types =========================== */

typedef uint32_t HAL_StatusTypeDef;
#define HAL_OK        0U
#define HAL_ERROR     1U
#define HAL_BUSY      2U
#define HAL_TIMEOUT   3U

#define HAL_MAX_DELAY 0xFFFFFFFFU

#ifndef __NOP
#define __NOP() ((void)0)
#endif

/* ========================= GPIO Types ============================ */

typedef struct {
    uint32_t id;   /* unique identifier for test assertions */
} GPIO_TypeDef;

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET   = 1
} GPIO_PinState;

/* GPIO pin bit masks (same as STM32 HAL) */
#define GPIO_PIN_0    ((uint16_t)0x0001)
#define GPIO_PIN_1    ((uint16_t)0x0002)
#define GPIO_PIN_2    ((uint16_t)0x0004)
#define GPIO_PIN_3    ((uint16_t)0x0008)
#define GPIO_PIN_4    ((uint16_t)0x0010)
#define GPIO_PIN_5    ((uint16_t)0x0020)
#define GPIO_PIN_6    ((uint16_t)0x0040)
#define GPIO_PIN_7    ((uint16_t)0x0080)
#define GPIO_PIN_8    ((uint16_t)0x0100)
#define GPIO_PIN_9    ((uint16_t)0x0200)
#define GPIO_PIN_10   ((uint16_t)0x0400)
#define GPIO_PIN_11   ((uint16_t)0x0800)
#define GPIO_PIN_12   ((uint16_t)0x1000)
#define GPIO_PIN_13   ((uint16_t)0x2000)
#define GPIO_PIN_14   ((uint16_t)0x4000)
#define GPIO_PIN_15   ((uint16_t)0x8000)

/* GPIO port stubs -- each gets a distinct static instance */
extern GPIO_TypeDef gpio_a, gpio_b, gpio_c, gpio_d, gpio_e, gpio_f, gpio_g;

#define GPIOA (&gpio_a)
#define GPIOB (&gpio_b)
#define GPIOC (&gpio_c)
#define GPIOD (&gpio_d)
#define GPIOE (&gpio_e)
#define GPIOF (&gpio_f)
#define GPIOG (&gpio_g)

/* ========================= Peripheral Handle Types ================ */

typedef struct {
    uint32_t id;
} SPI_HandleTypeDef;

typedef struct {
    uint32_t id;
} I2C_HandleTypeDef;

typedef struct {
    uint32_t id;
} UART_HandleTypeDef;

typedef struct {
    uint32_t id;
} TIM_HandleTypeDef;

typedef struct {
    uint32_t id;
} ADC_HandleTypeDef;

/* ========================= Extern Peripheral Instances ============ */

extern SPI_HandleTypeDef  hspi1, hspi4;
extern I2C_HandleTypeDef  hi2c1, hi2c2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;  /* GPS UART */
extern ADC_HandleTypeDef  hadc3;
extern TIM_HandleTypeDef  htim3;  /* Timer for DELADJ PWM */

/* ========================= SPY / RECORDING LAYER ================== */

/* Each HAL call is recorded in a ring buffer for test inspection */

typedef enum {
    SPY_GPIO_WRITE,
    SPY_GPIO_READ,
    SPY_GPIO_TOGGLE,
    SPY_HAL_DELAY,
    SPY_HAL_GET_TICK,
    SPY_UART_TX,
    SPY_ADF4382_INIT,
    SPY_ADF4382_REMOVE,
    SPY_ADF4382_SET_OUT_POWER,
    SPY_ADF4382_SET_EN_CHAN,
    SPY_ADF4382_SET_TIMED_SYNC,
    SPY_ADF4382_SET_EZSYNC,
    SPY_ADF4382_SET_SW_SYNC,
    SPY_ADF4382_SPI_READ,
    SPY_AD9523_INIT,
    SPY_AD9523_SETUP,
    SPY_AD9523_STATUS,
    SPY_AD9523_SYNC,
    SPY_AD9523_REMOVE,
    SPY_NO_OS_UDELAY,
    SPY_ADS7830_MEASURE,
    SPY_TIM_PWM_START,
    SPY_TIM_PWM_STOP,
    SPY_TIM_SET_COMPARE,
    SPY_SPI_TRANSMIT_RECEIVE,
    SPY_SPI_TRANSMIT,
    SPY_UART_RX,
} SpyCallType;

typedef struct {
    SpyCallType type;
    void       *port;       /* GPIO port or NULL */
    uint16_t    pin;        /* GPIO pin or 0 */
    uint32_t    value;      /* pin state, delay ms, tick value, etc. */
    void       *extra;      /* device pointer, etc. */
} SpyRecord;

#define SPY_MAX_RECORDS 512

extern SpyRecord spy_log[SPY_MAX_RECORDS];
extern int       spy_count;

/* Reset spy log */
void spy_reset(void);

/* Read a specific spy record (returns NULL if index out of range) */
const SpyRecord *spy_get(int index);

/* Count records of a specific type */
int spy_count_type(SpyCallType type);

/* Find the Nth record of a given type (0-based). Returns index or -1. */
int spy_find_nth(SpyCallType type, int n);

/* ========================= Mock tick control ====================== */

/* Set the value HAL_GetTick() will return */
void mock_set_tick(uint32_t tick);

/* Advance the mock tick by `delta` ms */
void mock_advance_tick(uint32_t delta);

/* ========================= Mock GPIO read returns ================= */

/* Set the value HAL_GPIO_ReadPin will return for a specific port/pin */
void mock_gpio_set_read(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState val);

/* ========================= HAL Function Declarations ============== */

void          HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void          HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint32_t      HAL_GetTick(void);
void          HAL_Delay(uint32_t Delay);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

/* ========================= Mock UART RX buffer ======================= */

/* Inject bytes into the mock UART RX buffer for a specific UART handle.
 * HAL_UART_Receive will return these bytes one at a time. */
#define MOCK_UART_RX_BUF_SIZE 2048

void mock_uart_rx_load(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len);
void mock_uart_rx_clear(UART_HandleTypeDef *huart);

/* Capture buffer for UART TX data (to verify commands sent to GPS module) */
#define MOCK_UART_TX_BUF_SIZE 2048

extern uint8_t  mock_uart_tx_buf[MOCK_UART_TX_BUF_SIZE];
extern uint16_t mock_uart_tx_len;
void mock_uart_tx_clear(void);

/* ========================= SPI stubs ============================== */

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);

/* ========================= no_os compat layer ===================== */

void no_os_udelay(uint32_t usecs);
void no_os_mdelay(uint32_t msecs);

/* ========================= TIM / PWM stubs ======================== */

#define TIM_CHANNEL_1  0x00U
#define TIM_CHANNEL_2  0x04U
#define TIM_CHANNEL_3  0x08U
#define TIM_CHANNEL_4  0x0CU

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
void mock_tim_set_compare(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t Compare);

/* Macro form that the real STM32 HAL uses */
#define __HAL_TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__) \
    mock_tim_set_compare((__HANDLE__), (__CHANNEL__), (__COMPARE__))

/* ========================= ADS7830 stub =========================== */

uint8_t ADS7830_Measure_SingleEnded(ADC_HandleTypeDef *hadc, uint8_t channel);

/* ========================= Printf stub ============================ */

/* Allow printf to work normally (it's libc), but we silence it during tests
 * if desired via a global flag. */
extern int mock_printf_enabled;

#ifdef __cplusplus
}
#endif

#endif /* STM32_HAL_MOCK_H */
