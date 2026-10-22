/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "USBHandler.h"
#include "usbd_cdc_if.h"
#include "ADAR1000_Manager.h"
#include "ADAR1000_AGC.h"
extern "C" {
#include "ad9523.h"
}
#include "no_os_delay.h"
#include "no_os_alloc.h"
#include "no_os_print_log.h"
#include "no_os_error.h"
#include "no_os_units.h"
#include "no_os_dma.h"
#include "no_os_spi.h"
#include "no_os_uart.h"
#include "no_os_util.h"
#include <stdint.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <iostream>
#include <vector>
#include "stm32_spi.h"
#include "stm32_delay.h"
extern "C" {
#include "um982_gps.h"
}
extern "C" {
#include "GY_85_HAL.h"
}
#include "BMP180.h"
extern "C" {
#include "platform_noos_stm32.h"
}
extern "C" {

#include "adf4382a_manager.h"
#include "adf4382.h"
#include "no_os_delay.h"
#include "no_os_error.h"
#include "diag_log.h"
}
#include <cmath>
#include "DAC5578.h"
#include "ADS7830.h"
#include "gps_handler.h"

#include "stm32f7xx_hal.h"




/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* -------- GPIO index mapping for no-OS interface -------- */
#define GPIO_IDX_TX_CS       0
#define GPIO_IDX_RX_CS       1
#define GPIO_IDX_TX_DELSTR   2
#define GPIO_IDX_TX_DELADJ   3
#define GPIO_IDX_RX_DELSTR   4
#define GPIO_IDX_RX_DELADJ   5
#define GPIO_IDX_TX_CE       6
#define GPIO_IDX_RX_CE       7
#define GPIO_IDX_SW_SYNC     8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;  // B15 fix: DELADJ PWM timer (CH2=TX, CH3=RX)

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// UM982 dual-antenna GPS receiver
UM982_GPS_t um982;

// Global data structures
GPS_Data_t current_gps_data = {0};

// Global USB handler
USBHandler usbHandler;
extern USBD_HandleTypeDef hUsbDeviceFS;

// Buffer for USB reception
uint8_t usb_rx_buffer[64];

//IMU variable
GY85_t imu;

float Pitch_Sensor, Yaw_Sensor, Roll_Sensor;

float abias[3] = {-0.108, -0.038, -0.006}, gbias[3] = {-10, 6, -12}, mbias[3]={0.0,0.0,0.0};
float M11=1.0,  M12=-0.0,  M13=0.0,
      M21=0.0,  M22=1.0,   M23=0.0,
      M31=0.0,  M32=-0.0,  M33=1.0;
float ax, ay, az, gx, gy, gz, mx, my, mz,mxc,myc,mzc; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float temperature;
float Mag_Declination = -0.61; //0°

float RxAcc,RyAcc,RzAcc;
float Rate_Ayz_1,Rate_Axz_1,Rate_Axy_1;
float Axz_1,Ayz_1,Axy_1, Axz_0,Ayz_0,Axy_0;
float RxEst_0,RyEst_0,RzEst_0, RxEst_1,RyEst_1,RzEst_1,Azr_Est;
float RxGyro,RyGyro,RzGyro, R_Est;

unsigned long now_timeperiod;
unsigned long lasttime_timeperiod = 0;
int Time_Period = 0;

//Barometer BMP180

/*resolution:
BMP180_ULTRALOWPOWER - pressure oversampled 1 time  & power consumption 3μA
BMP180_STANDARD      - pressure oversampled 2 times & power consumption 5μA
BMP180_HIGHRES       - pressure oversampled 4 times & power consumption 7μA
BMP180_ULTRAHIGHRES  - pressure oversampled 8 times & power consumption 12μA, library default
*/
BMP180 myBMP(BMP180_ULTRAHIGHRES);
float RADAR_Altitude;

//GPS
double RADAR_Longitude = 0;
double RADAR_Latitude = 0;

extern uint8_t GUI_start_flag_received;  // [STM32-006] Legacy, unused -- kept for linker compat


//RADAR
// Radar parameters
const int m_max = 32;  // Number of chirps per beam position
const int n_max = 31;   // Number of beam positions
const float T1 = 30.0f; // Chirp duration in microseconds
const float PRI1 = 167.0f; // Pulse repetition interval in microseconds
const float T2 = 0.5f;  // Short chirp duration in microseconds
const float PRI2 = 175.0f; // Short PRI in microseconds
const float Guard = 175.4f; // Guard time in microseconds

uint8_t	m = 1; // m = N° of chirp/ position = 16 (made of T1 and PRF1)+ Guard = 175µs +16 (made of T2 and PRF2)
uint8_t	n = 1; // n =[1-31] N° of elevations/azimuth
uint8_t	y = 1;// y = N° of azimuths/revolution = 50 controlled by a microcontroller connected to a stepper motor
uint8_t y_max = 50;// y = N° of azimuths/revolution
uint64_t IF_freq = 120000000ULL;//120MHz

#define PowerAmplifier 1

//Stepper Motor
float Stepper_steps = 200.0f;//step per revolution

// DAC5578 handles (RF Power Amplifier DAC controlling Vg)
DAC5578_HandleTypeDef hdac1, hdac2;
/* Phase accumulators */
float phase_dac1[8] = {0};
float phase_dac2[8] = {0};
const uint32_t sampleRate = 370;    // Sample rate in Hz
const uint32_t period = 2700;       // Period in microseconds
uint8_t DAC_val = 126;

// ADC handles (RF Power Amplifier ADC measuring Idq)
ADS7830_HandleTypeDef hadc1, hadc2;
uint8_t adc1_readings[8] = {0};
uint8_t adc2_readings[8] = {0};
float Idq_reading[16]={0.0f};

/* [GAP-3 FIX 2] Hardware IWDG watchdog handle
 * Prescaler=256, Reload=500 → timeout ≈ 4.096 s from 32 kHz LSI.
 * If the main loop stalls, the MCU resets automatically. */
IWDG_HandleTypeDef hiwdg;


// Global manager instance ADF4382A
ADF4382A_Manager lo_manager;
extern SPI_HandleTypeDef hspi4;

//ADAR1000

ADAR1000Manager adarManager;
ADAR1000_AGC    outerAgc;
static uint8_t matrix1[15][16];
static uint8_t matrix2[15][16];
static uint8_t vector_0[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/* Radar parameters
const int m_max = 32;  // Number of chirps per beam position
const int n_max = 31;   // Number of beam positions
const float T1 = 30.0f; // Chirp duration in microseconds
const float PRI1 = 167.0f; // Pulse repetition interval in microseconds
const float T2 = 0.5f;  // Short chirp duration in microseconds
const float T2 = 175.0f; // Short PRI in microseconds
const float Guard = 175.40f; // Guard time in microseconds*/

//Temperature Sensors
ADS7830_HandleTypeDef hadc3;
float Temperature_1 = 0.0f, Temperature_2 = 0.0f, Temperature_3 = 0.0f, Temperature_4 = 0.0f;
float Temperature_5 = 0.0f, Temperature_6 = 0.0f, Temperature_7 = 0.0f, Temperature_8 = 0.0f;

// Phase differences for 31 beam positions
const float phase_differences[31] = {
    160.0f, 80.0f, 53.333f, 40.0f, 32.0f, 26.667f, 22.857f, 20.0f, 17.778f, 16.0f,
    14.545f, 13.333f, 12.308f, 11.429f, 10.667f, 0.0f,
    -10.667f, -11.429f, -12.308f, -13.333f, -14.545f, -16.0f, -17.778f, -20.0f,
    -22.857f, -26.667f, -32.0f, -40.0f, -53.333f, -80.0f, -160.0f
};

// Convenience HAL wrappers for AD9523 control pins (GPIOF as you said)
static inline void AD9523_PD(bool v)        { HAL_GPIO_WritePin(GPIOF, AD9523_PD_Pin, v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static inline void AD9523_REF_SEL(bool v)   { HAL_GPIO_WritePin(GPIOF, AD9523_REF_SEL_Pin, v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static inline void AD9523_SYNC_PULSE(void)  { HAL_GPIO_WritePin(GPIOF, AD9523_SYNC_Pin, GPIO_PIN_SET); HAL_Delay(1); HAL_GPIO_WritePin(GPIOF, AD9523_SYNC_Pin, GPIO_PIN_RESET); }
static inline void AD9523_RESET_ASSERT()    { HAL_GPIO_WritePin(GPIOF, AD9523_RESET_Pin, GPIO_PIN_RESET); } // check polarity
static inline void AD9523_RESET_RELEASE()   { HAL_GPIO_WritePin(GPIOF, AD9523_RESET_Pin, GPIO_PIN_SET); }
static inline void AD9523_CS(bool v)        { HAL_GPIO_WritePin(GPIOF, AD9523_CS_Pin, v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static inline void AD9523_EEPROM_SEL(bool v){ HAL_GPIO_WritePin(GPIOF, AD9523_EEPROM_SEL_Pin, v ? GPIO_PIN_SET : GPIO_PIN_RESET); }

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);  // B15 fix: DELADJ PWM timer init
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_IWDG_Init(void);  /* GAP-3 FIX 2: hardware watchdog */
/* USER CODE BEGIN PFP */

// Function prototypes
void systemPowerUpSequence();
void systemPowerDownSequence();
void initializeBeamMatrices();
void runRadarPulseSequence();
void executeChirpSequence(int num_chirps, uint32_t T1, uint32_t PRI1, uint32_t T2, uint32_t PRI2);
void printSystemStatus();

//////////////////////////////////////////////
////////////////////micros()//////////////////
//////////////////////////////////////////////
unsigned long micros(void){
unsigned long Micros = __HAL_TIM_GET_COUNTER(&htim1);
return Micros; //Clock TIM1 -> AHB/APB1 is set to 72MHz/presc+1   presc = 71
}

//////////////////////////////////////////////
//////////////////Delay_us()//////////////////
//////////////////////////////////////////////

void delay_us(volatile uint32_t us){
__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a
while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // //Clock TIMx -> AHB/APB1 is set to 72MHz/presc+1   presc = 71
}

//////////////////////////////////////////////
//////////////////delay_ns()//////////////////
//////////////////////////////////////////////
void delay_ns(uint32_t nanoseconds)
{
    // Get the starting cycle count
    uint32_t start_cycles = DWT->CYCCNT;

    // Convert nanoseconds to the number of CPU clock cycles
    // Use `SystemCoreClock` to get the current CPU frequency.
    // Use 64-bit math to prevent overflow for large nanosecond values.
    uint32_t cycles = (uint32_t)(((uint64_t)nanoseconds * SystemCoreClock) / 1000000000);

    // Busy-wait until the required number of cycles has passed
    while ((DWT->CYCCNT - start_cycles) < cycles);
}

//////////////////////////////////////////////
//////////////////////RADAR///////////////////
//////////////////////////////////////////////
uint8_t degreesTo7BitPhase(float degrees) {
    // Normalize to 0-360 range
    while(degrees < 0) degrees += 360.0f;
    while(degrees >= 360.0f) degrees -= 360.0f;

    // Convert to 7-bit (0-127)
    uint8_t phase_7bit = (uint8_t)((degrees / 360.0f) * 128.0f);

    return phase_7bit % 128;
}

extern "C" {

    // USB CDC receive callback (called by STM32 HAL)
    void CDC_Receive_FS(uint8_t* Buf, uint32_t *Len) {
        DIAG("USB", "CDC_Receive_FS callback: %lu bytes received", *Len);
        // Process received USB data
        usbHandler.processUSBData(Buf, *Len);

        // Prepare for next reception
        USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &usb_rx_buffer[0]);
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    }
}

void systemPowerUpSequence() {
    DIAG_SECTION("PWR: systemPowerUpSequence");
    uint8_t msg[] = "Starting Power Up Sequence...\r\n";
    HAL_UART_Transmit(&huart3, msg, sizeof(msg)-1, 1000);

    // Step 1: Initialize ADTR1107 power sequence
    DIAG("PWR", "Step 1: initializeADTR1107Sequence()");
    if (!adarManager.initializeADTR1107Sequence()) {
        DIAG_ERR("PWR", "ADTR1107 power sequence FAILED -- calling Error_Handler()");
        uint8_t err[] = "ERROR: ADTR1107 power sequence failed!\r\n";
        HAL_UART_Transmit(&huart3, err, sizeof(err)-1, 1000);
        Error_Handler();
    }
    DIAG("PWR", "Step 1 OK: ADTR1107 sequence complete");

    // Step 2: Initialize all ADAR1000 devices
    DIAG("PWR", "Step 2: initializeAllDevices()");
    if (!adarManager.initializeAllDevices()) {
        DIAG_ERR("PWR", "ADAR1000 initialization FAILED -- calling Error_Handler()");
        uint8_t err[] = "ERROR: ADAR1000 initialization failed!\r\n";
        HAL_UART_Transmit(&huart3, err, sizeof(err)-1, 1000);
        Error_Handler();
    }
    DIAG("PWR", "Step 2 OK: All ADAR1000 devices initialized");

    // Step 3: Perform system calibration
    DIAG("PWR", "Step 3: performSystemCalibration()");
    if (!adarManager.performSystemCalibration()) {
        DIAG_WARN("PWR", "System calibration returned issues (non-fatal)");
        uint8_t warn[] = "WARNING: System calibration issues\r\n";
        HAL_UART_Transmit(&huart3, warn, sizeof(warn)-1, 1000);
    } else {
        DIAG("PWR", "Step 3 OK: System calibration passed");
    }

    // Step 4: Set to safe TX mode
    DIAG("PWR", "Step 4: setAllDevicesTXMode()");
    if (!adarManager.setAllDevicesTXMode()) {
        DIAG_ERR("PWR", "setAllDevicesTXMode() FAILED -- calling Error_Handler()");
        uint8_t err[] = "ERROR: ADAR1000 TX-mode setup failed!\r\n";
        HAL_UART_Transmit(&huart3, err, sizeof(err)-1, 1000);
        Error_Handler();
    }
    DIAG("PWR", "Step 4 OK: All devices set to TX mode");

    uint8_t success[] = "Power Up Sequence Completed Successfully\r\n";
    HAL_UART_Transmit(&huart3, success, sizeof(success)-1, 1000);
    DIAG("PWR", "systemPowerUpSequence COMPLETE");
}

void systemPowerDownSequence() {
    DIAG_SECTION("PWR: systemPowerDownSequence");
    uint8_t msg[] = "Starting Power Down Sequence...\r\n";
    HAL_UART_Transmit(&huart3, msg, sizeof(msg)-1, 1000);

    // Step 1: Set all devices to RX mode (safest state). Failure here is logged
    // but NOT fatal -- power-down must always proceed to cut the rails below.
    // Leaving a stuck PA bias would be more dangerous than losing RX-mode telemetry.
    DIAG("PWR", "Step 1: setAllDevicesRXMode()");
    if (!adarManager.setAllDevicesRXMode()) {
        DIAG_ERR("PWR", "setAllDevicesRXMode() FAILED during power-down -- continuing to cut rails");
        uint8_t warn[] = "WARNING: RX-mode setup failed during power-down, cutting rails anyway\r\n";
        HAL_UART_Transmit(&huart3, warn, sizeof(warn)-1, 1000);
    }
    HAL_Delay(10);

    // Step 2: Disable PA power supplies
    DIAG("PWR", "Step 2: Disable PA 5V supplies (EN_P_5V0_PA1/PA2/PA3 LOW)");
    HAL_GPIO_WritePin(EN_P_5V0_PA1_GPIO_Port, EN_P_5V0_PA1_Pin | EN_P_5V0_PA2_Pin | EN_P_5V0_PA3_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    // Step 3: Set PA biases to safe values
    DIAG("PWR", "Step 3: Setting PA bias to safe value 0x20 on all 4 devices, 4 channels each");
    for (uint8_t dev = 0; dev < 4; dev++) {
        adarManager.adarWrite(dev, REG_PA_CH1_BIAS_ON, 0x20, BROADCAST_OFF);
        adarManager.adarWrite(dev, REG_PA_CH2_BIAS_ON, 0x20, BROADCAST_OFF);
        adarManager.adarWrite(dev, REG_PA_CH3_BIAS_ON, 0x20, BROADCAST_OFF);
        adarManager.adarWrite(dev, REG_PA_CH4_BIAS_ON, 0x20, BROADCAST_OFF);
    }
    HAL_Delay(10);

    // Step 4: Disable LNA power supply
    DIAG("PWR", "Step 4: Disable LNA 3.3V supply (EN_P_3V3_ADTR LOW)");
    HAL_GPIO_WritePin(EN_P_3V3_ADTR_GPIO_Port, EN_P_3V3_ADTR_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    // Step 5: Set LNA bias to safe value
    DIAG("PWR", "Step 5: Setting LNA bias to 0x00 on all 4 devices");
    for (uint8_t dev = 0; dev < 4; dev++) {
        adarManager.adarWrite(dev, REG_LNA_BIAS_ON, 0x00, BROADCAST_OFF);
    }
    HAL_Delay(10);

    // Step 6: Disable switch power supplies
    DIAG("PWR", "Step 6: Disable switch supplies (EN_P_3V3_VDD_SW, EN_P_3V3_SW LOW)");
    HAL_GPIO_WritePin(EN_P_3V3_VDD_SW_GPIO_Port, EN_P_3V3_VDD_SW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_P_3V3_SW_GPIO_Port, EN_P_3V3_SW_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    uint8_t success[] = "Power Down Sequence Completed\r\n";
    HAL_UART_Transmit(&huart3, success, sizeof(success)-1, 1000);
    DIAG("PWR", "systemPowerDownSequence COMPLETE");
}

void initializeBeamMatrices() {
    uint8_t msg[] = "Initializing Beam Matrices...\r\n";
    HAL_UART_Transmit(&huart3, msg, sizeof(msg)-1, 1000);

    // Matrix1: Positions 1-15 (positive phase differences)
    for(int beam_pos = 0; beam_pos < 15; beam_pos++) {
        float phase_diff_degrees = phase_differences[beam_pos];

        for(int element = 0; element < 16; element++) {
            float cumulative_phase_degrees = element * phase_diff_degrees;
            matrix1[beam_pos][element] = degreesTo7BitPhase(cumulative_phase_degrees);
        }
    }

    // Matrix2: Positions 17-31 (negative phase differences)
    for(int beam_pos = 0; beam_pos < 15; beam_pos++) {
        float phase_diff_degrees = phase_differences[beam_pos + 16];

        for(int element = 0; element < 16; element++) {
            float cumulative_phase_degrees = element * phase_diff_degrees;
            matrix2[beam_pos][element] = degreesTo7BitPhase(cumulative_phase_degrees);
        }
    }

    // Vector_0: Position 16 (zero phase - broadside)
    for(int element = 0; element < 16; element++) {
        vector_0[element] = 0; // 0 in 7-bit = 0° phase
    }

    uint8_t done[] = "Beam Matrices Initialized\r\n";
    HAL_UART_Transmit(&huart3, done, sizeof(done)-1, 1000);
}

void executeChirpSequence(int num_chirps, float T1, float PRI1, float T2, float PRI2) {
    // NOTE: No per-chirp DIAG — this is a us/ns timing-critical path.
    // Only log entry params for post-mortem analysis.
    
    DIAG("SYS", "executeChirpSequence: num_chirps=%d T1=%.2f PRI1=%.2f T2=%.2f PRI2=%.2f",
         num_chirps, T1, PRI1, T2, PRI2);
    // First chirp sequence (microsecond timing)
    for(int i = 0; i < num_chirps; i++) {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_8); // New chirp signal to FPGA
        (void)adarManager.pulseTXMode();
        delay_us((uint32_t)T1);
        (void)adarManager.pulseRXMode();
        delay_us((uint32_t)(PRI1 - T1));
    }

    delay_us((uint32_t)Guard);

    // Second chirp sequence (nanosecond timing)
    for(int i = 0; i < num_chirps; i++) {
    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_8); // New chirp signal to FPGA
        (void)adarManager.pulseTXMode();
        delay_ns((uint32_t)(T2 * 1000));
        (void)adarManager.pulseRXMode();
        delay_ns((uint32_t)((PRI2 - T2) * 1000));

    }
}

void runRadarPulseSequence() {
    static int sequence_count = 0;
    char msg[50];

    snprintf(msg, sizeof(msg), "Starting RADAR Sequence #%d\r\n", ++sequence_count);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 1000);
    DIAG("SYS", "runRadarPulseSequence #%d: m_max=%d n_max=%d y_max=%d",
         sequence_count, m_max, n_max, y_max);

    // Configure for fast switching
    DIAG("BF", "Enabling fast-switch mode for beam sweep");
    adarManager.setFastSwitchMode(true);

    int m = 1; // Chirp counter
    int n = 1; // Beam Elevation position counter
    int y = 1; // Beam Azimuth counter

    // Main beam steering sequence
    for(int beam_pos = 0; beam_pos < 15; beam_pos++) {
    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);// Notify FPGA of elevation change
    	DIAG("SYS", "Beam pos %d/15: elevation GPIO toggle, patterns matrix1/vector_0/matrix2", beam_pos);
        // Pattern 1: matrix1 (positive steering angles)
        adarManager.setCustomBeamPattern16(matrix1[beam_pos], ADAR1000Manager::BeamDirection::TX);
        adarManager.setCustomBeamPattern16(matrix1[beam_pos], ADAR1000Manager::BeamDirection::RX);

        executeChirpSequence(m_max/2, T1, PRI1, T2, PRI2);
        m += m_max/2;

        // Pattern 2: vector_0 (broadside)
        adarManager.setCustomBeamPattern16(vector_0, ADAR1000Manager::BeamDirection::TX);
        adarManager.setCustomBeamPattern16(vector_0, ADAR1000Manager::BeamDirection::RX);

        executeChirpSequence(m_max/2, T1, PRI1, T2, PRI2);
        m += m_max/2;

        // Pattern 3: matrix2 (negative steering angles)
        adarManager.setCustomBeamPattern16(matrix2[beam_pos], ADAR1000Manager::BeamDirection::TX);
        adarManager.setCustomBeamPattern16(matrix2[beam_pos], ADAR1000Manager::BeamDirection::RX);

        executeChirpSequence(m_max/2, T1, PRI1, T2, PRI2);
        m += m_max/2;

        // Reset chirp counter if needed
        if(m > m_max) m = 1;

        n++;
        if(n > n_max) n = 1;

    }

    HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_10);//Tell FPGA that there is a new azimuth
    DIAG("SYS", "Azimuth GPIO toggle (GPIOD pin 10), stepping motor");

    y++; if(y>y_max)y=1;
	  //Rotate stepper to next y position
	  for(int k= 0;k<(int)(Stepper_steps/y_max);k++){
		  HAL_GPIO_WritePin(STEPPER_CLK_P_GPIO_Port, STEPPER_CLK_P_Pin, GPIO_PIN_SET);
		  delay_us(500);
		  HAL_GPIO_WritePin(STEPPER_CLK_P_GPIO_Port, STEPPER_CLK_P_Pin, GPIO_PIN_RESET);
		  delay_us(500);
	  }
    DIAG("MOT", "Stepper moved %d steps for azimuth position y=%d", (int)(Stepper_steps/y_max), y);



    snprintf(msg, sizeof(msg), "RADAR Sequence #%d Completed\r\n", sequence_count);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 1000);
    DIAG("SYS", "runRadarPulseSequence #%d COMPLETE", sequence_count);
}




void printSystemStatus() {
    char status_msg[100];

    // Print device status
    for(int i = 0; i < 4; i++) {
        if(adarManager.verifyDeviceCommunication(i)) {
            float temp = adarManager.readTemperature(i);
            snprintf(status_msg, sizeof(status_msg),
                    "ADAR1000 #%d: OK, Temperature: %.1fC\r\n", i+1, temp);
            HAL_UART_Transmit(&huart3, (uint8_t*)status_msg, strlen(status_msg), 1000);
        } else {
            snprintf(status_msg, sizeof(status_msg),
                    "ADAR1000 #%d: COMMUNICATION ERROR\r\n", i+1);
            HAL_UART_Transmit(&huart3, (uint8_t*)status_msg, strlen(status_msg), 1000);
        }
    }

    // Print matrix info
    snprintf(status_msg, sizeof(status_msg),
            "Beam Matrices: 31 positions, %d chirps/position\r\n", m_max);
    HAL_UART_Transmit(&huart3, (uint8_t*)status_msg, strlen(status_msg), 1000);
}

/***************************************************************/
/**********************SYSTEM ERROR HANDLER*********************/
/***************************************************************/

typedef enum {
    ERROR_NONE = 0,
    ERROR_AD9523_CLOCK,
    ERROR_ADF4382_TX_UNLOCK,
    ERROR_ADF4382_RX_UNLOCK,
    ERROR_ADAR1000_COMM,
    ERROR_ADAR1000_TEMP,
    ERROR_IMU_COMM,
    ERROR_BMP180_COMM,
    ERROR_GPS_COMM,
    ERROR_RF_PA_OVERCURRENT,
    ERROR_RF_PA_BIAS,
    ERROR_STEPPER_MOTOR,
    ERROR_FPGA_COMM,
    ERROR_POWER_SUPPLY,
    ERROR_TEMPERATURE_HIGH,
    ERROR_MEMORY_ALLOC,
    ERROR_WATCHDOG_TIMEOUT,
    ERROR_COUNT  // must be last — used for bounds checking error_strings[]
} SystemError_t;

static SystemError_t last_error = ERROR_NONE;
static uint32_t error_count = 0;
static volatile bool system_emergency_state = false;

// Error handler function
SystemError_t checkSystemHealth(void) {
    SystemError_t current_error = ERROR_NONE;

    // 0. Watchdog: detect main-loop stall (checkSystemHealth not called for >60 s).
    //    Timestamp is captured at function ENTRY and updated unconditionally, so
    //    any early return from a sub-check below cannot leave a stale value that
    //    would later trip a spurious ERROR_WATCHDOG_TIMEOUT. A dedicated cold-start
    //    branch ensures the first call after boot never trips (last_health_check==0
    //    would otherwise make `HAL_GetTick() - 0 > 60000` true forever after the
    //    60-s mark of the init sequence).
    static uint32_t last_health_check = 0;
    uint32_t now_tick = HAL_GetTick();
    if (last_health_check == 0) {
        last_health_check = now_tick;          // cold start: seed only
    } else {
        uint32_t elapsed = now_tick - last_health_check;
        last_health_check = now_tick;          // update BEFORE any early return
        if (elapsed > 60000) {
            current_error = ERROR_WATCHDOG_TIMEOUT;
            DIAG_ERR("SYS", "Health check: Watchdog timeout (>60s since last check)");
            return current_error;
        }
    }

    // 1. Check AD9523 Clock Generator
    static uint32_t last_clock_check = 0;
    if (HAL_GetTick() - last_clock_check > 5000) {
        GPIO_PinState s0 = HAL_GPIO_ReadPin(AD9523_STATUS0_GPIO_Port, AD9523_STATUS0_Pin);
        GPIO_PinState s1 = HAL_GPIO_ReadPin(AD9523_STATUS1_GPIO_Port, AD9523_STATUS1_Pin);
        DIAG_GPIO("CLK", "AD9523 STATUS0", s0);
        DIAG_GPIO("CLK", "AD9523 STATUS1", s1);
        if (s0 == GPIO_PIN_RESET || s1 == GPIO_PIN_RESET) {
            current_error = ERROR_AD9523_CLOCK;
            DIAG_ERR("CLK", "AD9523 clock health check FAILED (STATUS0=%d STATUS1=%d)", s0, s1);
            return current_error;
        }
        last_clock_check = HAL_GetTick();
    }

    // 2. Check ADF4382 Lock Status
    bool tx_locked, rx_locked;
    if (ADF4382A_CheckLockStatus(&lo_manager, &tx_locked, &rx_locked) == ADF4382A_MANAGER_OK) {
        if (!tx_locked) {
            current_error = ERROR_ADF4382_TX_UNLOCK;
            DIAG_ERR("LO", "Health check: TX LO UNLOCKED");
            return current_error;
        }
        if (!rx_locked) {
            current_error = ERROR_ADF4382_RX_UNLOCK;
            DIAG_ERR("LO", "Health check: RX LO UNLOCKED");
            return current_error;
        }
    }

    // 3. Check ADAR1000 Communication and Temperature
    for (int i = 0; i < 4; i++) {
        if (!adarManager.verifyDeviceCommunication(i)) {
            current_error = ERROR_ADAR1000_COMM;
            DIAG_ERR("BF", "Health check: ADAR1000 #%d comm FAILED", i);
            return current_error;
        }

        float temp = adarManager.readTemperature(i);
        // NaN signals an ADC timeout / comm failure inside the manager. Previously
        // a hung ADC returned 0, which mapped to -50 C and looked healthy. Map
        // it to the comm-error bucket so attemptErrorRecovery re-inits the chip.
        if (isnan(temp)) {
            current_error = ERROR_ADAR1000_COMM;
            DIAG_ERR("BF", "Health check: ADAR1000 #%d temperature read returned NaN (ADC timeout)", i);
            return current_error;
        }
        if (temp > 85.0f) {
            current_error = ERROR_ADAR1000_TEMP;
            DIAG_ERR("BF", "Health check: ADAR1000 #%d OVERTEMP %.1fC > 85C", i, temp);
            return current_error;
        }
    }

    // 4. Check IMU Communication
    static uint32_t last_imu_check = 0;
    if (HAL_GetTick() - last_imu_check > 10000) {
        if (!GY85_Update(&imu)) {
            current_error = ERROR_IMU_COMM;
            DIAG_ERR("IMU", "Health check: GY85_Update() FAILED");
            return current_error;
        }
        last_imu_check = HAL_GetTick();
    }

    // 5. Check BMP180 Communication
    static uint32_t last_bmp_check = 0;
    if (HAL_GetTick() - last_bmp_check > 15000) {
        double pressure = myBMP.getPressure();
        if (pressure < 30000.0 || pressure > 110000.0 || isnan(pressure)) {
            current_error = ERROR_BMP180_COMM;
            DIAG_ERR("SYS", "Health check: BMP180 pressure out of range: %.0f", pressure);
            return current_error;
        }
        last_bmp_check = HAL_GetTick();
    }

    // 6. Check GPS Communication (30s grace period from boot / last valid fix)
    uint32_t gps_fix_age = um982_position_age(&um982);
    if (gps_fix_age > 30000) {
        current_error = ERROR_GPS_COMM;
        DIAG_WARN("SYS", "Health check: GPS no fix for >30s (age=%lu ms)", (unsigned long)gps_fix_age);
        return current_error;
    }

    // 7. Check RF Power Amplifier Current
    if (PowerAmplifier) {
        for (int i = 0; i < 16; i++) {
            if (Idq_reading[i] > 2.5f) {
                current_error = ERROR_RF_PA_OVERCURRENT;
                DIAG_ERR("PA", "Health check: PA ch%d OVERCURRENT Idq=%.3fA > 2.5A", i, Idq_reading[i]);
                return current_error;
            }
            if (Idq_reading[i] < 0.1f) {
                current_error = ERROR_RF_PA_BIAS;
                DIAG_ERR("PA", "Health check: PA ch%d BIAS FAULT Idq=%.3fA < 0.1A", i, Idq_reading[i]);
                return current_error;
            }
        }
    }

    // 8. Check System Temperature
    if (temperature > 75.0f) {
        current_error = ERROR_TEMPERATURE_HIGH;
        DIAG_ERR("SYS", "Health check: System OVERTEMP %.1fC > 75C", temperature);
        return current_error;
    }

    // 9. Watchdog check is performed at function entry (see step 0).

    if (current_error != ERROR_NONE) {
        DIAG_ERR("SYS", "checkSystemHealth returning error code %d", current_error);
    }
    return current_error;
}

// Error recovery function
void attemptErrorRecovery(SystemError_t error) {
    DIAG_SECTION("SYS: attemptErrorRecovery");
    DIAG("SYS", "Attempting recovery from error code %d", error);
    char recovery_msg[80];
    snprintf(recovery_msg, sizeof(recovery_msg),
             "Attempting recovery from error: %d\r\n", error);
    HAL_UART_Transmit(&huart3, (uint8_t*)recovery_msg, strlen(recovery_msg), 1000);

    switch (error) {
        case ERROR_ADF4382_TX_UNLOCK:
        case ERROR_ADF4382_RX_UNLOCK:
            // Re-initialize LO
            DIAG("LO", "Recovery: Re-initializing LO manager (SYNC_METHOD_TIMED)");
            ADF4382A_Manager_Init(&lo_manager, SYNC_METHOD_TIMED);
            HAL_Delay(100);
            DIAG("LO", "Recovery: LO re-init complete");
            break;

        case ERROR_ADAR1000_COMM:
            // Reset ADAR1000 communication. Previously this discarded the bool
            // return, so a re-init that failed silently looked like recovery
            // succeeded -- the next health check would loop right back here.
            DIAG("BF", "Recovery: Re-initializing all ADAR1000 devices");
            if (!adarManager.initializeAllDevices()) {
                DIAG_ERR("BF", "Recovery FAILED: ADAR1000 re-init still failing -- escalating to emergency state");
                uint8_t err[] = "ERROR: ADAR1000 recovery failed, entering emergency state\r\n";
                HAL_UART_Transmit(&huart3, err, sizeof(err)-1, 1000);
                system_emergency_state = true;
                error_count++;
            } else {
                HAL_Delay(50);
                DIAG("BF", "Recovery: ADAR1000 re-init complete");
            }
            break;

        case ERROR_IMU_COMM:
            // Re-initialize IMU
            DIAG("IMU", "Recovery: Re-initializing GY85 IMU");
            GY85_Init();
            HAL_Delay(100);
            DIAG("IMU", "Recovery: IMU re-init complete");
            break;

        case ERROR_GPS_COMM:
            // GPS will auto-recover when signal returns
            DIAG("SYS", "Recovery: GPS error -- no action (auto-recover on signal)");
            break;

        default:
            // For other errors, just log and continue
            DIAG_WARN("SYS", "Recovery: No specific handler for error %d", error);
            break;
    }

    snprintf(recovery_msg, sizeof(recovery_msg),
             "Recovery attempt completed.\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)recovery_msg, strlen(recovery_msg), 1000);
    DIAG("SYS", "attemptErrorRecovery COMPLETE");
}

////////////////////////////////////////////////////////////////////////////////
//:::::RF POWER AMPLIFIER DAC5578 Emergency stop function using CLR pin/////////
////////////////////////////////////////////////////////////////////////////////
void Emergency_Stop(void) {
    DIAG_ERR("PA", ">>> EMERGENCY_STOP ACTIVATED <<<");
    /* Immediately clear all DAC outputs to zero using hardware CLR */
    DIAG_ERR("PA", "Clearing DAC1 outputs via CLR pin");
    DAC5578_ActivateClearPin(&hdac1);
    DIAG_ERR("PA", "Clearing DAC2 outputs via CLR pin");
    DAC5578_ActivateClearPin(&hdac2);

    /* [GAP-3 FIX 1] Cut RF and PA power rails — DAC CLR alone is not enough.
     * With gate voltage cleared but VDD still energized, PAs can self-bias
     * or oscillate.  Disable everything in fast-to-slow order:
     *   1. TX mixers (stop RF immediately)
     *   2. PA 5V per-element supplies
     *   3. PA 5.5V bulk supply
     *   4. RFPA VDD enable
     */
    DIAG_ERR("PA", "Disabling TX mixers (GPIOD pin 11 LOW)");
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);

    DIAG_ERR("PA", "Cutting PA 5V supplies (PA1/PA2/PA3 LOW)");
    HAL_GPIO_WritePin(EN_P_5V0_PA1_GPIO_Port, EN_P_5V0_PA1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_P_5V0_PA2_GPIO_Port, EN_P_5V0_PA2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_P_5V0_PA3_GPIO_Port, EN_P_5V0_PA3_Pin, GPIO_PIN_RESET);

    DIAG_ERR("PA", "Cutting PA 5.5V supply (EN_P_5V5_PA LOW)");
    HAL_GPIO_WritePin(EN_P_5V5_PA_GPIO_Port, EN_P_5V5_PA_Pin, GPIO_PIN_RESET);

    DIAG_ERR("PA", "Disabling RFPA VDD (EN_DIS_RFPA_VDD LOW)");
    HAL_GPIO_WritePin(EN_DIS_RFPA_VDD_GPIO_Port, EN_DIS_RFPA_VDD_Pin, GPIO_PIN_RESET);

    DIAG_ERR("PA", "All PA rails cut -- entering infinite hold loop (manual reset required)");
    /* Keep outputs cleared until reset.
     * MUST refresh IWDG here — otherwise the watchdog would reset the MCU,
     * re-running startup code which re-energizes PA rails. */
    while (1) {
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(100);
    }
}

// Error response function
void handleSystemError(SystemError_t error) {
    if (error == ERROR_NONE) return;

    error_count++;
    last_error = error;
    DIAG_ERR("SYS", "handleSystemError: error=%d error_count=%lu", error, error_count);

    char error_msg[100];
    static const char* const error_strings[] = {
        "No error",
        "AD9523 Clock failure",
        "ADF4382 TX LO unlocked",
        "ADF4382 RX LO unlocked",
        "ADAR1000 communication error",
        "ADAR1000 temperature high",
        "IMU communication error",
        "BMP180 communication error",
        "GPS communication lost",
        "RF PA overcurrent detected",
        "RF PA bias fault",
        "Stepper motor fault",
        "FPGA communication error",
        "Power supply fault",
        "System temperature high",
        "Memory allocation failed",
        "Watchdog timeout"
    };

    static_assert(sizeof(error_strings) / sizeof(error_strings[0]) == ERROR_COUNT,
                  "error_strings[] and SystemError_t enum are out of sync");

    const char* err_name = (error >= 0 && error < (int)(sizeof(error_strings) / sizeof(error_strings[0])))
                           ? error_strings[error]
                           : "Unknown error";

    snprintf(error_msg, sizeof(error_msg),
             "ERROR #%d: %s (Count: %lu)\r\n",
             error, err_name, error_count);
    HAL_UART_Transmit(&huart3, (uint8_t*)error_msg, strlen(error_msg), 1000);

    // Blink LED pattern based on error code
    DIAG("SYS", "Blinking LED3 %d times for error code %d", (error % 5) + 1, error);
    for (int i = 0; i < (error % 5) + 1; i++) {
        HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
        HAL_Delay(200);
    }

    // Critical errors trigger emergency shutdown.
    //
    // Safety-critical range: any fault that can damage the PAs or leave the
    // system in an undefined state must cut the RF rails via Emergency_Stop().
    // This covers:
    //   ERROR_RF_PA_OVERCURRENT .. ERROR_POWER_SUPPLY (9..13) -- PA/supply faults
    //   ERROR_TEMPERATURE_HIGH  (14) -- >75 C on the PA thermal sensors;
    //                                  without cutting bias + 5V/5V5/RFPA rails
    //                                  the GaN QPA2962 stage can thermal-runaway.
    //   ERROR_WATCHDOG_TIMEOUT  (16) -- health-check loop has stalled (>60 s);
    //                                  transmitter state is unknown, safest to
    //                                  latch Emergency_Stop rather than rely on
    //                                  IWDG reset (which re-energises the rails).
    if ((error >= ERROR_RF_PA_OVERCURRENT && error <= ERROR_POWER_SUPPLY) ||
        error == ERROR_TEMPERATURE_HIGH ||
        error == ERROR_WATCHDOG_TIMEOUT) {
        DIAG_ERR("SYS", "CRITICAL ERROR (code %d: %s) -- initiating Emergency_Stop()", error, err_name);
        snprintf(error_msg, sizeof(error_msg),
                 "CRITICAL ERROR! Initiating emergency shutdown.\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)error_msg, strlen(error_msg), 1000);

        /* [GAP-3 FIX 5] Set flag BEFORE Emergency_Stop() — the function
         * never returns (infinite loop), so the line after it was dead code. */
        system_emergency_state = true;
        Emergency_Stop();
        /* NOTREACHED — Emergency_Stop() loops forever */
    }

    // For non-critical errors, attempt recovery
    if (!system_emergency_state) {
        DIAG("SYS", "Non-critical error -- attempting recovery");
        attemptErrorRecovery(error);
    }
}


// System health monitoring function
bool checkSystemHealthStatus(void) {
    SystemError_t error = checkSystemHealth();

    if (error != ERROR_NONE) {
        DIAG_ERR("SYS", "checkSystemHealthStatus: error detected (code %d), calling handleSystemError()", error);
        handleSystemError(error);

        // If we're in emergency state or too many errors, shutdown
        if (system_emergency_state || error_count > 10) {
            DIAG_ERR("SYS", "checkSystemHealthStatus returning FALSE (emergency=%s error_count=%lu)",
                     system_emergency_state ? "true" : "false", error_count);
            return false;
        }
    }

    return true;
}

// Get system status for GUI
// Get system status for GUI with 8 temperature variables
void getSystemStatusForGUI(char* status_buffer, size_t buffer_size) {
    // Build status string directly in the output buffer using offset-tracked
    // snprintf.  Each call returns the number of chars written (excluding NUL),
    // so we advance 'off' and shrink 'rem' to guarantee we never overflow.
    size_t off = 0;
    size_t rem = buffer_size;
    int w;

    // Basic status
    if (system_emergency_state) {
        w = snprintf(status_buffer + off, rem, "System Status: EMERGENCY_STOP|");
    } else {
        w = snprintf(status_buffer + off, rem, "System Status: NORMAL|");
    }
    if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }

    // Error information
    w = snprintf(status_buffer + off, rem, "LastError:%d|ErrorCount:%lu|",
                 last_error, error_count);
    if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }

    // Sensor status
    w = snprintf(status_buffer + off, rem, "IMU:%.1f,%.1f,%.1f|GPS:%.6f,%.6f|ALT:%.1f|",
                 Pitch_Sensor, Roll_Sensor, Yaw_Sensor,
                 RADAR_Latitude, RADAR_Longitude, RADAR_Altitude);
    if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }

    // LO Status
    bool tx_locked, rx_locked;
    ADF4382A_CheckLockStatus(&lo_manager, &tx_locked, &rx_locked);
    w = snprintf(status_buffer + off, rem, "LO_TX:%s|LO_RX:%s|",
                 tx_locked ? "LOCKED" : "UNLOCKED",
                 rx_locked ? "LOCKED" : "UNLOCKED");
    if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }

    // Temperature readings (8 variables)
    Temperature_1 = ADS7830_Measure_SingleEnded(&hadc3, 0);
    Temperature_2 = ADS7830_Measure_SingleEnded(&hadc3, 1);
    Temperature_3 = ADS7830_Measure_SingleEnded(&hadc3, 2);
    Temperature_4 = ADS7830_Measure_SingleEnded(&hadc3, 3);
    Temperature_5 = ADS7830_Measure_SingleEnded(&hadc3, 4);
    Temperature_6 = ADS7830_Measure_SingleEnded(&hadc3, 5);
    Temperature_7 = ADS7830_Measure_SingleEnded(&hadc3, 6);
    Temperature_8 = ADS7830_Measure_SingleEnded(&hadc3, 7);

    // Format all 8 temperature variables
    w = snprintf(status_buffer + off, rem,
                 "T1:%.1f|T2:%.1f|T3:%.1f|T4:%.1f|T5:%.1f|T6:%.1f|T7:%.1f|T8:%.1f|",
                 Temperature_1, Temperature_2, Temperature_3, Temperature_4,
                 Temperature_5, Temperature_6, Temperature_7, Temperature_8);
    if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }

    // RF Power Amplifier status (if enabled)
    if (PowerAmplifier) {
        float avg_current = 0.0f;
        for (int i = 0; i < 16; i++) {
            avg_current += Idq_reading[i];
        }
        avg_current /= 16.0f;

        w = snprintf(status_buffer + off, rem, "PA_AvgCurrent:%.2f|PA_Enabled:%d|",
                     avg_current, PowerAmplifier);
        if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }
    }

    // Radar operation status
    w = snprintf(status_buffer + off, rem, "BeamPos:%d|Azimuth:%d|ChirpCount:%d|",
                 n, y, m);
    if (w > 0 && (size_t)w < rem) { off += (size_t)w; rem -= (size_t)w; }

    // NUL termination guaranteed by snprintf, but be safe
    status_buffer[buffer_size - 1] = '\0';
}

/* ---------- UART printing helpers (Bug #8 FIXED: uncommented) ---------- */
static void uart_print(const char *msg)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

static void uart_println(const char *msg)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    const char crlf[] = "\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)crlf, 2, HAL_MAX_DELAY);
}

/* ---------- Helper delay wrappers ---------- */
static inline void delay_ms(uint32_t ms) { HAL_Delay(ms); }



// smartDelay removed -- replaced by non-blocking um982_process() in main loop

// Small helper to enable DWT cycle counter for microdelay
static void DWT_Init(void)
{
    // Enable DWT CYCCNT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Configure and program AD9523 via no-os driver
static int configure_ad9523(void)
{
    int32_t ret;
    struct ad9523_dev *dev = NULL;

    static struct ad9523_platform_data pdata;
    memset(&pdata, 0, sizeof(pdata));

    // VCXO + refs
    pdata.vcxo_freq = 100000000; // 100 MHz VCXO on OSC_IN
    pdata.refa_diff_rcv_en = 0;  // REFA 10 MHz single-ended
    pdata.refb_diff_rcv_en = 0;  // REFB 100 MHz single-ended
    pdata.osc_in_diff_en = 0;

    // PLL1: keep bypass disabled so VCXO can be used as reference cleanup if desired
    pdata.pll1_bypass_en = 0;
    pdata.refa_r_div = 1;
    pdata.refb_r_div = 1;

    // PLL2: 100 MHz PFD (R2=1), N = 36 to get VCO = 3.6 GHz
    pdata.pll2_ndiv_a_cnt = 0;
    pdata.pll2_ndiv_b_cnt = 9; // 4*9 + 0 = 36
    pdata.pll2_r2_div = 0;     // R2=1
    pdata.pll2_charge_pump_current_nA = 3500; // example

    // Loop filters: reasonable starting values from examples
    pdata.rpole2 = RPOLE2_900_OHM;
    pdata.rzero = RZERO_2000_OHM;
    pdata.cpole1 = CPOLE1_24_PF;
    pdata.rzero_bypass_en = 0;

    // Channels array (must allocate AD9523_NUM_CHAN entries)
    static struct ad9523_channel_spec channels[AD9523_NUM_CHAN];
    pdata.channels = channels;
    pdata.num_channels = AD9523_NUM_CHAN;

    // Initialize channels to disabled defaults
    for (int i=0; i<AD9523_NUM_CHAN; ++i) {
        channels[i].channel_num = i;
        channels[i].driver_mode = TRISTATE;
        channels[i].channel_divider = 0;
        channels[i].divider_phase = 0;
        channels[i].use_alt_clock_src = 0;
        channels[i].output_dis = 1;
    }

    // Map your required outputs (3.6 GHz PLL2)
    // OUT0 = ADF4382A_TX= 300 MHz LVDS ( /12 )
    channels[0].channel_divider = 12;
    channels[0].driver_mode = LVDS_7mA;
    channels[0].divider_phase = 0;
    channels[0].output_dis = 0;

    // OUT1 = ADF4382A_RX = 300 MHz LVDS (phase aligned with OUT0)
    channels[1].channel_divider = 12;
    channels[1].driver_mode = LVDS_7mA;
    channels[1].divider_phase = 0;
    channels[1].output_dis = 0;

    // OUT4 = ADC = 400 MHz LVDS ( /9 )
    channels[4].channel_divider = 9;
    channels[4].driver_mode = LVDS_7mA;
    channels[4].divider_phase = 0;
    channels[4].output_dis = 0;

    // OUT5 = FPGA_ADC = 400 MHz LVDS (phase aligned with OUT4)
    channels[5].channel_divider = 9;
    channels[5].driver_mode = LVDS_7mA;
    channels[5].divider_phase = 0;
    channels[5].output_dis = 0;

    // OUT6 = FPGA_SYSTEM_CLOCK = 100 MHz LVCMOS ( /36 )
    channels[6].channel_divider = 36;
    channels[6].driver_mode = CMOS_CONF1;
    channels[6].divider_phase = 0;
    channels[6].output_dis = 0;

    // OUT7 = FPGA_TEST_CLOCK = 20 MHz LVCMOS ( /180 )
    channels[7].channel_divider = 180;
    channels[7].driver_mode = CMOS_CONF1;
    channels[7].divider_phase = 0;
    channels[7].output_dis = 0;

    // OUT8 = SYNC_TX = 60 MHz LVDS ( /60 )
    channels[8].channel_divider = 60;
    channels[8].driver_mode = LVDS_4mA;
    channels[8].divider_phase = 0;
    channels[8].output_dis = 0;

    // OUT9 = SYNC_RX = 60 MHz LVDS (phase aligned with OUT8)
    channels[9].channel_divider = 60;
    channels[9].driver_mode = LVDS_4mA;
    channels[9].divider_phase = 0;
    channels[9].output_dis = 0;

    // OUT10 = DAC = 120 MHz LVCMOS ( /30 )
    channels[10].channel_divider = 30;
    channels[10].driver_mode = CMOS_CONF1;
    channels[10].divider_phase = 0;
    channels[10].output_dis = 0;

    // OUT11 = FPGA_DAC = 120 MHz LVCMOS (phase aligned with OUT10)
    channels[11].channel_divider = 30;
    channels[11].driver_mode = CMOS_CONF1;
    channels[11].divider_phase = 0;
    channels[11].output_dis = 0;

    // Fill ad9523 init param
    struct ad9523_init_param init_param;
    memset(&init_param, 0, sizeof(init_param));

    // SPI init (no_os type)
    init_param.spi_init.max_speed_hz = 10000000; // 10 MHz SPI
    init_param.spi_init.chip_select = 0;
    init_param.spi_init.mode = NO_OS_SPI_MODE_0;
    init_param.spi_init.platform_ops = &stm32_spi_ops;
    init_param.spi_init.extra = &hspi4; // pass HAL handle via extra


    init_param.pdata = &pdata;

    DIAG_SECTION("AD9523 CONFIGURE");
    DIAG("CLK", "VCXO=%lu Hz, PLL2 N=%d (4*%d+%d)=%d, R2=%d",
         (unsigned long)pdata.vcxo_freq,
         4 * pdata.pll2_ndiv_b_cnt + pdata.pll2_ndiv_a_cnt,
         pdata.pll2_ndiv_b_cnt, pdata.pll2_ndiv_a_cnt,
         4 * pdata.pll2_ndiv_b_cnt + pdata.pll2_ndiv_a_cnt,
         pdata.pll2_r2_div);
    DIAG("CLK", "Enabled channels: 0,1(/12=300M) 4,5(/9=400M) 6(/36=100M) 7(/180=20M) 8,9(/60=60M) 10,11(/30=120M)");

    // init ad9523 defaults (fills any missing pdata defaults)
    DIAG("CLK", "Calling ad9523_init() -- fills pdata defaults");
    {
        int32_t init_ret = ad9523_init(&init_param);
        DIAG("CLK", "ad9523_init() returned %ld", (long)init_ret);
        if (init_ret != 0) {
            DIAG_ERR("CLK", "ad9523_init() FAILED (ret=%ld)", (long)init_ret);
            return -1;
        }
    }

    /* [Bug #2 FIXED] Removed first ad9523_setup() call that was here.
     * It wrote to the chip while still in reset — writes were lost.
     * Only the post-reset setup call below is needed. */

    // Bring AD9523 out of reset
    DIAG("CLK", "Releasing AD9523 reset (AD9523_RESET_RELEASE)");
    AD9523_RESET_RELEASE();
    HAL_Delay(5);
    DIAG("CLK", "AD9523 reset released, waited 5 ms");

    // Select REFB (100MHz) using REF_SEL pin (true selects REFB here)
    DIAG("CLK", "Selecting REFB (100 MHz) via REF_SEL pin");
    AD9523_REF_SEL(true);

    // Call setup which uses no_os_spi to write registers, io_update, calibrate, sync
    DIAG("CLK", "Calling ad9523_setup() -- post-reset configuration");
    uint32_t setup_start = HAL_GetTick();
    ret = ad9523_setup(&dev, &init_param);
    DIAG("CLK", "ad9523_setup() returned %ld (took %lu ms)",
         (long)ret, (unsigned long)(HAL_GetTick() - setup_start));
    if (ret != 0) {
        // handle error: lock missing or SPI error
        DIAG_ERR("CLK", "ad9523_setup() FAILED (ret=%ld) -- lock missing or SPI error", (long)ret);
        return -1;
    }

    // Final check
    DIAG("CLK", "Checking AD9523 status (PLL lock, etc.)");
    ret = ad9523_status(dev);
    DIAG("CLK", "ad9523_status() returned %ld", (long)ret);
    if (ret != 0) {
        // log/handle
        DIAG_ERR("CLK", "AD9523 status check FAILED (ret=%ld)", (long)ret);
        return -1;
    }

    // optionally manual sync
    DIAG("CLK", "Triggering manual ad9523_sync()");
    ad9523_sync(dev);
    DIAG("CLK", "AD9523 configuration complete -- all outputs should be active");

    // keep device pointer globally if needed (dev)
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////ADAR1000//////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Alternative approach using the beam sequence feature
void setupBeamSequences(ADAR1000Manager& manager) {
    std::vector<ADAR1000Manager::BeamConfig> tx_sequence;
    std::vector<ADAR1000Manager::BeamConfig> rx_sequence;

    // Create beam configurations for each pattern
    // Matrix1 sequences (14 steps)
    for(int i = 0; i < 14; i++) {
        ADAR1000Manager::BeamConfig tx_config;
        ADAR1000Manager::BeamConfig rx_config;

        // Calculate angles or use your matrix directly
        // For now, we'll use a placeholder angle and set phases manually later
        tx_config.angle_degrees = 0; // This would be calculated from your matrix
        rx_config.angle_degrees = 0;

        // Copy phase settings from matrix1
        for(int ch = 0; ch < 16; ch++) {
            if(ch < 4) {
                tx_config.phase_settings[ch] = matrix1[i][ch];
                rx_config.phase_settings[ch] = matrix1[i][ch];
            }
        }

        tx_sequence.push_back(tx_config);
        rx_sequence.push_back(rx_config);
    }

    // You would similarly add the zero phase and matrix2 sequences

    // Note: The beam sequence feature in the manager currently uses calculated phases
    // You might need to modify the manager to support pre-calculated phase arrays
}
void setCustomBeamPattern(ADAR1000Manager& manager, uint8_t phase_pattern[16]) {
    // Set TX phases for all 4 ADAR1000 devices
    for(int dev = 0; dev < 4; dev++) {
        for(int ch = 0; ch < 4; ch++) {
            uint8_t phase = phase_pattern[dev * 4 + ch];
            manager.adarSetTxPhase(dev, ch + 1, phase, BROADCAST_OFF);
        }
    }

    // Set RX phases for all 4 ADAR1000 devices
    for(int dev = 0; dev < 4; dev++) {
        for(int ch = 0; ch < 4; ch++) {
            uint8_t phase = phase_pattern[dev * 4 + ch];
            manager.adarSetRxPhase(dev, ch + 1, phase, BROADCAST_OFF);
        }
    }
}

void initializeBeamMatricesWithSteeringAngles() {
    const float wavelength = 0.02857f; // 10.5 GHz wavelength in meters
    const float element_spacing = wavelength / 2.0f;

    // Calculate steering angles from phase differences
    float steering_angles[31];
    for(int i = 0; i < 31; i++) {
        float phase_diff_rad = phase_differences[i] * M_PI / 180.0f;
        steering_angles[i] = asin((phase_diff_rad * wavelength) / (2 * M_PI * element_spacing)) * 180.0f / M_PI;
    }

    // Matrix1: Positive angles (positions 1-15)
    for(int beam_pos = 0; beam_pos < 15; beam_pos++) {
        float angle = steering_angles[beam_pos];

        for(int element = 0; element < 16; element++) {
            // Calculate phase shift for linear array
            float phase_shift_rad = (2 * M_PI * element_spacing * element * sin(angle * M_PI / 180.0f)) / wavelength;
            float phase_degrees = phase_shift_rad * 180.0f / M_PI;

            // Wrap to 0-360 degrees
            while(phase_degrees < 0) phase_degrees += 360;
            while(phase_degrees >= 360) phase_degrees -= 360;

            matrix1[beam_pos][element] = (uint8_t)((phase_degrees / 360.0f) * 128);
        }
    }

    // Matrix2: Negative angles (positions 17-31)
    for(int beam_pos = 0; beam_pos < 15; beam_pos++) {
        float angle = steering_angles[beam_pos + 16];

        for(int element = 0; element < 16; element++) {
            float phase_shift_rad = (2 * M_PI * element_spacing * element * sin(angle * M_PI / 180.0f)) / wavelength;
            float phase_degrees = phase_shift_rad * 180.0f / M_PI;

            while(phase_degrees < 0) phase_degrees += 360;
            while(phase_degrees >= 360) phase_degrees -= 360;

            matrix2[beam_pos][element] = (uint8_t)((phase_degrees / 360.0f) * 128);
        }
    }
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();  // B15 fix: init DELADJ PWM timer before LO manager uses it
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_IWDG_Init();  /* GAP-3 FIX 2: start hardware watchdog (~4 s timeout) */
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);

  /* --- Enable DWT cycle counter for accurate microsecond delays --- */
  DWT_Init();

  DIAG_SECTION("SYSTEM INIT");
  DIAG("SYS", "DWT cycle counter initialized, TIM1 started");
  DIAG("SYS", "HAL tick at init start: %lu ms", (unsigned long)HAL_GetTick());

  //Wait for OCXO 3mn
  DIAG("CLK", "OCXO warmup starting -- waiting 180 s (3 min)");
  uint32_t ocxo_start = HAL_GetTick();
  /* [GAP-3 FIX 2] Cannot use HAL_Delay(180000) — IWDG would reset MCU.
   * Instead loop in 1-second steps, kicking the watchdog each iteration. */
  for (int ocxo_sec = 0; ocxo_sec < 180; ocxo_sec++) {
      HAL_IWDG_Refresh(&hiwdg);
      HAL_Delay(1000);
  }
  DIAG_ELAPSED("CLK", "OCXO warmup", ocxo_start);

  DIAG_SECTION("AD9523 POWER SEQUENCING");
  DIAG("PWR", "Asserting AD9523 reset (pin LOW)");
  HAL_GPIO_WritePin(AD9523_RESET_GPIO_Port,AD9523_RESET_Pin,GPIO_PIN_RESET);

  //Power sequencing AD9523
  DIAG("PWR", "Enabling 1.8V clock rail");
  HAL_GPIO_WritePin(EN_P_1V8_CLOCK_GPIO_Port,EN_P_1V8_CLOCK_Pin,GPIO_PIN_SET);
  HAL_Delay(100);
  DIAG("PWR", "Enabling 3.3V clock rail");
  HAL_GPIO_WritePin(EN_P_3V3_CLOCK_GPIO_Port,EN_P_3V3_CLOCK_Pin,GPIO_PIN_SET);
  HAL_Delay(100);
  DIAG("PWR", "Releasing AD9523 reset (pin HIGH)");
  HAL_GPIO_WritePin(AD9523_RESET_GPIO_Port,AD9523_RESET_Pin,GPIO_PIN_SET);
  HAL_Delay(100);
  DIAG("PWR", "AD9523 power sequencing complete -- all rails up, reset released");

  //Set planned Clocks on AD9523
  /*
  o Channel 0 = 300MHz LVDS (ADF4382 TX)
  o Channel 1 = 300MHz LVDS (ADF4382 RX) (Phase aligned with Channel 0)
  o Channel 4 = 400MHz LVDS (ADC)
  o Channel 5 = 400MHz LVDS (FPGA_ADC)(Phase aligned with Channel 4)
  o Channel 6 = 100MHz LVCMOS (FPGA system clock)
  o Channel 7 = 20MHz LVCMOS (FPGA test)
  o Channel 8 = 60MHz LVDS (ADF4382 TX sync)
  o Channel 9 = 60MHz LVDS (ADF4382 RX sync)(Phase aligned with Channel 8)
  o Channel 10 = 120MHz LVCMOS
  o Channel 11 = 120MHz LVCMOS (Phase aligned with Channel 10)
   */


  // Ensure stm32_spi.c and stm32_delay.c are compiled and linked

  DIAG("CLK", "Calling configure_ad9523()...");
  uint32_t ad9523_cfg_start = HAL_GetTick();
  if (configure_ad9523() != 0) {
      // configuration error - handle as needed
      DIAG_ERR("CLK", "configure_ad9523() FAILED -- entering infinite halt loop");
      while (1) { HAL_Delay(1000); }
  }
  DIAG_ELAPSED("CLK", "configure_ad9523()", ad9523_cfg_start);
  DIAG("CLK", "AD9523 configured successfully");

  //Power sequencing FPGA
  DIAG_SECTION("FPGA POWER SEQUENCING");
  DIAG("PWR", "Enabling 1.0V FPGA rail");
  HAL_GPIO_WritePin(EN_P_1V0_FPGA_GPIO_Port,EN_P_1V0_FPGA_Pin,GPIO_PIN_SET);
  HAL_Delay(100);
  DIAG("PWR", "Enabling 1.8V FPGA rail");
  HAL_GPIO_WritePin(EN_P_1V8_FPGA_GPIO_Port,EN_P_1V8_FPGA_Pin,GPIO_PIN_SET);
  HAL_Delay(100);
  DIAG("PWR", "Enabling 3.3V FPGA rail");
  HAL_GPIO_WritePin(EN_P_3V3_FPGA_GPIO_Port,EN_P_3V3_FPGA_Pin,GPIO_PIN_SET);
  HAL_Delay(100);
  DIAG("PWR", "FPGA power sequencing complete -- 1.0V -> 1.8V -> 3.3V");


// Initialize module IMU
  DIAG_SECTION("IMU INIT (GY-85)");
  DIAG("IMU", "Initializing GY-85 IMU...");
  if (!GY85_Init()) {
      DIAG_ERR("IMU", "GY85_Init() FAILED -- calling Error_Handler()");
      Error_Handler();
  }
  DIAG("IMU", "GY-85 initialized OK, running 10 calibration samples");
  for(int i=0; i<10;i++){
  if (!GY85_Update(&imu)) {
      Error_Handler();
  }

  ax = imu.ax;
  ay = imu.ay;
  az = imu.az;
  gx = -imu.gx;
  gy = -imu.gy;
  gz = imu.gz;
  mx = imu.mx;
  my = imu.my;
  mz = imu.mz;

  ax *= 6.10351e-5;   //2.0 / 32768.0;
  ay *= 6.10351e-5;
  az *= 6.10351e-5;

  ax -= abias[0];   // Convert to g's, remove accelerometer biases
  ay -= abias[1];
  az -= abias[2];

  gx *= 0.0152588f;   //500.0 / 32768.0;
  gy *= 0.0152588f;
  gz *= 0.0152588f;

  gx -= gbias[0];   // Convert to degrees per seconds, remove gyro biases
  gy -= gbias[1];
  gz -= gbias[2];

  mx *= 6.10351e-5;   //2.0 / 32768.0;
  my *= 6.10351e-5;
  mz *= 6.10351e-5;

  mxc = M11*(mx-mbias[0]) + M12*(my-mbias[1]) +M13*(mz-mbias[2]);
  myc = M21*(mx-mbias[0]) + M22*(my-mbias[1]) +M23*(mz-mbias[2]);
  mzc = M31*(mx-mbias[0]) + M32*(my-mbias[1]) +M33*(mz-mbias[2]);

  mx = mxc;
  my = myc;
  mz = mzc;

  float norm = sqrt((mx*mx) + (my*my) + (mz*mz));
  mx /=norm;
  my /=norm;
  mz /=norm;

  /***************************************************************/
  /*********************IMU Complementary Filter******************/
  /***************************************************************/
    RxAcc = ax;
    RyAcc = ay;
    RzAcc = az;

    Rate_Ayz_1 = gx;
    Rate_Axz_1 = gy;
    Rate_Axy_1 = gz;

    Rate_Ayz_1 = Rate_Ayz_1*0.01745;//Convertion from °/s to Radians/s
    Rate_Axz_1 = Rate_Axz_1*0.01745;
    Rate_Axy_1 = Rate_Axy_1*0.01745;

    /*Combining Accelerometer & Gyroscope Data*/
    Axz_0 = atan2(RxEst_0,RzEst_0);
    Ayz_0 = atan2(RyEst_0,RzEst_0);
    Axy_0 = atan2(RxEst_0,RyEst_0);

    now_timeperiod = micros();

    Time_Period = now_timeperiod - lasttime_timeperiod;

    Axz_1= Axz_0 + Rate_Axz_1*Time_Period*0.000001;
    Ayz_1= Ayz_0 + Rate_Ayz_1*Time_Period*0.000001;
    Axy_1= Axy_0 + Rate_Axy_1*Time_Period*0.000001;

    lasttime_timeperiod = now_timeperiod;

    RxGyro = sin(Axz_1)/(sqrt(1.0 + cos(Axz_1)*cos(Axz_1) * tan(Ayz_1)*tan(Ayz_1)));
    RyGyro = sin(Ayz_1)/(sqrt(1.0 + cos(Ayz_1)*cos(Ayz_1) * tan(Axz_1)*tan(Axz_1)));
    if(RzEst_0 >= 0){
      RzGyro = 1.0*sqrt(1 - RxGyro*RxGyro - RyGyro*RyGyro);
    }
    else {
      RzGyro = -1*sqrt(1 - RxGyro*RxGyro - RyGyro*RyGyro);
    }

    RxEst_1 = RxAcc*0.5 + RxGyro*0.5 ;// Weight of Gyro. over Acc.
    RyEst_1 = RyAcc*0.5 + RyGyro*0.5 ;
    RzEst_1 = RzAcc*0.5 + RzGyro*0.5 ;

    R_Est = sqrt(RxEst_1*RxEst_1+RyEst_1*RyEst_1+RzEst_1*RzEst_1);

    Pitch_Sensor = atan2(RxEst_1,sqrt(RyEst_1*RyEst_1)+(RzEst_1*RzEst_1))*180/PI;
    Roll_Sensor = atan2(RyEst_1,sqrt(RxEst_1*RxEst_1)+(RzEst_1*RzEst_1))*180/PI;

    float magRawX = mx*cos(Pitch_Sensor*PI/180.0f)  - mz*sin(Pitch_Sensor*PI/180.0f);
	float magRawY = mx*sin(Roll_Sensor*PI/180.0f)*sin(Pitch_Sensor*PI/180.0f) + my*cos(Roll_Sensor*PI/180.0f)- mz*sin(Roll_Sensor*PI/180.0f)*cos(Pitch_Sensor*PI/180.0f);
    Yaw_Sensor = (180*atan2(magRawY,magRawX)/PI) - Mag_Declination;

    if(Yaw_Sensor<0)Yaw_Sensor+=360;

    // Override magnetometer heading with UM982 dual-antenna heading when available
    if (um982_is_heading_valid(&um982)) {
        Yaw_Sensor = um982_get_heading(&um982);
    }

    RxEst_0 = RxEst_1;
    RyEst_0 = RyEst_1;
    RzEst_0 = RzEst_1;

    HAL_Delay(300);

  }

    ///////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////BAROMETER BMP180////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    DIAG_SECTION("BAROMETER INIT (BMP180)");
    DIAG("IMU", "Reading 5 barometer samples for altitude baseline");
  	for(int i = 0; i<5 ; i++){
		float BMP_Perssure = myBMP.getPressure();
		RADAR_Altitude = 44330*(1-(pow((BMP_Perssure/101325),(1/5.255))));
		HAL_Delay(100);
  	}
    DIAG("IMU", "Barometer init complete, RADAR_Altitude baseline set");

    ///////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////ADF4382/////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    DIAG_SECTION("ADF4382A LO INIT");
    printf("Starting ADF4382A Radar LO System...\n");
    printf("Using SPI4 with TIMED SYNCHRONIZATION (60 MHz clocks)\n");

    // Initialize LO manager with TIMED synchronization
    DIAG("LO", "Calling ADF4382A_Manager_Init(sync=SYNC_METHOD_TIMED)");
    uint32_t lo_init_start = HAL_GetTick();
    int ret = ADF4382A_Manager_Init(&lo_manager, SYNC_METHOD_TIMED);
    DIAG("LO", "ADF4382A_Manager_Init returned %d (took %lu ms)",
         ret, (unsigned long)(HAL_GetTick() - lo_init_start));

    /* [Bug #4 FIXED] Check init return code BEFORE calling phase shift.
     * Previously SetPhaseShift/Strobe were called before this check. */
    if (ret != ADF4382A_MANAGER_OK) {
        printf("LO Manager initialization failed: %d\n", ret);
        DIAG_ERR("LO", "Manager init FAILED (ret=%d) -- calling Error_Handler()", ret);
        Error_Handler();
    }

    // Set phase shift (e.g., 500 ps for TX, 500 ps for RX)
    int ps_ret = ADF4382A_SetPhaseShift(&lo_manager, 500, 500);
    DIAG("LO", "ADF4382A_SetPhaseShift(500, 500) returned %d", ps_ret);

    // Strobe to apply phase shifts
    int strobe_tx_ret = ADF4382A_StrobePhaseShift(&lo_manager, 0); // TX device
    int strobe_rx_ret = ADF4382A_StrobePhaseShift(&lo_manager, 1); // RX device
    DIAG("LO", "StrobePhaseShift TX returned %d, RX returned %d", strobe_tx_ret, strobe_rx_ret);

    // Check initial lock status
    bool tx_locked, rx_locked;
    DIAG("LO", "Checking initial lock status...");
    ret = ADF4382A_CheckLockStatus(&lo_manager, &tx_locked, &rx_locked);
    if (ret == ADF4382A_MANAGER_OK) {
        printf("Initial Lock Status - TX: %s, RX: %s\n",
               tx_locked ? "LOCKED" : "UNLOCKED",
               rx_locked ? "LOCKED" : "UNLOCKED");
        DIAG("LO", "Initial lock: TX=%s RX=%s", tx_locked ? "LOCKED" : "UNLOCKED", rx_locked ? "LOCKED" : "UNLOCKED");
    } else {
        DIAG_ERR("LO", "CheckLockStatus returned %d", ret);
    }
    // Wait for both LOs to lock
        DIAG("LO", "Entering lock-wait loop (max 10 s = 100 x 100 ms)");
        uint32_t lock_wait_start = HAL_GetTick();
        uint32_t lock_timeout = 0;
        while (!(tx_locked && rx_locked) && lock_timeout < 100) {
            HAL_Delay(100);
            ADF4382A_CheckLockStatus(&lo_manager, &tx_locked, &rx_locked);
            lock_timeout++;

            if (lock_timeout % 10 == 0) {
                printf("Waiting for LO lock... TX: %s, RX: %s\n",
                       tx_locked ? "LOCKED" : "UNLOCKED",
                       rx_locked ? "LOCKED" : "UNLOCKED");
                DIAG("LO", "Lock poll #%lu: TX=%s RX=%s",
                     (unsigned long)lock_timeout,
                     tx_locked ? "LOCKED" : "UNLOCKED",
                     rx_locked ? "LOCKED" : "UNLOCKED");
            }
        }
        DIAG_ELAPSED("LO", "Lock wait loop", lock_wait_start);

        if (tx_locked && rx_locked) {
            printf("Both LOs locked successfully!\n");
            printf("Synchronization ready - 60 MHz clocks on SYNCP/SYNCN pins\n");
            DIAG("LO", "Both LOs LOCKED after %lu iterations", (unsigned long)lock_timeout);
            // - 60 MHz phase-aligned clocks on SYNCP/SYNCN pins
            // - SYNC pin connected for triggering

            // When ready to synchronize:
            DIAG("LO", "Calling TriggerTimedSync to pulse sw_sync on both PLLs");
            ADF4382A_TriggerTimedSync(&lo_manager);
            // At this point, the SYNC pin can be toggled to trigger synchronization
        } else {
            printf("LO lock timeout! TX: %s, RX: %s\n",
                   tx_locked ? "LOCKED" : "UNLOCKED",
                   rx_locked ? "LOCKED" : "UNLOCKED");
            DIAG_ERR("LO", "Lock TIMEOUT after %lu iterations! TX=%s RX=%s",
                     (unsigned long)lock_timeout,
                     tx_locked ? "LOCKED" : "UNLOCKED",
                     rx_locked ? "LOCKED" : "UNLOCKED");
        }

  // check if there is a lock via direct GPIO (independent of register read above)
  DIAG("LO", "Direct GPIO lock detect pins:");
  DIAG_GPIO("LO", "ADF4382_TX_LKDET", ADF4382_TX_LKDET_GPIO_Port, ADF4382_TX_LKDET_Pin);
  DIAG_GPIO("LO", "ADF4382_RX_LKDET", ADF4382_RX_LKDET_GPIO_Port, ADF4382_RX_LKDET_Pin);
  if(HAL_GPIO_ReadPin (ADF4382_TX_LKDET_GPIO_Port, ADF4382_TX_LKDET_Pin))
  {
      // Set The LED_1 ON!
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
      DIAG("LO", "TX lock detected via GPIO -- LED_1 ON");
  }
  else
  {
      // Else .. Turn LED_1 OFF!
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
      DIAG_WARN("LO", "TX NOT locked via GPIO -- LED_1 OFF");
  }

  if(HAL_GPIO_ReadPin (ADF4382_RX_LKDET_GPIO_Port, ADF4382_RX_LKDET_Pin))
  {
      // Set The LED_2 ON!
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
      DIAG("LO", "RX lock detected via GPIO -- LED_2 ON");
  }
  else
  {
      // Else .. Turn LED_2 OFF!
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
      DIAG_WARN("LO", "RX NOT locked via GPIO -- LED_2 OFF");
  }


  //////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////ADAR1000/////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////

  //Power sequencing ADAR1000
  DIAG_SECTION("ADAR1000 POWER SEQUENCING");

  //Tell FPGA to turn off TX RF signal by disabling Mixers
  DIAG("BF", "Disabling TX mixers (GPIOD pin 11 LOW)");
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);

  DIAG("PWR", "Enabling 3.3V ADAR12 + ADAR34 rails");
  HAL_GPIO_WritePin(EN_P_3V3_ADAR12_GPIO_Port,EN_P_3V3_ADAR12_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_P_3V3_ADAR34_GPIO_Port,EN_P_3V3_ADAR34_Pin,GPIO_PIN_SET);
  HAL_Delay(500);
  DIAG("PWR", "Enabling 5.0V ADAR rail");
  HAL_GPIO_WritePin(EN_P_5V0_ADAR_GPIO_Port,EN_P_5V0_ADAR_Pin,GPIO_PIN_SET);
  HAL_Delay(500);
  DIAG("PWR", "ADAR1000 power sequencing complete");

  // System startup message
      uint8_t startup_msg[] = "Starting Phased Array RADAR System...\r\n";
      HAL_UART_Transmit(&huart3, startup_msg, sizeof(startup_msg)-1, 1000);

      DIAG("BF", "Calling systemPowerUpSequence()");
      // Power up sequence
      systemPowerUpSequence();

      DIAG("BF", "Calling initializeBeamMatrices()");
      // Initialize beam matrices
      initializeBeamMatrices();

      // Print system status
      printSystemStatus();
      DIAG("SYS", "System init complete -- entering main loop");

  	//////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////GPS/////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////
  DIAG_SECTION("GPS INIT (UM982)");
  DIAG("GPS", "Initializing UM982 on UART5 @ 115200 (baseline=50cm, tol=3cm)");
  if (!um982_init(&um982, &huart5, 50.0f, 3.0f)) {
      DIAG_WARN("GPS", "UM982 init: no VERSIONA response -- module may need more time");
      // Not fatal: module may still start sending NMEA data after boot
  } else {
      DIAG("GPS", "UM982 init OK -- VERSIONA received");
  }

  // Collect GPS data for a few seconds (non-blocking pump)
  DIAG("GPS", "Pumping GPS for 5 seconds to acquire initial fix...");
  {
      uint32_t gps_start = HAL_GetTick();
      while (HAL_GetTick() - gps_start < 5000) {
          um982_process(&um982);
          HAL_Delay(10);
      }
  }
  RADAR_Longitude = um982_get_longitude(&um982);
  RADAR_Latitude = um982_get_latitude(&um982);
  DIAG("GPS", "Initial position: lat=%.6f lon=%.6f fix=%d sats=%d",
       RADAR_Latitude, RADAR_Longitude,
       um982_get_fix_quality(&um982), um982_get_num_sats(&um982));

  // Re-apply heading after GPS init so the north-alignment stepper move uses
  // UM982 dual-antenna heading when available.
  if (um982_is_heading_valid(&um982)) {
      Yaw_Sensor = um982_get_heading(&um982);
  }

  //move Stepper to position 1 = 0°
  HAL_GPIO_WritePin(STEPPER_CW_P_GPIO_Port, STEPPER_CW_P_Pin, GPIO_PIN_RESET);//Set stepper motor spinning direction to CCW
  //Point Stepper to North
  for(int i= 0;i<(int)(Yaw_Sensor*Stepper_steps/360);i++){
	  HAL_GPIO_WritePin(STEPPER_CLK_P_GPIO_Port, STEPPER_CLK_P_Pin, GPIO_PIN_SET);
	  delay_us(500);
	  HAL_GPIO_WritePin(STEPPER_CLK_P_GPIO_Port, STEPPER_CLK_P_Pin, GPIO_PIN_RESET);
	  delay_us(500);
  }

  /***************************************************************/
  /**********wait for GUI start flag and Send Lat/Long/alt********/
  /***************************************************************/

  GPS_Data_t gps_data;
  // Binary packet structure:
  // [Header 4 bytes][Latitude 8 bytes][Longitude 8 bytes][Altitude 4 bytes][Pitch 4 bytes][CRC 2 bytes]
  gps_data = {RADAR_Latitude, RADAR_Longitude, RADAR_Altitude, Pitch_Sensor, HAL_GetTick()};
  if (!GPS_SendBinaryToGUI(&gps_data)) {
      const uint8_t gps_send_error[] = "GPS binary send failed\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)gps_send_error, sizeof(gps_send_error) - 1, 1000);
  }

  /* [STM32-006 FIXED] Removed blocking do-while loop that waited for
   * usbHandler.isStartFlagReceived().  The production V7 PyQt GUI does not
   * send the legacy 4-byte start flag [23,46,158,237], so this loop hung
   * the MCU at boot indefinitely.  The USB settings handshake (if ever
   * re-enabled) should be handled non-blocking in the main loop. */

  /***************************************************************/
  /************RF Power Amplifier Powering up sequence************/
  /***************************************************************/
  if(PowerAmplifier){
	  DIAG_SECTION("PA: RF Power Amplifier power-up sequence");
	  /* Initialize DACs */
	  /* DAC1: Address 0b1001000 = 0x48 */
	  DIAG("PA", "Initializing DAC1 (I2C1, addr=0x48, 8-bit)");
	  if (!DAC5578_Init(&hdac1, &hi2c1, 0x48, 8,
			  DAC_1_VG_LDAC_GPIO_Port, DAC_1_VG_LDAC_Pin,
			  DAC_1_VG_CLR_GPIO_Port, DAC_1_VG_CLR_Pin)) {
		  DIAG_ERR("PA", "DAC1 init FAILED -- calling Error_Handler()");
		  Error_Handler();
	  }
	  DIAG("PA", "DAC1 init OK");

	  /* DAC2: Address 0b1001001 = 0x49 */
	  DIAG("PA", "Initializing DAC2 (I2C1, addr=0x49, 8-bit)");
	  if (!DAC5578_Init(&hdac2, &hi2c1, 0x49, 8,
			  DAC_2_VG_LDAC_GPIO_Port, DAC_2_VG_LDAC_Pin,
			  DAC_2_VG_CLR_GPIO_Port, DAC_2_VG_CLR_Pin)) {
		  DIAG_ERR("PA", "DAC2 init FAILED -- calling Error_Handler()");
		  Error_Handler();
	  }
	  DIAG("PA", "DAC2 init OK");

	  /* Configure clear code behavior */
	  DIAG("PA", "Setting clear code to ZERO on both DACs");
	  DAC5578_SetClearCode(&hdac1, DAC5578_CLR_CODE_ZERO); // Clear to 0V on CLR pulse
	  DAC5578_SetClearCode(&hdac2, DAC5578_CLR_CODE_ZERO); // Clear to 0V on CLR pulse

	  /* Configure LDAC so all channels update simultaneously on hardware LDAC */
	  DIAG("PA", "Setting LDAC mask=0xFF on both DACs (all channels respond)");
	  DAC5578_SetupLDAC(&hdac1, 0xFF); // All channels respond to LDAC
	  DAC5578_SetupLDAC(&hdac2, 0xFF); // All channels respond to LDAC

	  //set Vg [1-8] to -3.98V -> input opamp = 1.63058V->126(8bits)
	  DIAG("PA", "Writing initial DAC_val=%d to DAC1 channels 0-7", DAC_val);
	  for(int channel = 0; channel < 8; channel++){
		  DAC5578_WriteAndUpdateChannelValue(&hdac1, channel, DAC_val);
	  }

	  //set Vg [9-16] to -3.98V -> input opamp = 1.63058V->126(8bits)
	  DIAG("PA", "Writing initial DAC_val=%d to DAC2 channels 0-7", DAC_val);
	  for(int channel = 0; channel < 8; channel++){
		  DAC5578_WriteAndUpdateChannelValue(&hdac2, channel, DAC_val);
	  }

	  /* Optional: Use hardware LDAC for simultaneous update of all channels */
	  DIAG("PA", "Pulsing LDAC on both DACs for simultaneous update");
	  HAL_GPIO_WritePin(DAC_1_VG_LDAC_GPIO_Port, DAC_1_VG_LDAC_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(DAC_2_VG_LDAC_GPIO_Port, DAC_2_VG_LDAC_Pin, GPIO_PIN_RESET);
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(DAC_1_VG_LDAC_GPIO_Port, DAC_1_VG_LDAC_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(DAC_2_VG_LDAC_GPIO_Port, DAC_2_VG_LDAC_Pin, GPIO_PIN_SET);

	  //Enable RF Power Amplifier VDD = 22V
	  DIAG("PA", "Enabling RFPA VDD=22V (EN_DIS_RFPA_VDD HIGH)");
	  HAL_GPIO_WritePin(EN_DIS_RFPA_VDD_GPIO_Port, EN_DIS_RFPA_VDD_Pin, GPIO_PIN_SET);

	  /* Initialize ADCs with correct addresses */
	  /* ADC1: Address 0x48, Single-Ended mode, Internal Ref ON + ADC ON */
	  DIAG("PA", "Initializing ADC1 (I2C2, addr=0x48, single-ended, ref+adc ON)");
	  if (!ADS7830_Init(&hadc1, &hi2c2, 0x48,
						ADS7830_SDMODE_SINGLE, ADS7830_PDIRON_ADON)) {
		  DIAG_ERR("PA", "ADC1 init FAILED -- calling Error_Handler()");
		  Error_Handler();
	  }
	  DIAG("PA", "ADC1 init OK");

	  /* ADC2: Address 0x4A, Single-Ended mode, Internal Ref ON + ADC ON */
	  DIAG("PA", "Initializing ADC2 (I2C2, addr=0x4A, single-ended, ref+adc ON)");
	  if (!ADS7830_Init(&hadc2, &hi2c2, 0x4A,
						ADS7830_SDMODE_SINGLE, ADS7830_PDIRON_ADON)) {
		  DIAG_ERR("PA", "ADC2 init FAILED -- calling Error_Handler()");
		  Error_Handler();
	  }
	  DIAG("PA", "ADC2 init OK");

	  /* Read all 8 channels from ADC1 and calculate Idq */
	  DIAG("PA", "Reading initial Idq from ADC1 channels 0-7");
	  for (uint8_t channel = 0; channel < 8; channel++) {
		  adc1_readings[channel] = ADS7830_Measure_SingleEnded(&hadc1, channel);
		  Idq_reading[channel]= (3.3/255)*adc1_readings[channel]/(50*0.005);//Idq=Vadc/(GxRshunt)//G_INA241A3=50;Rshunt=5mOhms
		  DIAG("PA", "  ADC1 ch%d: raw=%d Idq=%.3fA", channel, adc1_readings[channel], Idq_reading[channel]);
	  }

	  /* Read all 8 channels from ADC2 and calculate Idq*/
	  DIAG("PA", "Reading initial Idq from ADC2 channels 0-7");
	  for (uint8_t channel = 0; channel < 8; channel++) {
		  adc2_readings[channel] = ADS7830_Measure_SingleEnded(&hadc2, channel);
		  Idq_reading[channel+8]= (3.3/255)*adc2_readings[channel]/(50*0.005);//Idq=Vadc/(GxRshunt)//G_INA241A3=50;Rshunt=5mOhms
		  DIAG("PA", "  ADC2 ch%d: raw=%d Idq=%.3fA", channel, adc2_readings[channel], Idq_reading[channel+8]);
	  }

	  DIAG("PA", "Starting Idq calibration loop for DAC1 channels 0-7 (target=1.680A)");
	  for (uint8_t channel = 0; channel < 8; channel++){
	      uint8_t safety_counter = 0;
	      DAC_val = 126; // Reset for each channel

	      do {
	          if (safety_counter++ > 50) { // Prevent infinite loop
	              DIAG_WARN("PA", "  DAC1 ch%d: safety limit reached (50 iterations), DAC_val=%d Idq=%.3fA",
	                        channel, DAC_val, Idq_reading[channel]);
	              break;
	          }
	          DAC_val = DAC_val - 4;
	          DAC5578_WriteAndUpdateChannelValue(&hdac1, channel, DAC_val);
	          adc1_readings[channel] = ADS7830_Measure_SingleEnded(&hadc1, channel);
	          Idq_reading[channel] = (3.3/255) * adc1_readings[channel] / (50 * 0.005);
	      } while (DAC_val > 38 && abs(Idq_reading[channel] - 1.680) > 0.2); // B12 fix: loop while FAR from target
	      DIAG("PA", "  DAC1 ch%d calibrated: DAC_val=%d Idq=%.3fA iters=%d",
	           channel, DAC_val, Idq_reading[channel], safety_counter);
	  }

	  DIAG("PA", "Starting Idq calibration loop for DAC2 channels 0-7 (target=1.680A)");
	  for (uint8_t channel = 0; channel < 8; channel++){
	      uint8_t safety_counter = 0;
	      DAC_val = 126; // Reset for each channel

	      do {
	          if (safety_counter++ > 50) { // Prevent infinite loop
	              DIAG_WARN("PA", "  DAC2 ch%d: safety limit reached (50 iterations), DAC_val=%d Idq=%.3fA",
	                        channel, DAC_val, Idq_reading[channel+8]);
	              break;
	          }
	          DAC_val = DAC_val - 4;
	          DAC5578_WriteAndUpdateChannelValue(&hdac2, channel, DAC_val);
	          adc2_readings[channel] = ADS7830_Measure_SingleEnded(&hadc2, channel); // B13 fix: was adc1_readings
	          Idq_reading[channel+8] = (3.3/255) * adc2_readings[channel] / (50 * 0.005);
	      } while (DAC_val > 38 && abs(Idq_reading[channel+8] - 1.680) > 0.2); // B12 fix: loop while FAR from target
	      DIAG("PA", "  DAC2 ch%d calibrated: DAC_val=%d Idq=%.3fA iters=%d",
	           channel, DAC_val, Idq_reading[channel+8], safety_counter);
	  }
	  DIAG("PA", "PA IDQ calibration sequence COMPLETE");
  }

  //RESET FPGA
  DIAG("FPGA", "Resetting FPGA (GPIOD pin 12: LOW -> 10ms -> HIGH)");
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  DIAG("FPGA", "FPGA reset complete");



  //Tell FPGA to apply TX RF by enabling Mixers
  DIAG("FPGA", "Enabling TX mixers (GPIOD pin 11 HIGH)");
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

  /* T°C sensor TMP37 ADC3: Address 0x49, Single-Ended mode, Internal Ref ON + ADC ON */
  DIAG("SYS", "Initializing temperature sensor ADC3 (I2C2, addr=0x49, TMP37)");
  if (!ADS7830_Init(&hadc3, &hi2c2, 0x49,
					ADS7830_SDMODE_SINGLE, ADS7830_PDIRON_ADON)) {
	  DIAG_ERR("SYS", "Temperature sensor ADC3 init FAILED -- calling Error_Handler()");
	  Error_Handler();
  }
  DIAG("SYS", "Temperature sensor ADC3 init OK");

  /***************************************************************/
  /************************ERRORS HANDLERS************************/
  /***************************************************************/

  // Initialize error handler system
  DIAG("SYS", "Initializing error handler: clearing error_count, last_error, emergency_state");
  error_count = 0;
  last_error = ERROR_NONE;
  system_emergency_state = false;

  /***************************************************************/
  /*********************SEND TO GUI STATUS************************/
  /***************************************************************/

  // Send initial status to GUI
  DIAG("USB", "Sending initial system status to GUI via USB CDC");
  char initial_status[500];
  getSystemStatusForGUI(initial_status, sizeof(initial_status));
  // Send via USB to GUI
  CDC_Transmit_FS((uint8_t*)initial_status, strlen(initial_status));
  DIAG("USB", "Initial status sent (%d bytes)", (int)strlen(initial_status));

  DIAG("SYS", "=== INIT COMPLETE -- entering main loop ===");






  // main loop

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //////////////////////////////////////////////////////////////////////////////////////
	  //////////////////////// Check system health at the start of each loop////////////////
	  //////////////////////////////////////////////////////////////////////////////////////

	    if (!checkSystemHealthStatus()) {
	        // System is in bad state, enter safe mode
	    	//Tell FPGA to turn off TX RF signal by disabling Mixers
	    	DIAG_ERR("SYS", "checkSystemHealthStatus() FAILED -- entering SAFE MODE");
	    	DIAG("SYS", "Disabling TX mixers (GPIOD pin 11 LOW)");
	    	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
	    	DIAG("SYS", "Calling systemPowerDownSequence()");
	        systemPowerDownSequence();

	        char emergency_msg[] = "SYSTEM IN SAFE MODE - Manual intervention required\r\n";
	        HAL_UART_Transmit(&huart3, (uint8_t*)emergency_msg, strlen(emergency_msg), 1000);
	        DIAG_ERR("SYS", "SAFE MODE ACTIVE -- blinking all LEDs, waiting for system_emergency_state clear");

	        // Blink all LEDs to indicate safe mode (500ms period, visible to operator)
	        while (system_emergency_state) {
	            HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	            HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
	            HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
	            HAL_GPIO_TogglePin(LED_4_GPIO_Port, LED_4_Pin);
	            HAL_Delay(250);
	            if (usbHandler.isFaultAckReceived()) {
	                system_emergency_state = false;
	                usbHandler.clearFaultAck();
	            }
	        }
	        DIAG("SYS", "Exited safe mode blink loop -- system_emergency_state cleared");
	    }

	  //////////////////////////////////////////////////////////////////////////////////////
	  ////////////////////////// GPS: Non-blocking NMEA processing ////////////////////////
	  //////////////////////////////////////////////////////////////////////////////////////
	  um982_process(&um982);

	  // Update position globals continuously
	  if (um982_is_position_valid(&um982)) {
	      RADAR_Latitude = um982_get_latitude(&um982);
	      RADAR_Longitude = um982_get_longitude(&um982);
	  }

	  //////////////////////////////////////////////////////////////////////////////////////
	  ////////////////////////// Monitor ADF4382A lock status periodically//////////////////
	  //////////////////////////////////////////////////////////////////////////////////////

      // Monitor lock status periodically
      static uint32_t last_check = 0;
      if (HAL_GetTick() - last_check > 5000) {
          ADF4382A_CheckLockStatus(&lo_manager, &tx_locked, &rx_locked);

          if (!tx_locked || !rx_locked) {
              printf("LO Lock Lost! TX: %s, RX: %s\n",
                     tx_locked ? "LOCKED" : "UNLOCKED",
                     rx_locked ? "LOCKED" : "UNLOCKED");
              DIAG_ERR("LO", "Lock LOST in main loop! TX=%s RX=%s",
                       tx_locked ? "LOCKED" : "UNLOCKED",
                       rx_locked ? "LOCKED" : "UNLOCKED");
              DIAG_GPIO("LO", "TX_LKDET (direct)", ADF4382_TX_LKDET_GPIO_Port, ADF4382_TX_LKDET_Pin);
              DIAG_GPIO("LO", "RX_LKDET (direct)", ADF4382_RX_LKDET_GPIO_Port, ADF4382_RX_LKDET_Pin);
          } else {
              DIAG("LO", "Lock poll OK: TX=LOCKED RX=LOCKED");
          }

          last_check = HAL_GetTick();
      }
	  //////////////////////////////////////////////////////////////////////////////////////
	  ////////////////////////// Monitor Temperature Sensors periodically//////////////////
	  //////////////////////////////////////////////////////////////////////////////////////
	  /* Read all 8 channels from ADC2 and calculate Idq*/
      // Monitor T°C periodically
      static uint32_t last_check1 = 0;
		  if (HAL_GetTick() - last_check1 > 5000) {
		  //TMP37 3.3V-->165°C & ADS7830 3.3V-->255 => Temperature_n °C = ADC_val_n * 165/255
		  Temperature_1 = ADS7830_Measure_SingleEnded(&hadc3, 0)*0.64705f;
		  Temperature_2 = ADS7830_Measure_SingleEnded(&hadc3, 1)*0.64705f;
		  Temperature_3 = ADS7830_Measure_SingleEnded(&hadc3, 2)*0.64705f;
		  Temperature_4 = ADS7830_Measure_SingleEnded(&hadc3, 3)*0.64705f;
		  Temperature_5 = ADS7830_Measure_SingleEnded(&hadc3, 4)*0.64705f;
		  Temperature_6 = ADS7830_Measure_SingleEnded(&hadc3, 5)*0.64705f;
		  Temperature_7 = ADS7830_Measure_SingleEnded(&hadc3, 6)*0.64705f;
		  Temperature_8 = ADS7830_Measure_SingleEnded(&hadc3, 7)*0.64705f;

		  DIAG("PA", "Temps: T1=%.1f T2=%.1f T3=%.1f T4=%.1f T5=%.1f T6=%.1f T7=%.1f T8=%.1f",
		       (double)Temperature_1, (double)Temperature_2, (double)Temperature_3, (double)Temperature_4,
		       (double)Temperature_5, (double)Temperature_6, (double)Temperature_7, (double)Temperature_8);

		  /* [GAP-3 FIX 3] Populate `temperature` with hottest sensor reading.
		   * checkSystemHealth() uses `temperature` for the >75 °C overtemp check;
		   * previously it was uninitialized. */
		  {
		      float temps[8] = { Temperature_1, Temperature_2, Temperature_3, Temperature_4,
		                         Temperature_5, Temperature_6, Temperature_7, Temperature_8 };
		      temperature = temps[0];
		      for (int ti = 1; ti < 8; ti++) {
		          if (temps[ti] > temperature) temperature = temps[ti];
		      }
		      DIAG("PA", "System temperature (max of 8 sensors) = %.1f C", (double)temperature);
		  }

		  //(20 mV/°C on TMP37) QPA2962 RF amplifier Operating Temp. Range, TBASE min−40 normal+25 max+85 °C
		  int Max_Temp = 25;
		  if((Temperature_1>Max_Temp)||(Temperature_2>Max_Temp)||(Temperature_3>Max_Temp)||(Temperature_4>Max_Temp)
				  ||(Temperature_5>Max_Temp)||(Temperature_6>Max_Temp)||(Temperature_7>Max_Temp)||(Temperature_8>Max_Temp))
			{
			  HAL_GPIO_WritePin(EN_DIS_COOLING_GPIO_Port, EN_DIS_COOLING_Pin, GPIO_PIN_SET);
			  DIAG_WARN("PA", "Over-temp detected (>%d C) -- cooling ENABLED", Max_Temp);
			}
		  else{
			  HAL_GPIO_WritePin(EN_DIS_COOLING_GPIO_Port, EN_DIS_COOLING_Pin, GPIO_PIN_RESET);
		  }

		  /* [GAP-3 FIX 4] Periodic IDQ re-read — the Idq_reading[] array was only
		   * populated during startup/calibration.  checkSystemHealth() compares
		   * stale values for overcurrent (>2.5 A) and bias fault (<0.1 A) checks.
		   * Re-read all 16 channels every 5 s alongside temperature. */
		  if (PowerAmplifier) {
		      DIAG("PA", "Periodic IDQ re-read (ADC1 + ADC2, 16 channels)");
		      for (uint8_t ch = 0; ch < 8; ch++) {
		          adc1_readings[ch] = ADS7830_Measure_SingleEnded(&hadc1, ch);
		          Idq_reading[ch] = (3.3f/255.0f) * adc1_readings[ch] / (50.0f * 0.005f);
		      }
		      for (uint8_t ch = 0; ch < 8; ch++) {
		          adc2_readings[ch] = ADS7830_Measure_SingleEnded(&hadc2, ch);
		          Idq_reading[ch + 8] = (3.3f/255.0f) * adc2_readings[ch] / (50.0f * 0.005f);
		      }
		      DIAG("PA", "IDQ[0..3]=%.3f %.3f %.3f %.3f  [4..7]=%.3f %.3f %.3f %.3f",
		           (double)Idq_reading[0], (double)Idq_reading[1],
		           (double)Idq_reading[2], (double)Idq_reading[3],
		           (double)Idq_reading[4], (double)Idq_reading[5],
		           (double)Idq_reading[6], (double)Idq_reading[7]);
		      DIAG("PA", "IDQ[8..11]=%.3f %.3f %.3f %.3f  [12..15]=%.3f %.3f %.3f %.3f",
		           (double)Idq_reading[8], (double)Idq_reading[9],
		           (double)Idq_reading[10], (double)Idq_reading[11],
		           (double)Idq_reading[12], (double)Idq_reading[13],
		           (double)Idq_reading[14], (double)Idq_reading[15]);
		  }


		  /* [BUG #6 FIXED] Was 'last_check' — now correctly writes 'last_check1'
		   * so the temperature timer runs independently of the lock-check timer. */
		  last_check1 = HAL_GetTick();
		  }
	  //////////////////////////////////////////////////////////////////////////////////////
	  /////////////////////////////////////ADAR1000/////////////////////////////////////////
	  //////////////////////////////////////////////////////////////////////////////////////
	 //phase_step = 0 => phase = 0°
	 //phase_step = 127 => phase = 360°
	 //steering angle (rad)= arcsin(phase_dif/Pi)

      runRadarPulseSequence();

      /* [AGC] Outer-loop AGC: sync enable from FPGA via DIG_6 (PD14),
       * then read saturation flag (DIG_5 / PD13) and adjust ADAR1000 VGA
       * common gain once per radar frame (~258 ms).
       * FPGA register host_agc_enable is the single source of truth —
       * DIG_6 propagates it to MCU every frame.
       * 2-frame confirmation debounce: only change outerAgc.enabled when
       * two consecutive frames read the same DIG_6 value. Prevents a
       * single-sample glitch from causing a spurious AGC state transition.
       * Added latency: 1 extra frame (~258 ms), acceptable for control plane. */
      {
          bool dig6_now = (HAL_GPIO_ReadPin(FPGA_DIG6_GPIO_Port,
                                            FPGA_DIG6_Pin) == GPIO_PIN_SET);
          static bool dig6_prev = false;  // matches boot default (AGC off)
          if (dig6_now == dig6_prev) {
              outerAgc.enabled = dig6_now;
          }
          dig6_prev = dig6_now;
      }
      if (outerAgc.enabled) {
          bool sat = HAL_GPIO_ReadPin(FPGA_DIG5_SAT_GPIO_Port,
                                      FPGA_DIG5_SAT_Pin) == GPIO_PIN_SET;
          outerAgc.update(sat);
          outerAgc.applyGain(adarManager);
      }

      /* [GAP-3 FIX 2] Kick hardware watchdog — if we don't reach here within
       * ~4 s, the IWDG resets the MCU automatically. */
      HAL_IWDG_Refresh(&hiwdg);

      // Optional: Add system monitoring here
      // Check temperatures, power levels, etc.


    /* USER CODE END WHILE */




    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_TIM;
  PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_ACTIVATED;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00808CD2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00808CD2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00808CD2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function — DELADJ PWM for ADF4382A phase shift
  *        CH2 = TX DELADJ, CH3 = RX DELADJ
  *        Period (ARR) = 999 → 1000 counts matching DELADJ_MAX_DUTY_CYCLE
  *        Prescaler = 71 → 1 MHz tick @ 72 MHz APB1 → 1 kHz PWM frequency
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  /* B15 fix: provide htim3 definition so adf4382a_manager.c extern resolves */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;                          // 72 MHz / (71+1) = 1 MHz tick
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;                             // ARR = 999 → 1000 counts = DELADJ_MAX_DUTY_CYCLE
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;                                // Start with 0% duty cycle
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  // Configure CH2 (TX DELADJ)
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  // Configure CH3 (RX DELADJ)
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, AD9523_PD_Pin|AD9523_REF_SEL_Pin|AD9523_SYNC_Pin|AD9523_RESET_Pin
                          |AD9523_CS_Pin|AD9523_EEPROM_SEL_Pin|LED_1_Pin|LED_2_Pin
                          |LED_3_Pin|LED_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADAR_1_CS_3V3_Pin|ADAR_2_CS_3V3_Pin|ADAR_3_CS_3V3_Pin|ADAR_4_CS_3V3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, EN_P_5V0_PA1_Pin|EN_P_5V0_PA2_Pin|EN_P_5V0_PA3_Pin|EN_P_5V5_PA_Pin
                          |EN_P_1V8_CLOCK_Pin|EN_P_3V3_CLOCK_Pin|ADF4382_RX_DELADJ_Pin|ADF4382_RX_DELSTR_Pin
                          |ADF4382_RX_CE_Pin|ADF4382_RX_CS_Pin|ADF4382_TX_DELSTR_Pin|ADF4382_TX_DELADJ_Pin
                          |ADF4382_TX_CS_Pin|ADF4382_TX_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, EN_P_1V0_FPGA_Pin|EN_P_1V8_FPGA_Pin|EN_P_3V3_FPGA_Pin|EN_P_5V0_ADAR_Pin
                          |EN_P_3V3_ADAR12_Pin|EN_P_3V3_ADAR34_Pin|EN_P_3V3_ADTR_Pin|EN_P_3V3_SW_Pin
                          |EN_P_3V3_VDD_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |STEPPER_CW_P_Pin|STEPPER_CLK_P_Pin|EN_DIS_RFPA_VDD_Pin|EN_DIS_COOLING_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DAC_1_VG_CLR_Pin|DAC_1_VG_LDAC_Pin|DAC_2_VG_CLR_Pin|DAC_2_VG_LDAC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AD9523_PD_Pin AD9523_REF_SEL_Pin AD9523_SYNC_Pin AD9523_RESET_Pin
                           AD9523_CS_Pin AD9523_EEPROM_SEL_Pin LED_1_Pin LED_2_Pin
                           LED_3_Pin LED_4_Pin */
  GPIO_InitStruct.Pin = AD9523_PD_Pin|AD9523_REF_SEL_Pin|AD9523_SYNC_Pin|AD9523_RESET_Pin
                          |AD9523_CS_Pin|AD9523_EEPROM_SEL_Pin|LED_1_Pin|LED_2_Pin
                          |LED_3_Pin|LED_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : AD9523_STATUS0_Pin AD9523_STATUS1_Pin */
  GPIO_InitStruct.Pin = AD9523_STATUS0_Pin|AD9523_STATUS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : ADAR_1_CS_3V3_Pin ADAR_2_CS_3V3_Pin ADAR_3_CS_3V3_Pin ADAR_4_CS_3V3_Pin */
  GPIO_InitStruct.Pin = ADAR_1_CS_3V3_Pin|ADAR_2_CS_3V3_Pin|ADAR_3_CS_3V3_Pin|ADAR_4_CS_3V3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_P_5V0_PA1_Pin EN_P_5V0_PA2_Pin EN_P_5V0_PA3_Pin EN_P_5V5_PA_Pin
                           EN_P_1V8_CLOCK_Pin EN_P_3V3_CLOCK_Pin ADF4382_RX_DELADJ_Pin ADF4382_RX_DELSTR_Pin
                           ADF4382_RX_CE_Pin ADF4382_RX_CS_Pin ADF4382_TX_DELSTR_Pin ADF4382_TX_DELADJ_Pin
                           ADF4382_TX_CS_Pin ADF4382_TX_CE_Pin */
  GPIO_InitStruct.Pin = EN_P_5V0_PA1_Pin|EN_P_5V0_PA2_Pin|EN_P_5V0_PA3_Pin|EN_P_5V5_PA_Pin
                          |EN_P_1V8_CLOCK_Pin|EN_P_3V3_CLOCK_Pin|ADF4382_RX_DELADJ_Pin|ADF4382_RX_DELSTR_Pin
                          |ADF4382_RX_CE_Pin|ADF4382_RX_CS_Pin|ADF4382_TX_DELSTR_Pin|ADF4382_TX_DELADJ_Pin
                          |ADF4382_TX_CS_Pin|ADF4382_TX_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_P_1V0_FPGA_Pin EN_P_1V8_FPGA_Pin EN_P_3V3_FPGA_Pin EN_P_5V0_ADAR_Pin
                           EN_P_3V3_ADAR12_Pin EN_P_3V3_ADAR34_Pin EN_P_3V3_ADTR_Pin EN_P_3V3_SW_Pin
                           EN_P_3V3_ADAR12EN_P_3V3_VDD_SW_Pin */
  GPIO_InitStruct.Pin = EN_P_1V0_FPGA_Pin|EN_P_1V8_FPGA_Pin|EN_P_3V3_FPGA_Pin|EN_P_5V0_ADAR_Pin
                          |EN_P_3V3_ADAR12_Pin|EN_P_3V3_ADAR34_Pin|EN_P_3V3_ADTR_Pin|EN_P_3V3_SW_Pin
                          |EN_P_3V3_VDD_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           STEPPER_CW_P_Pin STEPPER_CLK_P_Pin EN_DIS_RFPA_VDD_Pin EN_DIS_COOLING_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |STEPPER_CW_P_Pin|STEPPER_CLK_P_Pin|EN_DIS_RFPA_VDD_Pin|EN_DIS_COOLING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ADF4382_RX_LKDET_Pin ADF4382_TX_LKDET_Pin */
  GPIO_InitStruct.Pin = ADF4382_RX_LKDET_Pin|ADF4382_TX_LKDET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : MAG_DRDY_Pin ACC_INT_Pin GYR_INT_Pin */
  GPIO_InitStruct.Pin = MAG_DRDY_Pin|ACC_INT_Pin|GYR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DAC_1_VG_CLR_Pin DAC_1_VG_LDAC_Pin DAC_2_VG_CLR_Pin DAC_2_VG_LDAC_Pin */
  GPIO_InitStruct.Pin = DAC_1_VG_CLR_Pin|DAC_1_VG_LDAC_Pin|DAC_2_VG_CLR_Pin|DAC_2_VG_LDAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ============================================================
 *                  DEVICE INITIALIZATION
 * ============================================================ */

/**
 * [GAP-3 FIX 2] Initialize Independent Watchdog (IWDG).
 *
 * LSI clock ≈ 32 kHz.
 * Prescaler = 256  → IWDG counter clock ≈ 125 Hz (8 ms/tick).
 * Reload   = 500   → timeout ≈ 500 × 8 ms = 4.0 s.
 *
 * The main loop must call HAL_IWDG_Refresh() within this window
 * or the MCU hard-resets — protecting against firmware hangs when
 * PA rails may be energized.
 */
static void MX_IWDG_Init(void)
{
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Reload    = 500;
    hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        DIAG_ERR("SYS", "IWDG init FAILED -- continuing without hardware watchdog");
    } else {
        DIAG("SYS", "IWDG hardware watchdog started (timeout ~4s)");
    }
}



/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
