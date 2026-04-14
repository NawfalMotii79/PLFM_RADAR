/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t GUI_start_flag_received;
extern uint8_t USB_Buffer[64];

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AD9523_PD_Pin GPIO_PIN_3
#define AD9523_PD_GPIO_Port GPIOF
#define AD9523_REF_SEL_Pin GPIO_PIN_4
#define AD9523_REF_SEL_GPIO_Port GPIOF
#define AD9523_SYNC_Pin GPIO_PIN_5
#define AD9523_SYNC_GPIO_Port GPIOF
#define AD9523_RESET_Pin GPIO_PIN_6
#define AD9523_RESET_GPIO_Port GPIOF
#define AD9523_CS_Pin GPIO_PIN_7
#define AD9523_CS_GPIO_Port GPIOF
#define AD9523_STATUS0_Pin GPIO_PIN_8
#define AD9523_STATUS0_GPIO_Port GPIOF
#define AD9523_STATUS1_Pin GPIO_PIN_9
#define AD9523_STATUS1_GPIO_Port GPIOF
#define AD9523_EEPROM_SEL_Pin GPIO_PIN_10
#define AD9523_EEPROM_SEL_GPIO_Port GPIOF
#define ADAR_1_CS_3V3_Pin GPIO_PIN_0
#define ADAR_1_CS_3V3_GPIO_Port GPIOA
#define ADAR_2_CS_3V3_Pin GPIO_PIN_1
#define ADAR_2_CS_3V3_GPIO_Port GPIOA
#define ADAR_3_CS_3V3_Pin GPIO_PIN_2
#define ADAR_3_CS_3V3_GPIO_Port GPIOA
#define ADAR_4_CS_3V3_Pin GPIO_PIN_3
#define ADAR_4_CS_3V3_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_12
#define LED_1_GPIO_Port GPIOF
#define LED_2_Pin GPIO_PIN_13
#define LED_2_GPIO_Port GPIOF
#define LED_3_Pin GPIO_PIN_14
#define LED_3_GPIO_Port GPIOF
#define LED_4_Pin GPIO_PIN_15
#define LED_4_GPIO_Port GPIOF
#define EN_P_5V0_PA1_Pin GPIO_PIN_0
#define EN_P_5V0_PA1_GPIO_Port GPIOG
#define EN_P_5V0_PA2_Pin GPIO_PIN_1
#define EN_P_5V0_PA2_GPIO_Port GPIOG
#define EN_P_1V0_FPGA_Pin GPIO_PIN_7
#define EN_P_1V0_FPGA_GPIO_Port GPIOE
#define EN_P_1V8_FPGA_Pin GPIO_PIN_8
#define EN_P_1V8_FPGA_GPIO_Port GPIOE
#define EN_P_3V3_FPGA_Pin GPIO_PIN_9
#define EN_P_3V3_FPGA_GPIO_Port GPIOE
#define EN_P_5V0_ADAR_Pin GPIO_PIN_10
#define EN_P_5V0_ADAR_GPIO_Port GPIOE
#define EN_P_3V3_ADAR12_Pin GPIO_PIN_11
#define EN_P_3V3_ADAR12_GPIO_Port GPIOE
#define EN_P_3V3_ADAR34_Pin GPIO_PIN_12
#define EN_P_3V3_ADAR34_GPIO_Port GPIOE
#define EN_P_3V3_ADTR_Pin GPIO_PIN_13
#define EN_P_3V3_ADTR_GPIO_Port GPIOE
#define EN_P_3V3_SW_Pin GPIO_PIN_14
#define EN_P_3V3_SW_GPIO_Port GPIOE
#define EN_P_3V3_VDD_SW_Pin GPIO_PIN_15
#define EN_P_3V3_VDD_SW_GPIO_Port GPIOE
#define EN_P_5V0_PA3_Pin GPIO_PIN_2
#define EN_P_5V0_PA3_GPIO_Port GPIOG
#define EN_P_5V5_PA_Pin GPIO_PIN_3
#define EN_P_5V5_PA_GPIO_Port GPIOG
#define EN_P_1V8_CLOCK_Pin GPIO_PIN_4
#define EN_P_1V8_CLOCK_GPIO_Port GPIOG
#define EN_P_3V3_CLOCK_Pin GPIO_PIN_5
#define EN_P_3V3_CLOCK_GPIO_Port GPIOG
#define ADF4382_RX_LKDET_Pin GPIO_PIN_6
#define ADF4382_RX_LKDET_GPIO_Port GPIOG
#define ADF4382_RX_DELADJ_Pin GPIO_PIN_7
#define ADF4382_RX_DELADJ_GPIO_Port GPIOG
#define ADF4382_RX_DELSTR_Pin GPIO_PIN_8
#define ADF4382_RX_DELSTR_GPIO_Port GPIOG
#define MAG_DRDY_Pin GPIO_PIN_6
#define MAG_DRDY_GPIO_Port GPIOC
#define ACC_INT_Pin GPIO_PIN_7
#define ACC_INT_GPIO_Port GPIOC
#define GYR_INT_Pin GPIO_PIN_8
#define GYR_INT_GPIO_Port GPIOC
#define STEPPER_CW_P_Pin GPIO_PIN_4
#define STEPPER_CW_P_GPIO_Port GPIOD
#define STEPPER_CLK_P_Pin GPIO_PIN_5
#define STEPPER_CLK_P_GPIO_Port GPIOD
#define EN_DIS_RFPA_VDD_Pin GPIO_PIN_6
#define EN_DIS_RFPA_VDD_GPIO_Port GPIOD
#define EN_DIS_COOLING_Pin GPIO_PIN_7
#define EN_DIS_COOLING_GPIO_Port GPIOD
#define ADF4382_RX_CE_Pin GPIO_PIN_9
#define ADF4382_RX_CE_GPIO_Port GPIOG
#define ADF4382_RX_CS_Pin GPIO_PIN_10
#define ADF4382_RX_CS_GPIO_Port GPIOG
#define ADF4382_TX_LKDET_Pin GPIO_PIN_11
#define ADF4382_TX_LKDET_GPIO_Port GPIOG
#define ADF4382_TX_DELSTR_Pin GPIO_PIN_12
#define ADF4382_TX_DELSTR_GPIO_Port GPIOG
#define ADF4382_TX_DELADJ_Pin GPIO_PIN_13
#define ADF4382_TX_DELADJ_GPIO_Port GPIOG
#define ADF4382_TX_CS_Pin GPIO_PIN_14
#define ADF4382_TX_CS_GPIO_Port GPIOG
#define ADF4382_TX_CE_Pin GPIO_PIN_15
#define ADF4382_TX_CE_GPIO_Port GPIOG
#define DAC_1_VG_CLR_Pin GPIO_PIN_4
#define DAC_1_VG_CLR_GPIO_Port GPIOB
#define DAC_1_VG_LDAC_Pin GPIO_PIN_5
#define DAC_1_VG_LDAC_GPIO_Port GPIOB
#define DAC_2_VG_CLR_Pin GPIO_PIN_8
#define DAC_2_VG_CLR_GPIO_Port GPIOB
#define DAC_2_VG_LDAC_Pin GPIO_PIN_9
#define DAC_2_VG_LDAC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/*
 * PLFM phase-2 ADC I/Q capture map.
 * Default map uses ADC1_IN10/IN11 on PC0/PC1.
 * Override these defines for board-specific IF I/Q wiring.
 */
#define PLFM_ADC_I_CHANNEL       ADC_CHANNEL_10
#define PLFM_ADC_Q_CHANNEL       ADC_CHANNEL_11
#define PLFM_ADC_I_GPIO_Port     GPIOC
#define PLFM_ADC_I_GPIO_Pin      GPIO_PIN_0
#define PLFM_ADC_Q_GPIO_Port     GPIOC
#define PLFM_ADC_Q_GPIO_Pin      GPIO_PIN_1
#define PLFM_ADC_SAMPLE_TIME     ADC_SAMPLETIME_84CYCLES

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
