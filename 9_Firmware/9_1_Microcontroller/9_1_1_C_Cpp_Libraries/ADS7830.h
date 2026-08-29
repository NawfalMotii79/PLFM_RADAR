#ifndef __ADS7830_H
#define __ADS7830_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* I2C Address Definitions */
#define ADS7830_DEFAULT_ADDRESS             (0x48)      // 1001 000 (ADDR = GND)
#define ADS7830_VDD_ADDRESS                 (0x49)      // 1001 001 (ADDR = VDD)
#define ADS7830_SDA_ADDRESS                 (0x4A)      // 1001 010 (ADDR = SDA)
#define ADS7830_SCL_ADDRESS                 (0x4B)      // 1001 011 (ADDR = SCL)

/* Conversion Delay (in ms) */
#define ADS7830_CONVERSIONDELAY             (1)

/* Command Byte Register Masks */
#define ADS7830_REG_COMMAND_SD_MASK         (0x80)      // Single-Ended/Differential Inputs
#define ADS7830_REG_COMMAND_SD_DIFF         (0x00)      // Bit = 0, Differential Inputs
#define ADS7830_REG_COMMAND_SD_SINGLE       (0x80)      // Bit = 1, Single-Ended Inputs

#define ADS7830_REG_COMMAND_CH_MASK         (0x70)      // Input multiplexer Configuration
#define ADS7830_REG_COMMAND_CH_DIFF_0_1     (0x00)      // Differential P = CH0, N = CH1
#define ADS7830_REG_COMMAND_CH_DIFF_2_3     (0x10)      // Differential P = CH2, N = CH3
#define ADS7830_REG_COMMAND_CH_DIFF_4_5     (0x20)      // Differential P = CH4, N = CH5
#define ADS7830_REG_COMMAND_CH_DIFF_6_7     (0x30)      // Differential P = CH6, N = CH7
#define ADS7830_REG_COMMAND_CH_DIFF_1_0     (0x40)      // Differential P = CH1, N = CH0
#define ADS7830_REG_COMMAND_CH_DIFF_3_2     (0x50)      // Differential P = CH3, N = CH2
#define ADS7830_REG_COMMAND_CH_DIFF_5_4     (0x60)      // Differential P = CH5, N = CH4
#define ADS7830_REG_COMMAND_CH_DIFF_7_6     (0x70)      // Differential P = CH7, N = CH6
#define ADS7830_REG_COMMAND_CH_SINGLE_0     (0x00)      // Single-ended P = CH0, N = COM
#define ADS7830_REG_COMMAND_CH_SINGLE_1     (0x10)      // Single-ended P = CH1, N = COM
#define ADS7830_REG_COMMAND_CH_SINGLE_2     (0x20)      // Single-ended P = CH2, N = COM
#define ADS7830_REG_COMMAND_CH_SINGLE_3     (0x30)      // Single-ended P = CH3, N = COM
#define ADS7830_REG_COMMAND_CH_SINGLE_4     (0x40)      // Single-ended P = CH4, N = COM
#define ADS7830_REG_COMMAND_CH_SINGLE_5     (0x50)      // Single-ended P = CH5, N = COM
#define ADS7830_REG_COMMAND_CH_SINGLE_6     (0x60)      // Single-ended P = CH6, N = COM
#define ADS7830_REG_COMMAND_CH_SINGLE_7     (0x70)      // Single-ended P = CH7, N = COM

#define ADS7830_REG_COMMAND_PD_MASK         (0x0C)      // Power-Down Selection
#define ADS7830_REG_COMMAND_PD_PDADCONV     (0x00)      // Power Down Between A/D Converter Conversions
#define ADS7830_REG_COMMAND_PD_IROFF_ADON   (0x04)      // Internal Reference OFF and A/D Converter ON
#define ADS7830_REG_COMMAND_PD_IRON_ADOFF   (0x08)      // Internal Reference ON and A/D Converter OFF
#define ADS7830_REG_COMMAND_PD_IRON_ADON    (0x0C)      // Internal Reference ON and A/D Converter ON

/* Mode Enumerations */
typedef enum {
    ADS7830_SDMODE_DIFF         = ADS7830_REG_COMMAND_SD_DIFF,
    ADS7830_SDMODE_SINGLE       = ADS7830_REG_COMMAND_SD_SINGLE
} ADS7830_SDMode_t;

typedef enum {
    ADS7830_PDADCONV            = ADS7830_REG_COMMAND_PD_PDADCONV,
    ADS7830_PDIROFF_ADON        = ADS7830_REG_COMMAND_PD_IROFF_ADON,
    ADS7830_PDIRON_ADOFF        = ADS7830_REG_COMMAND_PD_IRON_ADOFF,
    ADS7830_PDIRON_ADON         = ADS7830_REG_COMMAND_PD_IRON_ADON
} ADS7830_PDMode_t;

/* ADC Handle Structure */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_addr;
    ADS7830_SDMode_t sdmode;
    ADS7830_PDMode_t pdmode;
    uint8_t conversion_delay;
    uint8_t last_conversion_result;
} ADS7830_HandleTypeDef;

/* Function Prototypes */
bool ADS7830_Init(ADS7830_HandleTypeDef *hadc, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, 
                  ADS7830_SDMode_t sdmode, ADS7830_PDMode_t pdmode);
bool ADS7830_SetSDMode(ADS7830_HandleTypeDef *hadc, ADS7830_SDMode_t sdmode);
bool ADS7830_SetPDMode(ADS7830_HandleTypeDef *hadc, ADS7830_PDMode_t pdmode);
uint8_t ADS7830_Measure_SingleEnded(ADS7830_HandleTypeDef *hadc, uint8_t channel);
int8_t ADS7830_Measure_Differential(ADS7830_HandleTypeDef *hadc, uint8_t channel);
uint8_t ADS7830_GetLastConversionResult(ADS7830_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif /* __ADS7830_H */