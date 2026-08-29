#ifndef __DAC5578_H
#define __DAC5578_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include <stdbool.h>



extern I2C_HandleTypeDef hi2c1;

/* Command definitions - DAC5578 specific */
#define DAC5578_CMD_WRITE              (0x0 << 4)  // Write to input register
#define DAC5578_CMD_UPDATE             (0x1 << 4)  // Update DAC register
#define DAC5578_CMD_WRITE_UPDATE       (0x2 << 4)  // Write and update
#define DAC5578_CMD_WRITE_ALL          (0x3 << 4)  // Write to all channels
#define DAC5578_CMD_POWERDOWN          (0x4 << 4)  // Power down
#define DAC5578_CMD_POWERDOWN_ALL      (0x5 << 4)  // Power down all channels
#define DAC5578_CMD_INT_REF_ENABLE     (0x6 << 4)  // Enable internal reference
#define DAC5578_CMD_INT_REF_DISABLE    (0x7 << 4)  // Disable internal reference
#define DAC5578_CMD_RESET              (0x8 << 4)  // Software reset
#define DAC5578_CMD_LDAC_SETUP         (0x9 << 4)  // LDAC setup register
#define DAC5578_CMD_SOFTWARE_LDAC      (0xA << 4)  // Software LDAC trigger

/* Broadcast channel for simultaneous updates */
#define DAC5578_CHANNEL_BROADCAST      0x8

/* Power down modes */
typedef enum {
    DAC5578_PWD_NORMAL = 0,     // Normal operation
    DAC5578_PWD_1K = 1,         // 1kΩ to GND
    DAC5578_PWD_100K = 2,       // 100kΩ to GND
    DAC5578_PWD_HIZ = 3         // High impedance
} DAC5578_PowerDownMode_t;

/* Clear code options - determines what happens when CLR pin is activated */
typedef enum {
    DAC5578_CLR_CODE_ZERO = 0,      // Clear to zero scale (0x00)
    DAC5578_CLR_CODE_MID = 1,       // Clear to midscale (0x80)
    DAC5578_CLR_CODE_FULL = 2,      // Clear to full scale (0xFF)
    DAC5578_CLR_CODE_NOP = 3        // No operation (retain current value)
} DAC5578_ClearCode_t;

/* DAC handle structure */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_addr;
    uint8_t resolution_bits;
    GPIO_TypeDef *ldac_port;
    uint16_t ldac_pin;
    GPIO_TypeDef *clr_port;
    uint16_t clr_pin;
    DAC5578_ClearCode_t clear_code; // Current clear code setting
} DAC5578_HandleTypeDef;

/* Function prototypes */
bool DAC5578_Init(DAC5578_HandleTypeDef *hdac, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, 
                  uint8_t resolution, GPIO_TypeDef *ldac_port, uint16_t ldac_pin,
                  GPIO_TypeDef *clr_port, uint16_t clr_pin);
bool DAC5578_Reset(DAC5578_HandleTypeDef *hdac);
bool DAC5578_WriteChannelValue(DAC5578_HandleTypeDef *hdac, uint8_t channel, uint16_t value);
bool DAC5578_UpdateChannel(DAC5578_HandleTypeDef *hdac, uint8_t channel);
bool DAC5578_WriteAndUpdateChannelValue(DAC5578_HandleTypeDef *hdac, uint8_t channel, uint16_t value);
bool DAC5578_WriteAllChannels(DAC5578_HandleTypeDef *hdac, uint16_t value);
bool DAC5578_ReadInputChannelValue(DAC5578_HandleTypeDef *hdac, uint8_t channel, uint16_t *value);
bool DAC5578_ReadDACChannelValue(DAC5578_HandleTypeDef *hdac, uint8_t channel, uint16_t *value);
bool DAC5578_SetPowerDownMode(DAC5578_HandleTypeDef *hdac, uint8_t channel, DAC5578_PowerDownMode_t mode);
bool DAC5578_SetPowerDownAll(DAC5578_HandleTypeDef *hdac, DAC5578_PowerDownMode_t mode);
bool DAC5578_SetInternalReference(DAC5578_HandleTypeDef *hdac, bool enable);
bool DAC5578_SetupLDAC(DAC5578_HandleTypeDef *hdac, uint8_t ldac_mask);
bool DAC5578_SoftwareLDAC(DAC5578_HandleTypeDef *hdac);

/* CLR Pin Functions */
bool DAC5578_SetClearCode(DAC5578_HandleTypeDef *hdac, DAC5578_ClearCode_t clear_code);
DAC5578_ClearCode_t DAC5578_GetClearCode(DAC5578_HandleTypeDef *hdac);
void DAC5578_ActivateClearPin(DAC5578_HandleTypeDef *hdac);
void DAC5578_DeactivateClearPin(DAC5578_HandleTypeDef *hdac);
void DAC5578_ClearOutputs(DAC5578_HandleTypeDef *hdac); // Pulse CLR pin to clear outputs

/* Private functions */
bool DAC5578_CommandWrite(DAC5578_HandleTypeDef *hdac, uint8_t command, uint16_t value);
bool DAC5578_CommandRead(DAC5578_HandleTypeDef *hdac, uint8_t command, uint16_t *value);

#ifdef __cplusplus
}
#endif

#endif /* __DAC5578_H */
