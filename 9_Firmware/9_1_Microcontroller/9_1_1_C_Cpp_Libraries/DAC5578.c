#include "DAC5578.h"
#include "diag_log.h"
#include <string.h>

/**
  * @brief  Initialize the DAC5578
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  hi2c: pointer to an I2C_HandleTypeDef structure
  * @param  i2c_addr: I2C address of the DAC
  * @param  resolution: DAC resolution (only 8 is valid for DAC5578)
  * @param  ldac_port: GPIO port for LDAC pin
  * @param  ldac_pin: GPIO pin for LDAC pin
  * @param  clr_port: GPIO port for CLR pin
  * @param  clr_pin: GPIO pin for CLR pin
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_Init(DAC5578_HandleTypeDef *hdac, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, 
                  uint8_t resolution, GPIO_TypeDef *ldac_port, uint16_t ldac_pin,
                  GPIO_TypeDef *clr_port, uint16_t clr_pin) {
    
    DIAG("PA", "DAC5578_Init: addr=0x%02X (shifted=0x%02X), res=%u", i2c_addr, i2c_addr << 1, resolution);
    
    if (hdac == NULL || hi2c == NULL) {
        DIAG_ERR("PA", "DAC5578_Init: NULL handle(s)");
        return false;
    }
    
    /* DAC5578 is 8-bit only */
    hdac->resolution_bits = 8;
    hdac->clear_code = DAC5578_CLR_CODE_ZERO; // Default clear to zero
    
    hdac->hi2c = hi2c;
    hdac->i2c_addr = i2c_addr << 1; // HAL requires 7-bit address shifted left
    hdac->ldac_port = ldac_port;
    hdac->ldac_pin = ldac_pin;
    hdac->clr_port = clr_port;
    hdac->clr_pin = clr_pin;
    
    /* Set LDAC high (inactive) and CLR high (normal operation) */
    if (ldac_port != NULL) {
        DIAG("PA", "  LDAC pin -> HIGH (inactive)");
        HAL_GPIO_WritePin(ldac_port, ldac_pin, GPIO_PIN_SET);
    }
    
    if (clr_port != NULL) {
        DIAG("PA", "  CLR pin -> HIGH (normal operation)");
        HAL_GPIO_WritePin(clr_port, clr_pin, GPIO_PIN_SET);
    }

    /* Reset the DAC and enable internal reference by default */
    DIAG("PA", "  Resetting DAC5578...");
    bool success = DAC5578_Reset(hdac);
    if (success) {
        DIAG("PA", "  Enabling internal reference...");
        success = DAC5578_SetInternalReference(hdac, true);
    } else {
        DIAG_ERR("PA", "  DAC5578_Reset FAILED");
    }

    /* Set the clear code in the device */
    if (success) {
        DIAG("PA", "  Setting clear code to ZERO...");
        success = DAC5578_SetClearCode(hdac, hdac->clear_code);
    }

    DIAG("PA", "DAC5578_Init: %s", success ? "OK" : "FAILED");
    return success;
}

/**
  * @brief  Reset the DAC5578
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_Reset(DAC5578_HandleTypeDef *hdac) {
    DIAG("PA", "DAC5578_Reset: addr=0x%02X", hdac->i2c_addr);
    uint8_t buffer[3];
    buffer[0] = DAC5578_CMD_RESET;
    buffer[1] = 0x00;
    buffer[2] = 0x00;

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hdac->hi2c, hdac->i2c_addr, buffer, 3, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        DIAG_ERR("PA", "DAC5578_Reset: I2C transmit FAILED, HAL status=%d", (int)status);
    }
    return (status == HAL_OK);
}

/**
  * @brief  Write a value to a specific channel's input register
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  channel: DAC channel (0-7)
  * @param  value: 8-bit value to write
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_WriteChannelValue(DAC5578_HandleTypeDef *hdac, uint8_t channel, uint16_t value) {
    if (channel > 7) {
        return false;
    }

    /* DAC5578 is 8-bit, so mask value */
    value &= 0xFF;

    return DAC5578_CommandWrite(hdac, DAC5578_CMD_WRITE | (channel & 0x7), value);
}

/**
  * @brief  Update a specific channel's DAC register from input register
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  channel: DAC channel (0-7) or DAC5578_CHANNEL_BROADCAST
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_UpdateChannel(DAC5578_HandleTypeDef *hdac, uint8_t channel) {
    if (channel > 7 && channel != DAC5578_CHANNEL_BROADCAST) {
        return false;
    }

    /* Update command with channel selection */
    uint8_t command = DAC5578_CMD_UPDATE | (channel & 0x7);
    if (channel == DAC5578_CHANNEL_BROADCAST) {
        command = DAC5578_CMD_UPDATE | 0x8; // Broadcast flag
    }

    uint8_t buffer[3];
    buffer[0] = command;
    buffer[1] = 0x00;
    buffer[2] = 0x00;

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hdac->hi2c, hdac->i2c_addr, buffer, 3, HAL_MAX_DELAY);
    return (status == HAL_OK);
}

/**
  * @brief  Write and update a specific channel in one operation
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  channel: DAC channel (0-7)
  * @param  value: 8-bit value to write and update
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_WriteAndUpdateChannelValue(DAC5578_HandleTypeDef *hdac, uint8_t channel, uint16_t value) {
    if (channel > 7) {
        DIAG_ERR("PA", "DAC5578_WriteAndUpdate: channel %u out of range", channel);
        return false;
    }

    /* DAC5578 is 8-bit, so mask value */
    value &= 0xFF;

    DIAG("PA", "DAC5578_WriteAndUpdate: addr=0x%02X ch=%u val=0x%02X (%u)", hdac->i2c_addr, channel, value, value);
    return DAC5578_CommandWrite(hdac, DAC5578_CMD_WRITE_UPDATE | (channel & 0x7), value);
}

/**
  * @brief  Write the same value to all channels' input registers
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  value: 8-bit value to write to all channels
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_WriteAllChannels(DAC5578_HandleTypeDef *hdac, uint16_t value) {
    /* DAC5578 is 8-bit, so mask value */
    value &= 0xFF;

    return DAC5578_CommandWrite(hdac, DAC5578_CMD_WRITE_ALL, value);
}

/**
  * @brief  Read the input register value of a specific channel
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  channel: DAC channel (0-7)
  * @param  value: pointer to store the read value
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_ReadInputChannelValue(DAC5578_HandleTypeDef *hdac, uint8_t channel, uint16_t *value) {
    if (channel > 7) {
        return false;
    }

    /* Note: DAC5578 has limited read capability - this reads back the input register */
    return DAC5578_CommandRead(hdac, DAC5578_CMD_WRITE | (channel & 0x7), value);
}

/**
  * @brief  Read the DAC register value (current output) of a specific channel
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  channel: DAC channel (0-7)
  * @param  value: pointer to store the read value
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_ReadDACChannelValue(DAC5578_HandleTypeDef *hdac, uint8_t channel, uint16_t *value) {
    if (channel > 7) {
        return false;
    }

    /* Note: DAC5578 has limited read capability - this reads back the DAC register */
    return DAC5578_CommandRead(hdac, DAC5578_CMD_UPDATE | (channel & 0x7), value);
}

/**
  * @brief  Set power down mode for a specific channel
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  channel: DAC channel (0-7)
  * @param  mode: power down mode
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_SetPowerDownMode(DAC5578_HandleTypeDef *hdac, uint8_t channel, DAC5578_PowerDownMode_t mode) {
    if (channel > 7 || mode > 3) {
        return false;
    }

    uint8_t buffer[3];
    buffer[0] = DAC5578_CMD_POWERDOWN | (channel & 0x7);
    buffer[1] = 0x00;
    buffer[2] = mode & 0x03;
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hdac->hi2c, hdac->i2c_addr, buffer, 3, HAL_MAX_DELAY);
    return (status == HAL_OK);
}

/**
  * @brief  Set power down mode for all channels
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  mode: power down mode
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_SetPowerDownAll(DAC5578_HandleTypeDef *hdac, DAC5578_PowerDownMode_t mode) {
    if (mode > 3) {
        return false;
    }
    
    uint8_t buffer[3];
    buffer[0] = DAC5578_CMD_POWERDOWN_ALL;
    buffer[1] = 0x00;
    buffer[2] = mode & 0x03;
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hdac->hi2c, hdac->i2c_addr, buffer, 3, HAL_MAX_DELAY);
    return (status == HAL_OK);
}

/**
  * @brief  Enable or disable the internal reference
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  enable: true to enable, false to disable
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_SetInternalReference(DAC5578_HandleTypeDef *hdac, bool enable) {
    uint8_t command = enable ? DAC5578_CMD_INT_REF_ENABLE : DAC5578_CMD_INT_REF_DISABLE;
    uint8_t buffer[3];
    buffer[0] = command;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hdac->hi2c, hdac->i2c_addr, buffer, 3, HAL_MAX_DELAY);
    return (status == HAL_OK);
}

/**
  * @brief  Setup LDAC mask for hardware LDAC control
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  ldac_mask: 8-bit mask where each bit corresponds to a channel
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_SetupLDAC(DAC5578_HandleTypeDef *hdac, uint8_t ldac_mask) {
    uint8_t buffer[3];
    buffer[0] = DAC5578_CMD_LDAC_SETUP;
    buffer[1] = 0x00;
    buffer[2] = ldac_mask & 0xFF; // Each bit corresponds to a channel (0-7)
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hdac->hi2c, hdac->i2c_addr, buffer, 3, HAL_MAX_DELAY);
    return (status == HAL_OK);
}

/**
  * @brief  Trigger software LDAC update
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_SoftwareLDAC(DAC5578_HandleTypeDef *hdac) {
    uint8_t buffer[3];
    buffer[0] = DAC5578_CMD_SOFTWARE_LDAC;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hdac->hi2c, hdac->i2c_addr, buffer, 3, HAL_MAX_DELAY);
    return (status == HAL_OK);
}

/* CLR Pin Functions */

/**
  * @brief  Set the clear code that determines what happens when CLR pin is activated
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  clear_code: desired clear code behavior
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_SetClearCode(DAC5578_HandleTypeDef *hdac, DAC5578_ClearCode_t clear_code) {
    if (clear_code > DAC5578_CLR_CODE_NOP) {
        return false;
    }

    /* The clear code is set using the RESET command with specific data bits */
    uint8_t buffer[3];
    buffer[0] = DAC5578_CMD_RESET;
    buffer[1] = 0x00;
    buffer[2] = (clear_code & 0x03); // Clear code in bits 1:0

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hdac->hi2c, hdac->i2c_addr, buffer, 3, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        hdac->clear_code = clear_code;
        return true;
    }

    return false;
}

/**
  * @brief  Get the current clear code setting
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @retval DAC5578_ClearCode_t: current clear code setting
  */
DAC5578_ClearCode_t DAC5578_GetClearCode(DAC5578_HandleTypeDef *hdac) {
    return hdac->clear_code;
}

/**
  * @brief  Activate the CLR pin (set low) to clear DAC outputs
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @retval None
  */
void DAC5578_ActivateClearPin(DAC5578_HandleTypeDef *hdac) {
    DIAG_WARN("PA", "DAC5578_ActivateClearPin: CLR -> LOW (emergency clear), addr=0x%02X", hdac->i2c_addr);
    if (hdac->clr_port != NULL) {
        HAL_GPIO_WritePin(hdac->clr_port, hdac->clr_pin, GPIO_PIN_RESET);
    } else {
        DIAG_ERR("PA", "  CLR port is NULL -- cannot activate hardware clear!");
    }
}

/**
  * @brief  Deactivate the CLR pin (set high) for normal operation
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @retval None
  */
void DAC5578_DeactivateClearPin(DAC5578_HandleTypeDef *hdac) {
    if (hdac->clr_port != NULL) {
        HAL_GPIO_WritePin(hdac->clr_port, hdac->clr_pin, GPIO_PIN_SET);
    }
}

/**
  * @brief  Pulse the CLR pin to clear all DAC outputs according to clear code
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @retval None
  */
void DAC5578_ClearOutputs(DAC5578_HandleTypeDef *hdac) {
    DIAG_WARN("PA", "DAC5578_ClearOutputs: pulsing CLR pin, addr=0x%02X", hdac->i2c_addr);
    if (hdac->clr_port != NULL) {
        /* Generate a pulse on CLR pin (active low) */
        HAL_GPIO_WritePin(hdac->clr_port, hdac->clr_pin, GPIO_PIN_RESET);
        HAL_Delay(1); // Hold for at least 50ns (1ms is plenty)
        HAL_GPIO_WritePin(hdac->clr_port, hdac->clr_pin, GPIO_PIN_SET);
    } else {
        DIAG_ERR("PA", "  CLR port is NULL -- cannot pulse clear!");
    }
}

/**
  * @brief  Software clear - uses I2C command to clear outputs without CLR pin
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_SoftwareClear(DAC5578_HandleTypeDef *hdac) {
    /* Use the reset command with the current clear code to perform software clear */
    return DAC5578_SetClearCode(hdac, hdac->clear_code);
}

/* Private functions */

/**
  * @brief  Write a command and value to the DAC
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  command: command byte
  * @param  value: 8-bit value to write
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_CommandWrite(DAC5578_HandleTypeDef *hdac, uint8_t command, uint16_t value) {
    uint8_t buffer[3];
    buffer[0] = command;
    buffer[1] = (value >> 8) & 0xFF; // MSB (should be 0 for 8-bit DAC)
    buffer[2] = value & 0xFF;        // LSB (actual 8-bit data)

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hdac->hi2c, hdac->i2c_addr, buffer, 3, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        DIAG_ERR("PA", "DAC5578 I2C write FAILED: addr=0x%02X cmd=0x%02X HAL=%d", hdac->i2c_addr, command, (int)status);
    }
    return (status == HAL_OK);
}

/**
  * @brief  Read a value from the DAC after sending a command
  * @param  hdac: pointer to a DAC5578_HandleTypeDef structure
  * @param  command: command byte
  * @param  value: pointer to store the read value
  * @retval bool: true if successful, false otherwise
  */
bool DAC5578_CommandRead(DAC5578_HandleTypeDef *hdac, uint8_t command, uint16_t *value) {
    uint8_t buffer[3];

    /* First write the command to set up readback */
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hdac->hi2c, hdac->i2c_addr, &command, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        DIAG_ERR("PA", "DAC5578 I2C read setup FAILED: addr=0x%02X cmd=0x%02X HAL=%d", hdac->i2c_addr, command, (int)status);
        return false;
    }

    /* Then read 3 bytes back */
    status = HAL_I2C_Master_Receive(hdac->hi2c, hdac->i2c_addr, buffer, 3, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        DIAG_ERR("PA", "DAC5578 I2C read data FAILED: addr=0x%02X cmd=0x%02X HAL=%d", hdac->i2c_addr, command, (int)status);
        return false;
    }

    /* Extract the 8-bit value from the response */
    *value = buffer[2] & 0xFF;
    DIAG("PA", "DAC5578_Read: addr=0x%02X cmd=0x%02X => 0x%02X", hdac->i2c_addr, command, *value);
    return true;
}
