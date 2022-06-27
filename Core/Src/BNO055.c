/*
 * BNO055.c
 *
 *  Created on: Jun 27, 2022
 *      Author: Jordi
 */

#include "BNO055.h"

uint8_t BNO055_Init(BNO055 *dev, I2C_HandleTypeDef *i2cHandle)
{
    dev->i2cHandle = i2cHandle;

    uint8_t errNum = 0;
    uint8_t regData;
    HAL_StatusTypeDef status;

    // Check if BNO055 is connected
    status = BNO055_readRegister(dev, BNO055_REG_CHIP_ID, &regData);
    errNum += (status != HAL_OK);

    if (regData != 0xA0)
        return 255;

    
}

HAL_StatusTypeDef BNO055_readRegister(BNO055 *dev, uint8_t reg, uint8_t *data){
    return HAL_I2C_Mem_Read(dev->i2cHandle, BNO055_ADDRESS_LOW, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BNO055_readRegisters(BNO055 *dev, uint8_t reg, uint8_t *data, uint8_t size)
{
    return HAL_I2C_Mem_Read(dev->i2cHandle, BNO055_ADDRESS_LOW, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BNO055_writeRegister(BNO055 *dev, uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Write(dev->i2cHandle, BNO055_ADDRESS_LOW, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BNO055_writeRegisters(BNO055 *dev, uint8_t reg, uint8_t *data, uint8_t size)
{
    return HAL_I2C_Mem_Write(dev->i2cHandle, BNO055_ADDRESS_LOW, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}