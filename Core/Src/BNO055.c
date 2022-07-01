/*
 * BNO055.c
 *
 *  Created on: Jun 27, 2022
 *      Author: Jordi
 */

#include "BNO055.h"

uint8_t BNO055_Init(BNO055 *dev, uint8_t ADR_PIN, I2C_HandleTypeDef *i2cHandle)
{
    dev->i2cHandle = i2cHandle;

    if (ADR_PIN)
        dev->ADDRESS = BNO055_ADDRESS_HIGH;
    else
        dev->ADDRESS = BNO055_ADDRESS_LOW;

    uint8_t errNum = 0;
    uint8_t regData = 0;
    HAL_StatusTypeDef status;

    // Check if BNO055 is connected
    status = BNO055_readRegister(dev, BNO055_REG_CHIP_ID, &regData);
    if (status == HAL_BUSY)
    {
        //	I2C_ClearBusyFla
    }
    errNum += (status != HAL_OK);

    // if (regData != 0xA0)
    //     return 255;

    dev->ACC_X = 0;
    dev->ACC_Y = 0;
    dev->ACC_Z = 0;
    dev->GYRO_X = 0;
    dev->GYRO_Y = 0;
    dev->GYRO_Z = 0;
    dev->MAG_X = 0;
    dev->MAG_Y = 0;
    dev->MAG_Z = 0;

    int8_t dataBuf[2];
    status = BNO055_readRegisters(dev, BNO055_REG_MAG_DATA_X_LSB, dataBuf, 2);
    errNum += (status != HAL_OK);

    dev->MAG_X = (dataBuf[0] << 8) | dataBuf[1];

    return errNum;
}

HAL_StatusTypeDef BNO055_readRegister(BNO055 *dev, uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(dev->i2cHandle, dev->ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BNO055_readRegisters(BNO055 *dev, uint8_t reg, uint8_t *data, uint8_t size)
{
    return HAL_I2C_Mem_Read(dev->i2cHandle, dev->ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BNO055_writeRegister(BNO055 *dev, uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Write(dev->i2cHandle, dev->ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BNO055_writeRegisters(BNO055 *dev, uint8_t reg, uint8_t *data, uint8_t size)
{
    return HAL_I2C_Mem_Write(dev->i2cHandle, dev->ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BNO055_readMAG(BNO055 *dev)
{
    int8_t data[2];
    // Get Data from REG_MAG_DATA_X
    BNO055_readRegisters(dev, BNO055_REG_MAG_DATA_X_LSB, data, 2);
    dev->MAG_X = (data[0] << 8) | data[1];

    // Get Data from REG_MAG_DATA_Y
    BNO055_readRegisters(dev, BNO055_REG_MAG_DATA_Y_LSB, data, 2);
    dev->MAG_Y = (data[0] << 8) | data[1];

    // Get Data from REG_MAG_DATA_X
    BNO055_readRegisters(dev, BNO055_REG_MAG_DATA_Z_LSB, data, 2);
    dev->MAG_Z = (data[0] << 8) | data[1];
}

HAL_StatusTypeDef BNO055_readGyro(BNO055 *dev)
{
    int8_t data[2];
    // Get Data from BNO055_REG_GYR_DATA_X
    BNO055_readRegisters(dev, BNO055_REG_GYR_DATA_X_LSB, data, 2);
    dev->GYRO_X = (data[0] << 8) | data[1];

    // Get Data from BNO055_REG_GYR_DATA_Y
    BNO055_readRegisters(dev, BNO055_REG_GYR_DATA_Y_LSB, data, 2);
    dev->GYRO_Y = (data[0] << 8) | data[1];

    // Get Data from BNO055_REG_GYR_DATA_Z
    BNO055_readRegisters(dev, BNO055_REG_GYR_DATA_Z_LSB, data, 2);
    dev->GYRO_Z = (data[0] << 8) | data[1];
}

HAL_StatusTypeDef BNO055_readAccel(BNO055 *dev)
{
    int8_t data[2];
    // Get Data from BNO055_REG_ACC_DATA_X
    BNO055_readRegisters(dev, BNO055_REG_ACC_DATA_X_LSB, data, 2);
    dev->ACC_X = (data[0] << 8) | data[1];

    // Get Data from BNO055_REG_ACC_DATA_Y
    BNO055_readRegisters(dev, BNO055_REG_ACC_DATA_Y_LSB, data, 2);
    dev->ACC_Y = (data[0] << 8) | data[1];

    // Get Data from BNO055_REG_ACC_DATA_Z
    BNO055_readRegisters(dev, BNO055_REG_ACC_DATA_Z_LSB, data, 2);
    dev->ACC_Z = (data[0] << 8) | data[1];
}

HAL_StatusTypeDef BNO055_readLIA(BNO055 *dev)
{
    int8_t data[2];
    // Get Data from BNO055_REG_LIA_DATA_X_LSB
    BNO055_readRegisters(dev, BNO055_REG_LIA_DATA_X_LSB, data, 2);
    dev->LIA_X = (data[0] << 8) | data[1];

    // Get Data from BNO055_REG_LIA_DATA_Y_LSB
    BNO055_readRegisters(dev, BNO055_REG_LIA_DATA_Y_LSB, data, 2);
    dev->LIA_Y = (data[0] << 8) | data[1];

    // Get Data from BNO055_REG_LIA_DATA_Z_LSB
    BNO055_readRegisters(dev, BNO055_REG_LIA_DATA_Z_LSB, data, 2);
    dev->LIA_Z = (data[0] << 8) | data[1];
}
