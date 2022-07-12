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
        dev->ADDRESS = BNO055_ADDRESS_HIGH << 1;
    else
        dev->ADDRESS = BNO055_ADDRESS_LOW << 1;

    uint8_t errNum = 0;
    uint8_t regData = 0;
    HAL_StatusTypeDef status;

    // Check if BNO055 is connected
    status = BNO055_readRegister(dev, BNO055_REG_CHIP_ID, &regData);
    if ((status == HAL_BUSY) || (status == HAL_ERROR))
    {
    	// delay and check again
        HAL_Delay(500);
        status = BNO055_readRegister(dev, BNO055_REG_CHIP_ID, &regData);
        // if failed, return fail code
        if ((status == HAL_BUSY) || (status == HAL_ERROR))
        	return 255;
    }
    errNum += (status != HAL_OK);

    if (regData != 0xA0)
         return 255;

    // Check IMU mode
    status = BNO055_readRegister(dev, BNO055_REG_OPR_MODE, &regData);
    errNum += (status != HAL_OK);

    status = BNO055_readRegister(dev, BNO055_REG_CALIB_STAT, &regData);
    errNum += (status != HAL_OK);

    // Check if IMU is fully calibrated
    //while(regData != 0xFF)
    //{
    //	// delay for 50ms then try again
    // 	  HAL_Delay(50);
    //    status = BNO055_readRegister(dev, BNO055_REG_CALIB_STAT, &regData);
    //    errNum += (status != HAL_OK);
    //}

    // Set IMU mode Page 21 Table 3-5
    // Set to NDOF
    regData = (regData & ~0b00001111) | 0b1100;
    status = BNO055_writeRegister(dev, BNO055_REG_OPR_MODE, &regData);
    errNum += (status != HAL_OK);

    // Get ACC Config
    status = BNO055_readRegister(dev, BNO055_REG_ACC_Config, &regData);
    errNum += (status != HAL_OK);

    // Get GYR C
    status = BNO055_readRegister(dev, BNO055_REG_GYR_Config_0, &regData);
    errNum += (status != HAL_OK);

    status = BNO055_readRegister(dev, BNO055_REG_GYR_Config_1, &regData);
    errNum += (status != HAL_OK);

    // Get MAG C
    status = BNO055_readRegister(dev, BNO055_REG_MAG_Config, &regData);
    errNum += (status != HAL_OK);

    // Get Unit Se
    status = BNO055_readRegister(dev, BNO055_REG_UNIT_SEL, &regData);
    errNum += (status != HAL_OK);

    dev->ACC_X = 0;
    dev->ACC_Y = 0;
    dev->ACC_Z = 0;
    dev->GYRO_X = 0;
    dev->GYRO_Y = 0;
    dev->GYRO_Z = 0;
    dev->MAG_X = 0.0f;
    dev->MAG_Y = 0.0f;
    dev->MAG_Z = 0.0f;

    // Magnetic declination from magnetic-declination.com
    // East is positive (+), west is negative (-)
    // mag_decl = (+/-)(deg + min/60 + sec/3600)
    // Set to 0 to get magnetic heading instead of geo heading
    dev->MAG_DECL = 11.52;

    int8_t dataBuf[2];
    status = BNO055_readRegisters(dev, BNO055_REG_MAG_DATA_X_LSB, dataBuf, 2);
    errNum += (status != HAL_OK);

    dev->MAG_X = (((int16_t)dataBuf[1] << 8) | dataBuf[0])/16.0;

    status = BNO055_readRegister(dev, BNO055_REG_MAG_DATA_X_LSB, &regData);
    errNum += (status != HAL_OK);

    int16_t temp = regData;

    status = BNO055_readRegister(dev, BNO055_REG_MAG_DATA_X_MSB, &regData);
    errNum += (status != HAL_OK);

    temp = (regData << 8) | temp;
    dev->MAG_X = temp / 16.0;

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
	HAL_StatusTypeDef status;

    int8_t data[2];
    // Get Data from REG_MAG_DATA_X
    status = BNO055_readRegister(dev, BNO055_REG_MAG_DATA_X_LSB, &data[0]);
    status = BNO055_readRegister(dev, BNO055_REG_MAG_DATA_X_MSB, &data[1]);
    dev->MAG_X = (((int16_t)data[1] << 8) | data[0])/16.0;

    // Get Data from REG_MAG_DATA_Y
    status = BNO055_readRegister(dev, BNO055_REG_MAG_DATA_Y_LSB, &data[0]);
    status = BNO055_readRegister(dev, BNO055_REG_MAG_DATA_Y_MSB, &data[1]);
    dev->MAG_Y = (((int16_t)data[1] << 8) | data[0])/16.0;

    // Get Data from REG_MAG_DATA_X
    status = BNO055_readRegister(dev, BNO055_REG_MAG_DATA_Z_LSB, &data[0]);
    status = BNO055_readRegister(dev, BNO055_REG_MAG_DATA_Z_MSB, &data[1]);
    dev->MAG_Z = (((int16_t)data[1] << 8) | data[0])/16.0;
}

HAL_StatusTypeDef BNO055_readGyro(BNO055 *dev)
{
    int8_t data[2];
    // Get Data from BNO055_REG_GYR_DATA_X
    BNO055_readRegisters(dev, BNO055_REG_GYR_DATA_X_LSB, data, 2);
    dev->GYRO_X = ((int16_t)data[0] << 8) | data[1];

    // Get Data from BNO055_REG_GYR_DATA_Y
    BNO055_readRegisters(dev, BNO055_REG_GYR_DATA_Y_LSB, data, 2);
    dev->GYRO_Y = ((int16_t)data[0] << 8) | data[1];

    // Get Data from BNO055_REG_GYR_DATA_Z
    BNO055_readRegisters(dev, BNO055_REG_GYR_DATA_Z_LSB, data, 2);
    dev->GYRO_Z = ((int16_t)data[0] << 8) | data[1];
}

HAL_StatusTypeDef BNO055_readAccel(BNO055 *dev)
{
    int8_t data[2];
    // Get Data from BNO055_REG_ACC_DATA_X
    BNO055_readRegisters(dev, BNO055_REG_ACC_DATA_X_LSB, data, 2);
    dev->ACC_X = ((int16_t)data[0] << 8) | data[1];

    // Get Data from BNO055_REG_ACC_DATA_Y
    BNO055_readRegisters(dev, BNO055_REG_ACC_DATA_Y_LSB, data, 2);
    dev->ACC_Y = ((int16_t)data[0] << 8) | data[1];

    // Get Data from BNO055_REG_ACC_DATA_Z
    BNO055_readRegisters(dev, BNO055_REG_ACC_DATA_Z_LSB, data, 2);
    dev->ACC_Z = ((int16_t)data[0] << 8) | data[1];
}

HAL_StatusTypeDef BNO055_readLIA(BNO055 *dev)
{
    int8_t data[2];
    // Get Data from BNO055_REG_LIA_DATA_X_LSB
    BNO055_readRegisters(dev, BNO055_REG_LIA_DATA_X_LSB, data, 2);
    dev->LIA_X = ((int16_t)data[0] << 8) | data[1];

    // Get Data from BNO055_REG_LIA_DATA_Y_LSB
    BNO055_readRegisters(dev, BNO055_REG_LIA_DATA_Y_LSB, data, 2);
    dev->LIA_Y = ((int16_t)data[0] << 8) | data[1];

    // Get Data from BNO055_REG_LIA_DATA_Z_LSB
    BNO055_readRegisters(dev, BNO055_REG_LIA_DATA_Z_LSB, data, 2);
    dev->LIA_Z = ((int16_t)data[0] << 8) | data[1];
}

HAL_StatusTypeDef BNO055_readEUL(BNO055 *dev)
{
	uint8_t data[2];
	// Get Data
	BNO055_readRegister(dev, BNO055_REG_EUL_DATA_HEADING_LSB, &data[0]);
	BNO055_readRegister(dev, BNO055_REG_EUL_DATA_HEADING_MSB, &data[1]);
	dev->EUL_HEADING = (((int16_t)data[1] << 8) | data[0])/16.0;
	dev->EUL_HEADING += dev->MAG_DECL;
	// If the heading is greater than 360 subtract
	if(dev->EUL_HEADING > 360)
		dev->EUL_HEADING -= 360;
	// If the heading is less than 360 add
	else if(dev->EUL_HEADING < 0)
		dev->EUL_HEADING += 360;

	BNO055_readRegister(dev, BNO055_REG_EUL_DATA_ROLL_LSB, &data[0]);
	BNO055_readRegister(dev, BNO055_REG_EUL_DATA_ROLL_MSB, &data[1]);
	dev->EUL_ROLL = (((int16_t)data[1] << 8) | data[0])/16.0;

	BNO055_readRegister(dev, BNO055_REG_EUL_DATA_PITCH_LSB, &data[0]);
	BNO055_readRegister(dev, BNO055_REG_EUL_DATA_PITCH_MSB, &data[1]);
	dev->EUL_PITCH = (((int16_t)data[1] << 8) | data[0])/16.0;
}
