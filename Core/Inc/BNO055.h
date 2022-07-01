/*
 * BNO055.h
 *
 *  Created on: Jun 27, 2022
 *      Author: Jordi
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include <stm32f4xx_hal.h>

// Data Sheet
// https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

#define BNO055_ADDRESS_LOW                  0x28                 // COM3_state is low
#define BNO055_ADDRESS_HIGH                 0x29                 // COM3_state is high

// Register Locations

#define BNO055_REG_CHIP_ID                  0x00                 // Read value 0xA0
#define BNO055_REG_ACC_DATA_X_LSB           0x08
#define BNO055_REG_ACC_DATA_X_MSB           0x09
#define BNO055_REG_ACC_DATA_Y_LSB           0x0A
#define BNO055_REG_ACC_DATA_Y_MSB           0x0B
#define BNO055_REG_ACC_DATA_Z_LSB           0x0C
#define BNO055_REG_ACC_DATA_Z_MSB           0x0D
#define BNO055_REG_MAG_DATA_X_LSB           0x0E
#define BNO055_REG_MAG_DATA_X_MSB           0x0F
#define BNO055_REG_MAG_DATA_Y_LSB           0x10
#define BNO055_REG_MAG_DATA_Y_MSB           0x11
#define BNO055_REG_MAG_DATA_Z_LSB           0x12
#define BNO055_REG_MAG_DATA_Z_MSB           0x13
#define BNO055_REG_GYR_DATA_X_LSB           0x14
#define BNO055_REG_GYR_DATA_X_MSB           0x15
#define BNO055_REG_GYR_DATA_Y_LSB           0x16
#define BNO055_REG_GYR_DATA_Y_MSB           0x17
#define BNO055_REG_GYR_DATA_Z_LSB           0x18
#define BNO055_REG_GYR_DATA_Z_MSB           0x19
#define BNO055_REG_EUL_DATA_X_LSB           0x1A
#define BNO055_REG_EUL_DATA_X_MSB           0x1B
#define BNO055_REG_EUL_DATA_Y_LSB           0x1C
#define BNO055_REG_EUL_DATA_Y_MSB           0x1D
#define BNO055_REG_EUL_DATA_Z_LSB           0x1E
#define BNO055_REG_EUL_DATA_Z_MSB           0x1F
#define BNO055_REG_QUA_DATA_W_LSB           0x20
#define BNO055_REG_QUA_DATA_W_MSB           0x21
#define BNO055_REG_QUA_DATA_X_LSB           0x22
#define BNO055_REG_QUA_DATA_X_MSB           0x23
#define BNO055_REG_QUA_DATA_Y_LSB           0x24
#define BNO055_REG_QUA_DATA_Y_MSB           0x25
#define BNO055_REG_QUA_DATA_Z_LSB           0x26
#define BNO055_REG_QUA_DATA_Z_MSB           0x27
#define BNO055_REG_LIA_DATA_X_LSB           0x28
#define BNO055_REG_LIA_DATA_X_MSB           0x29
#define BNO055_REG_LIA_DATA_Y_LSB           0x2A
#define BNO055_REG_LIA_DATA_Y_MSB           0x2B
#define BNO055_REG_LIA_DATA_Z_LSB           0x2C
#define BNO055_REG_LIA_DATA_Z_MSB           0x2D
#define BNO055_REG_GRV_DATA_X_LSB           0x2E
#define BNO055_REG_GRV_DATA_X_MSB           0x2F
#define BNO055_REG_GRV_DATA_Y_LSB           0x30
#define BNO055_REG_GRV_DATA_Y_MSB           0x31
#define BNO055_REG_GRV_DATA_Z_LSB           0x32
#define BNO055_REG_GRV_DATA_Z_MSB           0x33
#define BNO055_REG_TEMP                     0x34
#define BNO055_REG_CALIB_STAT               0x35
#define BNO055_REG_ST_RESULT                0x36
#define BNO055_REG_INT_STA                  0x37
#define BNO055_REG_SYS_CLK_STATUS           0x38
#define BNO055_REG_SYS_STATUS               0x39
#define BNO055_REG_SYS_ERR                  0x3A
#define BNO055_REG_UNIT_SEL                 0x3B
#define BNO055_REG_OPR_MODE                 0x3D
#define BNO055_REG_PWR_MODE                 0x3E
#define BNO055_REG_SYS_TRIGGER              0x3F
#define BNO055_REG_TEMP_SOURCE              0x40
#define BNO055_REG_AXIS_MAP_CONFIG          0x41
#define BNO055_REG_AXIS_MAP_SIGN            0x42
#define BNO055_REG_ACC_OFFSET_X_LSB         0x55
#define BNO055_REG_ACC_OFFSET_X_MSB         0x56
#define BNO055_REG_ACC_OFFSET_Y_LSB         0x57
#define BNO055_REG_ACC_OFFSET_Y_MSB         0x58
#define BNO055_REG_ACC_OFFSET_Z_LSB         0x59
#define BNO055_REG_ACC_OFFSET_Z_MSB         0x5A
#define BNO055_REG_MAG_OFFSET_X_LSB         0x5B
#define BNO055_REG_MAG_OFFSET_X_MSB         0x5C
#define BNO055_REG_MAG_OFFSET_Y_LSB         0x5D
#define BNO055_REG_MAG_OFFSET_Y_MSB         0x5E
#define BNO055_REG_MAG_OFFSET_Z_LSB         0x5F
#define BNO055_REG_MAG_OFFSET_Z_MSB         0x60
#define BNO055_REG_GYR_OFFSET_X_LSB         0x61
#define BNO055_REG_GYR_OFFSET_X_MSB         0x62
#define BNO055_REG_GYR_OFFSET_Y_LSB         0x63
#define BNO055_REG_GYR_OFFSET_Y_MSB         0x64
#define BNO055_REG_GYR_OFFSET_Z_LSB         0x65
#define BNO055_REG_GYR_OFFSET_Z_MSB         0x66
#define BNO055_REG_ACC_RADIUS_LSB           0x67
#define BNO055_REG_ACC_RADIUS_MSB           0x68
#define BNO055_REG_MAG_RADIUS_LSB           0x69
#define BNO055_REG_MAG_RADIUS_MSB           0x6A
#define BNO055_REG_ACC_Config               0x08
#define BNO055_REG_MAG_Config               0x09
#define BNO055_REG_GYR_Config_0             0x0A
#define BNO055_REG_GYR_Config_1             0x0B
#define BNO055_REG_ACC_Sleep_Config         0x0C
#define BNO055_REG_GYR_Sleep_Config         0x0D
#define BNO055_REG_INT_MSK                  0x0F
#define BNO055_REG_INT_EN                   0x10
#define BNO055_REG_ACC_AM_THRES             0x11
#define BNO055_REG_ACC_INT_Settings         0x12
#define BNO055_REG_ACC_HG_DURATION          0x13
#define BNO055_REG_ACC_HG_THRES             0x14
#define BNO055_REG_ACC_NM_THRES             0x15
#define BNO055_REG_ACC_NM_SET               0x16
#define BNO055_REG_GYR_INT_SETTING          0x17
#define BNO055_REG_GYR_HR_X_SET             0x18
#define BNO055_REG_GYR_DUR_X                0x19
#define BNO055_REG_GYR_HR_Y_SET             0x1A
#define BNO055_REG_GYR_DUR_Y                0x1B
#define BNO055_REG_GYR_HR_Z_SET             0x1C
#define BNO055_REG_GYR_DUR_Z                0x1D
#define BNO055_REG_GYR_AM_THRES             0x1E
#define BNO055_REG_GYR_AM_SET               0x1F

typedef struct
{
    // I2C handle
	I2C_HandleTypeDef *i2cHandle;

	uint8_t ADDRESS;

	int GYRO_X;
	int GYRO_Y;
	int GYRO_Z;

	int ACC_X;
	int ACC_Y;
	int ACC_Z;

	int MAG_X;
	int MAG_Y;
	int MAG_Z;

}BNO055;

// Initializer
// Returns 0 if no errors
uint8_t BNO055_Init(BNO055 *dev, uint8_t ADR_PIN, I2C_HandleTypeDef *i2cHandle);

// Read register
HAL_StatusTypeDef BNO055_readRegister(BNO055 *dev, uint8_t reg, uint8_t *data);

// Read registers
HAL_StatusTypeDef BNO055_readRegisters(BNO055 *dev, uint8_t reg, uint8_t *data, uint8_t size);    

// Write register
HAL_StatusTypeDef BNO055_writeRegister(BNO055 *dev, uint8_t reg, uint8_t *data);

// Write registers
HAL_StatusTypeDef BNO055_writeRegisters(BNO055 *dev, uint8_t reg, uint8_t *data, uint8_t size);

// Read Magnetic field strength
HAL_StatusTypeDef BNO055_readMAG(BNO055 *dev);

#endif /* INC_BNO055_H_ */
