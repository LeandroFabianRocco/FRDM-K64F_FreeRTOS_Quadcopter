/*
 * MPU6050.c
 *
 *  Created on: 6 sep. 2020
 *      Author: leandro
 */


#include "MPU6050.h"



/*********************************************************************************************
 * @brief Gets the WHO_AM_I value
 *
 * @param I2C RTOS handle
 *
 * @return status flag. Return true if no error
 *********************************************************************************************/
bool MPU6050_ReadSensorWhoAmI(i2c_rtos_handle_t *master_handle)
{
	status_t status;
	uint8_t who_am_i_reg          = MPU6050_WHO_AM_I;
	uint8_t who_am_i_value        = 0x00;
	i2c_master_transfer_t masterXfer;
	memset(&masterXfer, 0, sizeof(masterXfer));

	// START + Slave_address (write_bit); Reg_address
	masterXfer.slaveAddress   = MPU6050_DEVICE_ADDRESS_0;
	masterXfer.direction      = kI2C_Write;
	masterXfer.subaddress     = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data           = &who_am_i_reg;
	masterXfer.dataSize       = 1;
	masterXfer.flags          = kI2C_TransferNoStopFlag;

	status = I2C_RTOS_Transfer(master_handle, &masterXfer);
	if (status != kStatus_Success)
	{
		PRINTF("I2C master: error during write transaction, %d", status);
		return false;
	}


	// START + Slave_address (read_bit); recibo dato
	masterXfer.direction      = kI2C_Read;
	masterXfer.subaddress     = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data           = &who_am_i_value;
	masterXfer.dataSize       = 1;
	masterXfer.flags          = kI2C_TransferRepeatedStartFlag;
	status = I2C_RTOS_Transfer(master_handle, &masterXfer);
	if (status != kStatus_Success)
	{
		PRINTF("I2C master: error during write transaction, %d", status);
		return false;
	}


	return true;
}



/*********************************************************************************************
 * @brief Configure MPU6050 module
 *
 * @param void
 *
 * @return void
 *********************************************************************************************/
bool MPU6050_Configure_Device(i2c_rtos_handle_t *master_handle)
{
	uint8_t databyte  = 0;
	uint8_t write_reg = 0;

	bool success;

	// Configuration register
	write_reg = MPU6050_CONFIG;
	databyte  = 4;
	success = MPU6050_WriteAccelReg(master_handle, MPU6050_DEVICE_ADDRESS_0, write_reg, databyte);
	if (success == false)
	{
		PRINTF("Error in configuration step 1");
	}

	// Power mode and clock source
	write_reg = MPU6050_PWR_MGMT_1;
	databyte  = 0;
	success = MPU6050_WriteAccelReg(master_handle, MPU6050_DEVICE_ADDRESS_0, write_reg, databyte);
	if (success == false)
	{
		PRINTF("Error in configuration step 2");
	}

	// Gyro - +-250°/s
	write_reg = MPU6050_GYRO_CONFIG;
	databyte  = 0;
	success = MPU6050_WriteAccelReg(master_handle, MPU6050_DEVICE_ADDRESS_0, write_reg, databyte);
	if (success == false)
	{
		PRINTF("Error in configuration step 3");
	}


	// Accel - +-2g
	write_reg = MPU6050_ACCEL_CONFIG;
	databyte  = 0;
	success = MPU6050_WriteAccelReg(master_handle, MPU6050_DEVICE_ADDRESS_0, write_reg, databyte);
	if (success == false)
	{
		PRINTF("Error in configuration step 4");
	}

	return true;
}




/*********************************************************************************************
 * @brief Write sensor register
 *
 * @param base I2C peripheral base address.
 * @param sensor device address
 * @param register address
 * @param data value address
 *
 * @return status flag. True if success
 *********************************************************************************************/
bool MPU6050_WriteAccelReg(i2c_rtos_handle_t *master_handle, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
	i2c_master_transfer_t masterXfer;
	status_t status;
	memset(&masterXfer, 0, sizeof(masterXfer));

	masterXfer.slaveAddress   = device_addr;
	masterXfer.direction      = kI2C_Write;
	masterXfer.subaddress     = reg_addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data           = &value;
	masterXfer.dataSize       = 1;
	masterXfer.flags          = kI2C_TransferDefaultFlag;

	//I2C_MasterTransferNonBlocking(I2C1, &mpu_g_m_handle, &masterXfer);

	status = I2C_RTOS_Transfer(master_handle, &masterXfer);
	if (status != kStatus_Success)
	{
		PRINTF("I2C master: error during write transaction, %d", status);
		return false;
	}

	return true;
}
