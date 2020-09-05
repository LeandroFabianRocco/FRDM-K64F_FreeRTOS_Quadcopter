/*
 * MPU6050_Tasks.c
 *
 *  Created on: 16 jun. 2020
 *      Author: leandro
 */

#include "MPU6050_Tasks.h"




/*****************************************************************************
 * @brief Task responsible for receive data from I2C module
 ****************************************************************************/
void I2C1_master_task(void *pvParameters)
{
    i2c_rtos_handle_t master_rtos_handle;
    i2c_master_config_t masterConfig;
    //i2c_master_transfer_t masterXfer;
    uint32_t sourceClock;
    status_t status;

    /*
     * masterConfig.baudRate_Bps = 100000U;
     * masterConfig.enableStopHold = false;
     * masterConfig.glitchFilterWidth = 0U;
     * masterConfig.enableMaster = true;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C1_BAUDRATE;
    sourceClock               = I2C1_CLK_FREQ;

    masterConfig.enableStopHold = true;

    status = I2C_RTOS_Init(&master_rtos_handle, I2C1, &masterConfig, sourceClock);
    if (status != kStatus_Success)
    {
        PRINTF("I2C master: error during init, %d", status);
    }


    PRINTF("I2C1 module initialized!\r\n");
    const TickType_t xDelay250ms = pdMS_TO_TICKS(250);
    bool isOK;
    for (;;)
    {
    	isOK = MPU6050_ReadSensorWhoAmI(&master_rtos_handle);
    	if (isOK)
    		PRINTF("WHO AM I received!!\r\n");
    	else
    		PRINTF("No device found!!\r\n");
    	vTaskDelay(xDelay250ms);
    }
}





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
