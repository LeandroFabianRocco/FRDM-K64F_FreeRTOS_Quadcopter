/*
 * SensorData_Tasks.c
 *
 *  Created on: 16 jun. 2020
 *      Author: leandro
 */

#include <SensorData_Tasks.h>
#include "MPU6050.h"
#include "PIDcontroller.h"




/*****************************************************************************
 * @brief Task responsible for receive data from Accel/Gyro/Magnet sensors
 ****************************************************************************/
void SensorData_task(void *pvParameters)
{
    i2c_rtos_handle_t master_rtos_handle;
    i2c_master_config_t masterConfig;
    //i2c_master_transfer_t masterXfer;
    uint32_t sourceClock;
    status_t status;

    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C1_BAUDRATE;
    sourceClock               = I2C1_CLK_FREQ;

    masterConfig.enableStopHold = true;

    status = I2C_RTOS_Init(&master_rtos_handle, I2C1, &masterConfig, sourceClock);
    if (status != kStatus_Success)
    {
        PRINTF("I2C master: error during init, %d", status);
    }


    status = MPU6050_Configure_Device(&master_rtos_handle);
    if (status != true)
	{
		PRINTF("I2C master: error during configuration MPU6050 mmodule, %d", status);
	}


    PRINTF("I2C1 module initialized!\r\n");
    const TickType_t xDelay250ms = pdMS_TO_TICKS(250);
    bool isOK;
    isOK = MPU6050_ReadSensorWhoAmI(&master_rtos_handle);
	if (isOK)
		PRINTF("WHO AM I received!!\r\n");
	else
		PRINTF("No device found!!\r\n");

	float xyz_gravity[3];
	float xyz_omega[3];
    for (;;)
    {
    	GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_PIN);
		GPIO_PortClear(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_PIN);
		GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_PIN);
		PRINTF("---> SensorData_Task!!\r\n");

    	MPU6050_GetgAcceleration(&master_rtos_handle, xyz_gravity);
    	MPU6050_GetAngularVelocity(&master_rtos_handle, xyz_omega);

    	PRINTF("Gxyz = [%.3f, %.3f, %.3f]; Wxyz = [%.3f, %.3f, %.3f]\r\n",
    			xyz_gravity[0], xyz_gravity[1], xyz_gravity[2],
				xyz_omega[0], xyz_omega[1], xyz_omega[2]);


    	vTaskDelay(xDelay250ms);
    }
}






