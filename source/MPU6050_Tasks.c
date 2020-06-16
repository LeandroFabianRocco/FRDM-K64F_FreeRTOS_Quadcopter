/*
 * MPU6050_Tasks.c
 *
 *  Created on: 16 jun. 2020
 *      Author: leandro
 */


static void master_task(void *pvParameters)
{
    i2c_rtos_handle_t master_rtos_handle;
    i2c_master_config_t masterConfig;
    i2c_master_transfer_t masterXfer;
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

    status = I2C_RTOS_Init(&master_rtos_handle, EXAMPLE_I2C_MASTER, &masterConfig, sourceClock);
    if (status != kStatus_Success)
    {
        PRINTF("I2C master: error during init, %d", status);
    }

    for (;;)
    {

    }
}
