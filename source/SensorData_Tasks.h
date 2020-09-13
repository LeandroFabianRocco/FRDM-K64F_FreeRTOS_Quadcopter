/*
 * SensorData_Tasks.h
 *
 *  Created on: 16 jun. 2020
 *      Author: leandro
 */

#ifndef SENSORDATA_TASKS_H_
#define SENSORDATA_TASKS_H_


#include "fsl_i2c.h"
#include "fsl_i2c_freertos.h"
#include "board.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2C1_IRQN 		I2C1_IRQn
#define I2C1_CLK_FREQ 	CLOCK_GetFreq(I2C1_CLK_SRC)

#define I2C1_BAUDRATE	400000



/*****************************************************************************
 * @brief Task responsible for receive data from Accel/Gyro/Magnet sensors
 ****************************************************************************/
void SensorData_task(void *pvParameters);






#endif /* SENSORDATA_TASKS_H_ */
