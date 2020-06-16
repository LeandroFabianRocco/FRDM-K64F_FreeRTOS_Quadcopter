/*
 * MPU6050_Tasks.h
 *
 *  Created on: 16 jun. 2020
 *      Author: leandro
 */

#ifndef MPU6050_TASKS_H_
#define MPU6050_TASKS_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2C_BASE 		I2C1_BASE
#define I2C_IRQN 		I2C1_IRQn
#define I2C_CLK_SRC 	I2C1_CLK_SRC
#define I2C_CLK_FREQ 	CLOCK_GetFreq(I2C1_CLK_SRC)

#define I2C1_BAUDRATE	400000




#endif /* MPU6050_TASKS_H_ */
