/*
 * MotorControl_Tasks.h
 *
 *  Created on: 6 sep. 2020
 *      Author: leandro
 */

#ifndef MOTORCONTROL_TASKS_H_
#define MOTORCONTROL_TASKS_H_



/*************************************************************************
 * Includes
 ************************************************************************/
#include "fsl_ftm.h"
#include "stdint.h"
#include "fsl_debug_console.h"
#include "stdint.h"
#include "pin_mux.h"
#include "fsl_gpio.h"


/*************************************************************************
 * Definitions
 ************************************************************************/
#define FTM_MODULE FTM0
#define PWM_CH0  0U
#define PWM_CH1  1U
#define PWM_CH2  2U
#define PWM_CH3  3U
#define LOOPTIME 4U

#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_McgFixedFreqClk)


#define CnV_MAX	3125U
#define CnV_MIN	1562U


/*********************************************************************************************
 * @brief Set the CnV field for PWM channels
 *
 * @param FTM peripheral base address.
 * @param value received from Quadcopter control
 * @param PWM channel
 *
 * @return void
 *********************************************************************************************/
void set_pwm_CnV(FTM_Type *base, int32_t value, uint8_t ch);



/*********************************************************************************************
 * @brief FTM channels initialization
 *
 * @param FTM module pointer
 *
 * @return void
 *********************************************************************************************/
void FTM0_init(FTM_Type *base);


/*******************************************************************************
 * Throttle to CnV value
 ******************************************************************************/
float throttle2CnV(uint8_t throttle);


/*********************************************************************************************
 * @brief Task to control the four Brushless Motors
 *
 * @param
 *
 * @return void
 *********************************************************************************************/
void ControllingMotors_task(void *pvParameters);



#endif /* MOTORCONTROL_TASKS_H_ */


