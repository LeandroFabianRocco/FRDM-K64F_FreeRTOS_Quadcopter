/*
 * MotorControl_Tasks.c
 *
 *  Created on: 6 sep. 2020
 *      Author: leandro
 */


#include "MotorControl_Tasks.h"



/*********************************************************************************************
 * @brief Set the CnV field for PWM channels
 *
 * @param FTM peripheral base address.
 * @param value received from Quadcopter control
 * @param PWM channel
 *
 * @return void
 *********************************************************************************************/
void set_pwm_CnV(FTM_Type *base, int32_t value, uint8_t ch)
{
	if (value > CnV_MAX)
		value = CnV_MAX;
	if (value < CnV_MIN)
		value = CnV_MIN;
	base->CONTROLS[ch].CnV = value;
	FTM_SetSoftwareTrigger(base, true);
}


/*********************************************************************************************
 * @brief FTM channels initialization
 *
 * @param FTM module pointer
 *
 * @return void
 *********************************************************************************************/
void FTM0_init(FTM_Type *base)
{
	ftm_chnl_pwm_signal_param_t ftmParam[4];
	ftm_config_t ftmInfo;

	// 400Hz
	uint8_t duty = 40U;
	uint16_t period = 400U;

	// 250Hz
	/*uint8_t duty = 25U;
	uint16_t period = 250U;*/

	// 50Hz
	/*uint8_t duty = 5U;
	uint8_t period = 50U;*/

	ftmParam[0].chnlNumber            = (ftm_chnl_t)PWM_CH0;
	ftmParam[0].level                 = kFTM_HighTrue;
	ftmParam[0].dutyCyclePercent      = duty;
	ftmParam[0].firstEdgeDelayPercent = 0U;
	ftmParam[0].enableDeadtime        = false;

	ftmParam[1].chnlNumber            = (ftm_chnl_t)PWM_CH1;
	ftmParam[1].level                 = kFTM_HighTrue;
	ftmParam[1].dutyCyclePercent      = duty;
	ftmParam[1].firstEdgeDelayPercent = 0U;
	ftmParam[1].enableDeadtime        = false;

	ftmParam[2].chnlNumber            = (ftm_chnl_t)PWM_CH2;
	ftmParam[2].level                 = kFTM_HighTrue;
	ftmParam[2].dutyCyclePercent      = duty;
	ftmParam[2].firstEdgeDelayPercent = 0U;
	ftmParam[2].enableDeadtime        = false;

	ftmParam[3].chnlNumber            = (ftm_chnl_t)PWM_CH3;
	ftmParam[3].level                 = kFTM_HighTrue;
	ftmParam[3].dutyCyclePercent      = duty;
	ftmParam[3].firstEdgeDelayPercent = 0U;
	ftmParam[3].enableDeadtime        = false;

	FTM_GetDefaultConfig(&ftmInfo);
	FTM_Init(base, &ftmInfo);
	FTM_SetupPwm(base, ftmParam, 4U, kFTM_EdgeAlignedPwm, period, FTM_SOURCE_CLOCK);
	FTM_StartTimer(base, kFTM_FixedClock);
}

/*******************************************************************************
 * Throttle to CnV value
 ******************************************************************************/
float throttle2CnV(uint8_t throttle)
{
	float x1 = (float)throttle * (CnV_MAX - CnV_MIN) * 0.01 + CnV_MIN;
	//return (float)(throttle * (CnV_MAX - CnV_MIN) * 0.01 + CnV_MIN);
	return x1;
}



/*********************************************************************************************
 * @brief Task to control the four Brushless Motors
 *
 * @param
 *
 * @return void
 *********************************************************************************************/
void ControllingMotors_task(void *pvParameters)
{


	Attitude_Joystick_Data_t motor_data;

    for (;;)
    {
    	GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_PIN);
		GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_PIN);
		GPIO_PortClear(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_PIN);
		PRINTF("---> Motors_Task!!\r\n");


		if (xQueueReceive(motors_queue, &motor_data, portMAX_DELAY) != pdTRUE)
		{
			PRINTF("Failed to receive queue.\r\n");
		}

		if (motor_data.evPitch == TRUE)
		{

		}

		if (motor_data.evRoll == TRUE)
		{

		}

		if (motor_data.evYaw == TRUE)
		{

		}

		if (motor_data.evJoystick == TRUE)
		{

		}

		if (motor_data.evThrottle == TRUE)
		{

		}
    }
}




