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

	// Variables to controlling the BLDC motors
	volatile float Mfront, Mfront_last;	// Front motor
	volatile float Mleft, Mleft_last;	// Left motor
	volatile float Mback, Mback_last;	// Back motor
	volatile float Mright, Mright_last;	// Right motor

	uint8_t throttle = 0;
	uint8_t joystick = 0;
	float pitch = 0.0, roll = 0.0, yaw = 0.0;

    for (;;)
    {
    	GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_PIN);
		GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_PIN);
		GPIO_PortClear(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_PIN);
		//PRINTF("---> Motors_Task!!\r\n");


		if (xQueueReceive(motors_queue, &motor_data, portMAX_DELAY) != pdTRUE)
		{
			PRINTF("Failed to receive queue.\r\n");
		}


		// Update pitch, roll and yaw PID signals
		if (motor_data.evPitch == TRUE)
			pitch = motor_data.ePidPitch;
		if (motor_data.evRoll == TRUE)
			roll = motor_data.ePidRoll;
		if (motor_data.evYaw == TRUE)
			yaw = motor_data.ePidYaw;

		// Update joystick and throttle signals
		if (motor_data.evJandT == TRUE)
		{
			joystick = motor_data.eJoystick;
			throttle = motor_data.eThrottle;
			PRINTF("throttle = %d\r\n", throttle);
		}


		// Update motor signals
		Mfront = throttle + pitch - yaw;
		Mback = throttle - pitch - yaw;
		Mleft = throttle - roll + yaw;
		Mright = throttle + roll + yaw;



		PRINTF("[front, back, left, right] = [%.3f, %.3f, %.3f, %.3f]\r\n",
				Mfront, Mback, Mleft, Mright);



		if (Mfront_last != Mfront)
		{
			set_pwm_CnV(FTM0, Mfront, PWM_CH0);
			Mfront_last = Mfront;
		}

		if (Mback_last != Mback)
		{
			set_pwm_CnV(FTM0, Mback, PWM_CH2);
			Mback_last = Mback;
		}

		if (Mleft_last != Mleft)
		{
			set_pwm_CnV(FTM0, Mleft, PWM_CH1);
			Mleft_last = Mleft;
		}

		if (Mright_last != Mright)
		{
			set_pwm_CnV(FTM0, Mright, PWM_CH3);
			Mright_last = Mright;
		}
    }
}




