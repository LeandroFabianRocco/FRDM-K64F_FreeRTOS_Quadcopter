/*
 * structs.h
 *
 *  Created on: 13 sep. 2020
 *      Author: leandro
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_

/*********************************************************************
 * Structures and enumerations
 ********************************************************************/
// Enumeration for data valid
typedef enum
{
	TRUE,
	FALSE
} DataValid_t;

// Structure for attitude and joystick data
typedef struct
{
	float ePidPITCH;
	DataValid_t evPitch;

	float ePidRoll;
	DataValid_t evRoll;

	float ePidYaw;
	DataValid_t evYaw;

	uint8_t eJoystick;
	DataValid_t evJoystick;

	uint8_t eThrottle;
	DataValid_t evThrottle;
} Attitude_Joystick_Data_t;



extern QueueHandle_t motors_queue = NULL;


#endif /* STRUCTS_H_ */
