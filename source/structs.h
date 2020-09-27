/*
 * structs.h
 *
 *  Created on: 13 sep. 2020
 *      Author: leandro
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

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
	float ePidPitch;
	DataValid_t evPitch;

	float ePidRoll;
	DataValid_t evRoll;

	float ePidYaw;
	DataValid_t evYaw;

	uint8_t eJoystick;
	uint8_t eThrottle;
	DataValid_t evJandT;
} Attitude_Joystick_Data_t;

/*******************************************************************************
 * Pitch and Roll structures definition
 ******************************************************************************/
struct pitchStruct{
	float reference;
	float angle;
	float last_iError;
	float last_pError;
	float dt;
};


struct rollStruct{
	float reference;
	float angle;
	float last_iError;
	float last_pError;
	float dt;
};


// Declaration of handle to update motors
extern QueueHandle_t motors_queue;


#endif /* STRUCTS_H_ */
