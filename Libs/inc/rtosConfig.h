/*
 * rtosConfig.h
 *
 *  Created on: 24.10.2018
 *      Author: PKijanski
 */

#ifndef UTLIS_RTOSCONFIG_H_
#define UTLIS_RTOSCONFIG_H_

#include "cmsis_os.h"

#define LED_DRIVER_TASK_PRIORITY			osPriorityLow
#define LOGGER_TASK_PRIORITY				osPriorityNormal
#define ULTRA_SOUND_TASK_PRIORITY			osPriorityLow
#define JOYSTICK_DRIVER_TASK_PRIORITY		osPriorityLow
#define LCD_NOKIA__DRIVER_TASK_PRIORITY		osPriorityLow

#define OS_ACTION_TIMEOUT					1000

typedef void (*taskFunctionHandle_t)(void const * argument);

typedef struct
{
	taskFunctionHandle_t		mainTask;
	uint8_t						instances;
	uint8_t						stackSize;
	int8_t						taskPriority;
	osThreadId					taskHandle;
}genericTaskConfig_t;

#endif /* UTLIS_RTOSCONFIG_H_ */
