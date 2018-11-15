/*
 * ultraSoundDriver.c
 *
 *  Created on: 29.10.2018
 *      Author: PKijanski
 */

#include <stdlib.h>
#include "logger.h"
#include "utils.h"
#include "rtosConfig.h"
#include "ultraSoundDriver.h"
#include <string.h>

#if !defined  (STM32F4)
#include "stm32f4xx_hal.h"
#elif
#error "Wrong STM32 family defined"
#endif /* STM32F4 */

/*
 * @brief Local types and defines
 */

typedef struct
{
	genericTaskConfig_t					taskConfig;
	uint32_t							*timerHandler;
	triggerUltraSoundMeasure_t			trgPin;
	uint32_t							distance;
	uint8_t								cDistance[8];
	osSemaphoreId						distnaceMeasuredSemaphore;
}taskConfig_t;

/**
 * @brief Local Variables
 */
static taskConfig_t taskConfig =
{
	.taskConfig.mainTask		= NULL,
	.taskConfig.instances		= 0,
	.taskConfig.stackSize		= 128,
	.taskConfig.taskPriority	= ULTRA_SOUND_TASK_PRIORITY,
	.taskConfig.taskHandle		= NULL,
	.timerHandler				= (uint32_t *) NULL,
	.trgPin.gpioPin				= 0,
	.trgPin.gpioPort			= 0,
	.distance					= 0,
	.cDistance					= "",
};

static uint8_t measurementDoneCallback = 0;

/**
 * @brief Function definition
 */
static void mainTask(void const * argument);
static void triggerMeasurement();
static void displayLog();
/**
 * @brief Global functions body
 *
 */
/**
 * @brief Creating Task function
 */
result_t createUltraSoundTask(uint32_t *timerHandle, triggerUltraSoundMeasure_t trgPin)
{
	result_t retVal_e = RESULT_SUCCESS;

	do
	{
		if (NULL == timerHandle)
		{
			retVal_e = ERR_ULTRA_SOUND_DRIVER_NOT_VALID_PARAMS;
		}

		if (0 != trgPin.gpioPin && 0 != trgPin.gpioPort)
		{
			taskConfig.trgPin.gpioPin = trgPin.gpioPin;
			taskConfig.trgPin.gpioPort = trgPin.gpioPort;

			HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.trgPin.gpioPort, taskConfig.trgPin.gpioPin, GPIO_PIN_RESET);
		}
		else
		{
			retVal_e = ERR_ULTRA_SOUND_TRIGGER_NOT_SET;
		}

		taskConfig.timerHandler = timerHandle;
		taskConfig.taskConfig.mainTask = mainTask;

		osSemaphoreDef(distnaceMeasuredSemaphore);
		taskConfig.distnaceMeasuredSemaphore = osSemaphoreCreate(osSemaphore(distnaceMeasuredSemaphore), 1);

		osThreadDef(UltraSoundTask,
					taskConfig.taskConfig.mainTask,
					taskConfig.taskConfig.taskPriority,
					taskConfig.taskConfig.instances,
					taskConfig.taskConfig.stackSize);

		taskConfig.taskConfig.taskHandle = osThreadCreate(osThread(UltraSoundTask), NULL);

		if (NULL == taskConfig.taskConfig.taskHandle)
		{
			retVal_e = ERR_ULTRA_SOUND_TASK_CRESTION_ERROR;
		}
	} while(0);

	return retVal_e;
}

result_t measurementDone()
{
	result_t retVal_e = RESULT_SUCCESS;

	measurementDoneCallback = 1;

	return retVal_e;
}

uint32_t getDistance(void)
{
	return taskConfig.distance;
}

void getDistanceText(uint8_t *distance)
{
	memcpy(distance, taskConfig.cDistance, 8);
}

void getDistanceTextTrimmed(uint8_t *distance)
{
	uint8_t		*p = taskConfig.cDistance + sizeof(taskConfig.cDistance) - 1;
	uint8_t		pos = 0;

	while(0 == *p--)
	{
		pos++;
	}

	memcpy(distance + pos, taskConfig.cDistance, 8);
}

/**
 * @brief Function definition body
 */
static void mainTask(void const * argument)
{
	while(1)
	{
		triggerMeasurement();

		while(!measurementDoneCallback)
		{
			osDelay(1);
		}
		measurementDoneCallback = 0;
		taskConfig.distance = (HAL_TIM_ReadCapturedValue((TIM_HandleTypeDef *) taskConfig.timerHandler, TIM_CHANNEL_2) * 34) / 200;

		displayLog();
		osDelay(1000);
	}
}

static void displayLog()
{
	uint8_t		length;
	uint8_t		data2Display[30] = "Distance measured: ";

	memset(taskConfig.cDistance, 0, sizeof(taskConfig.cDistance));
	uint32ToStr(taskConfig.distance, taskConfig.cDistance, &length);
	memcpy(data2Display+19, taskConfig.cDistance, length);

	logMessage(LOG_DEB, (uint8_t *) data2Display);
}

static void triggerMeasurement()
{

	HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.trgPin.gpioPort, taskConfig.trgPin.gpioPin, GPIO_PIN_SET);
	//HAL_TIM_Base_Start_IT((TIM_HandleTypeDef *) taskConfig.timerHandler);
	delayUs(10);
	HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.trgPin.gpioPort, taskConfig.trgPin.gpioPin, GPIO_PIN_RESET);

	HAL_TIM_IC_Start_IT((TIM_HandleTypeDef *) taskConfig.timerHandler ,TIM_CHANNEL_2);
}
