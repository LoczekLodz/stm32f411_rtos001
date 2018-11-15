/*
 * transmitterAM433.c
 *
 *  Created on: 15.11.2018
 *      Author: PKijanski
 */

#include "transmitterAM433.h"
#include "rtosConfig.h"
#include "logger.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "utils.h"
#include "oneDirList.h"

#if defined  __STM32F4_FAMILY__
	#include "stm32f4xx_hal.h"
#elif defined  __STM32F3_FAMILY__
	#include "stm32f3xx_hal.h"
#else
	#error "Wrong STM32 family defined"
#endif /* STM32 FAMILY */


/**
 * @brief Local types and defines
 */
#define MAX_LENGTH_MSG		100

typedef struct
{
	genericTaskConfig_t		taskConfig;
	transmitPinAM433_t		transmitPin;
} taskConfig_t;

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
	.transmitPin.gpioPin  		= 0,
	.transmitPin.gpioPort  		= 0,
};

bufferList_t *firstNodeTransmitterAM433 = NULL;
bufferList_t *lastNodeTransmitterAM433 	= NULL;

/**
 * @brief Function definition
 */
static void mainTask(void const * argument);
static void sendData();
/**
 * @brief Global functions body
 *
 */
/**
 * @brief Creating Task function
 */
result_t createTransmitterAM433DriverTask(transmitPinAM433_t transmitPin)
{
	result_t retVal_e = RESULT_SUCCESS;

	do
	{

		if (0 != transmitPin.gpioPin && 0 != transmitPin.gpioPort)
		{
			taskConfig.transmitPin.gpioPin = transmitPin.gpioPin;
			taskConfig.transmitPin.gpioPort = transmitPin.gpioPort;

			HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.transmitPin.gpioPort, taskConfig.transmitPin.gpioPin, GPIO_PIN_RESET);
		}
		else
		{
			retVal_e = ERR_TRANSMITTER_AM_433_NOT_VALID_PARAMS;
		}

		taskConfig.taskConfig.mainTask = mainTask;

		osThreadDef(TransmitterAM433,
					taskConfig.taskConfig.mainTask,
					taskConfig.taskConfig.taskPriority,
					taskConfig.taskConfig.instances,
					taskConfig.taskConfig.stackSize);

		taskConfig.taskConfig.taskHandle = osThreadCreate(osThread(TransmitterAM433), NULL);

		if (NULL == taskConfig.taskConfig.taskHandle)
		{
			retVal_e = ERR_TRANSMITTER_AM_433_TASK_CRESTION_ERROR;
		}
	} while(0);

	return retVal_e;
}

result_t transmitAM433Message(const uint8_t *data, uint8_t size)
{
	result_t retVal_e = RESULT_SUCCESS;

	uint8_t *buff = (uint8_t *) calloc(size+1, sizeof(uint8_t));

	memcpy(buff, data, size);

	pushNode2Buffer((uint8_t *) buff, firstNodeTransmitterAM433, lastNodeTransmitterAM433);

	return retVal_e;
}

/**
 * @brief Function definition body
 */
static void mainTask(void const * argument)
{
	while(1)
	{
		if (NULL != firstNodeTransmitterAM433)
		{
			sendData();
			if (RESULT_SUCCESS != popNodeFromBuffer(firstNodeTransmitterAM433))
			{
				//break;
			}
		}
		osDelay(100);
	}
}

static void sendData()
{
	uint8_t i;

	while(firstNodeTransmitterAM433->buffer)
	{
		for (i=0;i<8; i++)
		{
			if (1 == (*(firstNodeTransmitterAM433->buffer) & (1<<i)))
			{
				HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.transmitPin.gpioPort, taskConfig.transmitPin.gpioPin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.transmitPin.gpioPort, taskConfig.transmitPin.gpioPin, GPIO_PIN_RESET);
			}
			osDelay(1);
		}
		firstNodeTransmitterAM433->buffer++;
	}
}
