/*
 * joystickDriver.c
 *
 *  Created on: 30.10.2018
 *      Author: PKijanski
 */

#include "joystickDriver.h"
#include "rtosConfig.h"
#include "logger.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "utils.h"

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

typedef struct
{
	genericTaskConfig_t		taskConfig;
	uint32_t				*adcHandle;
	uint32_t				adcData;
	uint8_t					channel1[4];
	uint8_t					channel2[4];
} taskConfig_t;

/**
 * @brief Local Variables
 */
static taskConfig_t taskConfig =
{
	.taskConfig.mainTask		= NULL,
	.taskConfig.instances		= 0,
	.taskConfig.stackSize		= 128,
	.taskConfig.taskPriority	= JOYSTICK_DRIVER_TASK_PRIORITY,
	.taskConfig.taskHandle		= NULL,
	.adcHandle					= NULL,
	.adcData					= 0,
	.channel1					= "",
	.channel2					= "",
};

uint8_t buf[25]  = "ADC value is: ";

/**
 * @brief Function definition
 */
static void mainTask(void const * argument);
static void displayADCLog();
/**
 * @brief Global functions body
 *
 */
result_t createJoystickDriverTask(uint32_t *adcHandle)
{
	result_t retVal_e = RESULT_SUCCESS;

	do
	{
		if (NULL == adcHandle)
		{
			retVal_e = ERR_JOYSTICK_DRIVER_NO_ADC_PROVIDED;
		}

		taskConfig.taskConfig.mainTask = mainTask;
		taskConfig.adcHandle = adcHandle;

		osThreadDef(JoystickDriver,
					taskConfig.taskConfig.mainTask,
					taskConfig.taskConfig.taskPriority,
					taskConfig.taskConfig.instances,
					taskConfig.taskConfig.stackSize);

		taskConfig.taskConfig.taskHandle = osThreadCreate(osThread(JoystickDriver), NULL);

		if (NULL == taskConfig.taskConfig.taskHandle)
		{
			retVal_e = ERR_LED_DRIVER_TASK_CRESTION_ERROR;
			continue;
		}
	} while(0);

	return retVal_e;
}

void getJoystickPositionText(uint8_t *posX, uint8_t *posY)
{
	memcpy(posX, taskConfig.channel1, 4);
	memcpy(posY, taskConfig.channel2, 4);
}

void getJoystickPositionTextTrimmed(uint8_t *posX, uint8_t *posY)
{
	uint8_t		*p = taskConfig.channel1 + sizeof(taskConfig.channel1) - 1;
	uint8_t		pos = 0;

	while(0 == *p--)
	{
		pos++;
	}

	memcpy(posX+pos, taskConfig.channel1, 4);

	pos = 0;
	p = taskConfig.channel2 + sizeof(taskConfig.channel2) - 1;
	while(0 == *p--)
	{
		pos++;
	}
	memcpy(posY+pos, taskConfig.channel2, 4);
}

/**
 * @brief Function definition body
 */
static void mainTask(void const * argument)
{
	while(1)
	{

		displayADCLog();
		logMessage(LOG_INF, (uint8_t *) buf );

		HAL_ADC_Start_DMA((ADC_HandleTypeDef *) taskConfig.adcHandle, (uint32_t *) &taskConfig.adcData, 4);

		osDelay(500);
	}
}

static void displayADCLog()
{
	uint16_t channel1;
	uint16_t channel2;

	uint8_t adcLength = 0;
	uint8_t	strLength = 14;

	memset(taskConfig.channel1, 0, sizeof(taskConfig.channel1));
	memset(taskConfig.channel2, 0, sizeof(taskConfig.channel2));

	channel1 = (uint16_t) ((taskConfig.adcData & 0xffff0000) >> 16);
	channel2 = (uint16_t) (taskConfig.adcData & 0x0000ffff);

	uint16ToStr(channel1, taskConfig.channel1, &adcLength);
	memcpy(buf + strLength, taskConfig.channel1, adcLength);
	strLength += adcLength;

	memcpy(buf + strLength,  " ", 1);
	strLength++;

	uint16ToStr(channel2, taskConfig.channel2, &adcLength);
	memcpy(buf + strLength, taskConfig.channel2, adcLength);
	strLength += adcLength;

	memcpy(buf + strLength,  "\0", 1);
}
