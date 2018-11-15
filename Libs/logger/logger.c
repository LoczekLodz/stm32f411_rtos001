/*
 * logger.c
 *
 *  Created on: 24.10.2018
 *      Author: PKijanski
 */

#include <string.h>
#include <stdlib.h>
#include "logger.h"
#include "rtosConfig.h"
#include "oneDirList.h"
#include "main.h"

#if defined  __STM32F4_FAMILY__
	#include "stm32f4xx_hal.h"
#elif defined  __STM32F3_FAMILY__
	#include "stm32f3xx_hal.h"
#else
	#error "Wrong STM32 family defined"
#endif /* STM32 FAMILY */

#define MAX_LENGTH_MSG_HEADER			16
#define MAX_LENGTH_MSG_WITHOUT_HEADER	240

#define header(str) "[" #str "] | "

/**
 * @brief Local types and defines
 */
typedef struct
{
	genericTaskConfig_t		taskConfig;
	uint32_t				*usartHandle;
}taskConfig_t;


/**
 * @brief Local Variables
 */
static taskConfig_t taskConfig =
{
	.taskConfig.mainTask		= NULL,
	.taskConfig.instances		= 0,
	.taskConfig.stackSize		= 255,
	.taskConfig.taskPriority	= LOGGER_TASK_PRIORITY,
	.taskConfig.taskHandle		= NULL,
	.usartHandle				= (uint32_t *) NULL
};

static bufferList_t *firstNode = NULL;
static bufferList_t *lastNode = NULL;

/**
 * @brief Function definition
 */
static void mainTask(void const * argument);

/**
 * @brief Global functions body
 *
 */
/**
 * @brief Creating Task function
 */
result_t createLoggerTask(uint32_t *usartHandle)
{
	result_t retVal_e = RESULT_SUCCESS;

	do
	{
		if (NULL == usartHandle)
		{
			retVal_e = ERR_LOGGER_NO_USART_PROVIDED;
		}

		taskConfig.usartHandle = usartHandle;
		taskConfig.taskConfig.mainTask = mainTask;

		osThreadDef(LoggerTask,
					taskConfig.taskConfig.mainTask,
					taskConfig.taskConfig.taskPriority,
					taskConfig.taskConfig.instances,
					taskConfig.taskConfig.stackSize);

		taskConfig.taskConfig.taskHandle = osThreadCreate(osThread(LoggerTask), NULL);

		if (NULL == taskConfig.taskConfig.taskHandle)
		{
			retVal_e = ERR_LOGGER_TASK_CRESTION_ERROR;
		}
	} while(0);

	return retVal_e;
}

static void mainTask(void const * argument)
{
	while(1)
	{
		if (NULL != firstNode)
		{

			HAL_UART_Transmit((UART_HandleTypeDef *) taskConfig.usartHandle, (uint8_t *) firstNode->buffer, (uint16_t) strlen((const char *)firstNode->buffer), 1000);
			if (RESULT_SUCCESS != popNodeFromBuffer(firstNode))
			{
				//break;
			}
		}

		osDelay(20);
	}
}

result_t logMessage(logSeverity_e severityLevel, const uint8_t *format)
{
	result_t retVal_e = RESULT_SUCCESS;

	uint8_t *buffHeader = (uint8_t *) calloc(MAX_LENGTH_MSG_HEADER, sizeof(uint8_t));
	uint8_t *destBuffer;

	uint16_t headerSize = 0;
	uint16_t buffSize = 0;
	//va_list args;

	if (LOG_INF == severityLevel)
	{
		buffHeader = (uint8_t *) header(INFO);
	}
	else if (LOG_DEB == severityLevel)
	{
		buffHeader = (uint8_t *) header(DEBUG);
	}
	else if (LOG_WARN == severityLevel)
	{
		buffHeader = (uint8_t *) header(WARNING);
	}
	else if (LOG_ERR == severityLevel)
	{
		buffHeader = (uint8_t *) header(ERROR);
	}
	headerSize = strlen((char *) buffHeader);

	//va_start (args, format);
	//vsnprintf ((char *) (buffer), MAX_LENGTH_MSG_WITHOUT_HEADER, (const char *) format, args);
	//va_end (args);
	buffSize = strlen((char *) format);

	destBuffer = (uint8_t *) malloc(buffSize + headerSize + 2);
	memcpy(destBuffer, buffHeader, headerSize);
	memcpy(destBuffer+headerSize, format, buffSize);
	memcpy(destBuffer+buffSize+headerSize, "\n\0", 2);

	pushNode2Buffer((uint8_t *) destBuffer, firstNode, lastNode);

	return retVal_e;
}
