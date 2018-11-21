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
#include <stdarg.h>
#include <stdio.h>

#if defined  __STM32F4_FAMILY__
	#include "stm32f4xx_hal.h"
#elif defined  __STM32F3_FAMILY__
	#include "stm32f3xx_hal.h"
#else
	#error "Wrong STM32 family defined"
#endif /* STM32 FAMILY */

#define MAX_LENGTH_MSG_HEADER			16
#define MAX_LENGTH_MSG_WITHOUT_HEADER	240
#define LOG_HEAD_INFO_LENGTH			((uint16_t) 9)
#define LOG_HEAD_DEBUG_LENGTH			((uint16_t) 10)
#define LOG_HEAD_WARNING_LENGTH			((uint16_t) 12)
#define LOG_HEAD_ERROR_LENGTH			((uint16_t) 10)

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

static uint8_t buffHeader[MAX_LENGTH_MSG_HEADER];

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
			if (RESULT_SUCCESS != popNodeFromBuffer(&firstNode))
			{
				//break;
			}
		}

		osDelay(20);
	}
}

result_t logMessage(logSeverity_e severityLevel, const uint8_t *format, ...)
{
	result_t retVal_e = RESULT_SUCCESS;

	uint8_t *destBuffer = NULL;
	uint16_t buffSize = strlen((char *) format);
	uint16_t headerSize = 0;
	uint16_t overalBuffSize = 18 + buffSize;

	memset(buffHeader, 0, MAX_LENGTH_MSG_HEADER);

	if (LOG_INF == severityLevel)
	{
		memcpy(buffHeader, (uint8_t *)"[INFO] | ", LOG_HEAD_INFO_LENGTH);
	}
	else if (LOG_DEB == severityLevel)
	{
		memcpy(buffHeader, (uint8_t *)"[DEBUG] | ", LOG_HEAD_DEBUG_LENGTH);
	}
	else if (LOG_WARN == severityLevel)
	{
		memcpy(buffHeader, (uint8_t *)"[WARNING] | ", LOG_HEAD_WARNING_LENGTH);
	}
	else if (LOG_ERR == severityLevel)
	{
		memcpy(buffHeader, (uint8_t *)"[ERROR] | ", LOG_HEAD_ERROR_LENGTH);
	}
	headerSize = strlen((char *) buffHeader);

	destBuffer = (uint8_t *) malloc(overalBuffSize);
	memcpy(destBuffer, buffHeader, headerSize);
	memcpy(destBuffer + headerSize, format, buffSize);
	memcpy(destBuffer + headerSize + buffSize, "\n\0", 2);

	if (NULL == firstNode)
	{
		firstNode = (bufferList_t *) calloc(0, sizeof(bufferList_t));
		firstNode->buffer = NULL;
		firstNode->next = NULL;
		lastNode = firstNode;
	}

	pushNode2Buffer((uint8_t *) destBuffer, &firstNode, &lastNode);
	return retVal_e;
}
