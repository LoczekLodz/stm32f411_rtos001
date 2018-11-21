/*
 * eeprom24Cx.c
 *
 *  Created on: 16.11.2018
 *      Author: PKijanski
 */

#include "eeprom24Cx.h"
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


typedef struct
{
	genericTaskConfig_t		taskConfig;
	uint32_t				*i2cHandle;
	uint8_t					i2cAddress;
	uint8_t					isBusy;
	uint32_t				callback;
} taskConfig_t;

static taskConfig_t taskConfig =
{
	.taskConfig.mainTask		= NULL,
	.taskConfig.instances		= 0,
	.taskConfig.stackSize		= 128,
	.taskConfig.taskPriority	= ULTRA_SOUND_TASK_PRIORITY,
	.taskConfig.taskHandle		= NULL,
	.i2cHandle					= NULL,
	.i2cAddress					= 0,
	.isBusy						= 0,
	.callback					= 0,
};

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
result_t createEeprom24CxDriverTask(uint32_t *i2cHandle, uint8_t address)
{
	result_t retVal_e = RESULT_SUCCESS;

	do
	{

		if (NULL == i2cHandle)
		{
			retVal_e = ERR_EEPROM_24CX_NO_I2C_PROVIDED;
		}

		taskConfig.i2cHandle = i2cHandle;
		taskConfig.i2cAddress = address;
		taskConfig.taskConfig.mainTask = mainTask;

		osThreadDef(eeprom24Cx,
					taskConfig.taskConfig.mainTask,
					taskConfig.taskConfig.taskPriority,
					taskConfig.taskConfig.instances,
					taskConfig.taskConfig.stackSize);

		taskConfig.taskConfig.taskHandle = osThreadCreate(osThread(eeprom24Cx), NULL);

		if (NULL == taskConfig.taskConfig.taskHandle)
		{
			retVal_e = ERR_EEPROM_24CX_TASK_CREATION_ERROR;
		}

//		if (NULL == firstNodeTransmitterAM433)
//		{
//			firstNodeTransmitterAM433 = (bufferList_t *) calloc(0, sizeof(bufferList_t));
//			firstNodeTransmitterAM433->buffer = NULL;
//			firstNodeTransmitterAM433->next = NULL;
//			lastNodeTransmitterAM433 = firstNodeTransmitterAM433;
//		}

	} while(0);

	return retVal_e;
}

result_t readData(uint16_t address, uint8_t *readData, uint8_t readSize)
{
	result_t	retVal_e = RESULT_SUCCESS;
	uint8_t		buff[readSize];
	do
	{
		if (0 != taskConfig.isBusy)
		{
			retVal_e = ERR_EEPROM_24CX_BUSY_LINE;
			continue;
		}

		taskConfig.isBusy = 1;
		//buff = (uint8_t *) calloc(readSize, sizeof(uint8_t));

		HAL_I2C_Master_Transmit((I2C_HandleTypeDef *) taskConfig.i2cHandle, taskConfig.i2cAddress, (uint8_t *) &address, 2, 100);

		HAL_I2C_Master_Receive((I2C_HandleTypeDef *) taskConfig.i2cHandle, taskConfig.i2cAddress, buff, readSize, 1000);

		memcpy(readData, buff, readSize);


		//HAL_I2C_Master_Receive_DMA((I2C_HandleTypeDef *) taskConfig.i2cHandle, taskConfig.i2cAddress, &buffRec, 20);
		taskConfig.isBusy = 0;
	}while(0);

	return retVal_e;
}

/**
 * @brief Function definition body
 */
static void mainTask(void const * argument)
{
	while(1)
	{

		osDelay(100);
	}
}

/*
 * 	HAL_I2C_Master_Transmit(&hi2c1, 0xA0, buffTr, 2, 100);
	//HAL_I2C_Master_Transmit_DMA(&hi2c1, 0xA0, buffTr, 2);

	HAL_I2C_Master_Receive(&hi2c1, 0xA0, buffRec, 24, 100);
	//HAL_I2C_Master_Receive_DMA(&hi2c1, 0xA0, &buffRec, 20);


	if (0) {
		uint8_t table2Write[24] = "Test DMA Write I2C      ";
		uint8_t writeBuff[10];
		uint8_t i;
		writeBuff[0] = 0;
		writeBuff[1] = 0;

		for (i=0; i<3; i++) {
			HAL_Delay(10);
			writeBuff[1] = 8*i;
			memcpy(writeBuff+2, (const void*) table2Write+(i*8), 8);
			HAL_I2C_Master_Transmit(&hi2c1, 0xA0, writeBuff, 10, 100);
}
	}
 */
