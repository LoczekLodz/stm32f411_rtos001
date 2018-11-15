/*
 * lcd_Nokia5110_Driver.c
 *
 *  Created on: 10.11.2018
 *      Author: PKijanski
 */

#include "lcd_Nokia5110_Driver.h"
#include "rtosConfig.h"
#include <stdlib.h>

#if defined  __STM32F4_FAMILY__
	#include "stm32f4xx_hal.h"
#elif defined  __STM32F3_FAMILY__
	#include "stm32f3xx_hal.h"
#else
	#error "Wrong STM32 family defined"
#endif /* STM32 FAMILY */


/**
 * @brief Local defines
 */
/** Lcd Commands */
#define LCD_CMD_NOKIA5110_POWER_ON_STD		((uint8_t) 0x20)
#define LCD_CMD_NOKIA5110_POWER_ON_EXT		((uint8_t) 0x21)

#define LCD_CMD_NOKIA5110_DISPLAY_BLANK		((uint8_t) 0x08)
#define LCD_CMD_NOKIA5110_DISPLAY_NORMAL	((uint8_t) 0x0c)
#define LCD_CMD_NOKIA5110_DISPLAY_ALL		((uint8_t) 0x09)
#define LCD_CMD_NOKIA5110_DISPLAY_INVERSE	((uint8_t) 0x0d)

#define LCD_CMD_NOKIA5110_DISPLAY_X_ADR		((uint8_t) 0x80)		//bits 0-6 is the address
#define LCD_CMD_NOKIA5110_DISPLAY_Y_ADR		((uint8_t) 0x40)		//bits 0-2 is the address

//EXT Commands
#define LCD_CMD_NOKIA5110_TEMP_COEFFICIENT0 ((uint8_t) 0x04)
#define LCD_CMD_NOKIA5110_TEMP_COEFFICIENT1 ((uint8_t) 0x05)
#define LCD_CMD_NOKIA5110_TEMP_COEFFICIENT2 ((uint8_t) 0x06)
#define LCD_CMD_NOKIA5110_TEMP_COEFFICIENT3 ((uint8_t) 0x07)

#define LCD_CMD_NOKIA5110_SET_BIAS			((uint8_t) 0x10)		//bits 0-2 is the bias
#define LCD_CMD_NOKIA5110_V_OP				((uint8_t) 0x80)		//bits 0-6 is the bias

/**
 * @brief Local types and defines
 */

typedef struct
{
	genericTaskConfig_t		taskConfig;
	uint32_t				*spiHandle;
	spiLCDControl_t			*spiControlPins;
} taskConfig_t;

/**
 * @brief Local Variables
 */
static taskConfig_t taskConfig =
{
	.taskConfig.mainTask		= NULL,
	.taskConfig.instances		= 0,
	.taskConfig.stackSize		= 128,
	.taskConfig.taskPriority	= LCD_NOKIA__DRIVER_TASK_PRIORITY,
	.taskConfig.taskHandle		= NULL,
	.spiHandle					= (uint32_t *) NULL,
	.spiControlPins				= (spiLCDControl_t *) NULL,
};


//Screen buffer
//static uint8_t lcdScreenData[84][6];

/**
 * @brief Function definition
 */
static void mainTask(void const * argument);

static void initScreen(void);
//static void setPosition(uint8_t xAdrr, uint8_t yAddr);
static void sendSpiCmd(const uint8_t *data, uint8_t size, const uint8_t cmd2Send);
static void sendTest(void);

/**
 * @brief Global functions body
 *
 */
result_t createLCDNokia5110DriverTask(uint32_t *spiHandle, spiLCDControl_t *controlPins)
{
	result_t retVal_e = RESULT_SUCCESS;

	do
	{
		if (NULL == spiHandle)
		{
			retVal_e = ERR_LCD_NOKIA_5110_NO_SPI_PROVIDED;
			continue;
		}

		taskConfig.spiHandle = spiHandle;
		taskConfig.taskConfig.mainTask = mainTask;

		if ( (NULL == controlPins) ||
			((0 == controlPins->hwCS) && (0 == controlPins->chipSelectPin.gpioPort)) ||
			(0 == controlPins->dataCmdPin.gpioPort) ||
			(0 == controlPins->resetPin.gpioPort) )
		{
			retVal_e = ERR_LCD_NOKIA_5110_WRONG_INPUT_PARAMS;
			continue;
		}

		taskConfig.spiControlPins = controlPins;

		if (0 == controlPins->hwCS)
		{
			HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.spiControlPins->chipSelectPin.gpioPort, taskConfig.spiControlPins->chipSelectPin.gpioPin, GPIO_PIN_SET);
		}

		HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.spiControlPins->resetPin.gpioPort, taskConfig.spiControlPins->resetPin.gpioPin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.spiControlPins->dataCmdPin.gpioPort, taskConfig.spiControlPins->dataCmdPin.gpioPin, GPIO_PIN_SET);

		osThreadDef(LcdNokiaDriver,
					taskConfig.taskConfig.mainTask,
					taskConfig.taskConfig.taskPriority,
					taskConfig.taskConfig.instances,
					taskConfig.taskConfig.stackSize);

		taskConfig.taskConfig.taskHandle = osThreadCreate(osThread(LcdNokiaDriver), NULL);

		if (NULL == taskConfig.taskConfig.taskHandle)
		{
			retVal_e = ERR_LCD_NOKIA_5110_TASK_CREATION_ERROR;
			continue;
		}
	} while(0);

	return retVal_e;
}

/**
 * @brief Function definition body
 */
static void mainTask(void const * argument)
{
	initScreen();
	osDelay(1);
	sendTest();

	while(1)
	{
		osDelay(10);
	}
}

static void initScreen(void)
{
	uint8_t configData[4] = {
		LCD_CMD_NOKIA5110_POWER_ON_EXT,
		LCD_CMD_NOKIA5110_V_OP | 0x20,
		LCD_CMD_NOKIA5110_POWER_ON_STD,
		LCD_CMD_NOKIA5110_DISPLAY_NORMAL
	};
	HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.spiControlPins->resetPin.gpioPort, taskConfig.spiControlPins->resetPin.gpioPin, GPIO_PIN_SET);
	sendSpiCmd(configData, 4, 1);
}

static void sendTest(void)
{
	uint8_t testData[15] = {
		0x01,
		0x1f,
		0x01,
		0x00,
		0x1f,
		0x15,
		0x15,
		0x00,
		0x1c,
		0x15,
		0x05,
		0x00,
		0x01,
		0x1f,
		0x01
	};

	sendSpiCmd(testData, 15, 0);
}

static void sendSpiCmd(const uint8_t *data, uint8_t size, const uint8_t cmd2Send)
{
	if (0 == taskConfig.spiControlPins->hwCS)
	{
		HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.spiControlPins->chipSelectPin.gpioPort, taskConfig.spiControlPins->chipSelectPin.gpioPin, GPIO_PIN_RESET);
	}

	if (1 == cmd2Send)
	{
		HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.spiControlPins->dataCmdPin.gpioPort, taskConfig.spiControlPins->dataCmdPin.gpioPin, GPIO_PIN_RESET);
	}

	HAL_SPI_Transmit((SPI_HandleTypeDef *) taskConfig.spiHandle, (uint8_t *) data, size, 1000);
	//HAL_SPI_Transmit_DMA((SPI_HandleTypeDef *) taskConfig.spiHandle, (uint8_t *) data, size);
	//HAL_SPI_Transmit_IT((SPI_HandleTypeDef *) taskConfig.spiHandle, (uint8_t *) data, size);

	if (1 == cmd2Send)
	{
		HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.spiControlPins->dataCmdPin.gpioPort, taskConfig.spiControlPins->dataCmdPin.gpioPin, GPIO_PIN_SET);
	}
	if (0 == taskConfig.spiControlPins->hwCS)
	{
		HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.spiControlPins->chipSelectPin.gpioPort, taskConfig.spiControlPins->chipSelectPin.gpioPin, GPIO_PIN_SET);
	}
}
