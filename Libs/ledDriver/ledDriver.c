/*
 * ledDriver.c
 *
 *  Created on: 26.10.2018
 *      Author: PKijanski
 */

#include "logger.h"
#include "rtosConfig.h"
#include "ledDriver.h"
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
/** Led Commands */
#define LD_CMD_TEST_ON				((uint16_t) 0x0f01)
#define LD_CMD_TEST_OFF				((uint16_t) 0x0f00)

#define LD_CMD_SCAN_MODE			((uint16_t) 0x0b00)
#define LD_CMD_SCAN_MODE_SET(X)		((uint16_t) LD_CMD_SCAN_MODE | (uint16_t) (X))

#define LD_CMD_INTENSIVITY			((uint16_t) 0x0a00)
#define LD_CMD_INTENSIVITY_SET(X)	((uint16_t) LD_CMD_INTENSIVITY | (uint16_t) (X))

#define LD_CMD_DECODE_MODE			((uint16_t) 0x0900)
#define LD_CMD_DECODE_MODE_SET(X)	((uint16_t) LD_CMD_DECODE_MODE | (uint16_t) (X))

#define LD_CMD_POWER_ON				((uint16_t) 0x0c01)
#define LD_CMD_POWER_OFF			((uint16_t) 0x0c00)

/**
 * @brief Local types and defines
 */

typedef struct
{
	genericTaskConfig_t		taskConfig;
	uint32_t				*spiHandle;
	spiDeviceControl_t		*spiDevice;
	uint8_t					devices;
} taskConfig_t;

typedef struct
{
	uint8_t 	data[8];
} ledStateDisplay_t;

/**
 * @brief Local Variables
 */
static taskConfig_t taskConfig =
{
	.taskConfig.mainTask		= NULL,
	.taskConfig.instances		= 0,
	.taskConfig.stackSize		= 128,
	.taskConfig.taskPriority	= LED_DRIVER_TASK_PRIORITY,
	.taskConfig.taskHandle		= NULL,
	.spiHandle					= (uint32_t *) NULL,
	.spiDevice					= (spiDeviceControl_t *) NULL,
	.devices					= 0,
};

ledStateDisplay_t 	*ledCurrentState;
static uint8_t		changeRequested = 0;

/**
 * @brief Function definition
 */
static void mainTask(void const * argument);
static void initLedDevice(void);
static void sendSpiCmd(uint16_t cmd, spiDeviceControl_t *spiDevice);
static void refreshLeds(void);
/**
 * @brief Global functions body
 *
 */
result_t createLedDriverTask(uint32_t *spiHandle, spiDeviceControl_t *device, uint8_t deviceNumber)
{
	result_t retVal_e = RESULT_SUCCESS;
	uint8_t deviceIndex = 0;

	do
	{
		if (NULL == spiHandle)
		{
			retVal_e = ERR_LED_DRIVER_NO_SPI_PROVIDED;
			continue;
		}

		taskConfig.spiHandle = spiHandle;
		taskConfig.taskConfig.mainTask = mainTask;

		if (NULL == device || 0 == deviceNumber)
		{
			retVal_e = ERR_LED_DRIVER_NO_DEVICES;
			continue;
		}

		taskConfig.devices = deviceNumber;
		taskConfig.spiDevice = device;
		ledCurrentState = (ledStateDisplay_t *) calloc(deviceNumber, sizeof(ledStateDisplay_t));
		for(deviceIndex = 0; deviceIndex < deviceNumber; deviceIndex++)
		{
			HAL_GPIO_WritePin((GPIO_TypeDef *) taskConfig.spiDevice->gpioPort, taskConfig.spiDevice->gpioPin, GPIO_PIN_SET);
		}

		osThreadDef(LedDriver,
					taskConfig.taskConfig.mainTask,
					taskConfig.taskConfig.taskPriority,
					taskConfig.taskConfig.instances,
					taskConfig.taskConfig.stackSize);

		taskConfig.taskConfig.taskHandle = osThreadCreate(osThread(LedDriver), NULL);

		if (NULL == taskConfig.taskConfig.taskHandle)
		{
			retVal_e = ERR_LED_DRIVER_TASK_CRESTION_ERROR;
			continue;
		}
	} while(0);

	return retVal_e;
}

result_t displayLed(uint8_t device, uint8_t ledNumber, uint8_t value)
{
	result_t retVal_e = RESULT_SUCCESS;

	do
	{
		if(device >= taskConfig.devices)
		{
			retVal_e = ERR_LED_DRIVER_WRONG_DEVICE_SELECTED;
			continue;
		}

		(ledCurrentState + device)->data[ledNumber] = value;

	} while(0);

	return retVal_e;
}

result_t updateLedDriverConfig(spiDeviceControl_t device, uint8_t deviceNumber)
{
	result_t retVal_e = RESULT_SUCCESS;
	spiDeviceControl_t *spiDevice;

	do
	{
		if(deviceNumber >= taskConfig.devices)
		{
			retVal_e = ERR_LED_DRIVER_WRONG_DEVICE_SELECTED;
			continue;
		}

		spiDevice = taskConfig.spiDevice + deviceNumber;

		spiDevice->driverParams.decodeMode = device.driverParams.decodeMode;
		spiDevice->driverParams.intensivity = device.driverParams.intensivity;
		spiDevice->driverParams.powerOn = device.driverParams.powerOn;
		spiDevice->driverParams.scanMode = device.driverParams.scanMode;
		spiDevice->driverParams.testMode = device.driverParams.testMode;

		changeRequested = 1;
	} while(0);

	return retVal_e;
}

/**
 * @brief Function definition body
 */
static void mainTask(void const * argument)
{
	initLedDevice();

	osDelay(1);

	while(1)
	{
		if (1 == changeRequested)
		{
			initLedDevice();
			changeRequested = 0;
		}
		else
		{
			refreshLeds();
		}
		osDelay(10);
	}
}

static void initLedDevice(void)
{
	uint8_t device = 0;
	spiDeviceControl_t *spiDevice;

	do
	{
		spiDevice = taskConfig.spiDevice + device;

		(0 == spiDevice->driverParams.testMode) ? sendSpiCmd(LD_CMD_TEST_OFF, spiDevice) : sendSpiCmd(LD_CMD_TEST_ON, spiDevice);

		sendSpiCmd(LD_CMD_SCAN_MODE_SET( spiDevice->driverParams.scanMode ), spiDevice);
		sendSpiCmd(LD_CMD_INTENSIVITY_SET( spiDevice->driverParams.intensivity ), spiDevice);
		sendSpiCmd(LD_CMD_DECODE_MODE_SET( spiDevice->driverParams.decodeMode ), spiDevice);

		(0 == spiDevice->driverParams.powerOn) ? sendSpiCmd(LD_CMD_POWER_OFF, spiDevice) : sendSpiCmd(LD_CMD_POWER_ON, spiDevice);

	} while (++device < taskConfig.devices);
}

static void refreshLeds(void)
{
	uint8_t device = 0;
	uint8_t ledNumber = 0;
	spiDeviceControl_t *spiDevice;
	ledStateDisplay_t *ledState;

	do
	{
		spiDevice = taskConfig.spiDevice + device;
		ledState = (ledStateDisplay_t *) ledCurrentState + device;

		for (ledNumber=0; ledNumber <= spiDevice->driverParams.scanMode; ledNumber++)
		{
			sendSpiCmd((uint16_t) (( (uint8_t) ( ledNumber + 1 ) << 8) | ((uint8_t) ledState->data[ledNumber]))  , (spiDeviceControl_t *) (taskConfig.spiDevice + device));
		}
	} while (++device < taskConfig.devices);
}

static void sendSpiCmd(uint16_t cmd, spiDeviceControl_t *spiDevice)
{
	if (0 == spiDevice->hwCS)
	{
		HAL_GPIO_WritePin((GPIO_TypeDef *) spiDevice->gpioPort, spiDevice->gpioPin, GPIO_PIN_RESET);
	}
	HAL_SPI_Transmit((SPI_HandleTypeDef *) taskConfig.spiHandle, (uint8_t *) &cmd, 1, 1000);
	if (0 == spiDevice->hwCS)
	{
		HAL_GPIO_WritePin((GPIO_TypeDef *) spiDevice->gpioPort, spiDevice->gpioPin, GPIO_PIN_SET);
	}
}
