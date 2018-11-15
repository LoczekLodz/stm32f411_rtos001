/*
 * ledDriver.h
 *
 *  Created on: 26.10.2018
 *      Author: PKijanski
 */

#ifndef INC_LEDDRIVER_H_
#define INC_LEDDRIVER_H_

#include "resultCodes.h"

/**
 * Types needed for initialization
 */

/**
 * @brief ledDriverParam_t to store parameters per each led driver
 *
 * @input testMode 		-> 0 [OFF], 1 [ON] - when on all led digits are displayed
 * @input scanMode 		-> binary mode 0 only 1 is displayed 0x07 mean 0,1,2,3,4,5,6,7 displays will bed taken under consideration
 * @input intensivity 	-> min level is 0x00 max level is 0x0f
 * @input decodeMode	-> each byte represent 1 display, 0x0f -> 0,1,2,3 displayes are used with decode option
 * @input powerOn		-> 0 [OFF], 1 [ON] - when on numbers/chars will be displayed
 */
typedef struct
{
	uint8_t 		testMode;
	uint8_t			scanMode;
	uint8_t			intensivity;
	uint8_t			decodeMode;
	uint8_t			powerOn;
} ledDriverParam_t;

typedef struct
{
	uint32_t hwCS;
	uint32_t gpioPort;
	uint32_t gpioPin;
	ledDriverParam_t driverParams;
} spiDeviceControl_t;

/**
 * @brief
 */
result_t createLedDriverTask(uint32_t *spiHandle, spiDeviceControl_t *device, uint8_t deviceNumber);

result_t displayLed(uint8_t device, uint8_t ledNumber, uint8_t value);

result_t updateLedDriverConfig(spiDeviceControl_t device, uint8_t deviceNumber);

#endif /* INC_LEDDRIVER_H_ */
