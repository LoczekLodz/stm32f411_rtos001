/*
 * lcd_Nokia5110_Driver.h
 *
 *  Created on: 10.11.2018
 *      Author: PKijanski
 */

#ifndef INC_LCD_NOKIA5110_DRIVER_H_
#define INC_LCD_NOKIA5110_DRIVER_H_

#include "resultCodes.h"

typedef struct
{
	uint32_t gpioPort;
	uint32_t gpioPin;
} spiLCDControlPin_t;

typedef struct
{
	uint32_t hwCS;
	spiLCDControlPin_t resetPin;
	spiLCDControlPin_t dataCmdPin;
	spiLCDControlPin_t chipSelectPin;
} spiLCDControl_t;

result_t createLCDNokia5110DriverTask(uint32_t *spiHandle, spiLCDControl_t *controlPins);

#endif /* INC_LCD_NOKIA5110_DRIVER_H_ */
