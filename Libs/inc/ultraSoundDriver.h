/*
 * ultraSoundDriver.h
 *
 *  Created on: 29.10.2018
 *      Author: PKijanski
 */

#ifndef INC_ULTRASOUNDDRIVER_H_
#define INC_ULTRASOUNDDRIVER_H_

#include "resultCodes.h"

typedef struct
{
	uint32_t gpioPort;
	uint32_t gpioPin;
} triggerUltraSoundMeasure_t;

result_t createUltraSoundTask(uint32_t *timerHandle, triggerUltraSoundMeasure_t trgPin);

result_t stopTimer();

result_t measurementDone();

uint32_t getDistance(void);

void getDistanceText(uint8_t *distance);
void getDistanceTextTrimmed(uint8_t *distance);

#endif /* INC_ULTRASOUNDDRIVER_H_ */
