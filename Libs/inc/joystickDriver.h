/*
 * joystickDriver.h
 *
 *  Created on: 30.10.2018
 *      Author: PKijanski
 */

#ifndef INC_JOYSTICKDRIVER_H_
#define INC_JOYSTICKDRIVER_H_

#include "resultCodes.h"

result_t createJoystickDriverTask(uint32_t *adcHandle);

void getJoystickPositionText(uint8_t *posX, uint8_t *posY);
void getJoystickPositionTextTrimmed(uint8_t *posX, uint8_t *posY);

#endif /* INC_JOYSTICKDRIVER_H_ */
