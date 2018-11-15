/*
 * utils.h
 *
 *  Created on: 01.11.2018
 *      Author: PKijanski
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "resultCodes.h"

result_t uint16ToStr(uint16_t number2Convert, uint8_t *buffer, uint8_t *length);
result_t uint32ToStr(uint32_t number2Convert, uint8_t *buffer, uint8_t *length);
void delayUs(uint8_t delay);

#endif /* INC_UTILS_H_ */
