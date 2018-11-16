/*
 * eeprom24Cx.h
 *
 *  Created on: 16.11.2018
 *      Author: PKijanski
 */

#ifndef INC_EEPROM24CX_H_
#define INC_EEPROM24CX_H_

#include "resultCodes.h"

result_t createEeprom24CxDriverTask(uint32_t *i2cHandle, uint8_t address);
result_t readData(uint16_t address, uint8_t *readData, uint8_t readSize);

#endif /* INC_EEPROM24CX_H_ */
