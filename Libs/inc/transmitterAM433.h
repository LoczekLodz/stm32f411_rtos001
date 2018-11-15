/*
 * transmitterAM433.h
 *
 *  Created on: 15.11.2018
 *      Author: PKijanski
 */

#ifndef INC_TRANSMITTERAM433_H_
#define INC_TRANSMITTERAM433_H_

#include "resultCodes.h"

result_t createTransmitterAM433DriverTask(uint32_t *usartHandle);
result_t transmitAM433Message(const uint8_t *data, uint8_t size);

#endif /* INC_TRANSMITTERAM433_H_ */
