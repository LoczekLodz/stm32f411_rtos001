/*
 * oneDirList.h
 *
 *  Created on: 15.11.2018
 *      Author: PKijanski
 */

#ifndef INC_ONEDIRLIST_H_
#define INC_ONEDIRLIST_H_

#include "resultCodes.h"

typedef struct bufferList
{
	uint8_t				*buffer;
	struct bufferList	*next;
}bufferList_t;


result_t pushNode2Buffer(uint8_t *buffer, bufferList_t **firstNode, bufferList_t **lastNode);
result_t popNodeFromBuffer(bufferList_t **firstNode);

#endif /* INC_ONEDIRLIST_H_ */
