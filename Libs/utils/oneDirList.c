/*
 * oneDirList.c
 *
 *  Created on: 15.11.2018
 *      Author: PKijanski
 */

#include "oneDirList.h"
#include <stdlib.h>
#include <string.h>

result_t pushNode2Buffer(uint8_t *buffer, bufferList_t **firstNode, bufferList_t **lastNode)
{
	result_t retVal_e = RESULT_SUCCESS;

	do
	{
		bufferList_t *newNode = (bufferList_t *) calloc(0, sizeof(bufferList_t));

		if (NULL == newNode)
		{
			retVal_e = ERR_LOGGER_NEW_NODE_CREATION_ERROR;
			continue;
		}

		newNode->buffer = buffer;
		newNode->next = NULL;

		if (NULL == (*firstNode)->next)
		{
			(*firstNode)->next = newNode;
			*lastNode = newNode;
		}
		else
		{
			(*lastNode)->next = newNode;
			*lastNode = newNode;
		}

	} while(0);
	return retVal_e;
}

result_t popNodeFromBuffer(bufferList_t **firstNode)
{
	result_t retVal_e = RESULT_SUCCESS;
	bufferList_t *tmpNode = NULL;

	do
	{
		if(NULL == (*firstNode)->next)
		{
			retVal_e = ERR_LOGGER_FIRST_NODE_NULL;
			continue;
		}

		//we can delete buffer, we have 1st node
		free((*firstNode)->next->buffer);


		if (NULL != (*firstNode)->next->next)
		{
			tmpNode = (*firstNode)->next->next;
			free((*firstNode)->next);
			(*firstNode)->next = tmpNode;
		}
		else
        {
            (*firstNode)->next = NULL;
		}


	} while(0);
	return retVal_e;
}
