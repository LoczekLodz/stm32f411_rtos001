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
	bufferList_t *newNode = NULL;

	do
	{
		if (NULL == (*firstNode)->buffer && NULL == (*firstNode)->next)
		{
			(*firstNode)->buffer = buffer;
		}
		else
		{
			newNode = (bufferList_t *) malloc(sizeof(bufferList_t));

			if (NULL == newNode)
			{
				retVal_e = ERR_LOGGER_NEW_NODE_CREATION_ERROR;
				continue;
			}

			newNode->buffer = buffer;
			newNode->next = NULL;

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
		if(NULL == (*firstNode)->buffer && NULL == (*firstNode)->next)
		{
			retVal_e = ERR_LOGGER_FIRST_NODE_NULL;
			continue;
		}

		//we can delete buffer, we have 1st node
		free((*firstNode)->buffer);
		(*firstNode)->buffer = NULL;

		if (NULL != (*firstNode)->next)
		{
			tmpNode = (*firstNode)->next;
			free(*firstNode);
			*firstNode = tmpNode;
		}
		else
        {
            free(*firstNode);
            *firstNode = NULL;
		}
	} while(0);
	return retVal_e;
}
