/*
 * utils.c
 *
 *  Created on: 01.11.2018
 *      Author: PKijanski
 */

#include <stdlib.h>
#include "utils.h"
#include "resultCodes.h"

#define MAX_NUM_UINT16		5
#define US_DELAY_CONST		30 //100MHz_Clk

result_t uint16ToStr(uint16_t number2Convert, uint8_t *buffer, uint8_t *length)
{
	result_t	retVal_e = RESULT_SUCCESS;
	uint8_t		*buff = buffer;
	uint8_t		i = 0;
	uint8_t 	maxNumbers = 0;
	uint16_t	tmpNum = number2Convert;

	do
	{
		if (NULL == buffer)
		{
			retVal_e = ERR_UTILS_NULL_INPUT_BUF;
			continue;
		}

		if(0 == number2Convert)
		{
			maxNumbers = 1;
			*buff = '0';
		}
		else
		{
			do
			{
				tmpNum = tmpNum / 10;
				maxNumbers++;
			}while (tmpNum);

			while(number2Convert)
			{
				*(buff + maxNumbers - i - 1) = number2Convert % 10 + '0';
				i++;
				number2Convert = number2Convert / 10;
			}
		}

		*length = maxNumbers;
	} while(0);

	return retVal_e;
}

result_t uint32ToStr(uint32_t number2Convert, uint8_t *buffer, uint8_t *length)
{
	result_t	retVal_e = RESULT_SUCCESS;
	uint8_t		*buff = buffer;
	uint8_t		i = 0;
	uint8_t 	maxNumbers = 0;
	uint16_t	tmpNum = number2Convert;

	do
	{
		if (NULL == buffer)
		{
			retVal_e = ERR_UTILS_NULL_INPUT_BUF;
			continue;
		}

		if(0 == number2Convert)
		{
			maxNumbers = 1;
			*buff = '0';
		}
		else
		{
			do
			{
				tmpNum = tmpNum / 10;
				maxNumbers++;
			}while (tmpNum);

			if(maxNumbers > 8)
			{
				retVal_e = ERR_UTILS_TO_BIG_NUMBER;
				continue;
			}

			while(number2Convert)
			{
				*(buff + maxNumbers - i - 1) = number2Convert % 10 + '0';
				i++;
				number2Convert = number2Convert / 10;
			}
		}
		*length = maxNumbers;
	} while(0);

	return retVal_e;
}

void delayUs(uint8_t delay)
{
	uint8_t i;
	uint8_t usDelay;

	for (i=0; i<delay; i++)
	{
		for (usDelay = 0; usDelay<US_DELAY_CONST; usDelay++)
		{
			asm("nop");
		}
	}
}
