/*
 * logger.h
 *
 *  Created on: 24.10.2018
 *      Author: PKijanski
 */

#ifndef LOGGER_LOGGER_H_
#define LOGGER_LOGGER_H_

#include "resultCodes.h"

#define LOG_INF 		((uint8_t) 0x00)
#define LOG_DEB			((uint8_t) 0x01)
#define LOG_WARN		((uint8_t) 0x02)
#define LOG_ERR			((uint8_t) 0x03)

typedef uint8_t logSeverity_e;

/**
 * @brief
 */
result_t createLoggerTask(uint32_t *usartHandle);

result_t logMessage(logSeverity_e severityLevel, const uint8_t *format, ...);

#endif /* LOGGER_LOGGER_H_ */
