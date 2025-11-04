/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef DRIVERS_UART_H_
#define DRIVERS_UART_H_

#include <stdint.h>
#include <stdbool.h>
#include "messages.h"

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Initialize UART for ESP32 communication
 */
void UART_Init(void);

/*!
 * @brief Send string via UART
 * @param str Null-terminated string to send
 */
void UART_SendString(const char* str);

/*!
 * @brief Send telemetry data to ESP32
 * @param telemetry Pointer to telemetry structure
 */
void UART_SendTelemetry(const Telemetry* telemetry);

/*!
 * @brief Receive a complete line from UART
 * @param line Buffer to store received line
 * @param max_length Maximum length of line buffer
 * @return true if complete line received, false otherwise
 */
bool UART_ReceiveLine(char* line, uint16_t max_length);

/*!
 * @brief Parse temperature from received line
 * @param line Received line string
 * @param temperature Pointer to store parsed temperature
 * @return true if temperature successfully parsed, false otherwise
 */
bool UART_ParseTemperature(const char* line, float* temperature);

#endif /* DRIVERS_UART_H_ */
