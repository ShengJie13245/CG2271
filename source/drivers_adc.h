/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef DRIVERS_ADC_H_
#define DRIVERS_ADC_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Initialize ADC for LDR sensor reading
 */
void ADC_Init(void);

/*!
 * @brief Read raw ADC value from LDR sensor
 * @return 12-bit ADC value (0-4095)
 */
uint16_t ADC_ReadLDR(void);

/*!
 * @brief Convert ADC value to approximate lux reading
 * @param adc_value Raw ADC value from LDR
 * @return Approximate lux value (0-1023)
 */
uint16_t ADC_ConvertToLux(uint16_t adc_value);

#endif /* DRIVERS_ADC_H_ */
