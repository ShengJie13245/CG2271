/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    ldr_sensor.h
 * @brief   LDR (Light Dependent Resistor) sensor module header
 * @author  CG2271 Group
 */

#ifndef LDR_SENSOR_H_
#define LDR_SENSOR_H_

#include <stdint.h>

/* LDR sensor configuration */
#define LDR_PIN        20   // PTE20
#define ADC_CHANNEL    0    // ADC0_SE0 for PTE20

/**
 * @brief Initialize the LDR sensor and ADC
 * Sets up ADC0 for reading from PTE20 (ADC0_SE0)
 */
void LDR_Init(void);

/**
 * @brief Start continuous LDR sensor readings
 * Begins periodic ADC conversions and UART output
 */
void LDR_StartReading(void);

/**
 * @brief Trigger a single LDR sensor reading
 * Initiates one ADC conversion
 */
void LDR_TriggerReading(void);

/**
 * @brief Get the last LDR sensor reading
 * @return Last ADC value (0-4095)
 */
uint16_t LDR_GetLastValue(void);

/**
 * @brief Convert ADC value to voltage in millivolts
 * @param adc_value Raw ADC value (0-4095)
 * @return Voltage in millivolts (0-3300mV)
 */
uint16_t LDR_ADCToVoltage(uint16_t adc_value);

/**
 * @brief Convert ADC value to light level percentage
 * @param adc_value Raw ADC value (0-4095)
 * @return Light level percentage (0-100%)
 */
uint8_t LDR_ADCToPercentage(uint16_t adc_value);

#endif /* LDR_SENSOR_H_ */
