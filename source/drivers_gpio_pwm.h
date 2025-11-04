/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef DRIVERS_GPIO_PWM_H_
#define DRIVERS_GPIO_PWM_H_

#include <stdint.h>
#include <stdbool.h>
#include "messages.h"

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Initialize GPIO pins for RGB LED
 */
void GPIO_Init(void);

/*!
 * @brief Initialize PWM for buzzer
 */
void PWM_Init(void);

/*!
 * @brief Set RGB LED color
 * @param red Red component (0 or 1)
 * @param green Green component (0 or 1)
 * @param blue Blue component (0 or 1)
 */
void LED_SetColor(uint8_t red, uint8_t green, uint8_t blue);

/*!
 * @brief Set LED to predefined state
 * @param state LED state from LedState enum
 */
void LED_SetState(LedState state);

/*!
 * @brief Set buzzer frequency
 * @param frequency Frequency in Hz (0 to turn off)
 */
void Buzzer_SetFrequency(uint32_t frequency);

/*!
 * @brief Play a simple beep
 * @param frequency Frequency in Hz
 * @param duration_ms Duration in milliseconds
 */
void Buzzer_Beep(uint32_t frequency, uint32_t duration_ms);

/*!
 * @brief Play buzzer pattern
 * @param pattern Buzzer pattern from BuzzerState enum
 */
void Buzzer_PlayPattern(BuzzerState pattern);

#endif /* DRIVERS_GPIO_PWM_H_ */
