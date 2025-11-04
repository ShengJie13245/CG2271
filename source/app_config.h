/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include "fsl_device_registers.h"

/*******************************************************************************
 * Hardware Pin Definitions
 ******************************************************************************/

/* LDR Sensor - Analog Input */
#define LDR_ADC_BASE            ADC0
#define LDR_ADC_CHANNEL         8U      /* PTB0 -> ADC0_SE8 */
#define LDR_PORT                PORTB
#define LDR_GPIO                GPIOB
#define LDR_PIN                 0U

/* Sound Sensor - Digital Input with Interrupt */
#define SOUND_PORT              PORTC
#define SOUND_GPIO              GPIOC
#define SOUND_PIN               6U
#define SOUND_IRQ               PORTC_IRQn

/* Mode Button - Digital Input with Interrupt */
#define BUTTON_PORT             PORTA
#define BUTTON_GPIO             GPIOA
#define BUTTON_PIN              4U
#define BUTTON_IRQ              PORTA_IRQn

/* RGB LED Pins */
#define LED_R_PORT              PORTD
#define LED_R_GPIO              GPIOD
#define LED_R_PIN               5U

#define LED_G_PORT              PORTD
#define LED_G_GPIO              GPIOD
#define LED_G_PIN               7U

#define LED_B_PORT              PORTD
#define LED_B_GPIO              GPIOD
#define LED_B_PIN               6U

/* Buzzer - PWM Output */
#define BUZZER_PORT             PORTD
#define BUZZER_GPIO             GPIOD
#define BUZZER_PIN              3U
#define BUZZER_TPM              TPM0
#define BUZZER_TPM_CHANNEL      3U

/* UART for ESP32 Communication */
#define ESP32_UART              LPUART0
#define ESP32_UART_CLKSRC       kCLOCK_McgIrc48MClk
#define ESP32_UART_CLK_FREQ     CLOCK_GetFreq(kCLOCK_McgIrc48MClk)
#define ESP32_UART_IRQ          LPUART0_IRQn
#define ESP32_UART_TX_PORT      PORTC
#define ESP32_UART_TX_PIN       4U
#define ESP32_UART_RX_PORT      PORTC
#define ESP32_UART_RX_PIN       3U

/*******************************************************************************
 * Application Constants
 ******************************************************************************/

/* Task Priorities */
#define SENSOR_TASK_PRIORITY    (tskIDLE_PRIORITY + 2)
#define COMMS_TASK_PRIORITY     (tskIDLE_PRIORITY + 2)
#define ACTUATOR_TASK_PRIORITY  (tskIDLE_PRIORITY + 1)
#define MODE_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

/* Task Stack Sizes */
#define SENSOR_TASK_STACK_SIZE      (256)
#define COMMS_TASK_STACK_SIZE       (512)
#define ACTUATOR_TASK_STACK_SIZE    (256)
#define MODE_TASK_STACK_SIZE        (128)

/* Queue Sizes */
#define NOISE_EVENT_QUEUE_SIZE      (10)
#define UPLINK_QUEUE_SIZE           (5)
#define DOWNLINK_QUEUE_SIZE         (5)

/* Timing Constants */
#define SENSOR_TASK_PERIOD_MS       (100)
#define COMMS_TASK_PERIOD_MS        (100)
#define NOISE_WINDOW_MS             (3000)  /* 3 seconds for noise burst counting */

/* Comfort Score Constants */
#define COMFORT_SCORE_MAX           (100)
#define COMFORT_SCORE_GOOD_THRESH   (75)
#define COMFORT_SCORE_WARN_THRESH   (50)

/* Light Level Constants */
#define FOCUS_MODE_IDEAL_LUX        (300)
#define RELAX_MODE_IDEAL_LUX        (200)
#define LIGHT_PENALTY_FACTOR        (0.1f)

/* Temperature Constants */
#define FOCUS_MODE_IDEAL_TEMP       (25.0f)
#define RELAX_MODE_IDEAL_TEMP       (27.0f)
#define TEMP_PENALTY_FACTOR         (2.0f)

/* Noise Constants */
#define MAX_NOISE_BURSTS_3S         (5)
#define NOISE_PENALTY_PER_BURST     (10)

/* UART Protocol Constants */
#define UART_BUFFER_SIZE            (128)
#define UART_BAUD_RATE              (115200)

#endif /* APP_CONFIG_H_ */
