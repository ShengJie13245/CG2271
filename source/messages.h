/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MESSAGES_H_
#define MESSAGES_H_

#include <stdint.h>

/*******************************************************************************
 * Message Type Definitions
 ******************************************************************************/

/* Operating Mode */
typedef enum {
    MODE_RELAX = 0,
    MODE_FOCUS = 1
} Mode;

/* System State */
typedef enum {
    STATE_GOOD = 0,
    STATE_WARN = 1,
    STATE_ALERT = 2
} State;

/* LED States */
typedef enum {
    LED_OFF = 0,
    LED_GREEN = 1,
    LED_YELLOW = 2,
    LED_RED = 3,
    LED_CYAN = 4
} LedState;

/* Buzzer States */
typedef enum {
    BUZZ_OFF = 0,
    BUZZ_SHORT = 1,
    BUZZ_LONG = 2,
    BUZZ_CHIME = 3
} BuzzerState;

/*******************************************************************************
 * Message Structures
 ******************************************************************************/

/* Telemetry data sent from SensorTask to CommsTask */
typedef struct {
    Mode mode;              /* Current operating mode */
    State state;            /* Current system state */
    uint8_t score;          /* Comfort score (0-100) */
    uint16_t lux;           /* Light level (0-1023 ADC value) */
    float temp_c;           /* Temperature from ESP32 */
    uint8_t noise_b3s;      /* Noise bursts in last 3 seconds */
} Telemetry;

/* Action commands sent to ActuatorTask */
typedef struct {
    LedState led_state;     /* LED pattern to display */
    BuzzerState buzz_state; /* Buzzer pattern to play */
} Action;

/* Noise event timestamp for ISR -> SensorTask communication */
typedef struct {
    uint32_t timestamp;     /* Tick count when noise detected */
} NoiseEvent;

/* UART message structure for ESP32 communication */
typedef struct {
    char buffer[128];       /* Message buffer */
    uint16_t length;        /* Message length */
} UartMessage;

/*******************************************************************************
 * Global Variables (extern declarations)
 ******************************************************************************/

/* FreeRTOS Objects */
extern SemaphoreHandle_t uart_mutex;
extern SemaphoreHandle_t button_sem;
extern QueueHandle_t noise_evt_q;
extern QueueHandle_t uplink_q;
extern QueueHandle_t downlink_q;

/* Global State Variables */
extern volatile Mode current_mode;
extern volatile float latest_temp_c;

#endif /* MESSAGES_H_ */
