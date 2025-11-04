/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "sensor_task.h"
#include "app_config.h"
#include "messages.h"
#include "drivers_adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "fsl_debug_console.h"
#include <math.h>

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint32_t noise_events[MAX_NOISE_BURSTS_3S];
static uint8_t noise_event_count = 0;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

static void ProcessNoiseEvents(void)
{
    uint32_t current_time = xTaskGetTickCount();
    uint32_t window_start = current_time - pdMS_TO_TICKS(NOISE_WINDOW_MS);
    
    /* Process all pending noise events from ISR */
    uint32_t timestamp;
    while (xQueueReceive(noise_evt_q, &timestamp, 0) == pdTRUE) {
        /* Add to circular buffer if within time window */
        if (timestamp >= window_start) {
            noise_events[noise_event_count % MAX_NOISE_BURSTS_3S] = timestamp;
            noise_event_count++;
        }
    }
    
    /* Count events within the 3-second window */
    uint8_t valid_events = 0;
    for (uint8_t i = 0; i < MAX_NOISE_BURSTS_3S && i < noise_event_count; i++) {
        if (noise_events[i] >= window_start) {
            valid_events++;
        }
    }
    
    /* Reset counter if we've processed more than max events */
    if (noise_event_count > MAX_NOISE_BURSTS_3S) {
        noise_event_count = valid_events;
        
        /* Compact the array to keep only valid events */
        uint8_t write_idx = 0;
        for (uint8_t i = 0; i < MAX_NOISE_BURSTS_3S; i++) {
            if (noise_events[i] >= window_start) {
                noise_events[write_idx++] = noise_events[i];
            }
        }
    }
}

static uint8_t CalculateComfortScore(uint16_t lux, float temp_c, uint8_t noise_bursts, Mode mode)
{
    /* Determine ideal values based on mode */
    uint16_t ideal_lux = (mode == MODE_FOCUS) ? FOCUS_MODE_IDEAL_LUX : RELAX_MODE_IDEAL_LUX;
    float ideal_temp = (mode == MODE_FOCUS) ? FOCUS_MODE_IDEAL_TEMP : RELAX_MODE_IDEAL_TEMP;
    
    /* Calculate penalties */
    float light_penalty = LIGHT_PENALTY_FACTOR * abs((int)lux - (int)ideal_lux);
    float temp_penalty = TEMP_PENALTY_FACTOR * fabsf(temp_c - ideal_temp);
    float noise_penalty = NOISE_PENALTY_PER_BURST * noise_bursts;
    
    /* Calculate score */
    float score = COMFORT_SCORE_MAX - light_penalty - temp_penalty - noise_penalty;
    
    /* Clamp to valid range */
    if (score < 0) score = 0;
    if (score > COMFORT_SCORE_MAX) score = COMFORT_SCORE_MAX;
    
    return (uint8_t)score;
}

static State DetermineState(uint8_t score)
{
    if (score >= COMFORT_SCORE_GOOD_THRESH) {
        return STATE_GOOD;
    } else if (score >= COMFORT_SCORE_WARN_THRESH) {
        return STATE_WARN;
    } else {
        return STATE_ALERT;
    }
}

static void GenerateAction(State state, Action* action)
{
    switch (state) {
        case STATE_GOOD:
            action->led_state = LED_GREEN;
            action->buzz_state = BUZZ_OFF;
            break;
        case STATE_WARN:
            action->led_state = LED_YELLOW;
            action->buzz_state = BUZZ_SHORT;
            break;
        case STATE_ALERT:
            action->led_state = LED_RED;
            action->buzz_state = BUZZ_LONG;
            break;
        default:
            action->led_state = LED_OFF;
            action->buzz_state = BUZZ_OFF;
            break;
    }
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

void SensorTask(void *pvParameters)
{
    (void)pvParameters;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Telemetry telemetry;
    Action action;
    
    PRINTF("SensorTask started\r\n");
    
    /* Initialize ADC */
    ADC_Init();
    
    while (1) {
        /* Process noise events from ISR */
        ProcessNoiseEvents();
        
        /* Read LDR sensor */
        uint16_t adc_value = ADC_ReadLDR();
        uint16_t lux = ADC_ConvertToLux(adc_value);
        
        /* Count noise bursts in last 3 seconds */
        uint32_t current_time = xTaskGetTickCount();
        uint32_t window_start = current_time - pdMS_TO_TICKS(NOISE_WINDOW_MS);
        uint8_t noise_bursts = 0;
        
        for (uint8_t i = 0; i < MAX_NOISE_BURSTS_3S && i < noise_event_count; i++) {
            if (noise_events[i] >= window_start) {
                noise_bursts++;
            }
        }
        
        /* Prepare telemetry */
        telemetry.mode = current_mode;
        telemetry.lux = lux;
        telemetry.temp_c = latest_temp_c;
        telemetry.noise_b3s = noise_bursts;
        telemetry.score = CalculateComfortScore(lux, latest_temp_c, noise_bursts, current_mode);
        telemetry.state = DetermineState(telemetry.score);
        
        /* Send telemetry to CommsTask */
        if (xQueueSend(uplink_q, &telemetry, pdMS_TO_TICKS(10)) != pdTRUE) {
            PRINTF("Warning: Failed to send telemetry to uplink queue\r\n");
        }
        
        /* Generate action for ActuatorTask */
        GenerateAction(telemetry.state, &action);
        if (xQueueSend(downlink_q, &action, pdMS_TO_TICKS(10)) != pdTRUE) {
            PRINTF("Warning: Failed to send action to downlink queue\r\n");
        }
        
        /* Debug output */
        PRINTF("Sensor: LUX=%d, TEMP=%.1f, NOISE=%d, SCORE=%d, STATE=%d\r\n",
               lux, latest_temp_c, noise_bursts, telemetry.score, telemetry.state);
        
        /* Wait for next period */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}
