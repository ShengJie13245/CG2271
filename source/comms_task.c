/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "comms_task.h"
#include "app_config.h"
#include "messages.h"
#include "drivers_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

static void ProcessIncomingData(void)
{
    char line[UART_BUFFER_SIZE];
    float temperature;
    
    /* Check for incoming UART data */
    while (UART_ReceiveLine(line, sizeof(line))) {
        PRINTF("Received: %s\r\n", line);
        
        /* Try to parse temperature */
        if (UART_ParseTemperature(line, &temperature)) {
            latest_temp_c = temperature;
            PRINTF("Updated temperature: %.1f°C\r\n", temperature);
        }
    }
}

static void SendTelemetryData(void)
{
    Telemetry telemetry;
    
    /* Check for telemetry data from SensorTask */
    if (xQueueReceive(uplink_q, &telemetry, 0) == pdTRUE) {
        /* Take mutex to protect UART access */
        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            /* Send telemetry to ESP32 */
            UART_SendTelemetry(&telemetry);
            
            /* Release mutex */
            xSemaphoreGive(uart_mutex);
            
            PRINTF("Sent telemetry: MODE=%s, STATE=%d, SCORE=%d\r\n",
                   (telemetry.mode == MODE_FOCUS) ? "FOCUS" : "RELAX",
                   telemetry.state, telemetry.score);
        } else {
            PRINTF("Warning: Failed to acquire UART mutex\r\n");
        }
    }
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

void CommsTask(void *pvParameters)
{
    (void)pvParameters;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    PRINTF("CommsTask started\r\n");
    
    /* Initialize UART */
    UART_Init();
    
    while (1) {
        /* Process incoming data from ESP32 */
        ProcessIncomingData();
        
        /* Send outgoing telemetry data */
        SendTelemetryData();
        
        /* Wait for next period */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(COMMS_TASK_PERIOD_MS));
    }
}
