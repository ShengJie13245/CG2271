/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mode_task.h"
#include "app_config.h"
#include "messages.h"
#include "actuator_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

void ModeTask(void *pvParameters)
{
    (void)pvParameters;
    
    PRINTF("ModeTask started\r\n");
    
    while (1) {
        /* Wait for button press semaphore */
        if (xSemaphoreTake(button_sem, portMAX_DELAY) == pdTRUE) {
            /* Toggle mode */
            if (current_mode == MODE_RELAX) {
                current_mode = MODE_FOCUS;
                PRINTF("Mode changed to FOCUS\r\n");
            } else {
                current_mode = MODE_RELAX;
                PRINTF("Mode changed to RELAX\r\n");
            }
            
            /* Signal mode change to actuator task for visual/audio feedback */
            ActuatorTask_SignalModeChange();
            
            /* Small delay to prevent button bounce */
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}
