/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    CG2271_Assignment.c
 * @brief   Smart Desk Assistant - Main Application Entry Point
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Application includes */
#include "app_config.h"
#include "messages.h"
#include "sensor_task.h"
#include "comms_task.h"
#include "actuator_task.h"
#include "mode_task.h"
#include "isr_inputs.h"

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/* FreeRTOS Objects */
SemaphoreHandle_t uart_mutex = NULL;
SemaphoreHandle_t button_sem = NULL;
QueueHandle_t noise_evt_q = NULL;
QueueHandle_t uplink_q = NULL;
QueueHandle_t downlink_q = NULL;

/* Global State Variables */
volatile Mode current_mode = MODE_RELAX;
volatile float latest_temp_c = 25.0f;  /* Default temperature */

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

static void CreateFreeRTOSObjects(void)
{
    /* Create mutex for UART protection */
    uart_mutex = xSemaphoreCreateMutex();
    if (uart_mutex == NULL) {
        PRINTF("ERROR: Failed to create UART mutex\r\n");
        while(1);
    }

    /* Create binary semaphore for button events */
    button_sem = xSemaphoreCreateBinary();
    if (button_sem == NULL) {
        PRINTF("ERROR: Failed to create button semaphore\r\n");
        while(1);
    }

    /* Create queue for noise events (ISR -> SensorTask) */
    noise_evt_q = xQueueCreate(NOISE_EVENT_QUEUE_SIZE, sizeof(uint32_t));
    if (noise_evt_q == NULL) {
        PRINTF("ERROR: Failed to create noise event queue\r\n");
        while(1);
    }

    /* Create queue for uplink data (SensorTask -> CommsTask) */
    uplink_q = xQueueCreate(UPLINK_QUEUE_SIZE, sizeof(Telemetry));
    if (uplink_q == NULL) {
        PRINTF("ERROR: Failed to create uplink queue\r\n");
        while(1);
    }

    /* Create queue for downlink data (SensorTask -> ActuatorTask) */
    downlink_q = xQueueCreate(DOWNLINK_QUEUE_SIZE, sizeof(Action));
    if (downlink_q == NULL) {
        PRINTF("ERROR: Failed to create downlink queue\r\n");
        while(1);
    }

    PRINTF("FreeRTOS objects created successfully\r\n");
}

static void CreateTasks(void)
{
    BaseType_t result;

    /* Create SensorTask */
    result = xTaskCreate(SensorTask, "SensorTask", SENSOR_TASK_STACK_SIZE, 
                        NULL, SENSOR_TASK_PRIORITY, NULL);
    if (result != pdPASS) {
        PRINTF("ERROR: Failed to create SensorTask\r\n");
        while(1);
    }

    /* Create CommsTask */
    result = xTaskCreate(CommsTask, "CommsTask", COMMS_TASK_STACK_SIZE, 
                        NULL, COMMS_TASK_PRIORITY, NULL);
    if (result != pdPASS) {
        PRINTF("ERROR: Failed to create CommsTask\r\n");
        while(1);
    }

    /* Create ActuatorTask */
    result = xTaskCreate(ActuatorTask, "ActuatorTask", ACTUATOR_TASK_STACK_SIZE, 
                        NULL, ACTUATOR_TASK_PRIORITY, NULL);
    if (result != pdPASS) {
        PRINTF("ERROR: Failed to create ActuatorTask\r\n");
        while(1);
    }

    /* Create ModeTask */
    result = xTaskCreate(ModeTask, "ModeTask", MODE_TASK_STACK_SIZE, 
                        NULL, MODE_TASK_PRIORITY, NULL);
    if (result != pdPASS) {
        PRINTF("ERROR: Failed to create ModeTask\r\n");
        while(1);
    }

    PRINTF("All tasks created successfully\r\n");
}

/*******************************************************************************
 * Main Function
 ******************************************************************************/

int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("\r\n=== Smart Desk Assistant Starting ===\r\n");
    PRINTF("FreeRTOS Version: %s\r\n", tskKERNEL_VERSION_NUMBER);
    PRINTF("System Clock: %d Hz\r\n", SystemCoreClock);

    /* Initialize interrupt service routines */
    ISR_Init();
    PRINTF("ISR initialization complete\r\n");

    /* Create FreeRTOS objects */
    CreateFreeRTOSObjects();

    /* Create application tasks */
    CreateTasks();

    PRINTF("Starting FreeRTOS scheduler...\r\n");
    PRINTF("Initial mode: %s\r\n", (current_mode == MODE_FOCUS) ? "FOCUS" : "RELAX");

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never reach here */
    PRINTF("ERROR: FreeRTOS scheduler returned!\r\n");
    while(1) {
        __asm volatile ("nop");
    }

    return 0;
}
