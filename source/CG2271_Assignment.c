/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    CG2271_Assignment.c
 * @brief   Main application entry point - CG2271 Group Project
 * @author  CG2271 Group
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* Include sensor modules */
#include "ldr_sensor.h"
// TODO: Add other sensor modules here as group members develop them
// #include "temperature_sensor.h"
// #include "sound_sensor.h"

/*
 * @brief   Main application entry point
 */
int main(void) {
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("=== CG2271 Group Project - Multi-Sensor System ===\r\n");
    
    /* Initialize all sensor modules */
    LDR_Init();
    
    // TODO: Initialize other sensors as group members add them
    // Temperature_Init();
    // Sound_Init();
    
    PRINTF("All sensors initialized. Starting readings...\r\n");

    /* Start sensor readings */
    LDR_StartReading(); // This function contains the main loop
    
    return 0;
}
