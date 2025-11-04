/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "drivers_uart.h"
#include "app_config.h"
#include "fsl_lpuart.h"
#include "fsl_port.h"
#include "fsl_clock.h"
#include <string.h>
#include <stdio.h>

/*******************************************************************************
 * Variables
 ******************************************************************************/
static bool uart_initialized = false;
static char rx_buffer[UART_BUFFER_SIZE];
static volatile uint16_t rx_index = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

void UART_Init(void)
{
    if (uart_initialized) {
        return;
    }

    lpuart_config_t config;

    /* Enable PORTC clock */
    CLOCK_EnableClock(kCLOCK_PortC);

    /* Configure UART pins */
    PORT_SetPinMux(ESP32_UART_TX_PORT, ESP32_UART_TX_PIN, kPORT_MuxAlt3);
    PORT_SetPinMux(ESP32_UART_RX_PORT, ESP32_UART_RX_PIN, kPORT_MuxAlt3);

    /* Get default UART configuration */
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = UART_BAUD_RATE;
    config.enableTx = true;
    config.enableRx = true;

    /* Initialize UART */
    LPUART_Init(ESP32_UART, &config, ESP32_UART_CLK_FREQ);

    /* Enable RX interrupt */
    LPUART_EnableInterrupts(ESP32_UART, kLPUART_RxDataRegFullInterruptEnable);
    EnableIRQ(ESP32_UART_IRQ);

    /* Clear RX buffer */
    memset(rx_buffer, 0, sizeof(rx_buffer));
    rx_index = 0;

    uart_initialized = true;
}

void UART_SendString(const char* str)
{
    if (!uart_initialized) {
        UART_Init();
    }

    LPUART_WriteBlocking(ESP32_UART, (const uint8_t*)str, strlen(str));
}

void UART_SendTelemetry(const Telemetry* telemetry)
{
    if (!uart_initialized) {
        UART_Init();
    }

    char buffer[128];
    const char* mode_str = (telemetry->mode == MODE_FOCUS) ? "FOCUS" : "RELAX";
    const char* state_str;
    
    switch (telemetry->state) {
        case STATE_GOOD:  state_str = "GOOD"; break;
        case STATE_WARN:  state_str = "WARN"; break;
        case STATE_ALERT: state_str = "ALERT"; break;
        default:          state_str = "UNKNOWN"; break;
    }

    snprintf(buffer, sizeof(buffer), 
             "L=%d;S=%d;MODE=%s;STATE=%s;SCORE=%d;\n",
             telemetry->lux,
             telemetry->noise_b3s,
             mode_str,
             state_str,
             telemetry->score);

    UART_SendString(buffer);
}

bool UART_ReceiveLine(char* line, uint16_t max_length)
{
    if (!uart_initialized) {
        UART_Init();
    }

    /* Look for complete line (ending with \n) in buffer */
    for (uint16_t i = 0; i < rx_index; i++) {
        if (rx_buffer[i] == '\n') {
            /* Found complete line */
            uint16_t line_length = (i < max_length - 1) ? i : max_length - 1;
            memcpy(line, rx_buffer, line_length);
            line[line_length] = '\0';

            /* Remove processed line from buffer */
            if (i + 1 < rx_index) {
                memmove(rx_buffer, &rx_buffer[i + 1], rx_index - i - 1);
                rx_index -= (i + 1);
            } else {
                rx_index = 0;
            }

            return true;
        }
    }

    return false;  /* No complete line available */
}

bool UART_ParseTemperature(const char* line, float* temperature)
{
    /* Parse "TEMP=xx.x;" format */
    if (strncmp(line, "TEMP=", 5) == 0) {
        char* end_ptr;
        float temp = strtof(&line[5], &end_ptr);
        
        if (end_ptr != &line[5] && *end_ptr == ';') {
            *temperature = temp;
            return true;
        }
    }
    
    return false;
}

void LPUART0_IRQHandler(void)
{
    uint8_t data;

    /* Check if RX data is available */
    if ((kLPUART_RxDataRegFullFlag) & LPUART_GetStatusFlags(ESP32_UART)) {
        data = LPUART_ReadByte(ESP32_UART);
        
        /* Add to buffer if there's space */
        if (rx_index < (UART_BUFFER_SIZE - 1)) {
            rx_buffer[rx_index++] = data;
        } else {
            /* Buffer overflow - reset */
            rx_index = 0;
        }
    }
}
