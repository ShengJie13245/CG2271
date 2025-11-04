/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "isr_inputs.h"
#include "app_config.h"
#include "messages.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
static bool isr_initialized = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

void ISR_Init(void)
{
    if (isr_initialized) {
        return;
    }

    gpio_pin_config_t input_config = {
        kGPIO_DigitalInput,
        0,
    };

    /* Enable PORT clocks */
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortC);

    /* Configure sound sensor pin (PTC6) */
    PORT_SetPinMux(SOUND_PORT, SOUND_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(SOUND_GPIO, SOUND_PIN, &input_config);
    
    /* Configure sound sensor interrupt - rising edge */
    PORT_SetPinInterruptConfig(SOUND_PORT, SOUND_PIN, kPORT_InterruptRisingEdge);
    EnableIRQ(SOUND_IRQ);

    /* Configure button pin (PTA4) */
    PORT_SetPinMux(BUTTON_PORT, BUTTON_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(BUTTON_GPIO, BUTTON_PIN, &input_config);
    
    /* Configure button interrupt - falling edge (button press) */
    PORT_SetPinInterruptConfig(BUTTON_PORT, BUTTON_PIN, kPORT_InterruptFallingEdge);
    EnableIRQ(BUTTON_IRQ);

    isr_initialized = true;
}

void PORTC_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    /* Check if PTC6 (sound sensor) triggered the interrupt */
    if (PORTC->ISFR & (1U << SOUND_PIN)) {
        /* Clear interrupt flag */
        PORTC->ISFR |= (1U << SOUND_PIN);
        
        /* Get current tick count */
        uint32_t timestamp = xTaskGetTickCountFromISR();
        
        /* Send timestamp to noise event queue */
        if (noise_evt_q != NULL) {
            xQueueSendFromISR(noise_evt_q, &timestamp, &xHigherPriorityTaskWoken);
        }
    }
    
    /* Yield to higher priority task if necessary */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void PORTA_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    /* Check if PTA4 (button) triggered the interrupt */
    if (PORTA->ISFR & (1U << BUTTON_PIN)) {
        /* Clear interrupt flag */
        PORTA->ISFR |= (1U << BUTTON_PIN);
        
        /* Give semaphore to signal button press */
        if (button_sem != NULL) {
            xSemaphoreGiveFromISR(button_sem, &xHigherPriorityTaskWoken);
        }
    }
    
    /* Yield to higher priority task if necessary */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
