/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "actuator_task.h"
#include "app_config.h"
#include "messages.h"
#include "drivers_gpio_pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
static LedState current_led_state = LED_OFF;
static uint32_t led_animation_counter = 0;
static uint32_t buzzer_timer = 0;
static bool buzzer_active = false;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

static void UpdateLEDAnimation(void)
{
    led_animation_counter++;
    
    switch (current_led_state) {
        case LED_OFF:
            LED_SetState(LED_OFF);
            break;
            
        case LED_GREEN:
            /* Breathing green effect */
            if ((led_animation_counter / 10) % 2 == 0) {
                LED_SetState(LED_GREEN);
            } else {
                LED_SetState(LED_OFF);
            }
            break;
            
        case LED_YELLOW:
            /* Heartbeat yellow effect */
            uint32_t heartbeat_phase = led_animation_counter % 20;
            if (heartbeat_phase < 2 || (heartbeat_phase >= 4 && heartbeat_phase < 6)) {
                LED_SetState(LED_YELLOW);
            } else {
                LED_SetState(LED_OFF);
            }
            break;
            
        case LED_RED:
            /* Flashing red */
            if ((led_animation_counter / 5) % 2 == 0) {
                LED_SetState(LED_RED);
            } else {
                LED_SetState(LED_OFF);
            }
            break;
            
        case LED_CYAN:
            /* Cyan sweep for mode change */
            LED_SetState(LED_CYAN);
            break;
            
        default:
            LED_SetState(LED_OFF);
            break;
    }
}

static void UpdateBuzzer(void)
{
    if (buzzer_active) {
        buzzer_timer++;
        
        /* Simple timer-based buzzer control */
        if (buzzer_timer >= 50) {  /* ~500ms at 10ms task period */
            Buzzer_SetFrequency(0);  /* Turn off */
            buzzer_active = false;
            buzzer_timer = 0;
        }
    }
}

static void ProcessBuzzerCommand(BuzzerState buzz_state)
{
    static uint32_t last_short_beep = 0;
    uint32_t current_time = xTaskGetTickCount();
    
    switch (buzz_state) {
        case BUZZ_OFF:
            Buzzer_SetFrequency(0);
            buzzer_active = false;
            break;
            
        case BUZZ_SHORT:
            /* Short beep every 10 seconds */
            if (current_time - last_short_beep > pdMS_TO_TICKS(10000)) {
                Buzzer_PlayPattern(BUZZ_SHORT);
                last_short_beep = current_time;
            }
            break;
            
        case BUZZ_LONG:
            /* Long beep once */
            if (!buzzer_active) {
                Buzzer_PlayPattern(BUZZ_LONG);
                buzzer_active = true;
                buzzer_timer = 0;
            }
            break;
            
        case BUZZ_CHIME:
            /* Two-note chime for mode change */
            Buzzer_PlayPattern(BUZZ_CHIME);
            break;
            
        default:
            break;
    }
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

void ActuatorTask(void *pvParameters)
{
    (void)pvParameters;
    
    Action action;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    PRINTF("ActuatorTask started\r\n");
    
    /* Initialize GPIO and PWM */
    GPIO_Init();
    PWM_Init();
    
    /* Start with LEDs off */
    LED_SetState(LED_OFF);
    Buzzer_SetFrequency(0);
    
    while (1) {
        /* Check for new action commands */
        if (xQueueReceive(downlink_q, &action, 0) == pdTRUE) {
            PRINTF("Actuator: LED=%d, BUZZ=%d\r\n", action.led_state, action.buzz_state);
            
            /* Update LED state */
            current_led_state = action.led_state;
            led_animation_counter = 0;  /* Reset animation */
            
            /* Process buzzer command */
            ProcessBuzzerCommand(action.buzz_state);
        }
        
        /* Update LED animations */
        UpdateLEDAnimation();
        
        /* Update buzzer state */
        UpdateBuzzer();
        
        /* Run at 100Hz for smooth animations */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

void ActuatorTask_SignalModeChange(void)
{
    Action mode_change_action = {
        .led_state = LED_CYAN,
        .buzz_state = BUZZ_CHIME
    };
    
    /* Send mode change indication to actuator task */
    xQueueSend(downlink_q, &mode_change_action, 0);
}
