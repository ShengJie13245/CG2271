/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "drivers_gpio_pwm.h"
#include "app_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_clock.h"
#include "fsl_device_registers.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
static bool gpio_initialized = false;
static bool pwm_initialized = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

void GPIO_Init(void)
{
    if (gpio_initialized) {
        return;
    }

    gpio_pin_config_t gpio_config = {
        kGPIO_DigitalOutput,
        0,
    };

    /* Enable PORT clocks */
    CLOCK_EnableClock(kCLOCK_PortD);

    /* Configure RGB LED pins as GPIO outputs */
    PORT_SetPinMux(LED_R_PORT, LED_R_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(LED_G_PORT, LED_G_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(LED_B_PORT, LED_B_PIN, kPORT_MuxAsGpio);

    /* Initialize GPIO pins */
    GPIO_PinInit(LED_R_GPIO, LED_R_PIN, &gpio_config);
    GPIO_PinInit(LED_G_GPIO, LED_G_PIN, &gpio_config);
    GPIO_PinInit(LED_B_GPIO, LED_B_PIN, &gpio_config);

    /* Turn off all LEDs initially */
    GPIO_PinWrite(LED_R_GPIO, LED_R_PIN, 1);  /* Active low */
    GPIO_PinWrite(LED_G_GPIO, LED_G_PIN, 1);  /* Active low */
    GPIO_PinWrite(LED_B_GPIO, LED_B_PIN, 1);  /* Active low */

    gpio_initialized = true;
}

void PWM_Init(void)
{
    if (pwm_initialized) {
        return;
    }

    /* Enable TPM0 clock */
    CLOCK_EnableClock(kCLOCK_Tpm0);
    CLOCK_SetTpmClock(1U);

    /* Enable PORTC clock for buzzer */
    CLOCK_EnableClock(kCLOCK_PortC);

    /* Configure buzzer pin for TPM PWM */
    PORT_SetPinMux(BUZZER_PORT, BUZZER_PIN, kPORT_MuxAlt4);

    /* Initialize TPM0 for PWM */
    /* Disable TPM first */
    BUZZER_TPM->SC = 0;
    
    /* Set prescaler to divide by 16 and use system clock */
    BUZZER_TPM->SC = TPM_SC_PS(4);  /* Prescaler = 16 */
    
    /* Set modulo value for 1kHz PWM (assuming 48MHz system clock) */
    /* 48MHz / 16 / 3000 = 1kHz */
    BUZZER_TPM->MOD = 3000;
    
    /* Configure channel for PWM */
    BUZZER_TPM->CONTROLS[BUZZER_TPM_CHANNEL].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
    BUZZER_TPM->CONTROLS[BUZZER_TPM_CHANNEL].CnV = 0;  /* 0% duty cycle initially */
    
    /* Start TPM */
    BUZZER_TPM->SC |= TPM_SC_CMOD(1);  /* Enable TPM with system clock */

    pwm_initialized = true;
}

void LED_SetColor(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!gpio_initialized) {
        GPIO_Init();
    }

    /* LEDs are active low, so invert the values */
    GPIO_PinWrite(LED_R_GPIO, LED_R_PIN, red ? 0 : 1);
    GPIO_PinWrite(LED_G_GPIO, LED_G_PIN, green ? 0 : 1);
    GPIO_PinWrite(LED_B_GPIO, LED_B_PIN, blue ? 0 : 1);
}

void LED_SetState(LedState state)
{
    switch (state) {
        case LED_OFF:
            LED_SetColor(0, 0, 0);
            break;
        case LED_GREEN:
            LED_SetColor(0, 1, 0);
            break;
        case LED_YELLOW:
            LED_SetColor(1, 1, 0);
            break;
        case LED_RED:
            LED_SetColor(1, 0, 0);
            break;
        case LED_CYAN:
            LED_SetColor(0, 1, 1);
            break;
        default:
            LED_SetColor(0, 0, 0);
            break;
    }
}

void Buzzer_SetFrequency(uint32_t frequency)
{
    if (!pwm_initialized) {
        PWM_Init();
    }

    if (frequency == 0) {
        /* Turn off buzzer */
        BUZZER_TPM->CONTROLS[BUZZER_TPM_CHANNEL].CnV = 0;
    } else {
        /* Calculate modulo value for desired frequency */
        /* Assuming 48MHz system clock with prescaler of 16 */
        uint32_t mod_value = (48000000U / 16U) / frequency;
        if (mod_value > 0xFFFF) mod_value = 0xFFFF;  /* Clamp to 16-bit */
        
        /* Update modulo and duty cycle (50%) */
        BUZZER_TPM->MOD = mod_value;
        BUZZER_TPM->CONTROLS[BUZZER_TPM_CHANNEL].CnV = mod_value / 2;  /* 50% duty cycle */
    }
}

void Buzzer_Beep(uint32_t frequency, uint32_t duration_ms)
{
    Buzzer_SetFrequency(frequency);
    
    /* Simple blocking delay - in a real application you might use a timer */
    for (volatile uint32_t i = 0; i < (duration_ms * 1000); i++) {
        __NOP();
    }
    
    Buzzer_SetFrequency(0);  /* Turn off */
}

void Buzzer_PlayPattern(BuzzerState pattern)
{
    switch (pattern) {
        case BUZZ_OFF:
            Buzzer_SetFrequency(0);
            break;
        case BUZZ_SHORT:
            Buzzer_Beep(1000, 100);  /* 1kHz for 100ms */
            break;
        case BUZZ_LONG:
            Buzzer_Beep(800, 500);   /* 800Hz for 500ms */
            break;
        case BUZZ_CHIME:
            Buzzer_Beep(1000, 200);  /* First note */
            Buzzer_Beep(1500, 200);  /* Second note */
            break;
        default:
            break;
    }
}
