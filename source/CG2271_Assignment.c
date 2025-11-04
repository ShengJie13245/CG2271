/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    CG2271_Assignment.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/* Include sensor modules */
#include "ldr_sensor.h"

/* Sensor definitions */
#define SOUND_PIN      6    // PTC6 digital output from sound sensor

// LED pin numbers
#define RED_PIN		31	// PTE31
#define GREEN_PIN	5	// PTD5
#define BLUE_PIN	29	// PTE29

typedef enum tl {
	RED, GREEN, BLUE
} TLED;

/* Function prototypes */
void initSoundSensor(void);
void initGPIO(void);
void ledOn(TLED led);
void ledOff(TLED led);
void PORTC_PORTD_IRQHandler(void);
void delay(uint32_t count);

/*
 * @brief   Application entry point.
 */

void initGPIO() {
    // Set up the clock gating
    SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);

    // Set up the pin PCR values
    PORTD->PCR[GREEN_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[GREEN_PIN] = PORT_PCR_MUX(1);
    GPIOD->PDDR |= (1 << GREEN_PIN);

    PORTE->PCR[RED_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[BLUE_PIN] &= ~PORT_PCR_MUX_MASK;

    PORTE->PCR[RED_PIN] = PORT_PCR_MUX(1);
    PORTE->PCR[BLUE_PIN] = PORT_PCR_MUX(1);
    GPIOE->PDDR |= (1 << BLUE_PIN);
    GPIOE->PDDR |= (1 << RED_PIN);

    // All LEDs OFF at startup (active-low -> drive high)
    GPIOE->PSOR = (1 << RED_PIN) | (1 << BLUE_PIN);
    GPIOD->PSOR = (1 << GREEN_PIN);
}

void initSoundSensor(void) {
    NVIC_DisableIRQ(PORTC_PORTD_IRQn);

    // Enable clock gating for PORTC
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    // Configure PTC6 as GPIO input with rising edge interrupt
    PORTC->PCR[SOUND_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[SOUND_PIN] |= PORT_PCR_MUX(1); // GPIO

    // Rising edge interrupt: IRQC = 1001
    PORTC->PCR[SOUND_PIN] &= ~PORT_PCR_IRQC_MASK;
    PORTC->PCR[SOUND_PIN] |= PORT_PCR_IRQC(0b1001);

    GPIOC->PDDR &= ~(1 << SOUND_PIN); // input mode

    // Enable PORTC interrupt
    PORTC->ISFR = (1 << SOUND_PIN);
    NVIC_SetPriority(PORTC_PORTD_IRQn, 2);
    NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void ledOn(TLED led) {
    switch(led) {
    case RED:
        GPIOE->PCOR |= (1 << RED_PIN);
        break;
    case GREEN:
        GPIOD->PCOR |= (1 << GREEN_PIN);
        break;
    case BLUE:
        GPIOE->PCOR |= (1 << BLUE_PIN);
        break;
    }
}

void ledOff(TLED led) {
    switch(led) {
    case RED:
        GPIOE->PSOR |= (1 << RED_PIN);
        break;
    case GREEN:
        GPIOD->PSOR |= (1 << GREEN_PIN);
        break;
    case BLUE:
        GPIOE->PSOR |= (1 << BLUE_PIN);
        break;
    }
}

void PORTC_PORTD_IRQHandler(void) {
    if (PORTC->ISFR & (1 << SOUND_PIN)) {
        PORTC->ISFR = (1 << SOUND_PIN);  // clear flag
        GPIOE->PTOR = (1 << BLUE_PIN);   // toggle blue LED
        PRINTF("Sound detected!\r\n");
    }
}

void delay(uint32_t count) {
    volatile uint32_t i = 0;
    for (i = 0; i < count; ++i) {
        __asm("NOP"); /* delay */
    }
}

int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("=== CG2271 Multi-Sensor System ===\r\n");
    PRINTF("LDR Sensor: PTE20 (ADC0_SE0)\r\n");
    PRINTF("Sound Sensor: PTC6 (Digital)\r\n");
    
    /* Initialize GPIO and LEDs */
    initGPIO();
    ledOff(RED);
    ledOff(GREEN);
    ledOff(BLUE);
    
    /* Initialize all sensor modules */
    PRINTF("Initializing sensors...\r\n");
    
    // Initialize LDR sensor (using modular approach)
    LDR_Init();
    
    // Initialize Sound sensor
    initSoundSensor();
    PRINTF("Sound sensor initialized\r\n");
    
    PRINTF("All sensors ready! Make some noise to test sound detection.\r\n");

    /* Enter an infinite loop with controlled readings */
    while(1) {
        // Trigger a single ADC reading for LDR (using modular function)
        LDR_TriggerReading();
        
        // Wait for a reasonable interval between readings (about 1 second)
        delay(8000000); // Adjust delay as needed for your desired reading frequency
    }
    return 0 ;
}


