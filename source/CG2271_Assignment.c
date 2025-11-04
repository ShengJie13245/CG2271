/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    timer_demo.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

// LED pin numbers
#define RED_PIN		31	// PTE31
#define GREEN_PIN	5	// PTD5
#define BLUE_PIN	29	// PTE29

#define SOUND_PIN   6   // PTC6 digital output from sound sensor

typedef enum tl {
	RED, GREEN, BLUE
} TLED;

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

void initSoundSensor(void)
{
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

void PORTC_PORTD_IRQHandler(void)
{
	if (PORTC->ISFR & (1 << SOUND_PIN)) {
	        PORTC->ISFR = (1 << SOUND_PIN);  // clear flag
	        GPIOE->PTOR = (1 << BLUE_PIN);
	        PRINTF("Sound detected!\r\n");
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

    initGPIO();
    //initTimer();
    PRINTF("TIMER DEMO\r\n");
   // startTimer();

    ledOff(RED);
    ledOff(GREEN);
    ledOff(BLUE);

    initSoundSensor();
    PRINTF("Sound sensor test\r\n");

    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
    	__WFI();   // sleep until an interrupt
    }
    return 0 ;
}

