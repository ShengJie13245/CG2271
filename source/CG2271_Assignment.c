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

// LED pin numbers
#define RED_PIN		5	// PTD5
#define GREEN_PIN	7	// PTD7
#define BLUE_PIN	6	// PTD6

//Buzzer pin numbers
#define BUZZER_PIN 30   // PTE31

typedef enum tl {
	RED, GREEN, BLUE
} TLED;


void initGPIO() {

	// LED
	// Set up the clock gating
	SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK |
			SIM_SCGC5_PORTE_MASK);

	// Set up the pin PCR values
	PORTD->PCR[GREEN_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_PIN] = PORT_PCR_MUX(1); //Set to ALT1
	GPIOD->PDDR |= (1 << GREEN_PIN); //Set as output

	PORTD->PCR[RED_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_PIN] &= ~PORT_PCR_MUX_MASK;

	PORTD->PCR[RED_PIN] = PORT_PCR_MUX(1);
	PORTD->PCR[BLUE_PIN] = PORT_PCR_MUX(1);
	GPIOD->PDDR |= (1 << BLUE_PIN);
	GPIOD->PDDR |= (1 << RED_PIN);
}

// Configure the MCG Internal Reference Clock
void setMCGIRClk() {

	MCG->C1 &= ~MCG_C1_CLKS_MASK;
	// Choose MCG clock source of 01 for LIRC
	// and set IRCLKEN to 1 to enable LIRC
	MCG->C1 |= ((MCG_C1_CLKS(0b01) | MCG_C1_IRCLKEN_MASK));

	// Set IRCS to 1 to choose 8 MHz clock
	MCG->C2 |= MCG_C2_IRCS_MASK;

	// Choose FCRDIV of 0 for divisor of 1
	MCG->SC &= ~MCG_SC_FCRDIV_MASK;
	MCG->SC |= MCG_SC_FCRDIV(0b0);

	// Choose LIRC_DIV2 of 0 for divisor of 1
	MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
	MCG->MC |= MCG_MC_LIRC_DIV2(0b0);

}

void setTPMClock(){

	// Set MCGIRCLK
	setMCGIRClk();

	// Choose MCGIRCLK (8 MHz)
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);

}

// Buzzer: PTE30 TPM0 CH 3 ALT3

void initPWM() {

	// Turn on clock gating to TPM0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	// Turn on clock gating to Port D and E
	// Set up the clock gating
	SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK |
			SIM_SCGC5_PORTE_MASK);


	// Set the pin multiplexors
	PORTE->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;


	// PWM
	PORTE->PCR[BUZZER_PIN] |= PORT_PCR_MUX(0b11);

	// Set pins to output
	GPIOE->PDDR |= (1 << BUZZER_PIN);

	// Set up TPM0
	// Turn off TPM0 and clear the prescalar field
	TPM0->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);

	// Set prescalar of 128
	TPM0->SC |= TPM_SC_PS(0b011);

	// Select centre-aligned PWM mode
	TPM0->SC |= TPM_SC_CPWMS_MASK;

	// Initialize count to 0
	TPM0->CNT = 0;

	// We nominally choose a PWM frequency of 4KHz
	// TPM frequency = 128/8000 = 0.016
	// For 250Hz, our mod value = 4 / (2 x 0.016)
	TPM0->MOD=125;

	// Configure channel 3
	// MS=10, ELS=10.
	// Note that this configures a PWM signal

	TPM0->CONTROLS[3].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_ELSA_MASK);
	TPM0->CONTROLS[3].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSB(1));
}

void startPWM() {
	TPM0->SC |= TPM_SC_CMOD(0b01);
}

void stopPWM() {
	TPM0->SC &= ~TPM_SC_CMOD_MASK;
}

void setBuzzer(int percent, int frequency) {
	int clock_freq = CLOCK_GetBusClkFreq();
	int mod = (int) ((clock_freq/(frequency*2*8)));
	TPM0->MOD = mod;
	int value = (int)((percent / 100.0) * (double) TPM0->MOD);
	TPM0->CONTROLS[3].CnV = value; //set duty cycle on channel 3

}

void ledOff(TLED led) {
	switch(led) {
	case RED:
		GPIOD->PCOR |= (1 << RED_PIN);
		break;

	case GREEN:
		GPIOD->PCOR |= (1 << GREEN_PIN);
		break;

	case BLUE:
		GPIOD->PCOR |= (1 << BLUE_PIN);
		break;
	}
}

void ledOn(TLED led) {
	switch(led) {
	case RED:
		GPIOD->PSOR |= (1 << RED_PIN);
		break;

	case GREEN:
		GPIOD->PSOR |= (1 << GREEN_PIN);
		break;

	case BLUE:
		GPIOD->PSOR |= (1 << BLUE_PIN);
		break;
	}
}

/*
 * @brief   Application entry point.
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

    PRINTF("LED Demo\r\n");

    /* Force the counter to be placed into memory. */
    /* Enter an infinite loop, just incrementing a counter. */
    initGPIO();
    setTPMClock();
    initPWM();
    setBuzzer(90, 500);//set different values for different volume (duty cycle, frequency up to 2000)

    ledOn(RED);
    ledOn(GREEN);
    ledOn(BLUE);// set led when needed
    startPWM();
    //stopPWM() //to stop buzzer

    return 0 ;
}
