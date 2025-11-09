
/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    CG2271_Assignment.c
 * @brief   Application entry point with FreeRTOS support.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Include sensor modules */
#include "ldr_sensor.h"

#define SOUND_PIN      6
#define RED_PIN		4
#define GREEN_PIN	7
#define BLUE_PIN	6
#define BUZZER_PIN 30

typedef enum tl {
	RED, GREEN, BLUE
} TLED;


/* Function prototypes */
void initSoundSensor(void);
void initGPIO(void);
void setMCGIRClk(void);
void setTPMClock(void);
void initPWM(void);
void startPWM(void);
void stopPWM(void);
void setBuzzer(int percent, int frequency);
void ledOn(TLED led);
void ledOff(TLED led);
void PORTC_PORTD_IRQHandler(void);
void delay(uint32_t count);

static void sensorTask(void *pvParameters);
static void ledTask(void *pvParameters);
static void buzzerTask(void *pvParameters);

/*
 * @brief   Application entry point.
 */

void initGPIO() {
    SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);

    PORTD->PCR[RED_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[GREEN_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTD->PCR[BLUE_PIN] &= ~PORT_PCR_MUX_MASK;

    PORTD->PCR[RED_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[GREEN_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[BLUE_PIN] = PORT_PCR_MUX(1);

    GPIOD->PDDR |= (1 << RED_PIN);
    GPIOD->PDDR |= (1 << GREEN_PIN);
    GPIOD->PDDR |= (1 << BLUE_PIN);

    ledOff(RED);
    ledOff(GREEN);
    ledOff(BLUE);
}

void setMCGIRClk() {
    MCG->C1 &= ~MCG_C1_CLKS_MASK;
    MCG->C1 |= ((MCG_C1_CLKS(0b01) | MCG_C1_IRCLKEN_MASK));
    MCG->C2 |= MCG_C2_IRCS_MASK;
    MCG->SC &= ~MCG_SC_FCRDIV_MASK;
    MCG->SC |= MCG_SC_FCRDIV(0b0);
    MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
    MCG->MC |= MCG_MC_LIRC_DIV2(0b0);
}

void setTPMClock(){
    setMCGIRClk();
    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);
}

void initPWM() {
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    PORTE->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[BUZZER_PIN] |= PORT_PCR_MUX(0b11);
    GPIOE->PDDR |= (1 << BUZZER_PIN);

    TPM0->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
    TPM0->SC |= TPM_SC_PS(0b011);
    TPM0->SC |= TPM_SC_CPWMS_MASK;
    TPM0->CNT = 0;
    TPM0->MOD = 125;

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
    TPM0->CONTROLS[3].CnV = value;
}

void initSoundSensor(void) {
    NVIC_DisableIRQ(PORTC_PORTD_IRQn);
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

    PORTC->PCR[SOUND_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTC->PCR[SOUND_PIN] |= PORT_PCR_MUX(1);
    PORTC->PCR[SOUND_PIN] &= ~PORT_PCR_IRQC_MASK;
    PORTC->PCR[SOUND_PIN] |= PORT_PCR_IRQC(0b1001);
    GPIOC->PDDR &= ~(1 << SOUND_PIN);

    PORTC->ISFR = (1 << SOUND_PIN);
    NVIC_SetPriority(PORTC_PORTD_IRQn, 2);
    NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
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


SemaphoreHandle_t ledSema;
SemaphoreHandle_t buzzerSema;
TLED currentLED = GREEN;

void PORTC_PORTD_IRQHandler(void) {
    static uint32_t last_sound_time = 0;

    if (PORTC->ISFR & (1 << SOUND_PIN)) {
        PORTC->ISFR = (1 << SOUND_PIN);
        
        uint32_t current_time = xTaskGetTickCountFromISR();
        if ((current_time - last_sound_time) > pdMS_TO_TICKS(500)) {
            last_sound_time = current_time;
            PRINTF("Sound Detected\r\n");

            BaseType_t hpw = pdFALSE;
            xSemaphoreGiveFromISR(ledSema, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
    }
}

void delay(uint32_t count) {
    volatile uint32_t i = 0;
    for (i = 0; i < count; ++i) {
        __asm("NOP");
    }
}

static void sensorTask(void *pvParameters) {
    (void)pvParameters;
    PRINTF("Sensor Task started\r\n");

    while(1) {
        LDR_TriggerReading();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


static void ledTask(void *p){
    while(1) {
        if(xSemaphoreTake(ledSema,portMAX_DELAY) == pdTRUE){
            if(currentLED == RED){
                ledOff(RED);
                delay(50000);
                ledOn(RED);
                delay(50000);
                ledOff(RED);
                delay(50000);
                ledOn(RED);
            } else if(currentLED == BLUE){
                ledOff(BLUE);
                delay(50000);
                ledOn(BLUE);
                delay(50000);
                ledOff(BLUE);
                delay(50000);
                ledOn(BLUE);
            } else if(currentLED == GREEN){
                ledOff(GREEN);
                delay(50000);
                ledOn(GREEN);
                delay(50000);
                ledOff(GREEN);
                delay(50000);
                ledOn(GREEN);
            }
        }
    }
}

static void buzzerTask(void *p){
    while(1) {
        if(xSemaphoreTake(buzzerSema, portMAX_DELAY) == pdTRUE){
            setBuzzer(60, 800);
            startPWM();
            delay(150000);
            stopPWM();
            delay(50000);
            
            setBuzzer(60, 1000);
            startPWM();
            delay(150000);
            stopPWM();
        }
    }
}

int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    PRINTF("=== CG2271 Multi-Sensor System with FreeRTOS ===\r\n");
    PRINTF("LDR Sensor: PTE20 (ADC0_SE0)\r\n");
    PRINTF("Sound Sensor: PTC6 (Digital)\r\n");
    PRINTF("Buzzer: PTE30 (TPM0_CH3)\r\n");
    PRINTF("LEDs: PTD4(Red), PTD7(Green), PTD6(Blue)\r\n");
    
    initGPIO();
    setTPMClock();
    initPWM();
    LDR_Init();
    initSoundSensor();
    
    PRINTF("Testing LEDs...\r\n");
    ledOn(RED);
    delay(500000);
    ledOff(RED);
    ledOn(GREEN);
    delay(500000);
    ledOff(GREEN);
    ledOn(BLUE);
    delay(500000);
    ledOff(BLUE);
    
    PRINTF("System ready!\r\n");

    ledSema = xSemaphoreCreateBinary();
    buzzerSema = xSemaphoreCreateBinary();

    xTaskCreate(sensorTask, "SensorTask", configMINIMAL_STACK_SIZE + 100, NULL, 1, NULL);
    xTaskCreate(ledTask, "ledTask", configMINIMAL_STACK_SIZE + 100, NULL, 2, NULL);
    xTaskCreate(buzzerTask, "buzzerTask", configMINIMAL_STACK_SIZE + 100, NULL, 2, NULL);

    PRINTF("Make some noise to test sound detection!\r\n");

    vTaskStartScheduler();

    PRINTF("Scheduler failed to start!\r\n");
    while(1) {}
    return 0;
}
