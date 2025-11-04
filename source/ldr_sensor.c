/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    ldr_sensor.c
 * @brief   LDR (Light Dependent Resistor) sensor module implementation
 * @author  CG2271 Group
 */

#include "ldr_sensor.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/* Private variables */
static volatile uint16_t last_adc_value = 0;

/* Private function prototypes */
static void delay(uint32_t count);

/**
 * @brief Initialize the LDR sensor and ADC
 */
void LDR_Init(void) {
    // Configure interrupt
    NVIC_DisableIRQ(ADC0_IRQn);
    NVIC_ClearPendingIRQ(ADC0_IRQn);

    // Enable clock gating to ADC0
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

    // Enable clock gating to PTE
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Set PTE20 (ADC0_SE0) to ADC mode
    PORTE->PCR[LDR_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LDR_PIN] |= PORT_PCR_MUX(0); // MUX = 0 for ADC

    // Configure the ADC
    // Enable ADC interrupt
    ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;

    // Select single-ended ADC
    ADC0->SC1[0] &= ~ADC_SC1_DIFF_MASK;
    ADC0->SC1[0] |= ADC_SC1_DIFF(0b0);

    // Set 12 bit conversion
    ADC0->CFG1 &= ~ADC_CFG1_MODE_MASK;
    ADC0->CFG1 |= ADC_CFG1_MODE(0b01);

    // Use software trigger
    ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;

    // Use VREFH and VREFL as reference
    ADC0->SC2 &= ~ADC_SC2_REFSEL_MASK;
    ADC0->SC2 |= ADC_SC2_REFSEL(0b00);

    // Don't use averaging
    ADC0->SC3 &= ~ADC_SC3_AVGE_MASK;
    ADC0->SC3 |= ADC_SC3_AVGE(0);

    // Use single conversion (NOT continuous)
    ADC0->SC3 &= ~ADC_SC3_ADCO_MASK;
    ADC0->SC3 |= ADC_SC3_ADCO(0);

    // Set interrupt priority and enable
    NVIC_SetPriority(ADC0_IRQn, 192);
    NVIC_EnableIRQ(ADC0_IRQn);
    
    PRINTF("LDR Sensor initialized on PTE20 (ADC0_SE0)\r\n");
}

/**
 * @brief Start continuous LDR sensor readings
 */
void LDR_StartReading(void) {
    PRINTF("Starting LDR sensor readings...\r\n");
    
    while(1) {
        // Trigger a single ADC reading
        LDR_TriggerReading();
        
        // Wait for a reasonable interval between readings (about 1 second)
        delay(8000000); // Adjust delay as needed for your desired reading frequency
    }
}

/**
 * @brief Trigger a single LDR sensor reading
 */
void LDR_TriggerReading(void) {
    // Start a single conversion
    ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
    ADC0->SC1[0] |= ADC_SC1_ADCH(ADC_CHANNEL);
}

/**
 * @brief Get the last LDR sensor reading
 */
uint16_t LDR_GetLastValue(void) {
    return last_adc_value;
}

/**
 * @brief Convert ADC value to voltage in millivolts
 */
uint16_t LDR_ADCToVoltage(uint16_t adc_value) {
    return (adc_value * 3300) / 4095; // Voltage in millivolts
}

/**
 * @brief Convert ADC value to light level percentage
 */
uint8_t LDR_ADCToPercentage(uint16_t adc_value) {
    return (adc_value * 100) / 4095;
}

/**
 * @brief ADC0 Interrupt Handler
 * This function handles ADC conversion complete interrupts
 */
void ADC0_IRQHandler(void) {
    NVIC_ClearPendingIRQ(ADC0_IRQn);

    if(ADC0->SC1[0] & ADC_SC1_COCO_MASK){
        uint16_t result = ADC0->R[0];
        last_adc_value = result;
        
        // Convert ADC result to voltage and percentage
        uint16_t voltage_mv = LDR_ADCToVoltage(result);
        uint8_t percentage = LDR_ADCToPercentage(result);
        
        PRINTF("LDR ADC Value: %d, Voltage: %d.%03dV, Light Level: %d%%\r\n", 
               result, voltage_mv/1000, voltage_mv%1000, percentage);
    }
}

/**
 * @brief Simple delay function
 * @param count Number of delay cycles
 */
static void delay(uint32_t count) {
    volatile uint32_t i = 0;
    for (i = 0; i < count; ++i) {
        __asm("NOP"); /* delay */
    }
}
