/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "drivers_adc.h"
#include "app_config.h"
#include "fsl_port.h"
#include "fsl_clock.h"
#include "fsl_device_registers.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
static bool adc_initialized = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

void ADC_Init(void)
{
    if (adc_initialized) {
        return;
    }

    /* Enable ADC0 clock */
    CLOCK_EnableClock(kCLOCK_Adc0);

    /* Configure PTB0 as ADC input */
    PORT_SetPinMux(LDR_PORT, LDR_PIN, kPORT_PinDisabledOrAnalog);

    /* Configure ADC */
    /* CFG1: 12-bit mode, bus clock, normal power */
    ADC0->CFG1 = ADC_CFG1_ADIV(0) |     /* Clock divide by 1 */
                 ADC_CFG1_MODE(1) |     /* 12-bit mode */
                 ADC_CFG1_ADICLK(0);    /* Bus clock */

    /* CFG2: Default settings */
    ADC0->CFG2 = 0;

    /* SC2: Software trigger, compare function disabled */
    ADC0->SC2 = 0;

    /* SC3: Single conversion, no averaging */
    ADC0->SC3 = 0;

    adc_initialized = true;
}

uint16_t ADC_ReadLDR(void)
{
    if (!adc_initialized) {
        ADC_Init();
    }

    /* Start conversion on channel 8 (PTB0) */
    ADC0->SC1[0] = ADC_SC1_ADCH(LDR_ADC_CHANNEL);
    
    /* Wait for conversion to complete */
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK))
    {
        /* Wait for conversion complete */
    }

    /* Get conversion result */
    uint16_t result = ADC0->R[0];
    
    return result;
}

uint16_t ADC_ConvertToLux(uint16_t adc_value)
{
    /* Convert 12-bit ADC value (0-4095) to approximate lux value (0-1023)
     * This is a simplified conversion - in practice you would calibrate
     * based on your specific LDR characteristics */
    return (adc_value >> 2);  /* Simple divide by 4 to get 0-1023 range */
}
