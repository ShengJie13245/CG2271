
#include "ldr_sensor.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern SemaphoreHandle_t buzzerSema;

static volatile uint16_t last_adc_value = 0;
static void delay(uint32_t count);

void LDR_Init(void) {
    NVIC_DisableIRQ(ADC0_IRQn);
    NVIC_ClearPendingIRQ(ADC0_IRQn);

    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    PORTE->PCR[LDR_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[LDR_PIN] |= PORT_PCR_MUX(0);

    ADC0->SC1[0] |= ADC_SC1_AIEN_MASK;
    ADC0->SC1[0] &= ~ADC_SC1_DIFF_MASK;
    ADC0->SC1[0] |= ADC_SC1_DIFF(0b0);
    ADC0->CFG1 &= ~ADC_CFG1_MODE_MASK;
    ADC0->CFG1 |= ADC_CFG1_MODE(0b01);
    ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;
    ADC0->SC2 &= ~ADC_SC2_REFSEL_MASK;
    ADC0->SC2 |= ADC_SC2_REFSEL(0b00);
    ADC0->SC3 &= ~ADC_SC3_AVGE_MASK;
    ADC0->SC3 |= ADC_SC3_AVGE(0);
    ADC0->SC3 &= ~ADC_SC3_ADCO_MASK;
    ADC0->SC3 |= ADC_SC3_ADCO(0);

    NVIC_SetPriority(ADC0_IRQn, 192);
    NVIC_EnableIRQ(ADC0_IRQn);
    
    PRINTF("LDR Sensor initialized\r\n");
}

void LDR_StartReading(void) {
    PRINTF("Starting LDR sensor readings...\r\n");
    
    while(1) {
        LDR_TriggerReading();
        delay(8000000);
    }
}

void LDR_TriggerReading(void) {
    ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
    ADC0->SC1[0] |= ADC_SC1_ADCH(ADC_CHANNEL);
}

uint16_t LDR_GetLastValue(void) {
    return last_adc_value;
}

uint16_t LDR_ADCToVoltage(uint16_t adc_value) {
    return (adc_value * 3300) / 4095;
}

uint8_t LDR_ADCToPercentage(uint16_t adc_value) {
    return (adc_value * 100) / 4095;
}

void ADC0_IRQHandler(void) {
    static uint32_t last_buzzer_time = 0;

    NVIC_ClearPendingIRQ(ADC0_IRQn);

    if(ADC0->SC1[0] & ADC_SC1_COCO_MASK){
        uint16_t result = ADC0->R[0];
        last_adc_value = result;
        
        uint16_t voltage_mv = LDR_ADCToVoltage(result);
        uint8_t percentage = LDR_ADCToPercentage(result);
        
        if(result < 100) {
            PRINTF("LDR ADC Value: %d, Voltage: %d.%03dV, Light Level: BRIGHT\r\n", 
                   result, voltage_mv/1000, voltage_mv%1000, percentage);
        } else {
            PRINTF("LDR ADC Value: %d, Voltage: %d.%03dV, Light Level: DARK\r\n", 
                   result, voltage_mv/1000, voltage_mv%1000, percentage);
        }
        
        if(result < 100) {
            uint32_t current_time = xTaskGetTickCountFromISR();
            if ((current_time - last_buzzer_time) > pdMS_TO_TICKS(800)) {
                last_buzzer_time = current_time;
                PRINTF("Bright light detected! Buzzer triggered!\r\n");

                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xSemaphoreGiveFromISR(buzzerSema, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}

static void delay(uint32_t count) {
    volatile uint32_t i = 0;
    for (i = 0; i < count; ++i) {
        __asm("NOP");
    }
}
