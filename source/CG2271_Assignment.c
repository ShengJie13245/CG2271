/*
 * Smart Desk Assistant - FreeRTOS-based Environmental Monitoring System
 * 
 * Features:
 * - Environmental monitoring (Light, Temperature, Noise)
 * - RGB LED feedback with animations
 * - Buzzer alerts
 * - ESP32 communication via UART
 * - Focus/Relax mode switching
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "fsl_clock.h"
#include "fsl_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "device/MCXC444.h"
#include "device/periph2/PERI_ADC.h"
#include "device/periph2/PERI_PORT.h"
#include "device/periph2/PERI_TPM.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Task priorities */
#define SENSOR_TASK_PRIORITY    (configMAX_PRIORITIES - 1)
#define ACTUATOR_TASK_PRIORITY (configMAX_PRIORITIES - 2)
#define COMMS_TASK_PRIORITY    (configMAX_PRIORITIES - 3)

/* Task stack sizes */
#define SENSOR_TASK_STACK_SIZE    512
#define ACTUATOR_TASK_STACK_SIZE  512
#define COMMS_TASK_STACK_SIZE     512

/* Timing constants */
#define SENSOR_POLL_INTERVAL_MS   100  /* Poll sensors every 100ms */
#define COMMS_TX_INTERVAL_MS      500  /* Send telemetry every 500ms */
#define BUTTON_DEBOUNCE_MS        50   /* Button debounce time */
#define LONG_PRESS_MS             2000 /* Long press duration */
#define NOISE_WINDOW_MS           3000 /* Noise event window (3 seconds) */
#define BUZZER_RATE_LIMIT_MS      1000 /* Minimum time between buzzer beeps */

/* Sensor channels - adjust based on your hardware connections */
#define ADC_LDR_CHANNEL           4U   /* AD4 for LDR */
#define ADC_TEMP_CHANNEL          5U   /* AD5 for temperature sensor */
#define SOUND_SENSOR_PIN          5U   /* Port A pin 5 for sound sensor */
#define SOUND_SENSOR_PORT         PORTA
#define SOUND_SENSOR_GPIO         GPIOA
#define BUZZER_PIN                6U   /* Port A pin 6 for buzzer */
#define BUZZER_PORT               PORTA
#define BUZZER_GPIO               GPIOA

/* ESP32 UART configuration */
#define ESP32_LPUART_BASE         LPUART1
#define ESP32_LPUART_BAUDRATE     115200
#define ESP32_LPUART_CLKSRC       kCLOCK_McgIrc48MClk

/* Mode definitions */
typedef enum {
    MODE_RELAX = 0,
    MODE_FOCUS = 1
} system_mode_t;

/* System state definitions */
typedef enum {
    STATE_GOOD = 0,
    STATE_WARN = 1,
    STATE_ALERT = 2
} system_state_t;

/* Event types */
typedef enum {
    EVENT_NOISE_DETECTED = 0,
    EVENT_BUTTON_PRESSED = 1,
    EVENT_MODE_CHANGED = 2
} event_type_t;

/* Sensor event structure */
typedef struct {
    event_type_t type;
    uint32_t timestamp;
    uint8_t data;
} sensor_event_t;

/* Telemetry data structure */
typedef struct {
    float temperature;
    uint16_t light;
    uint8_t noise_count;
    uint8_t comfort_score;
    system_mode_t mode;
    system_state_t state;
} telemetry_data_t;

/* Actuator command structure */
typedef struct {
    system_state_t state;
    system_mode_t mode;
    uint8_t buzzer_enable;
    uint8_t led_enable;
} actuator_cmd_t;

/* Global variables */
static QueueHandle_t sensor_event_q;
static QueueHandle_t uplink_q;
static QueueHandle_t downlink_q;
static SemaphoreHandle_t uart_mutex;
static SemaphoreHandle_t button_sem;

static volatile system_mode_t current_mode = MODE_RELAX;
static volatile uint8_t noise_event_count = 0;
static volatile uint32_t last_noise_time = 0;
static volatile uint32_t last_buzzer_time = 0;
static volatile uint8_t button_press_time = 0;

/* ADC reading buffer */
static volatile uint16_t adc_ldr_value = 0;
static volatile uint16_t adc_temp_value = 0;
static volatile float temperature_celsius = 25.0f;

/* Exponential moving average for smoothing */
static float light_ema = 0.0f;
static float temp_ema = 25.0f;
static const float EMA_ALPHA = 0.3f;

/*******************************************************************************
 * ADC Functions
 ******************************************************************************/

/* Initialize ADC */
static void ADC_Init(void) {
    /* Enable ADC clock */
    CLOCK_EnableClock(kCLOCK_Adc0);
    
    /* Configure ADC for 16-bit mode, single-ended */
    ADC0->CFG1 = ADC_CFG1_MODE(3) | ADC_CFG1_ADIV(0) | ADC_CFG1_ADLSMP(1);
    ADC0->CFG2 = 0;
    ADC0->SC2 = ADC_SC2_REFSEL(0); /* Use VREFH/VREFL */
    ADC0->SC3 = ADC_SC3_CAL_MASK;   /* Start calibration */
    
    /* Wait for calibration to complete */
    while (ADC0->SC3 & ADC_SC3_CAL_MASK) {
        /* Wait */
    }
}

/* Read ADC channel */
static uint16_t ADC_ReadChannel(uint8_t channel) {
    /* Start conversion */
    ADC0->SC1[0] = ADC_SC1_ADCH(channel);
    
    /* Wait for conversion complete */
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {
        /* Wait */
    }
    
    /* Read result */
    return ADC0->R[0];
}

/*******************************************************************************
 * GPIO and Peripheral Initialization
 ******************************************************************************/

/* Initialize sound sensor (interrupt input) */
static void SoundSensor_Init(void) {
    /* Enable PORT clock */
    CLOCK_EnableClock(kCLOCK_PortA);
    
    /* Configure as GPIO input with interrupt on falling edge */
    port_pin_config_t pinConfig = {0};
    pinConfig.pullSelect = kPORT_PullUp;
    pinConfig.interrupt = kPORT_InterruptFallingEdge;
    
    PORT_SetPinConfig(SOUND_SENSOR_PORT, SOUND_SENSOR_PIN, &pinConfig);
    
    /* Configure GPIO */
    gpio_pin_config_t gpioConfig = {
        kGPIO_DigitalInput,
        0,
    };
    GPIO_PinInit(SOUND_SENSOR_GPIO, SOUND_SENSOR_PIN, &gpioConfig);
    
    /* Enable interrupt */
    EnableIRQ(PORTA_IRQn);
}

/* Initialize buzzer (GPIO output) */
static void Buzzer_Init(void) {
    /* Configure as GPIO output */
    port_pin_config_t pinConfig = {0};
    pinConfig.mux = kPORT_MuxAsGpio;
    
    PORT_SetPinConfig(BUZZER_PORT, BUZZER_PIN, &pinConfig);
    
    gpio_pin_config_t gpioConfig = {
        kGPIO_DigitalOutput,
        0,
    };
    GPIO_PinInit(BUZZER_GPIO, BUZZER_PIN, &gpioConfig);
    GPIO_PinWrite(BUZZER_GPIO, BUZZER_PIN, 0); /* Turn off initially */
}

/* Initialize ESP32 UART */
static void ESP32_UART_Init(void) {
    lpuart_config_t config;
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = ESP32_LPUART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;
    
    /* Note: LPUART1 pins need to be configured in pin_mux */
    /* For now, we'll use LPUART0 for ESP32 if available */
    /* In production, configure LPUART1 pins properly */
    LPUART_Init(LPUART0, &config, CLOCK_GetFreq(ESP32_LPUART_CLKSRC));
}

/*******************************************************************************
 * Interrupt Service Routines
 ******************************************************************************/

/* Sound sensor ISR */
void PORTA_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t flags = PORT_GetPinsInterruptFlags(SOUND_SENSOR_PORT);
    
    if (flags & (1U << SOUND_SENSOR_PIN)) {
        /* Clear interrupt flag */
        PORT_ClearPinsInterruptFlags(SOUND_SENSOR_PORT, (1U << SOUND_SENSOR_PIN));
        
        /* Create event */
        sensor_event_t event = {
            .type = EVENT_NOISE_DETECTED,
            .timestamp = xTaskGetTickCountFromISR(),
            .data = 1
        };
        
        /* Send to queue from ISR */
        xQueueSendFromISR(sensor_event_q, &event, &xHigherPriorityTaskWoken);
        
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* Button ISR (SW2) */
void PORTC_PORTD_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t flags = PORT_GetPinsInterruptFlags(BOARD_SW2_PORT);
    
    if (flags & (1U << BOARD_SW2_GPIO_PIN)) {
        /* Clear interrupt flag */
        PORT_ClearPinsInterruptFlags(BOARD_SW2_PORT, (1U << BOARD_SW2_GPIO_PIN));
        
        /* Create event */
        sensor_event_t event = {
            .type = EVENT_BUTTON_PRESSED,
            .timestamp = xTaskGetTickCountFromISR(),
            .data = 0
        };
        
        /* Send to queue from ISR */
        xQueueSendFromISR(sensor_event_q, &event, &xHigherPriorityTaskWoken);
        
        /* Give semaphore */
        xSemaphoreGiveFromISR(button_sem, &xHigherPriorityTaskWoken);
        
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*******************************************************************************
 * Comfort Score Calculation
 ******************************************************************************/

static uint8_t CalculateComfortScore(system_mode_t mode, uint16_t light, float temp, uint8_t noise_penalty) {
    uint8_t score = 100;
    int16_t light_penalty = 0;
    int16_t temp_penalty = 0;
    
    if (mode == MODE_FOCUS) {
        /* Focus mode: strict thresholds */
        uint16_t ideal_light = 275; /* Middle of 200-350 range */
        float ideal_temp = 25.0f;   /* Middle of 22-28 range */
        
        /* Light penalty */
        if (light < 150) {
            light_penalty = (150 - light) * 2; /* Too dark */
        } else if (light > 350) {
            light_penalty = (light - 350) * 2; /* Too bright */
        } else {
            light_penalty = (int16_t)abs((int16_t)light - (int16_t)ideal_light) / 2;
        }
        
        /* Temperature penalty */
        if (temp < 22.0f) {
            temp_penalty = (uint8_t)((22.0f - temp) * 10);
        } else if (temp > 28.0f) {
            temp_penalty = (uint8_t)((temp - 28.0f) * 15);
        } else {
            temp_penalty = (uint8_t)(fabs(temp - ideal_temp) * 5);
        }
    } else {
        /* Relax mode: loose thresholds */
        uint16_t ideal_light = 300; /* Middle of 100-500 range */
        float ideal_temp = 25.0f;  /* Middle of 20-30 range */
        
        /* Light penalty */
        if (light < 100) {
            light_penalty = (100 - light) / 2;
        } else if (light > 500) {
            light_penalty = (light - 500) / 2;
        } else {
            light_penalty = (int16_t)abs((int16_t)light - (int16_t)ideal_light) / 5;
        }
        
        /* Temperature penalty */
        if (temp < 20.0f) {
            temp_penalty = (uint8_t)((20.0f - temp) * 5);
        } else if (temp > 30.0f) {
            temp_penalty = (uint8_t)((temp - 30.0f) * 8);
        } else {
            temp_penalty = (uint8_t)(fabs(temp - ideal_temp) * 3);
        }
    }
    
    score -= (light_penalty / 10);
    score -= temp_penalty;
    score -= (noise_penalty * 15);
    
    if (score > 100) score = 100;
    if (score < 0) score = 0;
    
    return score;
}

static system_state_t GetStateFromScore(uint8_t score) {
    if (score >= 75) {
        return STATE_GOOD;
    } else if (score >= 50) {
        return STATE_WARN;
    } else {
        return STATE_ALERT;
    }
}

/*******************************************************************************
 * LED Animation Functions
 ******************************************************************************/

static void LED_SetColor(uint8_t red, uint8_t green, uint8_t blue) {
    /* Simple PWM-like control using delay */
    /* For better control, use TPM/PWM hardware */
    LED_RED_OFF();
    LED_GREEN_OFF();
    LED_BLUE_OFF();
    
    if (red) LED_RED_ON();
    if (green) LED_GREEN_ON();
    if (blue) LED_BLUE_ON();
}

static void LED_Animation_Good(system_mode_t mode) {
    static uint32_t anim_counter = 0;
    anim_counter++;
    
    if (mode == MODE_FOCUS) {
        /* Slow green pulse in focus mode */
        uint8_t intensity = (anim_counter % 40) < 20 ? 1 : 0;
        LED_SetColor(0, intensity, 0);
    } else {
        /* Slow fade green/blue in relax mode */
        uint8_t phase = (anim_counter / 20) % 4;
        if (phase == 0) LED_SetColor(0, 1, 0);
        else if (phase == 1) LED_SetColor(0, 1, 1);
        else if (phase == 2) LED_SetColor(0, 0, 1);
        else LED_SetColor(0, 1, 1);
    }
}

static void LED_Animation_Warn(system_mode_t mode) {
    static uint32_t anim_counter = 0;
    anim_counter++;
    
    /* Yellow heartbeat */
    uint8_t intensity = ((anim_counter / 5) % 20) < 10 ? 1 : 0;
    LED_SetColor(1, intensity, 0);
}

static void LED_Animation_Alert(system_mode_t mode) {
    static uint32_t anim_counter = 0;
    anim_counter++;
    
    /* Fast red sawtooth */
    uint8_t intensity = ((anim_counter / 2) % 20);
    LED_SetColor(1, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(intensity));
    LED_SetColor(0, 0, 0);
}

static void LED_Animation_ModeChange(void) {
    /* Cyan sweep */
    for (int i = 0; i < 3; i++) {
        LED_SetColor(0, 1, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        LED_SetColor(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*******************************************************************************
 * FreeRTOS Tasks
 ******************************************************************************/

/* Sensor Task - Polls sensors and calculates comfort score */
static void SensorTask(void *pvParameters) {
    sensor_event_t event;
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t last_noise_check = 0;
    
    while (1) {
        /* Poll LDR */
        adc_ldr_value = ADC_ReadChannel(ADC_LDR_CHANNEL);
        light_ema = (EMA_ALPHA * adc_ldr_value) + ((1.0f - EMA_ALPHA) * light_ema);
        
        /* Poll temperature sensor */
        adc_temp_value = ADC_ReadChannel(ADC_TEMP_CHANNEL);
        /* Convert ADC to voltage (assuming 3.3V ref, 16-bit ADC) */
        float voltage = (adc_temp_value * 3.3f) / 65535.0f;
        /* TMP36: (voltage - 0.5) * 100 = temperature in Celsius */
        float temp_raw = (voltage - 0.5f) * 100.0f;
        temp_ema = (EMA_ALPHA * temp_raw) + ((1.0f - EMA_ALPHA) * temp_ema);
        temperature_celsius = temp_ema;
        
        /* Check for noise events in window */
        uint32_t current_time = xTaskGetTickCount();
        if (current_time - last_noise_check > pdMS_TO_TICKS(NOISE_WINDOW_MS)) {
            /* Reset noise count if window expired */
            if (current_time - last_noise_time > pdMS_TO_TICKS(NOISE_WINDOW_MS)) {
                noise_event_count = 0;
            }
            last_noise_check = current_time;
        }
        
        /* Calculate comfort score */
        uint8_t noise_penalty = 0;
        if (current_mode == MODE_FOCUS && noise_event_count > 2) {
            noise_penalty = 3;
        } else if (current_mode == MODE_RELAX && noise_event_count > 4) {
            noise_penalty = 2;
        }
        
        uint8_t score = CalculateComfortScore(current_mode, (uint16_t)light_ema, 
                                              temperature_celsius, noise_penalty);
        system_state_t state = GetStateFromScore(score);
        
        /* Send telemetry to CommsTask */
        telemetry_data_t telemetry = {
            .temperature = temperature_celsius,
            .light = (uint16_t)light_ema,
            .noise_count = noise_event_count,
            .comfort_score = score,
            .mode = current_mode,
            .state = state
        };
        
        if (xQueueSend(uplink_q, &telemetry, 0) != pdTRUE) {
            /* Queue full, drop oldest */
            xQueueReceive(uplink_q, &telemetry, 0);
            xQueueSend(uplink_q, &telemetry, 0);
        }
        
        /* Send state change to ActuatorTask */
        actuator_cmd_t cmd = {
            .state = state,
            .mode = current_mode,
            .buzzer_enable = (state == STATE_ALERT) ? 1 : 0,
            .led_enable = 1
        };
        xQueueOverwrite(downlink_q, &cmd);
        
        /* Process events from ISR */
        while (xQueueReceive(sensor_event_q, &event, 0) == pdTRUE) {
            if (event.type == EVENT_NOISE_DETECTED) {
                noise_event_count++;
                last_noise_time = xTaskGetTickCount();
            } else if (event.type == EVENT_BUTTON_PRESSED) {
                /* Button handling with debounce */
                vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
                
                /* Check if button is still pressed (long press detection) */
                uint32_t press_start = xTaskGetTickCount();
                while (GPIO_PinRead(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    if (xTaskGetTickCount() - press_start > pdMS_TO_TICKS(LONG_PRESS_MS)) {
                        /* Long press - force Focus mode */
                        current_mode = MODE_FOCUS;
                        LED_Animation_ModeChange();
                        break;
                    }
                }
                
                if (GPIO_PinRead(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN) == 1) {
                    /* Short press - toggle mode */
                    current_mode = (current_mode == MODE_FOCUS) ? MODE_RELAX : MODE_FOCUS;
                    LED_Animation_ModeChange();
                }
            }
        }
        
        /* Periodic delay */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SENSOR_POLL_INTERVAL_MS));
    }
}

/* Actuator Task - Controls LED and buzzer */
static void ActuatorTask(void *pvParameters) {
    actuator_cmd_t cmd;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        /* Receive command from queue */
        if (xQueueReceive(downlink_q, &cmd, pdMS_TO_TICKS(50)) == pdTRUE) {
            /* Update LED based on state */
            if (cmd.led_enable) {
                switch (cmd.state) {
                    case STATE_GOOD:
                        LED_Animation_Good(cmd.mode);
                        break;
                    case STATE_WARN:
                        LED_Animation_Warn(cmd.mode);
                        break;
                    case STATE_ALERT:
                        LED_Animation_Alert(cmd.mode);
                        break;
                }
            } else {
                LED_SetColor(0, 0, 0);
            }
            
            /* Update buzzer */
            if (cmd.buzzer_enable) {
                uint32_t current_time = xTaskGetTickCount();
                if (current_time - last_buzzer_time > pdMS_TO_TICKS(BUZZER_RATE_LIMIT_MS)) {
                    /* Short beep for warning, long beep for alert */
                    if (cmd.state == STATE_ALERT) {
                        GPIO_PinWrite(BUZZER_GPIO, BUZZER_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(200));
                        GPIO_PinWrite(BUZZER_GPIO, BUZZER_PIN, 0);
                    } else {
                        GPIO_PinWrite(BUZZER_GPIO, BUZZER_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        GPIO_PinWrite(BUZZER_GPIO, BUZZER_PIN, 0);
                    }
                    last_buzzer_time = current_time;
                }
            }
        }
        
        /* Continue LED animation even if no new command */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* Communication Task - Handles ESP32 UART */
static void CommsTask(void *pvParameters) {
    telemetry_data_t telemetry;
    char tx_buffer[128];
    char rx_buffer[64];
    uint8_t rx_index = 0;
    TickType_t last_tx_time = xTaskGetTickCount();
    
    while (1) {
        /* Receive telemetry from SensorTask */
        if (xQueueReceive(uplink_q, &telemetry, pdMS_TO_TICKS(100)) == pdTRUE) {
            /* Format telemetry message */
            const char *mode_str = (telemetry.mode == MODE_FOCUS) ? "FOCUS" : "RELAX";
            const char *state_str = "GOOD";
            if (telemetry.state == STATE_WARN) state_str = "WARN";
            else if (telemetry.state == STATE_ALERT) state_str = "ALERT";
            
            snprintf(tx_buffer, sizeof(tx_buffer),
                    "TEMP=%.1f;LIGHT=%d;NOISE=%d;MODE=%s;STATE=%s;SCORE=%d;\n",
                    telemetry.temperature,
                    telemetry.light,
                    telemetry.noise_count,
                    mode_str,
                    state_str,
                    telemetry.comfort_score);
            
            /* Send via UART (protected by mutex) */
            if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                LPUART_WriteBlocking(LPUART0, (uint8_t *)tx_buffer, strlen(tx_buffer));
                xSemaphoreGive(uart_mutex);
            }
        }
        
        /* Check for incoming commands from ESP32 */
        if (LPUART_GetStatusFlags(LPUART0) & kLPUART_RxDataRegFullFlag) {
            uint8_t byte;
            LPUART_ReadBlocking(LPUART0, &byte, 1);
            
            if (byte == '\n' || byte == '\r') {
                rx_buffer[rx_index] = '\0';
                
                /* Parse command */
                if (strstr(rx_buffer, "MODE=RELAX") != NULL) {
                    current_mode = MODE_RELAX;
                } else if (strstr(rx_buffer, "MODE=FOCUS") != NULL) {
                    current_mode = MODE_FOCUS;
                } else if (strstr(rx_buffer, "LED=OFF") != NULL) {
                    actuator_cmd_t cmd;
                    if (xQueuePeek(downlink_q, &cmd, 0) == pdTRUE) {
                        cmd.led_enable = 0;
                        xQueueOverwrite(downlink_q, &cmd);
                    }
                } else if (strstr(rx_buffer, "LED=ON") != NULL) {
                    actuator_cmd_t cmd;
                    if (xQueuePeek(downlink_q, &cmd, 0) == pdTRUE) {
                        cmd.led_enable = 1;
                        xQueueOverwrite(downlink_q, &cmd);
                    }
                }
                
                rx_index = 0;
            } else if (rx_index < sizeof(rx_buffer) - 1) {
                rx_buffer[rx_index++] = byte;
            }
        }
        
        /* Periodic transmission */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/*******************************************************************************
 * Main Function
 ******************************************************************************/

int main(void) {
    /* Init board hardware */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();
    
    PRINTF("Smart Desk Assistant Starting...\r\n");
    
    /* Initialize peripherals */
    ADC_Init();
    SoundSensor_Init();
    Buzzer_Init();
    ESP32_UART_Init();
    
    /* Initialize LEDs */
    LED_RED_INIT(LOGIC_LED_OFF);
    LED_GREEN_INIT(LOGIC_LED_OFF);
    LED_BLUE_INIT(LOGIC_LED_OFF);
    
    /* Create FreeRTOS queues */
    sensor_event_q = xQueueCreate(10, sizeof(sensor_event_t));
    uplink_q = xQueueCreate(5, sizeof(telemetry_data_t));
    downlink_q = xQueueCreate(1, sizeof(actuator_cmd_t));
    
    /* Create FreeRTOS semaphores */
    uart_mutex = xSemaphoreCreateMutex();
    button_sem = xSemaphoreCreateBinary();
    
    /* Create FreeRTOS tasks */
    xTaskCreate(SensorTask, "SensorTask", SENSOR_TASK_STACK_SIZE, NULL, 
                SENSOR_TASK_PRIORITY, NULL);
    xTaskCreate(ActuatorTask, "ActuatorTask", ACTUATOR_TASK_STACK_SIZE, NULL, 
                ACTUATOR_TASK_PRIORITY, NULL);
    xTaskCreate(CommsTask, "CommsTask", COMMS_TASK_STACK_SIZE, NULL, 
                COMMS_TASK_PRIORITY, NULL);
    
    PRINTF("FreeRTOS Scheduler Starting...\r\n");
    
    /* Start FreeRTOS scheduler */
    vTaskStartScheduler();
    
    /* Should never reach here */
    while (1) {
        __asm volatile ("nop");
    }
    
    return 0;
}
