# 🚀 Multi-Sensor IoT System - CG2271 FreeRTOS Project

A comprehensive IoT system built on the FRDM-MCXC444 board using FreeRTOS that integrates multiple sensors, actuators, and UART communication for real-time environmental monitoring and control.

## 🎯 Project Overview

This system demonstrates advanced embedded programming concepts including FreeRTOS task management, interrupt handling, sensor integration, and bidirectional UART communication with an ESP32/Arduino device.

### Key Features
- **Multi-sensor monitoring**: Light detection (LDR) and sound detection
- **Smart actuator control**: RGB LED patterns and buzzer alerts
- **UART communication**: Bidirectional communication with ESP32/Arduino
- **Remote LED control**: Change LED colors via UART commands
- **FreeRTOS architecture**: Task-based design with semaphores and queues
- **Interrupt-driven design**: Efficient sensor handling and UART communication

## 🔧 Hardware Requirements

### FRDM-MCXC444 Board Components
| Component | Pin | Type | Description |
|-----------|-----|------|-------------|
| LDR + 10kΩ resistor | PTE20 (ADC0_SE0) | Analog | Light level sensor (polling) |
| Sound sensor (DO) | PTC6 | Digital | Noise detection (interrupt) |
| RGB LED - Red | PTD4 | Output | Status indication |
| RGB LED - Green | PTD7 | Output | Status indication |
| RGB LED - Blue | PTD6 | Output | Status indication |
| Buzzer | PTE30 (TPM0_CH3) | PWM | Audio feedback |
| UART TX | PTE22 | Communication | ESP32/Arduino interface |
| UART RX | PTE23 | Communication | ESP32/Arduino interface |

### ESP32/Arduino Components
| Component | Pin | Description |
|-----------|-----|-------------|
| UART TX/RX | GPIO pins | FRDM communication |
| Any sensors/actuators | Various | Extended functionality |

## 🏗️ Software Architecture

### FreeRTOS Tasks
| Task | Priority | Period | Function |
|------|----------|--------|----------|
| **sensorTask** | 2 | 1000ms | Poll LDR sensor via ADC interrupt |
| **ledTask** | 3 | Event-driven | Control LED blinking patterns based on sound detection |
| **buzzerTask** | 3 | Event-driven | Control buzzer patterns based on light detection |
| **recvTask** | 2 | Event-driven | Handle incoming UART messages and LED control |
| **sendTask** | 1 | 2000ms | Send periodic "1" requests to ESP32/Arduino |

### Synchronization Objects
- **ledSema**: Binary semaphore for sound sensor → LED task communication
- **buzzerSema**: Binary semaphore for LDR sensor → buzzer task communication  
- **uart_queue**: Message queue for UART receive data handling
- **currentLED**: Global variable tracking current LED color (RED/GREEN/BLUE)

### Interrupt Service Routines
- **PORTC_PORTD_IRQHandler**: Sound sensor rising-edge detection with 500ms debouncing
- **ADC0_IRQHandler**: LDR sensor ADC conversion complete with light level analysis
- **UART2_FLEXIO_IRQHandler**: UART transmit/receive data handling

## 📊 Sensor Logic & Thresholds

### Light Detection (LDR Sensor)
```c
// ADC reading interpretation:
// Lower ADC value (<450) = BRIGHT light (more light = lower resistance)
// Higher ADC value (>450) = DARK conditions

if(adc_result < 450) {
    // Bright light detected → Trigger buzzer
    xSemaphoreGiveFromISR(buzzerSema, &hpw);
}
```

### Sound Detection
```c
// Digital sound sensor on PTC6
// Rising edge interrupt with 500ms debouncing
// Sound detected → Trigger LED blink pattern
xSemaphoreGiveFromISR(ledSema, &hpw);
```

## 🔌 Communication Protocol

### FRDM → ESP32/Arduino
```
"1\n"  // Periodic request every 2 seconds
```

### ESP32/Arduino → FRDM
```
"0\n"  // No action
"1\n"  // Change LED to GREEN
"2\n"  // Change LED to BLUE
```

### LED Control Logic
- **Receive "1"**: Switch to GREEN LED, update currentLED
- **Receive "2"**: Switch to BLUE LED, update currentLED
- **Sound Detection**: Blink the current LED color (RED/GREEN/BLUE)

## 🎨 LED Patterns & Audio Feedback

### LED Behavior
- **Default**: GREEN LED on (currentLED = GREEN)
- **Sound Triggered**: Current LED blinks (off→on→off→on pattern)
- **UART "1" Command**: Switch to solid GREEN
- **UART "2" Command**: Switch to solid BLUE

### Buzzer Patterns
- **Bright Light Detection**: Double beep (800Hz + 1000Hz, 150ms each)
- **Debouncing**: 800ms minimum interval between buzzer activations

## 📁 Project Structure

```
/source/
├── CG2271_Assignment.c      # Main application with all tasks and ISRs
├── ldr_sensor.c/.h          # LDR sensor driver and ADC handling
└── (other SDK files)        # Board support and peripheral drivers

/Debug/                      # Build output directory
/board/                      # Board configuration files
/drivers/                    # NXP SDK drivers
/freertos/                   # FreeRTOS kernel source
```

## 🚀 Getting Started

### 1. Hardware Setup
1. Connect LDR sensor with 10kΩ pull-down resistor to PTE20
2. Connect digital sound sensor output to PTC6
3. Connect RGB LED: Red→PTD4, Green→PTD7, Blue→PTD6
4. Connect buzzer to PTE30
5. Wire UART: FRDM PTE22(TX)→ESP RX, FRDM PTE23(RX)→ESP TX, common GND
6. Power both boards (3.3V logic levels)

### 2. Software Build
1. Import project into MCUXpresso IDE
2. Build the project (Release or Debug configuration)
3. Flash to FRDM-MCXC444 board
4. Open serial terminal (115200 baud for debug console)

### 3. Expected Demo Flow
1. **Startup**: System initializes, GREEN LED on, starts sending "1" via UART
2. **Light changes**: Cover LDR → buzzer double beep when bright light detected
3. **Sound detection**: Clap/noise → current LED blinks (default GREEN)
4. **UART commands**: Send "1" or "2" → LED changes to GREEN or BLUE
5. **Combined operation**: Sound detection blinks the currently selected LED color

## 🎛️ System Behavior

### Normal Operation
- LDR sensor readings every 1 second with voltage and light level display
- UART sends "1" request every 2 seconds
- Sound sensor monitors for noise events
- All sensors work independently and simultaneously

### Debug Output Examples
```
=== CG2271 Multi-Sensor System with FreeRTOS ===
LDR Sensor: PTE20 (ADC0_SE0)
Sound Sensor: PTC6 (Digital)
Buzzer: PTE30 (TPM0_CH3)
LEDs: PTD4(Red), PTD7(Green), PTD6(Blue)
UART2: PTE22(TX), PTE23(RX) - 9600 baud

LDR ADC Value: 234, Voltage: 0.188V, Light Level: BRIGHT
Bright light detected! Buzzer triggered!

Sound Detected
Received message: 2
changed LED to blue!
```

## 📋 CG2271 Marking Criteria Compliance

✅ **Multiple sensors**: LDR (polling via ADC), Sound (interrupt-driven)  
✅ **Multiple actuators**: RGB LED, Buzzer (PWM)  
✅ **Polling sensor**: LDR with 1-second periodic readings  
✅ **Interrupt sensor**: Sound sensor with proper ISR design  
✅ **Proper ISR design**: ISR → Semaphore → Task pattern  
✅ **Task design**: 5 modular tasks with clear responsibilities  
✅ **Semaphores**: Binary semaphores for sensor-actuator communication  
✅ **Queues**: UART message queue for communication  
✅ **UART communication**: Bidirectional with ESP32/Arduino  
✅ **Message passing**: Tasks communicate via semaphores and queues  
✅ **Professional implementation**: Clean, modular, well-documented code

## 🔧 Troubleshooting

### Common Issues
1. **No LED response**: Check GPIO connections and power supply
2. **No UART data**: Verify baud rate (9600) and TX/RX wiring
3. **ADC readings unstable**: Check LDR circuit and voltage reference
4. **Sound sensor not triggering**: Verify digital output and interrupt config
5. **Buzzer not working**: Check PWM configuration and buzzer connections

### Debug Features
- Comprehensive serial console output for all events
- Real-time sensor readings with voltage conversion
- UART message logging with clear status updates
- Task synchronization status monitoring

## 🔬 Technical Implementation Details

### ADC Configuration
- 12-bit resolution, single-ended mode
- Software trigger, interrupt-driven
- VREFH/VREFL reference, no averaging

### PWM Configuration  
- TPM0 Channel 3, center-aligned PWM
- Variable frequency and duty cycle for buzzer tones
- Prescaler of 8 for audio frequency range

### UART Configuration
- 9600 baud, 8-bit, no parity
- Interrupt-driven TX/RX with circular buffers
- Newline-terminated message protocol

---

**Author**: CG2271 Student  
**Date**: November 2025  
**Version**: 2.0  
**Platform**: FRDM-MCXC444 + ESP32/Arduino + FreeRTOS