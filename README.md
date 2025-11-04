# 🧠 Smart Desk Assistant - CG2271 FreeRTOS Project

A comprehensive IoT system built on the FRDM-MCXC444 board using FreeRTOS that monitors environmental comfort (light, temperature, noise) and provides real-time feedback through LEDs and a buzzer.

## 🎯 Project Overview

The Smart Desk Assistant integrates multiple sensors and actuators to create an intelligent workspace monitoring system that helps maintain optimal working conditions for both focus and relaxation modes.

### Key Features
- **Multi-sensor monitoring**: Light (LDR), Sound (digital), Temperature (ESP32 DHT11)
- **Dual operating modes**: Focus and Relax with different comfort thresholds
- **Real-time feedback**: RGB LED patterns and buzzer alerts
- **ESP32 integration**: Bidirectional UART communication for temperature data
- **FreeRTOS architecture**: Task-based design with proper synchronization

## 🔧 Hardware Requirements

### FRDM-MCXC444 Board Components
| Component | Pin | Type | Description |
|-----------|-----|------|-------------|
| LDR + 10kΩ resistor | PTB0 (ADC0_SE8) | Analog | Light level sensor (polling) |
| Sound sensor (DO) | PTC6 | Digital | Noise detection (interrupt) |
| Mode button | PTA4 | Digital | Mode switching (interrupt) |
| RGB LED (R,G,B) | PTD5, PTD7, PTD6 | Output | Status indication |
| Buzzer | PTC7 (TPM0_CH3) | PWM | Audio feedback |
| UART TX/RX | PTE22/PTE23 | Communication | ESP32 interface |

### ESP32 Components
| Component | Pin | Description |
|-----------|-----|-------------|
| DHT11 Temperature Sensor | GPIO 4 | Temperature/humidity reading |
| UART TX/RX | GPIO 1/2 | FRDM communication |

## 🏗️ Software Architecture

### FreeRTOS Tasks
| Task | Priority | Period | Function |
|------|----------|--------|----------|
| **SensorTask** | 2 | 100ms | Poll LDR, process noise events, calculate comfort score |
| **CommsTask** | 2 | 100ms | Handle UART communication with ESP32 |
| **ActuatorTask** | 1 | 10ms | Control LED animations and buzzer patterns |
| **ModeTask** | 2 | Event-driven | Handle button press for mode switching |

### Synchronization Objects
- **uart_mutex**: Protects UART writes (shared resource)
- **button_sem**: Signals mode toggle from button ISR
- **noise_evt_q**: ISR → SensorTask noise event timestamps
- **uplink_q**: SensorTask → CommsTask telemetry data
- **downlink_q**: SensorTask → ActuatorTask control commands

### Interrupt Service Routines
- **PORTC_IRQHandler**: Sound sensor rising-edge detection
- **PORTA_IRQHandler**: Button falling-edge detection
- **LPUART0_IRQHandler**: UART receive data handling

## 📊 Comfort Score Algorithm

The system calculates a comfort score (0-100) based on:

```c
score = 100 - light_penalty - temp_penalty - noise_penalty

Where:
- light_penalty = 0.1 × |current_lux - ideal_lux|
- temp_penalty = 2.0 × |current_temp - ideal_temp|
- noise_penalty = 10 × noise_bursts_in_3s
```

### Mode-Specific Thresholds
| Parameter | Focus Mode | Relax Mode |
|-----------|------------|------------|
| Ideal Light | 300 lux | 200 lux |
| Ideal Temperature | 25.0°C | 27.0°C |

### State Classification
- **GOOD** (75-100): Green breathing LED
- **WARN** (50-74): Yellow heartbeat LED + periodic beep
- **ALERT** (0-49): Red flashing LED + alert beep

## 🔌 Communication Protocol

### FRDM → ESP32 (Telemetry)
```
L=345;S=1;MODE=FOCUS;STATE=WARN;SCORE=68;\n
```

### ESP32 → FRDM (Temperature)
```
TEMP=27.4;\n
```

## 📁 Project Structure

```
/source/
├── CG2271_Assignment.c      # Main application entry point
├── app_config.h             # Hardware pin definitions and constants
├── messages.h               # Data structures and global variables
├── drivers_adc.c/.h         # ADC driver for LDR sensor
├── drivers_gpio_pwm.c/.h    # GPIO and PWM drivers for LED/buzzer
├── drivers_uart.c/.h        # UART driver for ESP32 communication
├── isr_inputs.c/.h          # Interrupt service routines
├── sensor_task.c/.h         # Sensor polling and comfort calculation
├── comms_task.c/.h          # UART communication management
├── actuator_task.c/.h       # LED/buzzer pattern control
└── mode_task.c/.h           # Mode switching logic

/ESP32_Temperature_Sensor.ino # Arduino sketch for ESP32
```

## 🚀 Getting Started

### 1. Hardware Setup
1. Connect all sensors and actuators according to the pin mapping
2. Wire ESP32 UART to FRDM board (TX→RX, RX→TX, common GND)
3. Upload the Arduino sketch to ESP32
4. Power both boards (3.3V logic levels)

### 2. Software Build
1. Import project into MCUXpresso IDE
2. Build the project (all drivers and tasks are included)
3. Flash to FRDM-MCXC444 board
4. Open serial terminal for debug output

### 3. Expected Demo Flow
1. **Startup**: System starts in Relax mode with soft green LED
2. **Light changes**: Cover LDR → LED changes to yellow/red
3. **Sound detection**: Clap near sensor → beep + red flash
4. **Temperature**: ESP32 sends temperature data → affects comfort score
5. **Mode switch**: Press button → cyan sweep + chime, mode toggles
6. **UART logs**: Both boards display telemetry data

## 🎛️ LED Patterns & Audio Feedback

### LED Animations
- **Green Breathing**: Comfort state GOOD
- **Yellow Heartbeat**: Comfort state WARN  
- **Red Flashing**: Comfort state ALERT
- **Cyan Sweep**: Mode change indication

### Buzzer Patterns
- **Short Beep**: Warning state (every 10s)
- **Long Beep**: Alert state (once)
- **Two-note Chime**: Mode change confirmation

## 📋 CG2271 Marking Criteria Compliance

✅ **3 sensors** (2 FRDM + 1 ESP32): LDR, Sound, Temperature  
✅ **2 actuators**: RGB LED, Buzzer  
✅ **Polling sensor**: LDR (ADC)  
✅ **Interrupt sensor**: Sound sensor (GPIO)  
✅ **Proper ISR design**: ISR → Queue → Task pattern  
✅ **Task design**: 4 modular tasks with clear responsibilities  
✅ **Semaphores**: Mutex (UART) + Binary (button)  
✅ **Queues**: 3 queues for message passing  
✅ **Time-slicing + Pre-emption**: Enabled in FreeRTOS config  
✅ **ESP32 integration**: UART data exchange  
✅ **Message passing**: Tasks communicate via queues and UART  
✅ **Professional implementation**: Comprehensive, well-documented code

## 🔧 Troubleshooting

### Common Issues
1. **No LED response**: Check GPIO pin connections and power
2. **No UART data**: Verify baud rate (115200) and wiring
3. **ADC readings unstable**: Check LDR circuit and reference voltage
4. **Button not responding**: Verify pull-up resistor and interrupt config
5. **ESP32 not sending data**: Check DHT11 connections and Arduino sketch

### Debug Features
- Serial console output for all major events
- Task status and queue usage monitoring
- Comfort score calculation debugging
- UART protocol message logging

## 📚 Technical References

- **FRDM-MCXC444 Reference Manual**: Hardware specifications
- **FreeRTOS Documentation**: Task management and synchronization
- **MCUXpresso SDK**: Driver APIs and examples
- **ESP32 Arduino Core**: ESP32 development framework

---

**Author**: CG2271 Student  
**Date**: November 2025  
**Version**: 1.0  
**Platform**: FRDM-MCXC444 + ESP32 + FreeRTOS
