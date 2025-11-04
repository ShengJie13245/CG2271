/*
 * ESP32 Temperature Sensor for Smart Desk Assistant
 * 
 * This Arduino sketch reads temperature from a DHT11 sensor connected to GPIO 4
 * and sends the data to the FRDM-MCXC444 board via UART (Serial2).
 * 
 * Hardware Connections:
 * - DHT11 VCC -> 3.3V
 * - DHT11 GND -> GND  
 * - DHT11 DATA -> GPIO 4
 * - UART TX (GPIO 17) -> FRDM PTC3 (RX)
 * - UART RX (GPIO 16) -> FRDM PTC4 (TX)
 * - Common GND between ESP32 and FRDM
 */

#include <DHT.h>

// DHT11 sensor configuration
#define DHT_PIN 4
#define DHT_TYPE DHT11

DHT dht(DHT_PIN, DHT_TYPE);

// Timing variables
unsigned long lastTempRead = 0;
const unsigned long TEMP_INTERVAL = 1000;  // Send temperature every 1 second

void setup() {
  // Initialize serial communications
  Serial.begin(115200);      // USB serial for debugging
  Serial2.begin(115200);     // UART for FRDM communication
  
  // Initialize DHT sensor
  dht.begin();
  
  Serial.println("ESP32 Smart Desk Assistant Temperature Sensor");
  Serial.println("DHT11 on GPIO 4, UART on GPIO 16/17");
  Serial.println("Sending temperature data to FRDM board...");
  
  delay(2000);  // Give DHT sensor time to stabilize
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read and send temperature data
  if (currentTime - lastTempRead >= TEMP_INTERVAL) {
    readAndSendTemperature();
    lastTempRead = currentTime;
  }
  
  // Mirror any data received from FRDM to USB serial for debugging
  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c);
  }
  
  // Allow other tasks to run
  delay(10);
}

void readAndSendTemperature() {
  // Read temperature from DHT11
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  // Check if reading was successful
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  
  // Send temperature to FRDM board in expected format
  Serial2.printf("TEMP=%.1f;\n", temperature);
  
  // Debug output to USB serial
  Serial.printf("Sent: TEMP=%.1f; (Humidity: %.1f%%)\n", temperature, humidity);
  
  // Optional: Add some sensor status indication
  if (temperature > 30.0) {
    Serial.println("Warning: High temperature detected!");
  } else if (temperature < 15.0) {
    Serial.println("Warning: Low temperature detected!");
  }
}
