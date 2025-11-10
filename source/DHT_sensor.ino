#include <SimpleDHT.h>
const int NEW_TX_PIN = 1;
const int NEW_RX_PIN = 2;
const int pinDHT11 = 4;
SimpleDHT11 dht11;

byte humidityS = 0;
byte temperatureS = 0;
byte dataS[40] = {0};
int readX = 1;

void setup() {
Serial.begin(9600);
Serial1.begin(9600, SERIAL_8N1, NEW_RX_PIN, NEW_TX_PIN);
delay(1000);

}

void loop() {
// start working...
Serial.println("=================================");
Serial.println("Sample DHT11...");

if (readX == 1) {
    if (dht11.read(pinDHT11, &temperatureS, &humidityS, dataS)) {
        Serial.print("Read DHT11 failed");
        return;
    }
    readX = 0;
    delay(1000);
}

// read with raw sample data.
byte temperature = 0;
byte humidity = 0;
byte data[40] = {0};
if (dht11.read(pinDHT11, &temperature, &humidity, data)) {
    Serial.print("Read DHT11 failed");
    return;
}
Serial.print("Avg humidity ");
Serial.println((int)humidityS);
Serial.print("Sample RAW Bits: ");
for (int i = 0; i < 40; i++) {
    Serial.print((int)data[i]);
    if (i > 0 && ((i + 1) % 4) == 0) {
    Serial.print(' ');
    }
}
Serial.println("");

while(Serial1.available() <=0);

String x = Serial1.readString();
Serial.print("Request received from MCXC444: ");
Serial.print(x);
Serial.print((int)temperature); Serial.print(" *C, ");
Serial.print((int)humidity); Serial.println(" %");
if ((int)humidity > (int)humidityS + 3) {
    Serial.println("2");
    Serial1.println("2");
}
else if ((int)humidity < (int)humidityS - 3) {
    Serial.println("0");
    Serial1.println("0");
}
else {
    Serial0.println("1");
    Serial1.println("1");
    }

// DHT11 sampling rate is 1HZ.
delay(1000);
}