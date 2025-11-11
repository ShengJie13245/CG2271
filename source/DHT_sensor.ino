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

Serial.println("=================================");
Serial.println("Initalize DHT");

if (dht11.read(pinDHT11, &temperatureS, &humidityS, dataS)) {
        Serial.print("Read DHT11 failed");
        return;
    }
delay(1000);
Serial.println("=================================");

}

void loop() {
// start working...
// read with raw sample data.
byte temperature = 0;
byte humidity = 0;
byte data[40] = {0};

Serial.println("Waiting for request");
while(Serial1.available() <=0);
String x = Serial1.readString();

Serial.print("Request received from MCXC444: ");
Serial.print(x);


Serial.println("=================================");
Serial.println("Sample DHT11");
if (dht11.read(pinDHT11, &temperature, &humidity, data)) {
    Serial.print("Read DHT11 failed");
    return;
}
Serial.print("Baseline humidity ");
Serial.print((int)humidityS); Serial.println("%"); 

Serial.print("Current Humidity: ");
Serial.print((int)humidity); Serial.println("%");

Serial.print("Message to be sent: ");
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

Serial.println("=================================");
// DHT11 sampling rate is 1HZ.
delay(1000);
}