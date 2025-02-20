#include <Arduino.h>

const int sensorPin1 = 34;    // First analog sensor pin
const int sensorPin2 = 27;    // Second analog sensor pin
#define RXD2 16              // GPIO16 for RX2
#define TXD2 17              // GPIO17 for TX2

void setup() {
    // Initialize serial communication using Serial2 (UART2)
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
}

void loop() {
    int sensorValue1 = analogRead(sensorPin1);
    int sensorValue2 = analogRead(sensorPin2);
    
    // Send both values separated by a comma
    Serial2.print(sensorValue1);
    Serial2.print(",");
    Serial2.println(sensorValue2);
    
    delay(100); // Adjust delay as needed
}