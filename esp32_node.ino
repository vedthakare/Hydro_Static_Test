#include <Arduino.h>

const int sensorPin = 34; // Example analog sensor pin

void setup() {
    Serial.begin(115200); // Initialize serial communication
}

void loop() {
    int sensorValue = analogRead(sensorPin);
    Serial.println(sensorValue); // Send data to Raspberry Pi
    delay(100); // Adjust as needed
}