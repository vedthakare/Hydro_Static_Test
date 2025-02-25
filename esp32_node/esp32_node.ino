#include <Arduino.h>

const int sensorPin1 = 34;    // First analog sensor pin
const int sensorPin2 = 27;    // Second analog sensor pin
#define RXD2 16              // GPIO16 for RX2
#define TXD2 17              // GPIO17 for TX2

bool startStreaming = false;

void setup() {
    // Initialize serial communication using Serial2 (UART2)
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
}

void loop() {

    // Only stream data if start signal received
    if (startStreaming) {
        // Convert raw ADC values to voltage (ESP32 has 12-bit ADC, 3.3V reference)
        float voltage1 = (analogRead(sensorPin1) * 5) / 4095.0; // outs the 5V equivalent
        float voltage2 = (analogRead(sensorPin2) * 5) / 4095.0;
        
        // Send both voltage values separated by a comma
        Serial2.print(voltage1, 4);  // 4 decimal places precision
        Serial2.print(",");
        Serial2.println(voltage2, 4);
        
        delay(5); // Adjust delay as needed
    }
}