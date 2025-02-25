#include <Arduino.h>

const int sensorPin1 = 34;    // First analog sensor pin
const int sensorPin2 = 27;    // Second analog sensor pin
#define RXD2 16              // GPIO16 for RX2
#define TXD2 17              // GPIO17 for TX2

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    delay(100); // Allow time for serial connection stabilization
}

void loop() {
    // Read raw sensor values
    int raw1 = analogRead(sensorPin1);
    int raw2 = analogRead(sensorPin2);

    // Convert raw ADC values to voltage (ESP32: 12-bit ADC, 3.3V reference)
    float voltage1 = (raw1 * 5) / 4095.0;
    float voltage2 = (raw2 * 5) / 4095.0;

    // Print data to Serial (for debugging)
    Serial.printf("Raw1: %d, Raw2: %d | Voltage1: %.4f, Voltage2: %.4f\n", 
                  raw1, raw2, voltage1, voltage2);

    // Send voltage values over Serial2
    Serial2.print(voltage1, 4);
    Serial2.print(",");
    Serial2.println(voltage2, 4);

    delay(5); // Adjust as needed
}
