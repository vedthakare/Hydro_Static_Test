#include <Wire.h>
#include <HX711.h>

// Define Serial2 pins for ESP32
#define RXD2 16
#define TXD2 17

// HX711 Load Cell Pins
const int LOADCELL_DOUT_PIN = 21;  // Use GPIO 32 instead of 21 if issues persist
const int LOADCELL_SCK_PIN = 22;   // Use GPIO 33 instead of 22

HX711 scale;

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

    Serial.println("ESP32 Load Cell Reader Initializing...");
    Serial2.println("ESP32 Load Cell Reader Initializing...");

    // Initialize HX711
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

    Serial.println("Checking HX711 connection...");
    Serial2.println("Checking HX711 connection...");

    // Wait for HX711 to be ready
    if (scale.wait_ready_timeout(5000)) { // Increased timeout for better detection
        Serial.println("HX711 detected!");
        Serial2.println("HX711 detected!");
    } else {
        Serial.println("ERROR: HX711 not detected! Check wiring.");
        Serial2.println("ERROR: HX711 not detected! Check wiring.");
        while (1) {  // Keep looping but allow debugging
            Serial.println("Recheck HX711 wiring...");
            delay(2000);
        }
    }

    // Apply calibration factor (adjust this for better accuracy)
    scale.set_scale(2280.f); 

    // Tare the scale to zero
    Serial.println("Taring scale... Please wait.");
    scale.tare();
    Serial.println("Scale zeroed. Ready to measure.");
    Serial2.println("Scale zeroed. Ready to measure.");
}

void loop() {
    if (scale.is_ready()) {
        float weight = scale.get_units(5);  // Average of 5 readings for stability

        // Print to Serial Monitor
        Serial.print("Weight: ");
        Serial.print(weight, 2);
        Serial.println(" kg");

        // Print to Serial2
        Serial2.print("Weight: ");
        Serial2.print(weight, 2);
        Serial2.println(" kg");
    } else {
        Serial.println("HX711 not responding...");
        Serial2.println("HX711 not responding...");
    }

    delay(1000);  // 1-second delay between readings
}
