#include <HX711.h>

// Pin definitions for HX711
#define DT_PIN 21 // GPIO pin connected to DT (data) of HX711
#define SCK_PIN 22 // GPIO pin connected to SCK (clock) of HX711

float rawValue;

HX711 scale;

void setup() {
  Serial.begin(115200);
  Serial.println("HX711 Offset Calibration");
  
  // Initialize the scale
  scale.begin(DT_PIN, SCK_PIN);

  // Check if the HX711 is connected
 // if (!scale.is_ready()) {
 //   Serial.println("Error: HX711 not found.");
 //   while (true) {
 //     delay(100);
 //  }
 // }

  Serial.println("Place no weight on the scale.");
  delay(5000); // Wait for user to remove all weight
  Serial.println("Calibrating...");

  // Tare the scale to zero it
  scale.tare();
  Serial.println("Scale offset calibrated.");
  
  Serial.print("Offset value: ");
  Serial.println(scale.get_offset());

  Serial.println("You can now add weight to measure readings.");
  delay(1000);
}

void loop() {
  if (scale.is_ready()) {
    // Read the raw value from the HX711
    rawValue = scale.get_units();
    Serial.print("Weight (units): ");
    Serial.println(rawValue);
  } else {
    Serial.println("Waiting for HX711...");
  }
  delay(100); // Update every 500 ms
}