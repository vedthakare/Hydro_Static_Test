#include <Wire.h>
#include "HX711.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

//Define Pi start pin
#define Pi_start 18
// Serial Communication Pins
#define RXD2 16
#define TXD2 17
// ADC Channel Definitions for Sensors
#define PT1_GPIO ADC_CHANNEL_6  // GPIO 34 (Pressure Sensor 1)
#define PT2_GPIO ADC_CHANNEL_7  // GPIO 35 (Pressure Sensor 2)
#define TC1_GPIO ADC_CHANNEL_7  // GPIO 27 (Thermocouple 1)
#define TC2_GPIO ADC_CHANNEL_6  // GPIO 14 (Thermocouple 2)

// Pin definitions for HX711
#define DT_PIN 21 // GPIO pin connected to DT (data) of HX711
#define SCK_PIN 22 // GPIO pin connected to SCK (clock) of HX711

float rawValue;
long loadcell_value;  // Added missing variable declaration

HX711 scale;

// Define a calibration factor for the load cell (adjust based on calibration)
#define LOADCELL_CALIBRATION_FACTOR 420.0 // Example value, needs to be calibrated

// Create ADC handles
adc_oneshot_unit_handle_t adc1_handle, adc2_handle;
adc_cali_handle_t adc1_cali_handle = NULL, adc2_cali_handle = NULL;





// Function to initialize ADC calibration
bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);


// Format: {slope, intercept} for each sensor
const float PT1_CALIBRATION[] = {0.9732, -0.0299};  // Example values - will calculate correct ones
const float PT2_CALIBRATION[] = {0.9967, -0.0299};  // Example values
const float TC1_CALIBRATION[] = {0.9950, -0.0142};  // Example values
const float TC2_CALIBRATION[] = {0.9950, -0.0142};  // Example values




void setup() {
    pinMode(Pi_start, INPUT_PULLDOWN);  // Add this line first
    
    
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    
    // Initialize the scale
    scale.begin(DT_PIN, SCK_PIN);
  
    // Check if the HX711 is connected
    // if (!scale.is_ready()) {
    //   Serial.println("Error: HX711 not found.");
    //   while (true) {
    //     delay(100);
    //   }
    // }
  
    //Serial.println("Place no weight on the scale.");
    delay(5000); // Wait for user to remove all weight
    //Serial.println("Calibrating...");
  
    // Tare the scale to zero it
    scale.tare();
    //Serial.println("Scale offset calibrated.");
    
    //Serial.print("Offset value: ");
    //Serial.println(scale.get_offset());
  
    //Serial.println("You can now add weight to measure readings.");
    delay(1000);

    // Correct ADC configuration
    adc_oneshot_unit_init_cfg_t adc1_config = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_unit_init_cfg_t adc2_config = { .unit_id = ADC_UNIT_2 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_config, &adc1_handle));
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc2_config, &adc2_handle));
    
    adc_oneshot_chan_cfg_t chan_config = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PT1_GPIO, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PT2_GPIO, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, TC1_GPIO, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, TC2_GPIO, &chan_config));
    
    // Initialize ADC calibration
    if (adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_12, &adc1_cali_handle)) {
        //Serial.println("ADC1 calibration success");
    } else {
        //Serial.println("ADC1 calibration failed");
    }
    
    if (adc_calibration_init(ADC_UNIT_2, ADC_ATTEN_DB_12, &adc2_cali_handle)) {
        //Serial.println("ADC2 calibration success");
    } else {
        //Serial.println("ADC2 calibration failed");
    }
}

void loop() {
    //Serial.println("23");
    while(digitalRead(Pi_start) == HIGH){

    
    int raw_pt1, raw_pt2, raw_tc1, raw_tc2;
    int voltage_pt1, voltage_pt2, voltage_tc1, voltage_tc2;
    float pt1_v, pt2_v, tc1_v, tc2_v;
    
    // Read raw ADC values
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PT1_GPIO, &raw_pt1));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PT2_GPIO, &raw_pt2));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, TC1_GPIO, &raw_tc1));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, TC2_GPIO, &raw_tc2));
    
    // Fix the problematic ternary operators with proper if-else statements
    if (adc1_cali_handle) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, raw_pt1, &voltage_pt1));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, raw_pt2, &voltage_pt2));
    } else {
        voltage_pt1 = raw_pt1;
        voltage_pt2 = raw_pt2;
    }
    
    if (adc2_cali_handle) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, raw_tc1, &voltage_tc1));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_handle, raw_tc2, &voltage_tc2));
    } else {
        voltage_tc1 = raw_tc1;
        voltage_tc2 = raw_tc2;
    }
    
    // Convert to volts
// Convert to volts with custom calibration
    pt1_v = applyCustomCalibration(voltage_pt1 / 1000.0, PT1_CALIBRATION);
    pt2_v = applyCustomCalibration(voltage_pt2 / 1000.0, PT2_CALIBRATION);
    tc1_v = applyCustomCalibration(voltage_tc1 / 1000.0, TC1_CALIBRATION);
    tc2_v = applyCustomCalibration(voltage_tc2 / 1000.0, TC2_CALIBRATION);
    
    // Get load cell reading with proper error handling
    if (scale.is_ready()) {
      // Read the raw value from the HX711
      loadcell_value = scale.get_units();
    } else {
      Serial.println("Waiting for HX711...");
    }
    
    Serial.printf("PT1_RAW: %d, PT1_V: %.3f, PT2_RAW: %d, PT2_V: %.3f, TC1_RAW: %d, TC1_V: %.3f, TC2_RAW: %d, TC2_V: %.3f, Loadcell: %ld\n", 
    raw_pt1, pt1_v, raw_pt2, pt2_v, raw_tc1, tc1_v, raw_tc2, tc2_v, loadcell_value);
    
    Serial2.printf("%d,%.3f,%d,%.3f,%d,%.3f,%d,%.3f,%ld\n", raw_pt1, pt1_v, raw_pt2, pt2_v, raw_tc1, tc1_v, raw_tc2, tc2_v, loadcell_value);
    
    delay(6); // Changed delay from 5ms to 100ms to reduce serial flooding
    }
}

float applyCustomCalibration(float measured_voltage, const float calibration[]) {
  // Apply linear correction: actual = measured * slope + intercept
  return measured_voltage * calibration[0] + calibration[1];
}

bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    adc_cali_line_fitting_config_t cali_config = { 
        .unit_id = unit, 
        .atten = atten, 
        .bitwidth = ADC_BITWIDTH_12, 
        .default_vref = 1100 
    };
    
    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
        *out_handle = handle;
        //Serial.printf("ADC calibration success for unit %d\n", unit);
        return true;
    }
    
    //Serial.printf("ADC calibration failed for unit %d with error code %d\n", unit, ret);
    return false;
}
