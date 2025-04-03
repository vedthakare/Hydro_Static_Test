#include <Arduino.h>
#include <WiFi.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define RXD2 16 // Pin for UART receive (Serial2)
#define TXD2 17 // Pin for UART transmit (Serial2)

#define ADC2_6_GPIO_CHANNEL  ADC_CHANNEL_6 // ADC channel for first sensor
#define ADC2_7_GPIO_CHANNEL  ADC_CHANNEL_7 // ADC channel for second sensor

adc_oneshot_unit_handle_t adc2_handle; // Handle for managing ADC unit 2
adc_cali_handle_t adc2_6_cali_handle = NULL; // Handle for ADC calibration of channel 6
adc_cali_handle_t adc2_7_cali_handle = NULL; // Handle for ADC calibration of channel 7

// Function prototype for initializing ADC calibration
bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);

void setup() {
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // Initialize Serial2 for communication (e.g., external device)
    Serial.begin(115200); // Initialize Serial Monitor for debugging

    // Configure ADC unit 2 to be used for reading analog values
    adc_oneshot_unit_init_cfg_t adc2_config = {
        .unit_id = ADC_UNIT_2, // Specify ADC unit 2
        .ulp_mode = ADC_ULP_MODE_DISABLE, // Disable ultra-low-power mode
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc2_config, &adc2_handle)); // Create new ADC unit

    // Configure ADC channels with specific settings
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12, // Set attenuation for wider voltage range (0-3.3V)
        .bitwidth = ADC_BITWIDTH_12, // Set resolution to 12 bits (values from 0 to 4095)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC2_6_GPIO_CHANNEL, &chan_config)); // Configure channel 6
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC2_7_GPIO_CHANNEL, &chan_config)); // Configure channel 7

    // Initialize ADC calibration for both channels
    if (!adc_calibration_init(ADC_UNIT_2, ADC_ATTEN_DB_12, &adc2_6_cali_handle)) {
        Serial.println("Calibration failed for ADC2_6");
    }
    if (!adc_calibration_init(ADC_UNIT_2, ADC_ATTEN_DB_12, &adc2_7_cali_handle)) {
        Serial.println("Calibration failed for ADC2_7");
    }
}

void loop() {
    int raw1 = 0, raw2 = 0; // Variables to store raw ADC readings
    
    // Read raw ADC values from channels 6 and 7
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC2_6_GPIO_CHANNEL, &raw1));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC2_7_GPIO_CHANNEL, &raw2));

    // Output raw ADC values to Serial Monitor for debugging
    Serial.print("Raw ADC2_6: ");
    Serial.print(raw1);
    Serial.print(", Raw ADC2_7: ");
    Serial.println(raw2);
    
    delay(5); // Short delay before taking the next ADC readings
}

// Function to initialize ADC calibration for better accuracy
bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    // Configure calibration settings
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit, // Specify ADC unit
        .atten = atten, // Set attenuation level
        .bitwidth = ADC_BITWIDTH_12, // 12-bit resolution
        .default_vref = 1100, // Default reference voltage in mV (used for accuracy)
    };
    
    // Attempt to create a calibration scheme for the ADC
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
        calibrated = true; // Calibration successful
    } else {
        Serial.println("Failed to create calibration scheme"); // Print error if calibration fails
    }

    *out_handle = handle; // Store the calibration handle
    return calibrated; // Return whether calibration was successful
}