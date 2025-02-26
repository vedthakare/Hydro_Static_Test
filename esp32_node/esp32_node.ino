#include <WiFi.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ADC Channels (ADC1 = GPIO34, ADC2 = GPIO27)
#define ADC1_GPIO_CHANNEL  ADC_CHANNEL_6  // GPIO34 is ADC1_CH6
#define ADC2_GPIO_CHANNEL  ADC_CHANNEL_7  // GPIO27 is ADC2_CH7

// ADC instances
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_handle_t adc2_handle;
adc_cali_handle_t adc_cali_handle = NULL;

// Function to initialize calibration
bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);

void setup() {
    Serial.begin(115200);
    
    // Turn off WiFi to avoid conflicts with ADC2
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    
    // Initialize ADC1 with oneshot mode
    adc_oneshot_unit_init_cfg_t adc1_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_config, &adc1_handle));
    
    // Initialize ADC2 with oneshot mode
    adc_oneshot_unit_init_cfg_t adc2_config = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc2_config, &adc2_handle));
    
    // Configure ADC1 channel
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_GPIO_CHANNEL, &chan_config));
    
    // Configure ADC2 channel
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC2_GPIO_CHANNEL, &chan_config));
    
    // Initialize ADC calibration
    bool calibration_enabled = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_12, &adc_cali_handle);
    if (!calibration_enabled) {
        Serial.println("Failed to enable ADC calibration");
    }
}

void loop() {
    int raw1 = 0;
    int raw2 = 0;
    
    // Read ADC values with oneshot API
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_GPIO_CHANNEL, &raw1));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC2_GPIO_CHANNEL, &raw2));
    
    int voltage1_mv = 0;
    int voltage2_mv = 0;
    
    if (adc_cali_handle != NULL) {
        // Convert raw readings to voltage
        adc_cali_raw_to_voltage(adc_cali_handle, raw1, &voltage1_mv);
        adc_cali_raw_to_voltage(adc_cali_handle, raw2, &voltage2_mv);
    }
    
    float voltage1 = voltage1_mv / 1000.0;
    float voltage2 = voltage2_mv / 1000.0;
    
    Serial.printf("ADC1 (GPIO34) Raw: %d, Voltage: %.6f V\n", raw1, voltage1);
    Serial.printf("ADC2 (GPIO27) Raw: %d, Voltage: %.6f V\n", raw2, voltage2);
    
    delay(1000);
}

// Helper function to initialize ADC calibration
bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_12,
        .default_vref = 1100,
    };
    
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
        calibrated = true;
    } else {
        Serial.println("Failed to create calibration scheme");
    }

    *out_handle = handle;
    return calibrated;
}