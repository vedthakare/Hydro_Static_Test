#include <Arduino.h>
#include <WiFi.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define RXD2 16
#define TXD2 17

#define ADC1_6_GPIO_CHANNEL  ADC_CHANNEL_6
#define ADC1_7_GPIO_CHANNEL  ADC_CHANNEL_7

adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc1_cali_handle = NULL;
adc_cali_handle_t adc1_7_cali_handle = NULL;

#define ADC1_7_CORRECTION_FACTOR 1

bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);

void setup() {
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    Serial.begin(115200);

    adc_oneshot_unit_init_cfg_t adc1_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc1_config, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_6_GPIO_CHANNEL, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_7_GPIO_CHANNEL, &chan_config));

    // **Initialize ADC calibration**
    if (!adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_12, &adc1_cali_handle)) {
        Serial.println("Calibration failed for ADC1_6");
    }
    if (!adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_12, &adc1_7_cali_handle)) {
        Serial.println("Calibration failed for ADC1_7");
    }
}

void loop() {
    int raw1 = 0, raw2 = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_6_GPIO_CHANNEL, &raw1));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_7_GPIO_CHANNEL, &raw2));

    int voltage1_mv = 0, voltage2_mv = 0; float voltage1 = 0; float voltage2 = 0;

    if (adc1_cali_handle != NULL) {
        adc_cali_raw_to_voltage(adc1_cali_handle, raw1, &voltage1_mv);
    } else {
        voltage1_mv = raw1;  // Use raw value if calibration fails
    }

    if (adc1_7_cali_handle != NULL) {
        adc_cali_raw_to_voltage(adc1_7_cali_handle, raw2, &voltage2_mv);
        voltage2_mv *= ADC1_7_CORRECTION_FACTOR;
    } else {
        voltage2_mv = raw2;  // Use raw value if calibration fails
    }

    Serial2.print(voltage1_mv);
    Serial2.print(",");
    Serial2.println(voltage2_mv);

    Serial.print(raw1);
    Serial.print(",");
    Serial.println(raw2);
    delay(5);
}

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
