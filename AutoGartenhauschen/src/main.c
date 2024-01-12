#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <sys/time.h>
#include <string.h>
#include <esp_log.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <iot_servo.h>

#define ADC_ATTEN_0db 0
#define ADC_WIDTH_12Bit 3
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF    1100 

#define LED_GPIO 14


void photoresistor_test()
{

    uint32_t reading;
    uint32_t voltage;

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11));

    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    // Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("eFuse Vref");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Two Point");
    }
    else
    {
        printf("Default");
    }
}

#define SERVO_CH1_PIN 25

void servo_test()
{

    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                SERVO_CH1_PIN
            },
            .ch = {
                LEDC_CHANNEL_1,
            },
        },
        .channel_number = 1,
    };
  
    while (true)
    {
         
        reading = adc1_get_raw(ADC1_CHANNEL_5);
        voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
        ESP_LOGI("photoresistor", "%lu mV", voltage );
       

        if(voltage >= 2500  ){
            ESP_LOGI("LED ON","");
            gpio_set_level(LED_GPIO, 1);
        }else if(voltage < 2500) {
             gpio_set_level(LED_GPIO, 0);
        }

         vTaskDelay(50 / portTICK_PERIOD_MS);

    }
}

void app_main()
{
    //xTaskCreate(photoresistor_test, "photoresistor_test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    //xTaskCreate(servo_test, "servo_test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}