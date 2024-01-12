#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <sys/time.h>
#include <string.h>
#include <esp_log.h>

#include <iot_servo.h>

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
        ESP_ERROR_CHECK(iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg));
        size_t i;
        float angle_ls;
        for (i = 0; i <= 180; i++)
        {
           iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, i);  
            vTaskDelay(50 / portTICK_PERIOD_MS);
            iot_servo_read_angle(LEDC_LOW_SPEED_MODE, 0, &angle_ls);
            ESP_LOGI("servo", "[%d|%.2f] ", i, angle_ls);
        }

        iot_servo_deinit(LEDC_LOW_SPEED_MODE);
    }
}

void app_main()
{

    xTaskCreate(servo_test, "servo_test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}