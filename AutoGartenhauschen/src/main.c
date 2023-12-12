#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <dht.h>

#define SENSOR_TYPE DHT_TYPE_AM2301

uint8_t dht_gpio_1 = 18;
uint8_t dht_gpio_2 = 5;
uint8_t dht_gpio_3 = 27;

void dht_test(void *pvParameters)
{
    uint8_t *dht_gpio_ptr = (uint8_t *)pvParameters;
    uint8_t dht_gpio = *dht_gpio_ptr;
    printf("Der Pin lautet: %" PRId8 "\n", dht_gpio);

    float temperature, humidity;

    while (1)
    {   
        if (dht_read_float_data(SENSOR_TYPE, dht_gpio, &humidity, &temperature) == ESP_OK)
            printf("Humidity: %.1f%% Temp: %.1fC an Pin %" PRId8 "\n", humidity, temperature, dht_gpio);
        else
            printf("Could not read data from sensor\n");

        // If you read the sensor data too often, it will heat up
        // http://www.kandrsmith.org/RJS/Misc/Hygrometers/dht_sht_how_fast.html
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main()
{  

    xTaskCreate(dht_test, "dht_pin1", configMINIMAL_STACK_SIZE * 3, &dht_gpio_1, 5, NULL);
    xTaskCreate(dht_test, "dht_pin2", configMINIMAL_STACK_SIZE * 3, &dht_gpio_2, 5, NULL);
    xTaskCreate(dht_test, "dht_pin3", configMINIMAL_STACK_SIZE * 3, &dht_gpio_3, 5, NULL);
}