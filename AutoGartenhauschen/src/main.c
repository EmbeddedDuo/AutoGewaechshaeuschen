#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <sys/time.h>
#include <string.h>
#include <esp_log.h>

#include <freertos/queue.h>

#include <driver/adc.h>
#include <esp_adc_cal.h>

#include <iot_servo.h>

#include <hd44780Gartenhaus.h>
#include <dht.h>
#include <wifiGartenhaus.h>

#define SENSOR_TYPE DHT_TYPE_AM2301

uint8_t dht_gpio_1 = 18;
uint8_t dht_gpio_2 = 5;
uint8_t dht_gpio_3 = 27;

#define ADC_ATTEN_0db 0
#define ADC_WIDTH_12Bit 3
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF 1100

#define LED_GPIO 14

#define SERVO_CH1_PIN 25

QueueHandle_t avgTempQueue;

TaskHandle_t dht_task1 = NULL;
TaskHandle_t dht_task2 = NULL;
TaskHandle_t dht_task3 = NULL;

QueueHandle_t temperatureQueue;

typedef struct DhtQueueMessage
{
    char *TaskName;
    float humidity;
    float temperature;
};

struct DhtQueueMessage dhtMessages[3];

static uint32_t get_time_sec()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec;
}

static const uint8_t char_data[] = {
    0x04, 0x0e, 0x0e, 0x0e, 0x1f, 0x00, 0x04, 0x00,
    0x1f, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x1f, 0x00};

void lcd_task(void *pvParameters)
{
    hd44780_t lcd = initializeHD44780();
    hd44780_switch_backlight(&lcd, true);

    hd44780_upload_character(&lcd, 0, char_data);
    hd44780_upload_character(&lcd, 1, char_data + 8);

    struct DhtQueueMessage ReceiveMessage;

    while (1)
    {

        float avgTemperature = 0.0;
        float avgHumidity = 0.0;

        while (uxQueueSpacesAvailable(temperatureQueue) != 3)
        {
            if (xQueueReceive(temperatureQueue, &ReceiveMessage, (TickType_t)5) == pdTRUE)
            {
                ESP_LOGI("Queue", "data successfully received from %s", ReceiveMessage.TaskName);
                printf("Humidity: %.1f  ,  Temperature: %.1f  \n", ReceiveMessage.humidity, ReceiveMessage.temperature);

                if (strcmp(ReceiveMessage.TaskName, "dht_task1") == 0)
                {
                    dhtMessages[0] = ReceiveMessage;
                }
                else if (strcmp(ReceiveMessage.TaskName, "dht_task2") == 0)
                {
                    dhtMessages[1] = ReceiveMessage;
                }
                else
                {
                    dhtMessages[2] = ReceiveMessage;
                }
            }
            else
            {
                ESP_LOGW("Queue", "couldnt recieve Data");
            }
        }

        for(uint8_t i = 0; i < 3; i++){
            avgHumidity += dhtMessages[i].humidity;
            avgTemperature += dhtMessages[i].temperature;
        }

        avgHumidity /= 3;
        avgTemperature /= 3;

        if (xQueueSend(avgTempQueue, (void *)&avgTemperature, (TickType_t)0) == pdTRUE)
        {
            ESP_LOGI("avgTempQueue", "Average temperature successfully sent");
        }
        else
        {
            ESP_LOGI("avgTempQueue", "nope");
        }

        printHumidity(lcd, avgHumidity);
        printTemperature(lcd, avgTemperature);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void dht_task(void *pvParameters)
{
    uint8_t *dht_gpio_ptr = (uint8_t *)pvParameters;
    uint8_t dht_gpio = *dht_gpio_ptr;
    printf("Der Pin lautet: %" PRId8 "\n", dht_gpio);

    float temperature, humidity;

    struct DhtQueueMessage sendMessage;

    while (1)
    {
        if (dht_read_float_data(SENSOR_TYPE, dht_gpio, &humidity, &temperature) == ESP_OK)
        {
            ESP_LOGI("DHT", "Humidity: %.1f%% Temp: %.1fC an Pin %u\n", humidity, temperature, dht_gpio);
            sendMessage.humidity = humidity;
            sendMessage.temperature = temperature;
            sendMessage.TaskName = pcTaskGetName(xTaskGetCurrentTaskHandle());
            if (xQueueSend(temperatureQueue, (void *)&sendMessage, (TickType_t)0) == pdTRUE)
            {
                ESP_LOGI("Queue", "temperature successfully sent");
            }
            else
            {
                ESP_LOGW("Queue", "Could not sent Data");
            }
        }
        else
        {
            printf("Could not read data from sensor\n");

            // If you read the sensor data too often, it will heat up
            // http://www.kandrsmith.org/RJS/Misc/Hygrometers/dht_sht_how_fast.html
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

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

    while (true)
    {

        reading = adc1_get_raw(ADC1_CHANNEL_5);
        voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
        ESP_LOGI("photoresistor", "%lu mV", voltage);

        if (voltage >= 2500)
        {
            ESP_LOGI("LED ON", "");
            gpio_set_level(LED_GPIO, 1);
        }
        else if (voltage < 2500)
        {
            gpio_set_level(LED_GPIO, 0);
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void servo_task()
{
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                SERVO_CH1_PIN},
            .ch = {
                LEDC_CHANNEL_1,
            },
        },
        .channel_number = 1,
    };
    ESP_ERROR_CHECK(iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg));

    bool isOpen = false;

    float avgTemperature;

    while (true)
    {
        TickType_t delayBuff = 0;
        if (xQueueReceive(avgTempQueue, &avgTemperature, (TickType_t)5) == pdTRUE)
        {
            ESP_LOGI("avgTempQueue", "Avg Temp recieved: %f", avgTemperature);
            if (avgTemperature >= 25 && !isOpen)
            {
                for (uint8_t i = 70; i <= 160; i++)
                {
                    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, i);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    ESP_LOGI("servo", "[%d] ", i);
                }
                isOpen = true;

                delayBuff = 2000;
            }
            else if (avgTemperature < 25 && isOpen)
            {

                for (uint8_t i = 160; i >= 70; i--)
                {
                    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, i);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    ESP_LOGI("servo", "[%d] ", i);
                }
                isOpen = false;

                delayBuff = 2000;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50 + delayBuff));
    }
}

void app_main()
{

    esp_err_t ret = nvs_flash_init(); // NVS-Flash-Speicher initialisieren
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret); // Fehlerprüfung

    init_wifi(); // WLAN initialisieren

    while (!checkWifiEstablished())
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    temperatureQueue = xQueueCreate(3, sizeof(struct DhtQueueMessage));

    if (temperatureQueue == NULL)
    {
        ESP_LOGE("Queue", "Queue couldnt be created");
    }

    avgTempQueue = xQueueCreate(2, sizeof(float));

    if (avgTempQueue == NULL)
    {
        ESP_LOGE("Queue2", "Queue couldnt be created");
    }

    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(lcd_task, "lcd_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);

    // xTaskCreate(photoresistor_test, "photoresistor_test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    xTaskCreate(dht_task, "dht_task1", configMINIMAL_STACK_SIZE * 3, &dht_gpio_1, 5, &dht_task1);
    xTaskCreate(dht_task, "dht_task2", configMINIMAL_STACK_SIZE * 3, &dht_gpio_2, 5, &dht_task2);
    xTaskCreate(dht_task, "dht_task3", configMINIMAL_STACK_SIZE * 3, &dht_gpio_3, 5, &dht_task3);

    xTaskCreate(servo_task, "servo_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);

    // test
}