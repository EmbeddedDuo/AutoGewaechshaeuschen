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

#include <hd44780.h>
#include <pcf8574.h>
#include <dht.h>

#define SENSOR_TYPE DHT_TYPE_AM2301

uint8_t dht_gpio_1 = 18;
uint8_t dht_gpio_2 = 5;
uint8_t dht_gpio_3 = 27;

static i2c_dev_t pcf8574;

#define ADC_ATTEN_0db 0
#define ADC_WIDTH_12Bit 3
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF 1100

#define LED_GPIO 14

#define SERVO_CH1_PIN 25

TaskHandle_t dht_task1 = NULL;
TaskHandle_t dht_task2 = NULL;
TaskHandle_t dht_task3 = NULL;

QueueHandle_t temperatureQueue;

QueueHandle_t windowQueue;

typedef struct DhtQueueMessage
{
    char *TaskName;
    float humidity;
    float temperature;
} Message;

typedef struct Messages
{
    Message Task1;
    Message Task2;
    Message Task3;
};

static uint32_t get_time_sec()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec;
}

static const uint8_t char_data[] = {
    0x04, 0x0e, 0x0e, 0x0e, 0x1f, 0x00, 0x04, 0x00,
    0x1f, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x1f, 0x00};

static esp_err_t write_lcd_data(const hd44780_t *lcd, uint8_t data)
{
    return pcf8574_port_write(&pcf8574, data);
}

bool isTask(char *recievedTaskname, char *taskname){
    if(strcmp(recievedTaskname, taskname) == 0){
        return true;
    }
    return false;
}

void lcd_task(void *pvParameters)
{
    hd44780_t lcd = {
        .write_cb = write_lcd_data, // use callback to send data to LCD by I2C GPIO expander
        .font = HD44780_FONT_5X8,
        .lines = 2,
        .pins = {
            .rs = 0,
            .e = 2,
            .d4 = 4,
            .d5 = 5,
            .d6 = 6,
            .d7 = 7,
            .bl = 3}};

    memset(&pcf8574, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, CONFIG_EXAMPLE_I2C_ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    ESP_ERROR_CHECK(hd44780_init(&lcd));

    hd44780_switch_backlight(&lcd, true);

    hd44780_upload_character(&lcd, 0, char_data);
    hd44780_upload_character(&lcd, 1, char_data + 8);

    hd44780_gotoxy(&lcd, 0, 0);
    hd44780_puts(&lcd, "Temp: ");
    hd44780_gotoxy(&lcd, 0, 1);
    hd44780_puts(&lcd, "Hum: ");

    char temp[16];
    char hum[16];

    struct DhtQueueMessage ReceiveMessage;
    struct Messages m;

    float avgTemperature = 0.0;
    float avgHumidity = 0.0;

    while (1)
    {

        float tmpAvgTemperature = 0.0;
        float tmpAvgHumidity = 0.0;

        if (uxQueueSpacesAvailable(temperatureQueue) == 0)
        {
            vTaskSuspend(dht_task1);
            vTaskSuspend(dht_task2);
            vTaskSuspend(dht_task3);
        }

        while (uxQueueSpacesAvailable(temperatureQueue) != 3)
        {
            if (xQueueReceive(temperatureQueue, &ReceiveMessage, (TickType_t)5) == pdTRUE)
            {
                ESP_LOGI("Queue", "data successfully received from %s", ReceiveMessage.TaskName);
                printf("Humidity: %.1f  ,  Temperature: %.1f  \n", ReceiveMessage.humidity, ReceiveMessage.temperature);
            }
            else
            {
                ESP_LOGW("Queue", "couldnt recieve Data");
            }
            if(isTask(ReceiveMessage.TaskName, "dht_task1")){
                m.Task1 = ReceiveMessage;
                tmpAvgHumidity+= ReceiveMessage.humidity;
                tmpAvgTemperature += ReceiveMessage.temperature;
            }else if(isTask(ReceiveMessage.TaskName, "dht_task2")){
                m.Task2 = ReceiveMessage;
                tmpAvgHumidity+= ReceiveMessage.humidity;
                tmpAvgTemperature += ReceiveMessage.temperature;
            }else{
                m.Task3 = ReceiveMessage;
                tmpAvgHumidity+= ReceiveMessage.humidity;
                tmpAvgTemperature += ReceiveMessage.temperature;
            }
        }

        tmpAvgTemperature /= 3;
        tmpAvgHumidity /= 3;

        avgHumidity = tmpAvgHumidity;
        avgTemperature = tmpAvgTemperature;


        hd44780_gotoxy(&lcd, 7, 0);

        snprintf(temp, 6, "%.1fC", avgTemperature);
        temp[sizeof(temp) - 1] = 0;

        hd44780_puts(&lcd, temp);

        hd44780_gotoxy(&lcd, 7, 1);

        snprintf(hum, 6, "%.1f%%", avgHumidity);
        hum[sizeof(hum) - 1] = 0;

        hd44780_puts(&lcd, hum);


        vTaskResume(dht_task1);
        vTaskResume(dht_task2);
        vTaskResume(dht_task3);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*
void lcd_test(void *pvParameters)
{
    hd44780_t lcd = {
        .write_cb = write_lcd_data, // use callback to send data to LCD by I2C GPIO expander
        .font = HD44780_FONT_5X8,
        .lines = 2,
        .pins = {
            .rs = 0,
            .e = 2,
            .d4 = 4,
            .d5 = 5,
            .d6 = 6,
            .d7 = 7,
            .bl = 3}};

    memset(&pcf8574, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, CONFIG_EXAMPLE_I2C_ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    ESP_ERROR_CHECK(hd44780_init(&lcd));

    hd44780_switch_backlight(&lcd, true);

    hd44780_upload_character(&lcd, 0, char_data);
    hd44780_upload_character(&lcd, 1, char_data + 8);

    hd44780_gotoxy(&lcd, 0, 0);
    hd44780_puts(&lcd, "\x08 Hello world!");
    hd44780_gotoxy(&lcd, 0, 1);
    hd44780_puts(&lcd, "\x09 ");

    char time[16];

    while (1)
    {
        hd44780_gotoxy(&lcd, 2, 1);

        snprintf(time, 7, "%" PRIu32 "  ", get_time_sec());
        time[sizeof(time) - 1] = 0;

        hd44780_puts(&lcd, time);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
} */

void dht_task(void *pvParameters)
{
    uint8_t *dht_gpio_ptr = (uint8_t *)pvParameters;
    uint8_t dht_gpio = *dht_gpio_ptr;
    printf("Der Pin lautet: %" PRId8 "\n", dht_gpio);

    float temperature, humidity;

    struct DhtQueueMessage sendMessage;
    temperatureQueue = xQueueCreate(3, sizeof(sendMessage));

    if (temperatureQueue == NULL)
    {
        ESP_LOGE("Queue", "Queue couldnt be created");
    }

    while (1)
    {
        if (dht_read_float_data(SENSOR_TYPE, dht_gpio, &humidity, &temperature) == ESP_OK)
        {
            printf("Humidity: %.1f%% Temp: %.1fC an Pin %" PRId8 "\n", humidity, temperature, dht_gpio);
            sendMessage.humidity = humidity;
            sendMessage.temperature = temperature;
            sendMessage.TaskName = pcTaskGetName(xTaskGetCurrentTaskHandle());
            if (xQueueSend(temperatureQueue, (void *)&sendMessage, (TickType_t)0) == pdTRUE)
            {
                ESP_LOGI("Queue", "temperature successfully sent");
            } else{
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

/*
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
*/
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
    while (true)
    {
        bool windowState;
        size_t i;
        if (xQueueReceive(windowQueue, &windowState, (TickType_t)5) == pdTRUE)
        {
            ESP_LOGI("Queue2", "windowState successfully received");

            if (windowState)
            {
                for (i = 70; i <= 160; i++)
                {
                    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, i);
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    ESP_LOGI("servo", "[%d] ", i);
                }
            }
            else
            {

                for (i = 160; i >= 50; i--)
                {
                    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 1, i);
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    ESP_LOGI("servo", "[%d] ", i);
                }
            }
        }
    }
}

/*

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
                SERVO_CH1_PIN},
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
} */

void app_main()
{

    bool windowState = true;
    windowQueue = xQueueCreate(1, sizeof(windowState));

    if (windowQueue == NULL)
    {
        ESP_LOGE("Queue2", "Queue couldnt be created");
    }

    if (xQueueSend(windowQueue, (void *)&windowState, (TickType_t)3) == pdTRUE)
    {
        ESP_LOGI("Queue2", "windowState successfully sent");
    }
    /*
    xTaskCreate(dht_test, "dht_pin1", configMINIMAL_STACK_SIZE * 3, &dht_gpio_1, 5, NULL);
    xTaskCreate(dht_test, "dht_pin2", configMINIMAL_STACK_SIZE * 3, &dht_gpio_2, 5, NULL);
    xTaskCreate(dht_test, "dht_pin3", configMINIMAL_STACK_SIZE * 3, &dht_gpio_3, 5, NULL);
    */

    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(lcd_task, "lcd_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);

    // xTaskCreate(photoresistor_test, "photoresistor_test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);

    xTaskCreate(dht_task, "dht_task1", configMINIMAL_STACK_SIZE * 3, &dht_gpio_1, 5, &dht_task1);
    xTaskCreate(dht_task, "dht_task2", configMINIMAL_STACK_SIZE * 3, &dht_gpio_2, 5, &dht_task2);
    xTaskCreate(dht_task, "dht_task3", configMINIMAL_STACK_SIZE * 3, &dht_gpio_3, 5, &dht_task3);

    xTaskCreate(servo_task, "servo_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);

}