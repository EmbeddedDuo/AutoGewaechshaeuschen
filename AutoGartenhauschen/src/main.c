// Include necessary libraries and headers
#include <stdio.h>
#include <stdlib.h>
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
#include "esp_event.h"
#include "esp_http_server.h"

// Definitions for DHT sensor type and GPIO pins
#define SENSOR_TYPE DHT_TYPE_AM2301
uint8_t dht_gpio_1 = 18;
uint8_t dht_gpio_2 = 5;
uint8_t dht_gpio_3 = 27;

// Definitions for ADC configuration
#define ADC_ATTEN_0db 0
#define ADC_WIDTH_12Bit 3
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF 1100

// LED GPIO pin
#define LED_GPIO 14

// Servo configuration
#define SERVO_CH1_PIN 25

// Queues for communication between tasks
QueueHandle_t avgTempQueue;
QueueHandle_t DhtDataQueue;
QueueHandle_t DataHandlerQueue;

// Task handles
TaskHandle_t dht_task1 = NULL;
TaskHandle_t dht_task2 = NULL;
TaskHandle_t dht_task3 = NULL;

// Structure for DHT sensor data
typedef struct DhtQueueMessage
{
    char *TaskName;
    float humidity;
    float temperature;
};

// Array to store DHT sensor data from different tasks
struct DhtQueueMessage dhtMessages[3];

// Semaphores for updating temperature and humidity
SemaphoreHandle_t tempUpdateSemaphore;
SemaphoreHandle_t humUpdateSemaphore;

// Threshold temperature for servo control
float thresholdTemperature = 25;

// Function to get current time in seconds
static uint32_t get_time_sec()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec;
}

// Character data for custom characters on LCD
static const uint8_t char_data[] = {
    0x04, 0x0e, 0x0e, 0x0e, 0x1f, 0x00, 0x04, 0x00,
    0x1f, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x1f, 0x00};

// LCD task function
void lcd_task(void *pvParameters)
{
    // Initialize LCD
    hd44780_t lcd = initializeHD44780();
    hd44780_switch_backlight(&lcd, true);

    // Upload custom characters to LCD
    hd44780_upload_character(&lcd, 0, char_data);
    hd44780_upload_character(&lcd, 1, char_data + 8);

    // Structure to receive DHT sensor data
    struct DhtQueueMessage ReceiveMessage;

    while (1)
    {
        // Variables for calculating average temperature and humidity
        float avgTemperature = 0.0;
        float avgHumidity = 0.0;

        // Receive DHT sensor data from the queue
        while (uxQueueSpacesAvailable(DhtDataQueue) != 3)
        {
            if (xQueueReceive(DhtDataQueue, &ReceiveMessage, (TickType_t)5) == pdTRUE)
            {
                ESP_LOGI("Queue", "data successfully received from %s", ReceiveMessage.TaskName);
                printf("Humidity: %.1f  ,  Temperature: %.1f  \n", ReceiveMessage.humidity, ReceiveMessage.temperature);

                // Store received data in the array based on task name
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
                ESP_LOGW("Queue", "couldnt receive Data");
            }
        }

        // Calculate average temperature and humidity
        for (uint8_t i = 0; i < 3; i++)
        {
            avgHumidity += dhtMessages[i].humidity;
            avgTemperature += dhtMessages[i].temperature;
        }

        avgHumidity /= 3;
        avgTemperature /= 3;

        // Send average temperature to the queue
        if (xQueueSend(avgTempQueue, (void *)&avgTemperature, (TickType_t)0) == pdTRUE)
        {
            ESP_LOGI("avgTempQueue", "Average temperature successfully sent");
        }
        else
        {
            ESP_LOGI("avgTempQueue", "nope");
        }

        // Send average temperature and humidity to DataHandlerQueue
        if (xQueueSend(DataHandlerQueue, (void *)&avgTemperature, (TickType_t)0) == pdTRUE &&
            xQueueSend(DataHandlerQueue, (void *)&avgHumidity, (TickType_t)0) == pdTRUE)
        {
            ESP_LOGI("DataHandlerQueue", "Data successfully sent to DataHandler");
        }
        else
        {
            ESP_LOGI("DataHandlerQueue", "nope");
        }

        // Print temperature and humidity on LCD
        printHumidity(lcd, avgHumidity);
        printTemperature(lcd, avgTemperature);

        // Delay for 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Function to read data from DHT sensor
void dht_task(void *pvParameters)
{
    uint8_t *dht_gpio_ptr = (uint8_t *)pvParameters;
    uint8_t dht_gpio = *dht_gpio_ptr;
    printf("Der Pin lautet: %" PRId8 "\n", dht_gpio);

    float temperature, humidity;

    // Structure to send DHT sensor data
    struct DhtQueueMessage sendMessage;

    while (1)
    {
        // Read data from DHT sensor
        if (dht_read_float_data(SENSOR_TYPE, dht_gpio, &humidity, &temperature) == ESP_OK)
        {
            ESP_LOGI("DHT", "Humidity: %.1f%% Temp: %.1fC an Pin %u\n", humidity, temperature, dht_gpio);
            sendMessage.humidity = humidity;
            sendMessage.temperature = temperature;
            sendMessage.TaskName = pcTaskGetName(xTaskGetCurrentTaskHandle());

            // Send data to DhtDataQueue
            if (xQueueSend(DhtDataQueue, (void *)&sendMessage, (TickType_t)0) == pdTRUE)
            {
                ESP_LOGI("Queue", "temperature successfully sent");
            }
            else
            {
                ESP_LOGW("Queue", "Could not send Data");
            }
        }
        else
        {
            printf("Could not read data from sensor\n");
        }

        // Delay for 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Function to test photoresistor
void photoresistor_test()
{
    uint32_t reading;
    uint32_t voltage;

    // Set up LED GPIO
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    // Configure ADC
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11));

    // Characterization of ADC
    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    while (true)
    {
        // Read data from photoresistor
        reading = adc1_get_raw(ADC1_CHANNEL_5);
        voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
        ESP_LOGI("photoresistor", "%lu mV", voltage);

        // Control LED based on voltage
        if (voltage >= 2500)
        {
            ESP_LOGI("LED ON", "");
            gpio_set_level(LED_GPIO, 1);
        }
        else if (voltage < 2500)
        {
            gpio_set_level(LED_GPIO, 0);
        }

        // Delay for 50 ms
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// Function to control servo motor
void servo_task()
{
    // Servo configuration
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
        // Initialize delay buffer
        TickType_t delayBuff = 0;

        // Receive average temperature from avgTempQueue
        if (xQueueReceive(avgTempQueue, &avgTemperature, (TickType_t)5) == pdTRUE)
        {
            ESP_LOGI("avgTempQueue", "Avg Temp received: %f", avgTemperature);

            // Control servo based on average temperature
            if (avgTemperature >= thresholdTemperature && !isOpen)
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
            else if (avgTemperature < thresholdTemperature && isOpen)
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

        // Delay with added buffer
        vTaskDelay(pdMS_TO_TICKS(50 + delayBuff));
    }
}

// Handler for the root URI (GET request)
esp_err_t get_root_handler(httpd_req_t *req)
{
    // HTML content for the root page
    char str1[] = "<!DOCTYPE html>\
                        <html>\
                        <head>\
                            <title> ESP32 Web Server</title>\
                            <style>\
                                // Styles for HTML elements\
                            </style>\
                            <title> ESP32 Web Server</title>\
                            <script>\
                                // JavaScript functions\
                            </script>\
                        </head>\
                        <body style = \"background-color : white; text-align: center \">\
                            <div>\
                                <h2>Temperature</h2>\
                                <p id='temperature'></p>";

    char str2[] = "</p>\
                            </div>\
                            <div>\
                                <h2>Humidity</h2>\
                                 <p id='humidity'></p>";

    char str3[] = "</p>\
                            </div>\
                            <div>\
                                <form id=\"myForm\">\
                                <input type=\"number\" name=\"data\" id=\"numberInput\" min=\"15\" max=\"35\">\
                                <input type=\"button\" value=\"Submit\" onclick= \"submitForm()\">\
                                <p style = \"font-size: 23px\" id='resultContainer'> Click To Submit Target Temperature </p>\
                        </form>";

    char str4[] = "     </body>\
                        </html>";

    // Send HTML chunks as response
    httpd_resp_sendstr_chunk(req, str1);
    httpd_resp_sendstr_chunk(req, str2);
    httpd_resp_sendstr_chunk(req, str3);
    httpd_resp_sendstr_chunk(req, str4);
    httpd_resp_sendstr_chunk(req, NULL);

    return ESP_OK;
}

// Handler for data URI (GET request)
esp_err_t get_data_handler(httpd_req_t *req)
{
    static float temperature = 0;
    static float humidity = 0;
    float tempTemperature, tempHumidity;

    // Receive data from DataHandlerQueue
    if (xQueueReceive(DataHandlerQueue, &tempTemperature, (TickType_t)5) == pdTRUE &&
        xQueueReceive(DataHandlerQueue, &tempHumidity, (TickType_t)5) == pdTRUE)
    {
        ESP_LOGI("DataHandlerQueue", "Data successfully received in DataHandler");
        temperature = tempTemperature;
        humidity = tempHumidity;
    }

    // Prepare JSON response
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "{\"temperature\": %.1f, \"humidity\": %.1f}", temperature, humidity);

    // Set response type as JSON
    httpd_resp_set_type(req, "application/json");
    // Send JSON response
    httpd_resp_send(req, buffer, strlen(buffer));

    return ESP_OK;
}

// Handler for threshold URI (POST request)
esp_err_t post_threshold_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

    // Read request content
    while (remaining > 0)
    {
        if ((ret = httpd_req_recv(req, buf, sizeof(buf))) <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {
                // Request timed out
                httpd_resp_send_408(req);
            }
            return ESP_FAIL;
        }

        remaining -= ret;
    }

    char data_param[32];
    // Parse data from the request
    if (httpd_query_key_value(buf, "data", data_param, sizeof(data_param)) == ESP_OK)
    {
        // Convert received data to integer
        int received_data = atoi(data_param);
        ESP_LOGI("threshold", "Received value: %d", received_data);
        
        // Set the threshold temperature
        thresholdTemperature = (float)received_data;

        const char *resp[64];
        // Send response to the client
        snprintf(resp, 64, "Target Successfully set to %d", received_data);
        httpd_resp_send(req, resp, strlen(resp));
    }
    else
    {
        // Error in data processing, send Internal Server Error status
        httpd_resp_set_status(req, "500 Internal Server Error");
    }

    return ESP_OK;
}

/* URI handler structure for GET /uri */
httpd_uri_t uri_get_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_root_handler,
    .user_ctx = NULL};

httpd_uri_t uri_get_data = {
    .uri = "/data",
    .method = HTTP_GET,
    .handler = get_data_handler,
    .user_ctx = NULL};

httpd_uri_t uri_post_threshold = {
    .uri = "/submit",
    .method = HTTP_POST,
    .handler = post_threshold_handler,
    .user_ctx = NULL};

httpd_handle_t start_webserver(void)
{
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    /* Empty handle to esp_http_server */
    httpd_handle_t server = NULL;
    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK)
    {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &uri_get_root);
        httpd_register_uri_handler(server, &uri_get_data);
        httpd_register_uri_handler(server, &uri_post_threshold);
    }
    return server;
}

void app_main()
{
    esp_err_t ret = nvs_flash_init(); // Initialize NVS Flash memory

    // Check for NVS initialization errors
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret); // Check for general initialization errors

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    init_wifi(); // Initialize Wi-Fi

    // Wait until Wi-Fi is established
    while (!checkWifiEstablished())
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    DhtDataQueue = xQueueCreate(3, sizeof(struct DhtQueueMessage));

    // Check if DhtDataQueue is created successfully
    if (DhtDataQueue == NULL)
    {
        ESP_LOGE("Queue", "Queue couldn't be created");
    }

    avgTempQueue = xQueueCreate(2, sizeof(float));

    // Check if avgTempQueue is created successfully
    if (avgTempQueue == NULL)
    {
        ESP_LOGE("Queue2", "Queue couldn't be created");
    }

    DataHandlerQueue = xQueueCreate(2, sizeof(float));

    // Check if DataHandlerQueue is created successfully
    if (DhtDataQueue == NULL)
    {
        ESP_LOGE("Queue", "Queue couldn't be created");
    }

    ESP_ERROR_CHECK(i2cdev_init());

    // Create LCD task
    xTaskCreate(lcd_task, "lcd_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);

    // Create photoresistor task
    xTaskCreate(photoresistor_test, "photoresistor_test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);

    // Create DHT sensor tasks
    xTaskCreate(dht_task, "dht_task1", configMINIMAL_STACK_SIZE * 3, &dht_gpio_1, 5, &dht_task1);
    xTaskCreate(dht_task, "dht_task2", configMINIMAL_STACK_SIZE * 3, &dht_gpio_2, 5, &dht_task2);
    xTaskCreate(dht_task, "dht_task3", configMINIMAL_STACK_SIZE * 3, &dht_gpio_3, 5, &dht_task3);

    // Create servo task
    xTaskCreate(servo_task, "servo_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);

    start_webserver();

    // test
}

