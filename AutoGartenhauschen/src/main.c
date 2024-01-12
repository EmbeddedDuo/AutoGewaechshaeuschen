#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <sys/time.h>
#include <string.h>

#include <hd44780.h>
#include <pcf8574.h>
#include <dht.h>

#define SENSOR_TYPE DHT_TYPE_AM2301

uint8_t dht_gpio_1 = 18;
uint8_t dht_gpio_2 = 5;
uint8_t dht_gpio_3 = 27;

static i2c_dev_t pcf8574;

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
}

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
    /*
    xTaskCreate(dht_test, "dht_pin1", configMINIMAL_STACK_SIZE * 3, &dht_gpio_1, 5, NULL);
    xTaskCreate(dht_test, "dht_pin2", configMINIMAL_STACK_SIZE * 3, &dht_gpio_2, 5, NULL);
    xTaskCreate(dht_test, "dht_pin3", configMINIMAL_STACK_SIZE * 3, &dht_gpio_3, 5, NULL);
    */

    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(lcd_test, "lcd_test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}