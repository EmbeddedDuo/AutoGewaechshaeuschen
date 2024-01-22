#include "hd44780Gartenhaus.h"
#include "string.h"
#include "esp_err.h"

static i2c_dev_t pcf8574;

static esp_err_t write_lcd_data(const hd44780_t *lcd, uint8_t data)
{
    return pcf8574_port_write(&pcf8574, data);
}

hd44780_t initializeHD44780(){
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

    return lcd;
}

void printTemperature(hd44780_t lcd, float temperature){
    char temp[16];

    hd44780_gotoxy(&lcd, 0, 0);
    hd44780_puts(&lcd, "Temp: ");

    hd44780_gotoxy(&lcd, 7, 0);

    snprintf(temp, 6, "%.1fC", temperature);
    temp[sizeof(temp) - 1] = 0;

    hd44780_puts(&lcd, temp);

}

void printHumidity(hd44780_t lcd, float humidity){
    char temp[16];

    hd44780_gotoxy(&lcd, 0, 1);
    hd44780_puts(&lcd, "Hum: ");

    hd44780_gotoxy(&lcd, 7, 1);

    snprintf(temp, 6, "%.1f%%", humidity);
    temp[sizeof(temp) - 1] = 0;

    hd44780_puts(&lcd, temp);

}