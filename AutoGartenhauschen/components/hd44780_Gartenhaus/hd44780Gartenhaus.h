/** @file hd44780Gartenhaus.h
 *  @brief Function prototypes for hd44780 specific for our Project.
 *
 *  drivers are all from third party repostitory: https://github.com/UncleRus/esp-idf-lib
 */

#ifndef __HD44780GARTENHAUS_H__
#define __HD44780GARTENHAUS_H__

#include "string.h"

#include <hd44780.h>
#include <pcf8574.h>
#include <esp_err.h>

/**
 * @brief data for LCD
 * 
 */
static const uint8_t char_data[] = {
    0x04, 0x0e, 0x0e, 0x0e, 0x1f, 0x00, 0x04, 0x00,
    0x1f, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x1f, 0x00};


/**
 * @brief initialize lcd with pcf8574 and 5x8 Font
 * 
 */
hd44780_t initializeHD44780();

/**
 * @brief callback to send data to LCD by I2C GPIO expander
 * @param
*/
static esp_err_t write_lcd_data(const hd44780_t *lcd, uint8_t data);

/**
 * @brief writes the humidity on the second first of LCD
 * @param lcd LCD descriptor
 * @param temperature temperature value
*/
void printTemperature(hd44780_t lcd, float temperature);

/**
 * @brief writes the humidity on the second line of LCD
 * @param lcd LCD descriptor
 * @param temperature humidity value
*/
void printHumidity(hd44780_t lcd, float humidity);

#endif