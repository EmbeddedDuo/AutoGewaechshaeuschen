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
 * @brief initialize lcd with pcf8574 and 5x8 Font
 * 
 */
hd44780_t initializeHD44780();

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