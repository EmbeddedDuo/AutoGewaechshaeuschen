/** @file wifiGartenhaus.h
 *  @brief Function prototypes for hd44780 specific for our Project.
 * */

#ifndef __WIFIGARTENHAUS_H__
#define __WIFIGARTENHAUS_H__

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "stdio.h"
#include "esp_netif.h"

/**
 * @brief function for initialising wifi
 * 
 */
void init_wifi();

/**
 * @brief checks if wifi is established
 * 
 * @return true 
 * @return false 
 */
bool checkWifiEstablished();

#endif