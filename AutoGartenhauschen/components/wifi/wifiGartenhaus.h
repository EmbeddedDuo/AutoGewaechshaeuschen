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
 * @brief wifi eventhanlder
 * 
 * @param arg
 * @param event_base 
 * @param event_id 
 * @param event_data 
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

#endif