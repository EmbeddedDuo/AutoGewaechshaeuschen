idf_component_register(SRCS "wifiGartenhaus.c"
                        INCLUDE_DIRS .
                        REQUIRES esp_wifi esp_netif esp_event nvs_flash)