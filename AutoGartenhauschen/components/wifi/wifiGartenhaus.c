#include "esp_event.h"
#include "esp_log.h"

#include <wifiGartenhaus.h>


#define ESP_WIFI_SSID CONFIG_EXAMPLE_WIFI_SSID
#define ESP_WIFI_PASS CONFIG_EXAMPLE_WIFI_PASSWORD

static const char *TAG = "wifi_station";
bool wifi_established = false;

bool checkWifiEstablished(){
    return wifi_established;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        wifi_established = false;
        ESP_LOGI(TAG, "connect to the AP"); // Verbindung zum Access Point (AP) herstellen
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_established = false;
        ESP_LOGI(TAG, "retry to connect to the AP"); // Erneuter Verbindungsversuch zum AP
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip : " IPSTR "\n", IP2STR(&event->ip_info.ip)); // IP-Adresse erhalten
        wifi_established = true;                                           // WLAN-Verbindung hergestellt
    }
    else
    {
        ESP_LOGI(TAG, "unhandled event (%s) with ID %lu!", event_base, event_id); // Nicht behandeltes Ereignis
    }
}

void init_wifi()
{
    esp_netif_init();                                    // Netzwerk-Interface initialisieren
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // WLAN-Initialisierungskonfiguration erstellen
    cfg.nvs_enable = false;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));                                                                   // WLAN-Subsystem initialisieren
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));   // WLAN-Ereignis-Handler registrieren
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));  // IP-Ereignis-Handler für erfolgreiche WLAN-Verbindung registrieren
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &wifi_event_handler, NULL)); // IP-Ereignis-Handler für verlorene WLAN-Verbindung registrieren

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,     // WLAN-SSID konfigurieren
            .password = ESP_WIFI_PASS, // WLAN-Passwort konfigurieren
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));                   // WLAN-Modus auf Station (STA) setzen
    esp_netif_create_default_wifi_sta();                                 // Standard-WiFi-Station erstellen
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config)); // WLAN-Konfiguration setzen
    ESP_ERROR_CHECK(esp_wifi_start());                                   // WLAN starten
}