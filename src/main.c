/*
    This is a BLE Central Node observing advertising packets.
*/

// Based from the IDF GATT Client example - https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/gatt_client

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include "controller.h"

#include "bt.h"
#include "bt_trace.h"
#include "bt_types.h"
#include "btm_api.h"
#include "bta_api.h"
#include "bta_gatt_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "user_config.h"

#define INFO(...) printf(__VA_ARGS__)

#include "mqtt.h"

const uint32_t scan_duration = 3000; // seconds ?

///Declare static functions
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

static esp_ble_scan_params_t ble_scan_params = {
  .scan_type              = BLE_SCAN_TYPE_PASSIVE,
  .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
  .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
  .scan_interval          = 0x10,
  .scan_window            = 0x08,
};

static mqtt_client *client;

typedef uint8_t mac_address_t[6];

static mac_address_t addresses[] = {
  DEVICE_ADDR_1,
  DEVICE_ADDR_1,
};

static void publish_mac(char *mac, int rssi) {
    char payload[256];
    int payload_len = sprintf(payload, "{\"topic\":\"beacon\",\"mac\":\"%s\",\"rssi\":%d}", mac, rssi);
    printf("Publishing payload: %s\n", payload);
    mqtt_publish(client, "gohome/beacon", payload, payload_len, 0, 0);
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        printf("scan param set complete, scanning...\n");
        esp_ble_gap_start_scanning(scan_duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            // forward beacon to MQTT
            uint8_t *addr = scan_result->scan_rst.bda;
            // check addresses
            bool matched = false;
            for (int i = 0; i < sizeof(addresses)/sizeof(mac_address_t); ++i) {
                uint8_t *a = (uint8_t *)addresses[i];
                if (addr[0] == a[0] && addr[1] == a[1] && addr[2] == a[2] && addr[3] == a[3] && addr[4] == a[4] && addr[5] == a[5]) {
                    matched = true;
                    break;
                }
            }

            if (!matched)
                return;

            char mac[18];
            sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
            int rssi = scan_result->scan_rst.rssi;            
            printf("Device %s, RSSI=%i\n", mac, rssi);
            publish_mac(mac, rssi);
            break;
        }
        case ESP_GAP_SEARCH_INQ_CMPL_EVT: {
            // scan params must be reset before starting another scan
            printf("scan completed, restarting...\n");
            esp_ble_gap_set_scan_params(&ble_scan_params);
            break;
        }
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

void ble_client_appRegister(void)
{
    esp_err_t status;

    printf("register callback\n");

    // register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        printf("ERROR: gap register error, error code = %x\n", status);
        return;
    }

    esp_ble_gap_set_scan_params(&ble_scan_params);
}

void mqtt_connected_cb(void *self, void *params)
{
}

void mqtt_disconnected_cb(void *self, void *params)
{

}

void mqtt_reconnect_cb(void *self, void *params)
{

}

void mqtt_subscribe_cb(void *self, void *params)
{
    INFO("[APP] Subscribe ok, test publish msg\n");
    mqtt_publish(client, "/test", "abcde", 5, 0, 0);
}

void mqtt_publish_cb(void *self, void *params)
{

}

mqtt_settings settings = {
    .host = "mqtt",
    .port = 1883,
    .client_id = "mqtt_client_id",
    .clean_session = 0,
    .keepalive = 120,
    .connected_cb = mqtt_connected_cb,
    .disconnected_cb = mqtt_disconnected_cb,
    .reconnect_cb = mqtt_reconnect_cb,
    .publish_cb = mqtt_publish_cb,
};

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;

    case SYSTEM_EVENT_STA_GOT_IP:

        client = mqtt_start(&settings);
        ble_client_appRegister();
        // Notice that, all callback will called in mqtt_task
        // All function publish, subscribe
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        
        mqtt_stop();
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    default:
        break;
    }
    return ESP_OK;


}

void wifi_conn_init(void)
{
    INFO("[APP] Start, connect to Wifi network: %s ..\n", WIFI_SSID);

    tcpip_adapter_init();

    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );

    wifi_init_config_t icfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&icfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        },
    };

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK( esp_wifi_start());
}

void app_main()
{
    INFO("[APP] Startup..\n");
    INFO("[APP] Free memory: %d bytes\n", esp_get_free_heap_size());

    nvs_flash_init();
    wifi_conn_init();

    esp_bt_controller_init();

    if (esp_bt_controller_enable(ESP_BT_MODE_BTDM) != ESP_OK) {
      return;
    }
    esp_bluedroid_init();
    esp_bluedroid_enable();
}
