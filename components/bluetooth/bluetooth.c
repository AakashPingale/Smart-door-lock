#include "bluetooth.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#define SPP_SERVER_NAME "SmartLock"
#define SPP_DEVICE_NAME "SmartLock_BT"

static const char *TAG = "BT_SPP";
static bt_spp_cmd_cb_t user_cmd_cb = NULL;
static uint32_t spp_handle = 0;

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
        esp_bt_gap_set_device_name(SPP_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT: Bluetooth Connected!");
        spp_handle = param->srv_open.handle;
        bluetooth_spp_send("Connected to SmartLock! Send 'SET,ID:xxx' to enroll.\n");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT: Bluetooth Disconnected.");
        spp_handle = 0;
        break;
    case ESP_SPP_DATA_IND_EVT: {
        ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT len=%d", param->data_ind.len);
        char buf[64];
        int len = param->data_ind.len < 63 ? param->data_ind.len : 63;
        memcpy(buf, param->data_ind.data, len);
        buf[len] = '\0';
        
        // Trim newlines
        for(int i=0; i<len; i++) {
            if(buf[i]=='\r' || buf[i]=='\n') buf[i]='\0';
        }
        
        ESP_LOGI(TAG, "BT Received: %s", buf);
        
        // Parse 'SET,ID:xxx'
        if (strncmp(buf, "SET,ID:", 7) == 0) {
            int id = atoi(&buf[7]);
            if (id > 0 && id <= 1000) {
                if (user_cmd_cb) user_cmd_cb(id);
            } else {
                ESP_LOGE(TAG, "Invalid ID received: %d", id);
                bluetooth_spp_send("ERROR: Invalid ID (Must be 1-1000)\n");
            }
        } else {
            bluetooth_spp_send("ERROR: Unknown Command. Format: SET,ID:123\n");
        }
        break;
    }
    default:
        break;
    }
}

void bluetooth_spp_send(const char *msg) {
    if (spp_handle != 0) {
        esp_spp_write(spp_handle, strlen(msg), (uint8_t*)msg);
    }
}

void bluetooth_spp_init(bt_spp_cmd_cb_t cmd_callback) {
    ESP_LOGI(TAG, "Initializing Classic Bluetooth SPP...");
    user_cmd_cb = cmd_callback;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    if (esp_bluedroid_init() != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid initialize failed");
        return;
    }

    if (esp_bluedroid_enable() != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed");
        return;
    }

    esp_spp_register_callback(esp_spp_cb);
    
    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0
    };
    esp_spp_enhanced_init(&spp_cfg);
    
    ESP_LOGI(TAG, "Classic Bluetooth SPP Initialized. Device Name: %s", SPP_DEVICE_NAME);
}
