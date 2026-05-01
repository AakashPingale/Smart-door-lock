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

// ================= GAP CALLBACK =================
static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            ESP_LOGI(TAG, "BT Auth Success");
            break;

        case ESP_BT_GAP_PIN_REQ_EVT: {
            ESP_LOGI(TAG, "PIN Requested");
            esp_bt_pin_code_t pin_code = {'1','2','3','4'};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            break;
        }

        default:
            break;
    }
}

// ================= SPP CALLBACK =================
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(TAG, "SPP INIT");
            esp_spp_start_srv(
                ESP_SPP_SEC_NONE,
                ESP_SPP_ROLE_SLAVE,
                0,
                SPP_SERVER_NAME
            );
            break;

        case ESP_SPP_SRV_OPEN_EVT:
            spp_handle = param->srv_open.handle;
            ESP_LOGI(TAG, "Client Connected (handle=%lu)", spp_handle);
            bluetooth_spp_send("Connected to SmartLock\n");
            bluetooth_spp_send("Send: SET,ID:123\n");
            break;

        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAG, "Client Disconnected");
            spp_handle = 0;
            break;

        case ESP_SPP_DATA_IND_EVT: {
            char buf[64];
            int len = param->data_ind.len > 63 ? 63 : param->data_ind.len;
            memcpy(buf, param->data_ind.data, len);
            buf[len] = '\0';

            // Remove newline
            for (int i = 0; i < len; i++) {
                if (buf[i] == '\r' || buf[i] == '\n') {
                    buf[i] = '\0';
                    break;
                }
            }

            ESP_LOGI(TAG, "Received: %s", buf);

            // ===== COMMAND PARSER =====
            if (strncmp(buf, "SET,ID:", 7) == 0) {
                int id = atoi(&buf[7]);

                if (id > 0 && id <= 1000) {
                    ESP_LOGI(TAG, "Valid ID: %d", id);
                    bluetooth_spp_send("Enrollment Started\n");

                    if (user_cmd_cb) {
                        user_cmd_cb(id);
                    }
                } else {
                    ESP_LOGE(TAG, "Invalid ID: %d", id);
                    bluetooth_spp_send("ERROR: ID must be 1-1000\n");
                }
            } else {
                bluetooth_spp_send("ERROR: Use SET,ID:123\n");
            }
            break;
        }

        default:
            break;
    }
}

void bluetooth_spp_send(const char *msg) {
    if (spp_handle != 0) {
        esp_spp_write(spp_handle, strlen(msg), (uint8_t *)msg);
    }
}

// ================= INIT FUNCTION =================
void bluetooth_spp_init(bt_spp_cmd_cb_t cmd_callback) {
    ESP_LOGI(TAG, "Initializing Classic Bluetooth SPP...");
    user_cmd_cb = cmd_callback;

    // FIXED: Do NOT release BLE memory before controller init!
    // This causes ESP_ERR_INVALID_ARG because controller isn't initialized yet
    // esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Controller init
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));

    // FIXED: Try CLASSIC_BT first, fallback to BTDM
    esp_err_t ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CLASSIC_BT failed: %s, trying BTDM", esp_err_to_name(ret));
        ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BTDM also failed: %s", esp_err_to_name(ret));
            return;
        }
    }

    // Bluedroid stack
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Set device name and scan mode AFTER bluedroid_enable
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(SPP_DEVICE_NAME));
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));

    // GAP callback
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(bt_gap_cb));

    // Set PIN (important for Android)
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code = {'1','2','3','4'};
    esp_bt_gap_set_pin(pin_type, 4, pin_code);

    // SPP callback
    ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));

    // FIXED: Use esp_spp_enhanced_init() instead of deprecated esp_spp_init()
    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,  // Use default
    };
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));

    ESP_LOGI(TAG, "Bluetooth SPP Ready. Device: %s", SPP_DEVICE_NAME);
}