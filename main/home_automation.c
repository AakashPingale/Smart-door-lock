/*
 * Smart Door Lock with Bluetooth Enrollment
 * Hardware: ESP32 + R307S Fingerprint + I2C LCD + Buzzer + Bluetooth SPP
 * 
 * Features:
 * - Fingerprint authentication via GR_Identify (0x34)
 * - Bluetooth SPP enrollment: Send "SET,ID:123" to enroll
 * - LCD status display
 * - Buzzer feedback
 * - I2C Scanner for debugging
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"

// ==================== BLUETOOTH INCLUDES ====================
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

// 👉 Set your password here
static char correct_pass[7] = "123456";
//static char admin_pass[7] = "999999";


static bool g_uart_busy = false;
// To visible password
bool showPassword = false;
// ==================== CONFIGURATION ====================

// R307S Protocol Constants
#define R307_HEADER_1       0xEF
#define R307_HEADER_2       0x01
#define R307_ADDRESS        0xFFFFFFFF

// Instruction Codes
#define CMD_GEN_IMAGE       0x01
#define CMD_IMG_2_TZ        0x02
#define CMD_SEARCH          0x04
#define CMD_REG_MODEL       0x05
#define CMD_STORE           0x06
#define CMD_DELETE          0x0C
#define CMD_READ_SYS_PARA   0x0F
#define CMD_VER_PASSWORD    0x13
#define CMD_TEMPLATE_NUM    0x1D
#define CMD_AURA_CONFIG     0x35
#define CMD_GR_IDENTIFY     0x34

// Confirmation Codes
#define CC_SUCCESS          0x00
#define CC_NO_FINGER        0x02
#define CC_FAIL_ENROLL      0x03
#define CC_IMAGE_MESSY      0x04
#define CC_FEATURE_FAIL     0x07
#define CC_NO_MATCH         0x09
#define CC_DUPLICATE        0x45

// Hardware Pins
#define FP_UART             UART_NUM_2
// #define FP_RX_PIN           27
// #define FP_TX_PIN           14
#define FP_RX_PIN           16
#define FP_TX_PIN           17
#define FP_TOUCH_PIN        4

#define RELAY_PIN           19 

#define BUZZER_PIN          18

#define I2C_PORT            I2C_NUM_0
#define I2C_SDA             21
#define I2C_SCL             22
#define LCD_ADDR            0x27    // Try 0x3F if needed

// Bluetooth Config
#define SPP_SERVER_NAME     "SmartLock"
#define SPP_DEVICE_NAME     "SmartLock_BT"
#define BT_PIN_CODE         "1234"

// System Parameters
#define FINGERPRINT_TIMEOUT_MS  10000
#define ENROLL_TIMEOUT_MS       30000

// Event Group Bits
#define EVT_TOUCH_DETECTED      BIT0
#define EVT_FP_MATCHED          BIT1
#define EVT_FP_FAILED           BIT2
#define EVT_ENROLL_REQUEST      BIT3
#define EVT_ENROLL_DONE         BIT4
#define EVT_DELETE_REQUEST      BIT5
#define EVT_COUNT_REQUEST       BIT6

// Logging tags
static const char *TAG_SYS = "SYS";
static const char *TAG_FP = "FP";
static const char *TAG_LCD = "LCD";
static const char *TAG_BT = "BT";

// ==================== SYSTEM MODE ====================
typedef enum {
    MODE_IDLE,
    MODE_ADMIN_AUTH,
    MODE_ADMIN_MENU
} system_mode_t;

//static system_mode_t system_mode = MODE_IDLE;

// Long press detection
//static int64_t key_press_time = 0;
//static char last_key = 0;

// ==================== NVS ====================
#define NVS_NAMESPACE "storage"
#define NVS_KEY_PASS  "user_pass"

static void nvs_save_password(const char *pass) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_str(handle, NVS_KEY_PASS, pass);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI("NVS", "Password Saved");
    } else {
        ESP_LOGE("NVS", "Failed to open NVS");
    }
}

static void nvs_load_password(void) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        size_t len = sizeof(correct_pass);
        esp_err_t err = nvs_get_str(handle, NVS_KEY_PASS, correct_pass, &len);
        if (err == ESP_OK) {
            ESP_LOGI("NVS", "Password Loaded: %s", correct_pass);
        } else {
            ESP_LOGW("NVS", "No saved password, using default");
            strcpy(correct_pass, "123456");
        }
        nvs_close(handle);
    } else {
        ESP_LOGE("NVS", "Failed to open NVS");
    }
}

// ==================== GLOBAL VARIABLES ====================
static EventGroupHandle_t evt_group;
static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t state_mutex;

static volatile uint16_t g_enroll_id = 0;
static volatile bool g_enroll_mode = false;
static volatile bool g_fp_busy = false; 

//static int menu_index = 0;
static uint32_t spp_handle = 0;

// ==================== KEYPAD ====================
#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4

static const gpio_num_t row_pins[4] = {32, 33, 25, 26};
static const gpio_num_t col_pins[4] = {27, 14, 12, 13};

static const char keymap[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

static char entered_pass[7] = {0};
static int pass_index = 0;
static bool show_password = false;

// ==================== LCD DRIVER ====================
#define LCD_RS          0x01    // P0
#define LCD_EN          0x04    // P2
#define LCD_BL          0x08    // P3
#define LCD_DATA_MASK   0xF0    // P7-P4

void lcd_cmd(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_clear(void);
void lcd_cursor(uint8_t col, uint8_t row);
void lcd_print(const char *str);

static void lcd_raw_write(uint8_t data) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    xSemaphoreGive(i2c_mutex);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_LCD, "I2C write failed: %s", esp_err_to_name(ret));
    }
}

static void lcd_init_pulse(uint8_t nibble) {
    uint8_t data = nibble & 0xF0;
    lcd_raw_write(data);
    esp_rom_delay_us(10);
    lcd_raw_write(data | LCD_EN);
    esp_rom_delay_us(10);
    lcd_raw_write(data);
    esp_rom_delay_us(10);
}

void lcd_init(void) {
    ESP_LOGI(TAG_LCD, "LCD Init");
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_init_pulse(0x30); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_init_pulse(0x30); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_init_pulse(0x30); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_init_pulse(0x20); vTaskDelay(pdMS_TO_TICKS(1));
    
    lcd_cmd(0x28); lcd_cmd(0x08); lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_cmd(0x06); lcd_cmd(0x0C);
    ESP_LOGI(TAG_LCD, "LCD Ready");
}

static void lcd_write_byte(uint8_t byte, uint8_t rs) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);

    uint8_t data = (byte & 0xF0) | (rs ? LCD_RS : 0) | LCD_BL;
    i2c_master_write_byte(cmd, data, true);
    i2c_master_write_byte(cmd, data | LCD_EN, true);
    i2c_master_write_byte(cmd, data & ~LCD_EN, true);

    data = ((byte << 4) & 0xF0) | (rs ? LCD_RS : 0) | LCD_BL;
    i2c_master_write_byte(cmd, data, true);
    i2c_master_write_byte(cmd, data | LCD_EN, true);
    i2c_master_write_byte(cmd, data & ~LCD_EN, true);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    xSemaphoreGive(i2c_mutex);

    if (ret != ESP_OK) ESP_LOGE(TAG_LCD, "LCD write failed: %s", esp_err_to_name(ret));
    if (byte == 0x01 || byte == 0x02) esp_rom_delay_us(2000);
    else esp_rom_delay_us(50);
}

void lcd_cmd(uint8_t cmd)  { lcd_write_byte(cmd, 0); }
void lcd_data(uint8_t data){ lcd_write_byte(data, 1); }
void lcd_clear(void)       { lcd_cmd(0x01); }
void lcd_cursor(uint8_t col, uint8_t row) {
    uint8_t addr = row ? 0x40 : 0x00;
    lcd_cmd(0x80 | (col + addr));
}
void lcd_print(const char *str) {
    while (*str) lcd_data(*str++);
}

static void lcd_show_idle(void) {
    lcd_clear();
    lcd_cursor(0, 0);
    lcd_print("Enter Password");
    lcd_cursor(0, 1);
    lcd_print("/Scan Finger");
    ESP_LOGI("LCD", "Idle Screen");
}

// ==================== HARDWARE ====================
static void buzzer_init(void) {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);
    ledc_channel_config_t ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BUZZER_PIN,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ch);
}

static void buzzer_beep(uint16_t ms, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(ms));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        if (i < count - 1) vTaskDelay(pdMS_TO_TICKS(ms / 2));
    }
}

static void relay_init(void) {
    gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_PIN, 0);
}

#define RELAY_ON  1
#define RELAY_OFF 0

static void door_unlock(void) {
    gpio_set_level(RELAY_PIN, RELAY_ON);
    ESP_LOGI("LOCK", "Door UNLOCKED");
}

static void door_lock(void) {
    gpio_set_level(RELAY_PIN, RELAY_OFF);
    ESP_LOGI("LOCK", "Door LOCKED");
}

// ==================== R307S FINGERPRINT DRIVER ====================
static uint16_t calc_checksum(uint8_t *packet, uint16_t len) {
    uint16_t sum = 0;
    for (uint16_t i = 6; i < len; i++) sum += packet[i];
    return sum;
}

static bool fp_send_cmd(uint8_t cmd, uint8_t *data, uint16_t data_len) {
    uart_flush_input(FP_UART); 
    uint8_t packet[256];
    uint16_t idx = 0;
    packet[idx++] = R307_HEADER_1; packet[idx++] = R307_HEADER_2;
    packet[idx++] = 0xFF; packet[idx++] = 0xFF; packet[idx++] = 0xFF; packet[idx++] = 0xFF;
    packet[idx++] = 0x01;
    uint16_t len = 1 + data_len + 2;
    packet[idx++] = (len >> 8) & 0xFF; packet[idx++] = len & 0xFF;
    packet[idx++] = cmd;
    for (uint16_t i = 0; i < data_len; i++) packet[idx++] = data[i];
    uint16_t chk = calc_checksum(packet, idx);
    packet[idx++] = (chk >> 8) & 0xFF; packet[idx++] = chk & 0xFF;
    


    int written = uart_write_bytes(FP_UART, (char*)packet, idx);
    return (written == idx);
}
static uint8_t fp_read_resp(uint8_t *data_buf, uint16_t max_data_len, uint16_t *out_data_len)
{
    uint8_t header[9];
    uint8_t byte;
    int total = 0;
    uint32_t start_time = xTaskGetTickCount();

    // 1. Seek for Header: 0xEF 0x01
    while (total < 2) {
        int r = uart_read_bytes(FP_UART, &byte, 1, pdMS_TO_TICKS(1000));
        if (r <= 0) {
            if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(3000)) {
                ESP_LOGE(TAG_FP, "Header timeout (seeking)");
                return 0xFF;
            }
            continue;
        }

        if (total == 0) {
            if (byte == R307_HEADER_1) {
                header[total++] = byte;
            }
        } else if (total == 1) {
            if (byte == R307_HEADER_2) {
                header[total++] = byte;
            } else if (byte == R307_HEADER_1) {
                // Stay at total=1
            } else {
                total = 0; 
            }
        }
    }

    // 2. Read the rest of the header
    int remaining = 7;
    while (remaining > 0) {
        int r = uart_read_bytes(FP_UART, header + (9 - remaining), remaining, pdMS_TO_TICKS(200));
        if (r <= 0) {
            ESP_LOGE(TAG_FP, "Header incomplete (%d/9)", 9 - remaining);
            return 0xFF;
        }
        remaining -= r;
    }

    uint16_t payload_len = ((uint16_t)header[7] << 8) | header[8];
    if (payload_len < 3 || payload_len > 128) {
        ESP_LOGE(TAG_FP, "Invalid payload length: %d", payload_len);
        return 0xFF;
    }

    // 3. Read payload
    uint8_t payload[128];
    int len = 0;
    while (len < payload_len) {
        int r = uart_read_bytes(FP_UART, payload + len, payload_len - len, pdMS_TO_TICKS(200));
        if (r <= 0) {
            ESP_LOGE(TAG_FP, "Payload incomplete (%d/%d)", len, payload_len);
            return 0xFF;
        }
        len += r;
    }

    uint8_t cc = payload[0];
    ESP_LOGI(TAG_FP, "Response CC = 0x%02X", cc);

    uint16_t data_len = payload_len - 3;
    if (data_buf && data_len > 0) {
        uint16_t to_copy = (data_len > max_data_len) ? max_data_len : data_len;
        memcpy(data_buf, &payload[1], to_copy);
        if (out_data_len) *out_data_len = to_copy;
    }

    return cc;
}


static bool fp_set_led(uint8_t mode, uint8_t color, uint8_t speed) {
    // R307 Aura LED Protocol: Mode, Speed, Color, Cycles
    // Modes: 01:Breathing, 02:Flashing, 03:Always On, 04:Always Off
    // Colors: 01:Red, 02:Blue, 03:Purple
    uint8_t data[] = {mode, speed, color, 0x00}; 
    if (!fp_send_cmd(CMD_AURA_CONFIG, data, 4)) return false;
    uint8_t resp[16];
    uint8_t cc = fp_read_resp(resp, sizeof(resp), NULL);
    if (cc != CC_SUCCESS) {
        ESP_LOGW(TAG_FP, "LED Config Failed: CC=0x%02X", cc);
        return false;
    }
    return true;
}

static bool fp_init(void) {
    ESP_LOGI(TAG_FP, "FP Init (Waiting for sensor to stabilize)...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Give sensor time to boot

    uint8_t pwd_data[] = {0x00, 0x00, 0x00, 0x00};
    uint8_t resp[16];
    uint8_t cc = 0xFF;

    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG_FP, "Attempting connection... (%d/3)", i + 1);
        if (fp_send_cmd(CMD_VER_PASSWORD, pwd_data, 4)) {
            cc = fp_read_resp(resp, sizeof(resp), NULL);
            if (cc == CC_SUCCESS) break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (cc != CC_SUCCESS) {
        ESP_LOGE(TAG_FP, "FP Init Failed: Check wiring and power! (CC=0x%02X)", cc);
        return false;
    }
    
    // Set Standby LED: Mode 0x01 (Breathing), Speed 0xFF, Color 0x02 (Blue)
    fp_set_led(0x01, 0x02, 0xFF); 
    
    ESP_LOGI(TAG_FP, "FP Ready (Standby Breathing Blue)");
    return true;
}
static int8_t fp_identify(uint16_t *matched_id, uint16_t *score)
{
    uint8_t resp[16];
    uint16_t len;

    // Step 1: Capture image
    if (!fp_send_cmd(CMD_GEN_IMAGE, NULL, 0)) return -1;
    uint8_t cc = fp_read_resp(resp, sizeof(resp), &len);

    if (cc == CC_NO_FINGER) return -2;
    if (cc != CC_SUCCESS) return -1;

    // Step 2: Convert to template
    uint8_t buf1[] = {0x01};
    if (!fp_send_cmd(CMD_IMG_2_TZ, buf1, 1)) return -1;
    if (fp_read_resp(resp, sizeof(resp), &len) != CC_SUCCESS) return -1;

    // Step 3: Search
    uint8_t search_data[] = {0x01, 0x00, 0x00, 0x03, 0xE8}; // search full DB (up to 1000)
    if (!fp_send_cmd(CMD_SEARCH, search_data, 5)) return -1;

    cc = fp_read_resp(resp, sizeof(resp), &len);

    if (cc == CC_SUCCESS && len >= 4) {
        *matched_id = (resp[0] << 8) | resp[1];
        *score      = (resp[2] << 8) | resp[3];
        return 0;
    }

    if (cc == CC_NO_MATCH) return -3;

    return -1;
}
static bool fp_delete(uint16_t id) {
    uint8_t resp[16]; uint16_t data_len;
    uint8_t del_data[] = {(id >> 8) & 0xFF, id & 0xFF, 0x00, 0x01};
    if (!fp_send_cmd(CMD_DELETE, del_data, 4)) return false;
    uint8_t cc = fp_read_resp(resp, sizeof(resp), &data_len);
    if (cc == CC_SUCCESS) {
        lcd_clear(); 
        lcd_cursor(0,0);
         lcd_print("Deleted ok");
          ESP_LOGI(TAG_FP, "Deleted ok");
          buzzer_beep(100, 2);
        vTaskDelay(pdMS_TO_TICKS(2000)); 
        lcd_show_idle();
        ESP_LOGI(TAG_FP, "IDLE");
        return true;
    }
    lcd_show_idle(); 
    return false;
}

static int fp_get_count(void) {
    uint8_t resp[16]; uint16_t data_len;
    if (!fp_send_cmd(CMD_TEMPLATE_NUM, NULL, 0)) return -1;
    uint8_t cc = fp_read_resp(resp, sizeof(resp), &data_len);
    if (cc == CC_SUCCESS && data_len >= 2) return (resp[0] << 8) | resp[1];
    return -1;
}

static bool fp_enroll_step1(void) {
    uint8_t resp[16]; uint16_t data_len;
    
    // 1. Wait for finger press
    ESP_LOGI(TAG_FP, "Place finger...");
    lcd_clear(); lcd_print("Place finger");
    fp_set_led(0x03, 0x02, 0x00); // Always ON Blue
    uint32_t start = xTaskGetTickCount();
    bool pressed = false;
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(10000)) {
        if (fp_send_cmd(CMD_GEN_IMAGE, NULL, 0)) {
            if (fp_read_resp(resp, sizeof(resp), &data_len) == CC_SUCCESS) {
                pressed = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (!pressed) {
        ESP_LOGE(TAG_FP, "Finger press timeout");
        return false;
    }
    
    // 2. Extract features to Buffer 1
    uint8_t buf1[] = {0x01};
    if (!fp_send_cmd(CMD_IMG_2_TZ, buf1, 1)) return false;
    if (fp_read_resp(resp, sizeof(resp), &data_len) != CC_SUCCESS) return false;
    
    buzzer_beep(100, 1);
    
    // 3. Wait for finger removal
    ESP_LOGI(TAG_FP, "Remove finger...");
    lcd_clear(); lcd_cursor(0,0); lcd_print("Remove finger   ");
    fp_set_led(0x02, 0x01, 0xFF); // Flashing Red
    while (1) {
        if (fp_send_cmd(CMD_GEN_IMAGE, NULL, 0)) {
            if (fp_read_resp(resp, sizeof(resp), &data_len) == CC_NO_FINGER) break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    return true;
}

static bool fp_enroll_step2(uint16_t id) {
    uint8_t resp[16]; uint16_t data_len; uint8_t cc;
    uint32_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(ENROLL_TIMEOUT_MS)) {
        if (!fp_send_cmd(CMD_GEN_IMAGE, NULL, 0)) return false;
        cc = fp_read_resp(resp, sizeof(resp), &data_len);
        if (cc == CC_SUCCESS) break;
        if (cc == CC_NO_FINGER) { 
            vTaskDelay(pdMS_TO_TICKS(500)); 
            continue; 
        }
        return false;
    }
    uint8_t buf_data[] = {0x02};
    if (!fp_send_cmd(CMD_IMG_2_TZ, buf_data, 1)) return false;

    if (fp_read_resp(resp, sizeof(resp), &data_len) != CC_SUCCESS) return false;
    if (!fp_send_cmd(CMD_REG_MODEL, NULL, 0)) return false;
    
    if (fp_read_resp(resp, sizeof(resp), &data_len) != CC_SUCCESS) return false;
    uint8_t store_data[] = {0x01, (id >> 8) & 0xFF, id & 0xFF};
    if (!fp_send_cmd(CMD_STORE, store_data, 3)) return false;
    return (fp_read_resp(resp, sizeof(resp), &data_len) == CC_SUCCESS);
}

// ==================== BLUETOOTH SPP ====================
static void bluetooth_spp_send(const char *msg) {
    if (spp_handle != 0) esp_spp_write(spp_handle, strlen(msg), (uint8_t *)msg);
}

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    if (event == ESP_BT_GAP_PIN_REQ_EVT) {
        esp_bt_pin_code_t pin_code = {'1','2','3','4'};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT: esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME); break;
        case ESP_SPP_SRV_OPEN_EVT: spp_handle = param->srv_open.handle; bluetooth_spp_send("SmartLock Ready\n"); break;
        case ESP_SPP_CLOSE_EVT: spp_handle = 0; break;
        case ESP_SPP_DATA_IND_EVT: {
            char buf[64]; int len = param->data_ind.len > 63 ? 63 : param->data_ind.len;
            memcpy(buf, param->data_ind.data, len); buf[len] = '\0';
            
            // Trim newlines and carriage returns for better matching
            for (int i = 0; i < len; i++) {
                if (buf[i] == '\r' || buf[i] == '\n') {
                    buf[i] = '\0';
                    break;
                }
            }
            
            ESP_LOGI(TAG_BT, "Received BT Data: [%s]", buf);

            if (strncmp(buf, "SET,ID:", 7) == 0) {
                int id = atoi(&buf[7]);
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                g_enroll_id = (uint16_t)id; g_enroll_mode = true;
                xSemaphoreGive(state_mutex);
                xEventGroupSetBits(evt_group, EVT_ENROLL_REQUEST);
                bluetooth_spp_send("Enrolling Finger...\n");
            } 
            else if (strncmp(buf, "DEL,ID:", 7) == 0) {
                int id = atoi(&buf[7]);
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                g_enroll_id = (uint16_t)id;
                xSemaphoreGive(state_mutex);
                xEventGroupSetBits(evt_group, EVT_DELETE_REQUEST);
            }
            else if (strstr(buf, "GET,COUNT") != NULL) {
                xEventGroupSetBits(evt_group, EVT_COUNT_REQUEST);
            }
            else if (strncmp(buf, "SET,PASS:", 9) == 0) {
                if (strlen(&buf[9]) >= 6) {
                    strncpy(correct_pass, &buf[9], 6);
                    correct_pass[6] = '\0';
                    nvs_save_password(correct_pass);
                    bluetooth_spp_send("Password Updated\n");
                } else {
                    bluetooth_spp_send("Pass must be 6 digits\n");
                }
            }
            else if (strstr(buf, "GET,STATUS") != NULL) {
                bluetooth_spp_send("Status: System Online\n");
            }
            break;
        }
        default: break;
    }
}

static void bluetooth_init(void) {
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    esp_bluedroid_init(); esp_bluedroid_enable();
    esp_bt_gap_set_device_name(SPP_DEVICE_NAME);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    esp_bt_gap_register_callback(bt_gap_cb);
    esp_spp_register_callback(esp_spp_cb);
    esp_spp_cfg_t spp_cfg = {.mode = ESP_SPP_MODE_CB, .enable_l2cap_ertm = true};
    esp_spp_enhanced_init(&spp_cfg);
}

// ==================== KEYPAD ====================
static char keypad_get_key(void) {
    for (int r = 0; r < 4; r++) {
        for (int i = 0; i < 4; i++) gpio_set_level(row_pins[i], 1);
        gpio_set_level(row_pins[r], 0);
        for (int c = 0; c < 4; c++) {
            if (gpio_get_level(col_pins[c]) == 0) {
                vTaskDelay(pdMS_TO_TICKS(50));
                if (gpio_get_level(col_pins[c]) == 0) return keymap[r][c];
            }
        }
    }
    return 0;
}

static void update_password_display(void) {
    lcd_cursor(0,1);
    char buf[17] = {0};
    for (int i = 0; i < pass_index; i++) buf[i] = show_password ? entered_pass[i] : '*';
    lcd_print("                "); 
    lcd_cursor(0,1); 
    lcd_print(buf);
}

static void task_keypad(void *pv) {
    while (1) {
        char key = keypad_get_key();
        if (key) {
            buzzer_beep(50, 1);
            if (key >= '0' && key <= '9' && pass_index < 6) {
                entered_pass[pass_index++] = key;
                entered_pass[pass_index] = '\0';
            } else if (key == '#' && pass_index > 0) {
                pass_index--;
                entered_pass[pass_index] = '\0';
            } else if (key == '*' && pass_index == 6) {
                if (strcmp(entered_pass, correct_pass) == 0) {
                    lcd_clear();
                    lcd_cursor(0, 0);
                    lcd_print("Welcome");
                    lcd_cursor(0, 1);
                    lcd_print("Home :) ");
                    door_unlock();
                    ESP_LOGI(TAG_FP, "DOOR OPENED BY PASSWORD");
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    door_lock();
                    ESP_LOGI(TAG_FP, "DOOR CLOSED");
                } else {
                    lcd_clear();
                    lcd_print("Access Denied");
                    buzzer_beep(300, 1);
                    ESP_LOGI(TAG_FP, "ACCESS DENIED");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                }
                pass_index = 0;
                memset(entered_pass, 0, sizeof(entered_pass));
                lcd_show_idle();
            } else if (key == 'A') {
                show_password = true;
                update_password_display();
            } else if (key == 'B') {
                show_password = false;
                update_password_display();
            }
            if (pass_index > 0) update_password_display();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==================== MAIN TASKS ====================
static void task_fingerprint(void *pv) {
    uint16_t matched_id = 0, score = 0;
    while (1) {
        // Wait for any system request (Touch, Enroll, Delete, or Count)
        EventBits_t evt = xEventGroupWaitBits(evt_group, 
            EVT_TOUCH_DETECTED | EVT_ENROLL_REQUEST | EVT_DELETE_REQUEST | EVT_COUNT_REQUEST, 
            pdTRUE, pdFALSE, portMAX_DELAY);
        
        if (evt & EVT_ENROLL_REQUEST) {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            g_uart_busy = true;
            uint16_t enroll_id = g_enroll_id;
            xSemaphoreGive(state_mutex);

            lcd_clear(); 
            lcd_print("Enroll Mode");
            ESP_LOGI(TAG_FP, "ENROLLING FINGER ID : %d", enroll_id);
            
            if (fp_enroll_step1() && fp_enroll_step2(enroll_id)) {
                lcd_clear(); 
                lcd_print("Enroll OK!");
                ESP_LOGI(TAG_FP, "FINGERPRINT ADDED SUCCESSFULLY");
                buzzer_beep(100, 2);
                vTaskDelay(pdMS_TO_TICKS(2000));
                lcd_show_idle();
            } else {
                lcd_clear(); 
                lcd_print("Enroll FAIL");
                ESP_LOGI(TAG_FP, "FINGERPRINT ADDED UNSUCCESSFULLY");
                buzzer_beep(300, 1);
                vTaskDelay(pdMS_TO_TICKS(2000));
                lcd_show_idle();
            }
            xSemaphoreTake(state_mutex, portMAX_DELAY); g_enroll_mode = false; xSemaphoreGive(state_mutex);
            g_uart_busy = false;
            g_enroll_mode = false;
        } 
        else if (evt & EVT_DELETE_REQUEST) {
            g_uart_busy = true;
            xSemaphoreTake(state_mutex, portMAX_DELAY); uint16_t del_id = g_enroll_id; xSemaphoreGive(state_mutex);
            if (fp_delete(del_id)) bluetooth_spp_send("Delete OK\n");
            else bluetooth_spp_send("Delete Fail\n");
            g_uart_busy = false;
        }
        else if (evt & EVT_COUNT_REQUEST) {
            g_uart_busy = true;
            int count = fp_get_count();
            char res[32]; sprintf(res, "Count: %d\n", count);
            bluetooth_spp_send(res);
            g_uart_busy = false;
        }
        // Identification Mode
        else if (evt & EVT_TOUCH_DETECTED) {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            bool busy = g_enroll_mode || g_uart_busy;
            xSemaphoreGive(state_mutex);

            if (!busy) {
                // Debounce: verify finger is still there (Active Low check)
                vTaskDelay(pdMS_TO_TICKS(50)); 
                if (gpio_get_level(FP_TOUCH_PIN) == 0) {
                    g_uart_busy = true;
                    int8_t result = fp_identify(&matched_id, &score);
                    g_uart_busy = false;

                    if (result == 0) {
                        lcd_clear();
                        lcd_cursor(0,0);
                        lcd_print("Welcome");
                        char buffer[20];
                        sprintf(buffer, "ID:%d", matched_id);
                        lcd_cursor(0,1);
                        
                        lcd_print(buffer);
                        door_unlock();
                        ESP_LOGI(TAG_FP, "FINGERPRINT VERIFIED ID: %d", matched_id);
                        vTaskDelay(pdMS_TO_TICKS(5000));
                        door_lock();
                        lcd_show_idle();
                    } 
                    else if (result == -3) {
                        lcd_clear();
                        lcd_print("Access Denied");
                        buzzer_beep(300, 1);
                        ESP_LOGI(TAG_FP, "ACCESS DENIED");
                        vTaskDelay(pdMS_TO_TICKS(2000));
                        lcd_show_idle();
                    }
                    else if (result == -2) {
                        ESP_LOGI(TAG_FP, "No finger detected");
                    }
                    else {
                        ESP_LOGE(TAG_FP, "Identification Error");
                    }
                    // Wait for finger removal to avoid multiple triggers
                   // while(gpio_get_level(FP_TOUCH_PIN))
                    while(gpio_get_level(FP_TOUCH_PIN) == 0) {
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    vTaskDelay(pdMS_TO_TICKS(300)); // Extra cool down
                }
            }
        }
    }
}

static void task_touch_monitor(void *pv) {
    bool last_state = true; // Start with HIGH (idle)
    while (1) {
        bool pin_level = gpio_get_level(FP_TOUCH_PIN);
        // Trigger on Falling Edge (HIGH to LOW)
        if (pin_level == 0 && last_state == true) {
            ESP_LOGI("TOUCH", "Finger Detected (Active Low)!");
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            if (!g_enroll_mode&& !g_uart_busy) xEventGroupSetBits(evt_group, EVT_TOUCH_DETECTED);
            xSemaphoreGive(state_mutex);
            ESP_LOGI("TOUCH", "Finger Detected on Sensor!");
        }
        last_state = pin_level;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void keypad_init(void) {
    for (int i = 0; i < 4; i++) {
        gpio_set_direction(row_pins[i], GPIO_MODE_OUTPUT);
         gpio_set_level(row_pins[i], 1);
        gpio_set_direction(col_pins[i], GPIO_MODE_INPUT);
         gpio_set_pull_mode(col_pins[i], GPIO_PULLUP_ONLY);
    }
}

void app_main(void) {
    ESP_LOGI(TAG_SYS, "Smart Lock Starting...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); ret = nvs_flash_init();
    }
    nvs_load_password();
    evt_group = xEventGroupCreate();
    i2c_mutex = xSemaphoreCreateMutex();
    state_mutex = xSemaphoreCreateMutex();

    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER, .sda_io_num = I2C_SDA, .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG_SYS, "Scanning I2C bus...");
    for (int i = 1; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        if (i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(10)) == ESP_OK) {
            ESP_LOGI(TAG_SYS, "Found I2C device at 0x%02X", i);
        }
        i2c_cmd_link_delete(cmd);
    }

    //uart_config_t uart_cfg = {.baud_rate = 57600, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1};
    
        uart_config_t uart_cfg = {
    .baud_rate = 57600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
    .source_clk = UART_SCLK_DEFAULT,
    };
    //uart_driver_install(FP_UART, 512, 0, 0, NULL, 0);
    uart_driver_install(FP_UART, 4096, 0, 0, NULL, 0);
    uart_param_config(FP_UART, &uart_cfg);
    uart_set_pin(FP_UART, FP_TX_PIN, FP_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Init UART for Fingerprint

   // uart_set_pin(FP_UART, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Using PULLUP for Active Low Touch Pin
    gpio_config_t io_conf = {.pin_bit_mask = (1ULL << FP_TOUCH_PIN), .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&io_conf);
    ESP_LOGI("TOUCH", "State: %d", gpio_get_level(FP_TOUCH_PIN));

    buzzer_init();
     relay_init(); 
     lcd_init();
    lcd_clear();

     lcd_print("System Ready");
     ESP_LOGI(TAG_SYS, "SYSTEM READY");
    bluetooth_init();
     keypad_init(); 
     fp_init();
    vTaskDelay(pdMS_TO_TICKS(1000)); lcd_show_idle();

    xTaskCreatePinnedToCore(task_fingerprint, "fp_task", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_keypad, "keypad", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(task_touch_monitor, "touch", 2048, NULL, 3, NULL, 1);
    ESP_LOGI(TAG_SYS, "System Ready");
}