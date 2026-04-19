/*
 * Smart Door Lock with Bluetooth Enrollment
 * Hardware: ESP32 + R307S Fingerprint + I2C LCD + Buzzer + Bluetooth SPP
 * 
 * Features:
 * - Fingerprint authentication via GR_Identify (0x34)
 * - Bluetooth SPP enrollment: Send "SET,ID:123" to enroll
 * - LCD status display
 * - Buzzer feedback
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
static char admin_pass[7] = "999999";

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
#define FP_RX_PIN           16
#define FP_TX_PIN           17
#define FP_TOUCH_PIN        4

#define RELAY_PIN  19 

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

// Logging tags
static const char *TAG_SYS = "SYS";
static const char *TAG_FP = "FP";
static const char *TAG_LCD = "LCD";
static const char *TAG_BT = "BT";
//static const char *TAG_BUZZ = "BUZZ";
// ==================== SYSTEM MODE ====================
typedef enum {
    MODE_IDLE,
    MODE_ADMIN_AUTH,
    MODE_ADMIN_MENU
} system_mode_t;

static system_mode_t system_mode = MODE_IDLE;

// Long press detection
static int64_t key_press_time = 0;
static char last_key = 0;
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

static int menu_index = 0;

static uint32_t spp_handle = 0;
// ==================== KEYPAD ====================

#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4

static const gpio_num_t row_pins[4] = {32, 33, 25, 26};
static const gpio_num_t col_pins[4] = {27, 14, 12, 13};

// Key map
static const char keymap[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

// Password system
static char entered_pass[7] = {0};
static int pass_index = 0;
static bool show_password = false;



// ==================== LCD DRIVER (PROVEN WORKING) ====================

#define LCD_RS          0x01    // P0
#define LCD_EN          0x04    // P2
#define LCD_BL          0x08    // P3
#define LCD_DATA_MASK   0xF0    // P7-P4

// Forward declarations (add these!)
void lcd_cmd(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_clear(void);
void lcd_home(void);
void lcd_cursor(uint8_t col, uint8_t row);
void lcd_print(const char *str);

// Raw I2C write WITHOUT backlight - critical for reset sequence
static void lcd_raw_write(uint8_t data) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);  // NO backlight during init!
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    xSemaphoreGive(i2c_mutex);
}

// Manual EN pulse for init (no backlight)
static void lcd_init_pulse(uint8_t nibble) {
    uint8_t data = nibble & 0xF0;  // Data on P7-P4, EN=0, RS=0, BL=0
    lcd_raw_write(data);           // Setup
    esp_rom_delay_us(10);
    lcd_raw_write(data | LCD_EN);  // EN high
    esp_rom_delay_us(10);
    lcd_raw_write(data);           // EN low
    esp_rom_delay_us(10);
}

void lcd_init(void) {
    ESP_LOGI(TAG_LCD, "LCD Init");
    
    // Wait >40ms after power-on
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Step 1: First 0x30, wait >4.1ms
    lcd_init_pulse(0x30);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Step 2: Second 0x30, wait >100us
    lcd_init_pulse(0x30);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Step 3: Third 0x30, wait >37us
    lcd_init_pulse(0x30);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Step 4: Switch to 4-bit mode
    lcd_init_pulse(0x20);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Now use normal writes (with backlight)
    lcd_cmd(0x28);  // 4-bit, 2 lines, 5x8
    lcd_cmd(0x08);  // Display OFF
    lcd_cmd(0x01);  // Clear
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_cmd(0x06);  // Entry mode
    lcd_cmd(0x0C);  // Display ON
    
    ESP_LOGI(TAG_LCD, "LCD Ready");
}

// Standard I2C write that includes the backlight
static void lcd_write_byte(uint8_t byte, uint8_t rs)
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);

    // High nibble
    uint8_t data = (byte & 0xF0) | (rs ? LCD_RS : 0) | LCD_BL;
    i2c_master_write_byte(cmd, data, true);
    i2c_master_write_byte(cmd, data | LCD_EN, true);
    i2c_master_write_byte(cmd, data & ~LCD_EN, true);

    // Low nibble
    data = ((byte << 4) & 0xF0) | (rs ? LCD_RS : 0) | LCD_BL;
    i2c_master_write_byte(cmd, data, true);
    i2c_master_write_byte(cmd, data | LCD_EN, true);
    i2c_master_write_byte(cmd, data & ~LCD_EN, true);

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(i2c_mutex);
    if (byte == 0x01 || byte == 0x02) {
        esp_rom_delay_us(2000);   // clear/home needs ~1.52ms
    } else {
        esp_rom_delay_us(50);     // normal command timing
    }
}

void lcd_cmd(uint8_t cmd)  { lcd_write_byte(cmd, 0); }
void lcd_data(uint8_t data){ lcd_write_byte(data, 1); }

void lcd_clear(void) { lcd_cmd(0x01); }
void lcd_home(void)  { lcd_cmd(0x02); }

void lcd_cursor(uint8_t col, uint8_t row) {
    uint8_t addr = row ? 0x40 : 0x00;
    lcd_cmd(0x80 | (col + addr));
}

void lcd_print(const char *str) {
    while (*str) lcd_data(*str++);
}
// ================= IDLE SCREEN =================
static void lcd_show_idle(void) {

    lcd_clear();
    lcd_cursor(0, 0);
    lcd_print("Enter password");
    lcd_cursor(0, 1);
    lcd_print("/scan Finger");

    ESP_LOGI("LCD", "Idle Screen");
}
// ==================== BUZZER DRIVER ====================

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
    gpio_set_level(RELAY_PIN, 0);  // LOCKED state
}
// Adjust logic depending on your relay (LOW or HIGH trigger)
#define RELAY_ON  1   // try 0 if reversed
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
    uint8_t packet[64];
    uint16_t idx = 0;
    
    packet[idx++] = R307_HEADER_1;
    packet[idx++] = R307_HEADER_2;
    packet[idx++] = 0xFF; packet[idx++] = 0xFF;
    packet[idx++] = 0xFF; packet[idx++] = 0xFF;
    packet[idx++] = 0x01;
    
    uint16_t len = 1 + data_len + 2;
    packet[idx++] = (len >> 8) & 0xFF;
    packet[idx++] = len & 0xFF;
    packet[idx++] = cmd;
    
    for (uint16_t i = 0; i < data_len; i++) packet[idx++] = data[i];
    
    uint16_t chk = calc_checksum(packet, idx);
    packet[idx++] = (chk >> 8) & 0xFF;
    packet[idx++] = chk & 0xFF;
    
  // uart_flush(FP_UART);
    // UART Stability: Clear RX buffer before sending new command
    uart_flush_input(FP_UART);

    int written = uart_write_bytes(FP_UART, (char*)packet, idx);
    return (written == idx);
}

static uint8_t fp_read_resp(uint8_t *data_buf, uint16_t max_data_len, uint16_t *out_data_len) {
    uint8_t header[9];
    int len;
    
    len = uart_read_bytes(FP_UART, header, 9, pdMS_TO_TICKS(1000));
   if (len < 9) {
        ESP_LOGE(TAG_FP, "Header timeout: %d/9", len);

        uart_flush_input(FP_UART);
        uart_flush(FP_UART);
        // clear garbage
        vTaskDelay(pdMS_TO_TICKS(100));

        return 0xFF;
    }
    
    if (header[0] != R307_HEADER_1 || header[1] != R307_HEADER_2) {
        ESP_LOGE(TAG_FP, "Invalid header: %02X %02X", header[0], header[1]);
        return 0xFF;
    }
    
    uint16_t payload_len = ((uint16_t)header[7] << 8) | header[8];
    uint8_t payload[64];
    
    if (payload_len > sizeof(payload)) {
        ESP_LOGE(TAG_FP, "Payload too large: %d", payload_len);
        return 0xFF;
    }
    
    len = uart_read_bytes(FP_UART, payload, payload_len, pdMS_TO_TICKS(1000));
    if (len < payload_len) {
        ESP_LOGE(TAG_FP, "Payload timeout: %d/%d", len, payload_len);
        return 0xFF;
    }
    
    uint8_t cc = payload[0];
    uint16_t data_len = payload_len - 3;
    
    if (data_buf && data_len > 0 && data_len <= max_data_len) {
        memcpy(data_buf, &payload[1], data_len);
        if (out_data_len) *out_data_len = data_len;
    }
    
    return cc;
}

static bool fp_init(void) {
    ESP_LOGI(TAG_FP, "FP Init");
    
    uint8_t pwd_data[] = {0x00, 0x00, 0x00, 0x00};
    if (!fp_send_cmd(CMD_VER_PASSWORD, pwd_data, 4)) return false;
    
    uint8_t resp[16];
    uint16_t data_len;
    uint8_t cc = fp_read_resp(resp, sizeof(resp), &data_len);
    
    if (cc != CC_SUCCESS) {
        ESP_LOGE(TAG_FP, "Password fail: 0x%02X", cc);
        return false;
    }
    
    ESP_LOGI(TAG_FP, "FP Ready");
    return true;
}
static int8_t fp_identify(uint16_t *matched_id, uint16_t *score) {

    uint8_t resp[16];
    uint16_t data_len;

    // 1. Capture image
    if (!fp_send_cmd(CMD_GEN_IMAGE, NULL, 0)) return -1;
    uint8_t cc = fp_read_resp(resp, sizeof(resp), &data_len);

    if (cc == CC_NO_FINGER) return -2;
    if (cc != CC_SUCCESS) return -1;

    // 2. Convert to template (buffer 1)
    uint8_t buf1[] = {0x01};
    if (!fp_send_cmd(CMD_IMG_2_TZ, buf1, 1)) return -1;
    cc = fp_read_resp(resp, sizeof(resp), &data_len);

    if (cc != CC_SUCCESS) return -1;

    // 3. Search in database
    uint8_t search[] = {0x01, 0x00, 0x00, 0x03, 0xE8}; // buffer1, start=0, count=1000
    if (!fp_send_cmd(CMD_SEARCH, search, 5)) return -1;
    cc = fp_read_resp(resp, sizeof(resp), &data_len);

    if (cc == CC_SUCCESS && data_len >= 4) {
        *matched_id = (resp[0] << 8) | resp[1];
        *score = (resp[2] << 8) | resp[3];
        return 0;
    }

    if (cc == CC_NO_MATCH) return -3;

    return -1;
}
// ================= DELETE FUNCTION =================
static bool fp_delete(uint16_t id) {

    uint8_t resp[16];
    uint16_t data_len;

    // Format: Start ID (2 bytes) + count (2 bytes)
    uint8_t del_data[] = {
        (id >> 8) & 0xFF,
        id & 0xFF,
        0x00,
        0x01   // delete only 1 ID
    };

    if (!fp_send_cmd(CMD_DELETE, del_data, 4)) return false;

    uint8_t cc = fp_read_resp(resp, sizeof(resp), &data_len);

    if (cc == CC_SUCCESS) {
        ESP_LOGI(TAG_FP, "Delete OK: ID=%d", id);
        return true;
    }

    ESP_LOGE(TAG_FP, "Delete Failed: 0x%02X", cc);
    return false;
}
static int fp_get_count(void) {

    uint8_t resp[16];
    uint16_t data_len;

    if (!fp_send_cmd(CMD_TEMPLATE_NUM, NULL, 0)) return -1;

    uint8_t cc = fp_read_resp(resp, sizeof(resp), &data_len);

    if (cc == CC_SUCCESS && data_len >= 2) {
        int count = (resp[0] << 8) | resp[1];
        return count;
    }

    return -1;
}
// ==================== ENROLLMENT FUNCTIONS ====================
// Per R307S datasheet: Enrollment requires 2 finger presses
// Step 1: GenImage -> Img2Tz(buffer1)
// Step 2: GenImage -> Img2Tz(buffer2) 
// Step 3: RegModel -> Store(ID)

static bool fp_enroll_step1(void) {
    ESP_LOGI(TAG_FP, "Enroll: Place finger (1st)");
    
    // Wait for finger
    uint8_t resp[16];
    uint16_t data_len;
    uint8_t cc;
    uint32_t start = xTaskGetTickCount();
    
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(ENROLL_TIMEOUT_MS)) {
        if (!fp_send_cmd(CMD_GEN_IMAGE, NULL, 0)) return false;
        cc = fp_read_resp(resp, sizeof(resp), &data_len);
        
        if (cc == CC_SUCCESS) break;
        if (cc == CC_NO_FINGER) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        ESP_LOGE(TAG_FP, "GenImage fail: 0x%02X", cc);
        return false;
    }
    
    // Convert to template (buffer 1)
    uint8_t buf_data[] = {0x01}; // Buffer ID 1
    if (!fp_send_cmd(CMD_IMG_2_TZ, buf_data, 1)) return false;
    cc = fp_read_resp(resp, sizeof(resp), &data_len);
    
    if (cc != CC_SUCCESS) {
        ESP_LOGE(TAG_FP, "Img2Tz(1) fail: 0x%02X", cc);
        return false;
    }
    ESP_LOGI(TAG_FP, "Enroll: Remove finger");
    buzzer_beep(100, 1);

    ESP_LOGI(TAG_FP, "Waiting for finger removal...");

    while (1) {
        if (!fp_send_cmd(CMD_GEN_IMAGE, NULL, 0)) return false;

    uint8_t resp[16];
    uint16_t data_len;
    uint8_t cc = fp_read_resp(resp, sizeof(resp), &data_len);

    if (cc == CC_NO_FINGER) {
        ESP_LOGI(TAG_FP, "Finger removed");
        buzzer_beep(80, 2);   
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(150));
}
    
    return true;
}

static bool fp_enroll_step2(uint16_t id) {
    ESP_LOGI(TAG_FP, "Enroll: Place same finger (2nd)");
    vTaskDelay(pdMS_TO_TICKS(500));
    uint8_t resp[16];
    uint16_t data_len;
    uint8_t cc;
    uint32_t start = xTaskGetTickCount();
    
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(ENROLL_TIMEOUT_MS)) {
        if (!fp_send_cmd(CMD_GEN_IMAGE, NULL, 0)) return false;
        cc = fp_read_resp(resp, sizeof(resp), &data_len);
        
        if (cc == CC_SUCCESS) break;
        if (cc == CC_NO_FINGER) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        ESP_LOGE(TAG_FP, "GenImage(2) fail: 0x%02X", cc);
        return false;
    }
    
    // Convert to template (buffer 2)
    uint8_t buf_data[] = {0x02}; // Buffer ID 2
    if (!fp_send_cmd(CMD_IMG_2_TZ, buf_data, 1)) return false;
    cc = fp_read_resp(resp, sizeof(resp), &data_len);
    
    if (cc != CC_SUCCESS) {
        ESP_LOGE(TAG_FP, "Img2Tz(2) fail: 0x%02X", cc);
        return false;
    }
    
    // Register model
    if (!fp_send_cmd(CMD_REG_MODEL, NULL, 0)) return false;
    cc = fp_read_resp(resp, sizeof(resp), &data_len);
    
    if (cc != CC_SUCCESS) {
        ESP_LOGE(TAG_FP, "RegModel fail: 0x%02X", cc);
        return false;
    }
    
    // Store to ID
    uint8_t store_data[] = {0x01, (id >> 8) & 0xFF, id & 0xFF};
    if (!fp_send_cmd(CMD_STORE, store_data, 3)) return false;
    cc = fp_read_resp(resp, sizeof(resp), &data_len);
    
    if (cc != CC_SUCCESS) {
        ESP_LOGE(TAG_FP, "Store fail: 0x%02X", cc);
        return false;
    }
    
    ESP_LOGI(TAG_FP, "Enroll success: ID=%d", id);
    return true;
}

// ==================== BLUETOOTH SPP ====================

static void bluetooth_spp_send(const char *msg) {
    if (spp_handle != 0) {
        esp_spp_write(spp_handle, strlen(msg), (uint8_t *)msg);
    }
}

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            ESP_LOGI(TAG_BT, "BT Auth Success");
            break;
        case ESP_BT_GAP_PIN_REQ_EVT: {
            ESP_LOGI(TAG_BT, "PIN Requested");
            esp_bt_pin_code_t pin_code = {'1','2','3','4'};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            break;
        }
        default:
            break;
    }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(TAG_BT, "SPP INIT");
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
            break;
            
        case ESP_SPP_SRV_OPEN_EVT:
            spp_handle = param->srv_open.handle;
            ESP_LOGI(TAG_BT, "Client Connected");
            bluetooth_spp_send("SmartLock Ready\n");
            bluetooth_spp_send("Send: SET,ID:123\n");
            break;
            
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAG_BT, "Client Disconnected");
            spp_handle = 0;
            break;
            
        case ESP_SPP_DATA_IND_EVT: {
            char buf[64];
            int len = param->data_ind.len > 63 ? 63 : param->data_ind.len;
            memcpy(buf, param->data_ind.data, len);
            buf[len] = '\0';
            
            for (int i = 0; i < len; i++) {
                if (buf[i] == '\r' || buf[i] == '\n') {
                    buf[i] = '\0';
                    break;
                }
            }
            
            ESP_LOGI(TAG_BT, "Received: %s", buf);
            // ================= COMMAND PARSER =================

if (strncmp(buf, "SET,ID:", 7) == 0) {

    int id = atoi(&buf[7]);

    if (id > 0 && id <= 1000) {

        ESP_LOGI(TAG_BT, "Enroll request: ID=%d", id);
        bluetooth_spp_send("Enroll Starting...\n");

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        g_enroll_id = (uint16_t)id;
        g_enroll_mode = true;
        xSemaphoreGive(state_mutex);

        xEventGroupSetBits(evt_group, EVT_ENROLL_REQUEST);
    } else {
        bluetooth_spp_send("ERROR: ID must be 1-1000\n");
    }
}

// ================= DELETE =================

else if (strncmp(buf, "DEL,ID:", 7) == 0) {

    int id = atoi(&buf[7]);

    if (id > 0 && id <= 1000) {

        ESP_LOGI(TAG_BT, "Delete request: ID=%d", id);
        bluetooth_spp_send("Deleting...\n");

        if (fp_delete((uint16_t)id)) {
            bluetooth_spp_send("Delete SUCCESS\n");
        } else {
            bluetooth_spp_send("Delete FAILED\n");
        }

    } else {
        bluetooth_spp_send("ERROR: ID must be 1-1000\n");
    }
}

// ================= GET STATUS =================

// else if (strncmp(buf, "GET,STATUS", 10) == 0)/
else if (strcmp(buf, "GET,STATUS") == 0) {

    int count = fp_get_count();

    char msg[128];
    snprintf(msg, sizeof(msg),
        "System: Ready\nMode: Idle\nFP Count: %d\n",
        (count >= 0) ? count : 0
    );

    bluetooth_spp_send(msg);
}

// ================= GET COUNT =================

// else if (strncmp(buf, "GET,COUNT", 9) == 0)
else if (strcmp(buf, "GET,COUNT") == 0) {

    int count = fp_get_count();

    char msg[64];
    if (count >= 0) {
        snprintf(msg, sizeof(msg), "Fingerprints: %d\n", count);
    } else {
        snprintf(msg, sizeof(msg), "Error reading count\n");
    }

    bluetooth_spp_send(msg);
}

// ================= SET PASSWORD =================

else if (strncmp(buf, "SET,PASS:", 9) == 0) {

    char *new_pass = &buf[9];

    if (strlen(new_pass) == 6) {

        strcpy(correct_pass, new_pass);
        nvs_save_password(correct_pass);

        bluetooth_spp_send("Password Updated\n");
        ESP_LOGI("BLUETOOTH","Password Updated");

    } else {
        bluetooth_spp_send("ERROR: Pass must be 6 digits\n");
        ESP_LOGI("BLUETOOTH","ERROR: Pass must be 6 digits");
    }
}

// ================= DEFAULT =================

else {
    bluetooth_spp_send("ERROR: Invalid Command\n");
    ESP_LOGI("BLUETOOTH","ERROR: Invalid Command");
}            
            break;
        }
        
        default:
            break;
    }
}

static void bluetooth_init(void) {
    ESP_LOGI(TAG_BT, "BT Init");
    
    // Release BLE memory since we are now correctly configured for BR/EDR only
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BT, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Now this will work because we've set CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY=y
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BT, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BT, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BT, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Set device name AFTER bluedroid_enable [^40^]
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(SPP_DEVICE_NAME));
    
    // Set scan mode (connectable + discoverable)
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));
    
    // GAP callback
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(bt_gap_cb));
    
    // Set PIN code for pairing
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code = {'1','2','3','4'};
    esp_bt_gap_set_pin(pin_type, 4, pin_code);
    
    // SPP callback
    ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));
    
    // FIXED: Use esp_spp_enhanced_init() instead of deprecated esp_spp_init() [^49^][^51^]
    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,  // Use default
    };
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));
    
    ESP_LOGI(TAG_BT, "BT Ready: %s", SPP_DEVICE_NAME);
}
static char keypad_get_key(void) {

    for (int r = 0; r < 4; r++) {

        // Set all rows HIGH
        for (int i = 0; i < 4; i++) gpio_set_level(row_pins[i], 1);

        // Pull one row LOW
        gpio_set_level(row_pins[r], 0);

        for (int c = 0; c < 4; c++) {

            if (gpio_get_level(col_pins[c]) == 0) {

                vTaskDelay(pdMS_TO_TICKS(50)); // debounce

                if (gpio_get_level(col_pins[c]) == 0) {
                    return keymap[r][c];
                }
            }
        }
    }
    return 0;
}
static void update_password_display(void) {

    // DO NOT clear full screen
    lcd_cursor(0,1);

    char buf[17] = {0};

    for (int i = 0; i < pass_index; i++) {
        buf[i] = show_password ? entered_pass[i] : '*';
    }

    lcd_print("                "); // clear only line
    lcd_cursor(0,1);
    lcd_print(buf);
}

static void task_keypad(void *pv) {


    while (1) {

        char key = keypad_get_key();
        //char key = keypad_get_key();

// ================= LONG PRESS DETECTION =================

        if (key == 'C') {

            if (last_key != 'C') {
                key_press_time = esp_timer_get_time();
                last_key = 'C';
            } else {
                int64_t duration = esp_timer_get_time() - key_press_time;

                if (duration > 2000000) {  // 2 seconds

                    if (system_mode == MODE_IDLE) {

                        system_mode = MODE_ADMIN_AUTH;

                        lcd_clear();
                        lcd_print("Admin Mode");
                        ESP_LOGI("KEYPAD","Admin Mode");
                        lcd_cursor(0,1);
                        lcd_print("Enter Pass");
                        ESP_LOGI("KEYPAD","Enter Pass");

                        buzzer_beep(100,2);

                        // Reset password input
                        pass_index = 0;
                        memset(entered_pass, 0, sizeof(entered_pass));
                    }
                }
            }

        } else {
            last_key = key;
        }

        if (key) {

            buzzer_beep(50,1);

            // DIGITS
           // if (key >= '0' && key <= '9')
            if (system_mode == MODE_IDLE && key >= '0' && key <= '9') {

                if (pass_index < 6) {
                    entered_pass[pass_index++] = key;
                    entered_pass[pass_index] = '\0';
                }
            }

            // BACKSPACE
            // else if (key == '#')
            else if (system_mode == MODE_IDLE && key == '#') {

                if (pass_index > 0) {
                    pass_index--;
                    entered_pass[pass_index] = '\0';
                }
            }

            // ENTER
            //else if (key == '*')
            else if (system_mode == MODE_IDLE && key == '*') {

    if (pass_index == 6) {

        if (strcmp(entered_pass, correct_pass) == 0) {

            lcd_clear();
            lcd_print("Welcome");
            ESP_LOGI("KEYPAD","Welcome");
            buzzer_beep(100,2);
             // 🔥 UNLOCK DOOR
            door_unlock();
            ESP_LOGI("DOOR","Door Unlocked");

            vTaskDelay(pdMS_TO_TICKS(5000));  // keep unlocked 5 sec

            // 🔒 LOCK BACK
            door_lock();
            ESP_LOGI("DOOR","Door Locked");

        } else {

            lcd_clear();
            lcd_print("Access Denied");
            ESP_LOGI("KEYPAD","Access Denied");
            buzzer_beep(300,1);
        }

        // HOLD MESSAGE (important)
        vTaskDelay(pdMS_TO_TICKS(2000));

        // RESET INPUT
        pass_index = 0;
        memset(entered_pass, 0, sizeof(entered_pass));

        // 🔥 RETURN TO IDLE SCREEN
        lcd_show_idle();
    }
}
           
            // SHOW
            else if (key == 'A') {
                show_password = true;
                ESP_LOGI("KEYPAD","Show Password");
            }

            // HIDE
            else if (key == 'B') {
                show_password = false;
                ESP_LOGI("KEYPAD","Hide Password");
            }
             // ================= ADMIN PASSWORD INPUT =================

            if (system_mode == MODE_ADMIN_AUTH) {

                if (key >= '0' && key <= '9') {

                    if (pass_index < 6) {
                        entered_pass[pass_index++] = key;
                        entered_pass[pass_index] = '\0';
                        ESP_LOGI("KEYPAD","Password: %s", entered_pass);
                    }
                }

                else if (key == '#') {

                    if (pass_index > 0) {
                        pass_index--;
                        entered_pass[pass_index] = '\0';
                        ESP_LOGI("KEYPAD","BACKSPACE: %s", entered_pass);
                    }
                }

                else if (key == 'D') {

                    // TEMP: hardcoded admin password
                    if (strcmp(entered_pass, admin_pass) == 0) {

                        lcd_clear();
                        lcd_print("Admin OK");
                        ESP_LOGI("KEYPAD","Admin OK");
                        buzzer_beep(100,2);

                        vTaskDelay(pdMS_TO_TICKS(1000));

                        // ✅ MOVE TO ADMIN MENU (NOT IDLE)
                        system_mode = MODE_ADMIN_MENU;
                        menu_index = 0;

                        lcd_clear();
                        lcd_print(">Change Pass");
                        lcd_cursor(0,1);
                        lcd_print(" Change Admin");
                        ESP_LOGI("KEYPAD","Admin Menu");


                    
                    } else {

                        lcd_clear();
                        lcd_print("Admin Denied");
                        ESP_LOGI("KEYPAD","Admin Denied");

                        buzzer_beep(300,1);

                        vTaskDelay(pdMS_TO_TICKS(1500));

                        system_mode = MODE_IDLE;
                        lcd_show_idle();
                    }

                    // Reset input
                    pass_index = 0;
                    memset(entered_pass, 0, sizeof(entered_pass));
                }

                // Update display (masked)
                lcd_cursor(0,1);

                char buf[17] = {0};
                for (int i = 0; i < pass_index; i++) {
                    buf[i] = '*';
                }

                lcd_print("                "); // clear line
                lcd_cursor(0,1);
                lcd_print(buf);
            }
            if (system_mode == MODE_IDLE && pass_index > 0) {
                update_password_display();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==================== MAIN TASKS ====================

static void task_fingerprint(void *pv) {
    static int last_state = -1;
    uint16_t matched_id = 0, score = 0;
    //static int fail_count = 0;
    
    while (1) {
        EventBits_t evt = xEventGroupWaitBits(
            evt_group,
            EVT_TOUCH_DETECTED | EVT_ENROLL_REQUEST,
            pdTRUE, pdFALSE, pdMS_TO_TICKS(100)
        );
        
        if (evt & EVT_ENROLL_REQUEST) {
            // Enrollment mode
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            uint16_t enroll_id = g_enroll_id;
            xSemaphoreGive(state_mutex);
            
            lcd_clear();
            lcd_cursor(0, 0);
            lcd_print("Enroll Mode");
            ESP_LOGI(TAG_FP,"Enroll Mode");
            lcd_cursor(0, 1);
            char buf[17];
            snprintf(buf, 16, "ID:%d", enroll_id);
            lcd_print(buf);
            
            bluetooth_spp_send("Place finger (1st press)\n");
            ESP_LOGI(TAG_FP,"Place finger (1st press)\n");
            
            if (fp_enroll_step1()) {
                lcd_clear();
                lcd_cursor(0, 0);
                lcd_print("Remove Finger");
                ESP_LOGI(TAG_FP,"Remove Finger");
                bluetooth_spp_send("Remove finger...\n");
               

                
                vTaskDelay(pdMS_TO_TICKS(2000));
                
                lcd_clear();
                lcd_cursor(0, 0);
                lcd_print("Place Again");
                ESP_LOGI(TAG_FP,"Place Again");
                bluetooth_spp_send("Place finger (2nd press)\n");
                ESP_LOGI(TAG_FP,"Place  2 Again");
                
                if (fp_enroll_step2(enroll_id)) {
                    lcd_clear();
                    lcd_cursor(0, 0);
                    lcd_print("Enroll OK!");
                    ESP_LOGI(TAG_FP,"Enroll OK!");
                    lcd_cursor(0, 1);
                    snprintf(buf, 16, "ID:%d Stored", enroll_id);
                    lcd_print(buf);
                    buzzer_beep(100, 2);
                    bluetooth_spp_send("Enroll SUCCESS\n");
                    ESP_LOGI(TAG_FP,"Enroll SUCCESS");
                } else {
                    lcd_clear();
                    lcd_cursor(0, 0);
                    lcd_print("Enroll FAIL");
                    ESP_LOGI(TAG_FP,"Enroll FAIL");
                    bluetooth_spp_send("Enroll FAILED\n");
                }
            } else {
                lcd_clear();
                lcd_cursor(0, 0);
                lcd_print("Enroll FAIL");
                ESP_LOGI(TAG_FP,"Enroll FAIL");
                bluetooth_spp_send("Enroll FAILED\n");
            }
            
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            g_enroll_mode = false;
            xSemaphoreGive(state_mutex);
            
            vTaskDelay(pdMS_TO_TICKS(3000));
            
            lcd_clear();
            lcd_cursor(0, 0);
            lcd_print("Place Finger");
            ESP_LOGI(TAG_FP,"Place Finger");
            lcd_cursor(0, 1);
            lcd_print("Or BT Enroll");
            ESP_LOGI(TAG_FP,"Or BT Enroll");
        }
        // ================= REAL-TIME SCAN LOOP =================
        // uint8_t resp[16];
        // uint16_t data_len;

            if (!g_enroll_mode) {

            int8_t result = fp_identify(&matched_id, &score);
            vTaskDelay(pdMS_TO_TICKS(150));
            if (result == 0) {

            if (last_state != 1) {
                lcd_clear();
                lcd_print("Welcome");
                ESP_LOGI(TAG_FP,"Welcome");

                char buf[16];
                snprintf(buf, 16, "ID:%d", matched_id);
                lcd_cursor(0, 1);
                lcd_print(buf);
                ESP_LOGI(TAG_FP,"ID:%d", matched_id);

                last_state = 1;
            }

            buzzer_beep(100, 2);
             // 🔥 UNLOCK DOOR
            door_unlock();
            ESP_LOGI("DOOR","Door Unlocked");

            vTaskDelay(pdMS_TO_TICKS(5000));

            // 🔒 LOCK BACK
            door_lock();
            ESP_LOGI("DOOR","Door Locked");

            // Wait until finger removed
            while (gpio_get_level(FP_TOUCH_PIN) == 0) {
                vTaskDelay(pdMS_TO_TICKS(3000));
            }

            vTaskDelay(pdMS_TO_TICKS(300));

            // NOW go back to idle
            // Show ready screen
            lcd_clear();
            lcd_show_idle(); 
            last_state = 0;
        
            
             }
            
            else if (result == -2) {
                // NO FINGER → normal idle
            }
            else if (result == -3) {
                 if (last_state != 2) {
                    lcd_clear();
                    lcd_print("Access Denied");
                    ESP_LOGI("LCD","Access Denied");
                    last_state = 2;
                }

                ESP_LOGI(TAG_FP,"Access Denied");
               
                buzzer_beep(300, 1);
                // 🔥 WAIT USING TOUCH PIN
                while (gpio_get_level(FP_TOUCH_PIN) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(3000));
                }

                // Back to idle
                if (last_state != 0) {
                 // Show ready screen
                    lcd_clear();
                    lcd_cursor(0, 0);
                    lcd_print("Enter password");
                    lcd_cursor(0, 1);
                    lcd_print("/scan Finger");
                    ESP_LOGI("LCD","Enter password");
                    ESP_LOGI("LCD","/scan Finger");
                    last_state = 0;
                }
            }
            else {

                    ESP_LOGE(TAG_FP, "Sensor Error → Recovering");
                    

                    uart_flush_input(FP_UART);
                    uart_flush(FP_UART);

                    if (last_state != 3) {
                        lcd_clear();
                        lcd_print("Recovering...");
                        ESP_LOGI("LCD","Recovering...");
                        last_state = 3;
                    }

                    vTaskDelay(pdMS_TO_TICKS(3000));
                    ;
                    if (last_state != 0) {
                        // Show ready screen
                        lcd_clear();
                        lcd_cursor(0, 0);
                        lcd_print("Enter password");
                        lcd_cursor(0, 1);
                        lcd_print("/scan Finger");
                        last_state = 0;
                        ESP_LOGI("LCD","Enter password");
                        ESP_LOGI("LCD","/scan Finger");
                    }
                    

                    vTaskDelay(pdMS_TO_TICKS(500));
            }
                    }
    }
}

        static void task_touch_monitor(void *pv) {
    
            bool last_state = false;
    
            while (1) {
                bool touched = gpio_get_level(FP_TOUCH_PIN);
        
                if (touched && !last_state) {
            // Rising edge - finger detected
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                bool in_enroll = g_enroll_mode;
                xSemaphoreGive(state_mutex);
            
                if (!in_enroll) {
                    xEventGroupSetBits(evt_group, EVT_TOUCH_DETECTED);
            }
        }
        
                last_state = touched;
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
static void keypad_init(void) {

    // Rows → OUTPUT
    for (int i = 0; i < 4; i++) {
        gpio_set_direction(row_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(row_pins[i], 1);
    }

    // Columns → INPUT_PULLUP
    for (int i = 0; i < 4; i++) {
        gpio_set_direction(col_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(col_pins[i], GPIO_PULLUP_ONLY);
    }
}

// ==================== MAIN ====================

void app_main(void) {
    ESP_LOGI(TAG_SYS, "Smart Lock Starting...");
    
    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    nvs_load_password();
    ESP_ERROR_CHECK(ret);
    
    // Create sync objects
    evt_group = xEventGroupCreate();
    i2c_mutex = xSemaphoreCreateMutex();
    state_mutex = xSemaphoreCreateMutex();
    
    // Init I2C for LCD
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_PORT, &i2c_cfg);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    
    // Init UART for Fingerprint
    uart_config_t uart_cfg = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(FP_UART, 512, 0, 0, NULL, 0);
    uart_param_config(FP_UART, &uart_cfg);
    uart_set_pin(FP_UART, FP_TX_PIN, FP_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // 🔥 Configure TOUCH PIN globally (IMPORTANT)
    gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << FP_TOUCH_PIN),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
    .intr_type = GPIO_INTR_DISABLE
};
    gpio_config(&io_conf);
    ESP_LOGI("TOUCH", "State: %d", gpio_get_level(FP_TOUCH_PIN));
    // Init hardware
    buzzer_init();
    relay_init();
    lcd_init();
    lcd_clear();
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_print("System Ready");
    ESP_LOGI("LCD","System Ready");
    bluetooth_init();
    keypad_init();
    
    // Init fingerprint
    vTaskDelay(pdMS_TO_TICKS(500));
    
    lcd_clear();
    lcd_cursor(0, 0);
    lcd_print("FP Init...");
    ESP_LOGI("LCD","FP Init...");
    
    if (fp_init()) {
        lcd_cursor(0, 1);
        lcd_print("FP OK");
        buzzer_beep(80, 2);
    } else {
        lcd_cursor(0, 1);
        lcd_print("FP FAIL");
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Show ready screen
    lcd_clear();
    lcd_show_idle();
    // lcd_cursor(0, 0);
    // lcd_print("Enter password");
    // lcd_cursor(0, 1);
    // lcd_print("/scan Finger");
    // ESP_LOGI(TAG_SYS,"Enter password");
    // ESP_LOGI(TAG_SYS,"/scan Finger");
    // Create tasks
    xTaskCreatePinnedToCore(task_fingerprint, "fp_task", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_keypad, "keypad", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(task_touch_monitor, "touch", 2048, NULL, 3, NULL, 1);
    
    ESP_LOGI(TAG_SYS, "System Ready");
}