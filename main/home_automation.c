/*
 * Smart Door Lock System - ESP-IDF Implementation
 * Hardware: ESP32 + R307S Fingerprint + 4x4 Keypad + I2C LCD + Relay
 * 
 * Features:
 * - Fingerprint auth via R307S internal storage (1000 templates)
 * - PIN backup authentication
 * - LittleFS for metadata and logs
 * - FreeRTOS task-based architecture
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
#include "esp_littlefs.h"
#include "cJSON.h"

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

#define RELAY_PIN           23
#define BUZZER_PIN          5
#define LED_PIN             2

#define KEYPAD_ROW1         14
#define KEYPAD_ROW2         27
#define KEYPAD_ROW3         26
#define KEYPAD_ROW4         25
#define KEYPAD_COL1         33
#define KEYPAD_COL2         32
#define KEYPAD_COL3         19

#define I2C_PORT            I2C_NUM_0
#define I2C_SDA             21
#define I2C_SCL             22
#define LCD_ADDR            0x27

// System Parameters
#define UNLOCK_DURATION_MS      5000
#define MAX_FAILED_ATTEMPTS     3
#define LOCKOUT_DURATION_MS     30000
#define FINGERPRINT_TIMEOUT_MS  10000
#define PIN_MAX_DIGITS          6
#define KEYPAD_SCAN_MS          50

// Event Group Bits
#define EVT_TOUCH_DETECTED      BIT0
#define EVT_KEYPAD_PRESS        BIT1
#define EVT_FP_MATCHED          BIT2
#define EVT_FP_FAILED           BIT3
#define EVT_PIN_MATCHED         BIT4
#define EVT_PIN_FAILED          BIT5
#define EVT_TIMEOUT             BIT6

// Logging tags
static const char *TAG_SYS = "SYS";
static const char *TAG_FP = "FP";
static const char *TAG_KEYPAD = "KEYPAD";
static const char *TAG_LCD = "LCD";
static const char *TAG_RELAY = "RELAY";
static const char *TAG_STORAGE = "STORAGE";

// ==================== TYPE DEFINITIONS ====================

typedef enum {
    STATE_IDLE,
    STATE_FINGERPRINT_SCAN,
    STATE_PIN_ENTRY,
    STATE_UNLOCKED,
    STATE_LOCKED_OUT,
    STATE_ADMIN_ENROLL
} SystemState_t;

typedef enum {
    LCD_CMD_CLEAR,
    LCD_CMD_LINE1,
    LCD_CMD_LINE2,
    LCD_CMD_BACKLIGHT
} LCDCommandType_t;

typedef struct {
    LCDCommandType_t type;
    char text[17];
} LCDCommand_t;

typedef struct {
    char key;
    uint32_t timestamp;
} KeypadEvent_t;

typedef struct {
    uint16_t finger_id;
    uint16_t match_score;
    bool success;
} FPResult_t;

// ==================== GLOBAL VARIABLES ====================

static EventGroupHandle_t evt_group;
static QueueHandle_t lcd_queue;
static QueueHandle_t keypad_queue;
static SemaphoreHandle_t state_mutex;
static SemaphoreHandle_t i2c_mutex;

static volatile SystemState_t g_state = STATE_IDLE;
static volatile uint8_t g_failed_attempts = 0;
static volatile uint32_t g_lockout_start = 0;
static volatile uint32_t g_unlock_start = 0;

static char g_pin_buffer[PIN_MAX_DIGITS + 1];
static uint8_t g_pin_count = 0;

static const char *MASTER_PIN = "123456";
static const char *ADMIN_PIN = "999999";
static uint16_t g_template_count = 0;
static uint16_t g_next_free_id = 0;

static const char keypad_map[4][3] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}
};

static const uint8_t row_pins[4] = {KEYPAD_ROW1, KEYPAD_ROW2, KEYPAD_ROW3, KEYPAD_ROW4};
static const uint8_t col_pins[3] = {KEYPAD_COL1, KEYPAD_COL2, KEYPAD_COL3};

// ==================== UTILITY FUNCTIONS ====================

static uint16_t calc_checksum(uint8_t *packet, uint16_t len) {
    uint16_t sum = 0;
    for (uint16_t i = 6; i < len; i++) sum += packet[i];
    return sum;
}

static void set_state(SystemState_t state) {
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    g_state = state;
    ESP_LOGI(TAG_SYS, "State changed to: %d", state);
    xSemaphoreGive(state_mutex);
}

static SystemState_t get_state(void) {
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    SystemState_t s = g_state;
    xSemaphoreGive(state_mutex);
    return s;
}

// ==================== LCD DRIVER (HD44780 I2C) ====================

#define LCD_EN 0x04
#define LCD_RS 0x01
#define LCD_BACKLIGHT 0x08

static void lcd_i2c_write(uint8_t data) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data | LCD_BACKLIGHT, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    xSemaphoreGive(i2c_mutex);
}

static void lcd_pulse(uint8_t data) {
    lcd_i2c_write(data | LCD_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_i2c_write(data & ~LCD_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void lcd_send_cmd(uint8_t cmd) {
    uint8_t upper = cmd & 0xF0;
    uint8_t lower = (cmd << 4) & 0xF0;
    lcd_pulse(upper);
    lcd_pulse(lower);
    vTaskDelay(pdMS_TO_TICKS(2));
}

static void lcd_send_data(uint8_t data) {
    uint8_t upper = data & 0xF0;
    uint8_t lower = (data << 4) & 0xF0;
    lcd_pulse(upper | LCD_RS);
    lcd_pulse(lower | LCD_RS);
    vTaskDelay(pdMS_TO_TICKS(2));
}

static void lcd_init(void) {
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_i2c_write(0x30); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_pulse(0x30); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_pulse(0x30); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_pulse(0x20); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_cmd(0x28); // 4-bit, 2 lines
    lcd_send_cmd(0x0C); // Display on, no cursor
    lcd_send_cmd(0x06); // Entry mode
    lcd_send_cmd(0x01); // Clear
    ESP_LOGI(TAG_LCD, "LCD initialized");
}

static void lcd_clear(void) { lcd_send_cmd(0x01); }
static void lcd_cursor(uint8_t col, uint8_t row) {
    uint8_t addr = row ? 0x40 : 0x00;
    lcd_send_cmd(0x80 | (col + addr));
}
static void lcd_print(const char *str) {
    while (*str) lcd_send_data(*str++);
}

// ==================== R307S FINGERPRINT PROTOCOL ====================

static bool fp_send_cmd(uint8_t cmd, uint8_t *data, uint16_t data_len) {
    uint8_t packet[64];
    uint16_t idx = 0;
    
    packet[idx++] = R307_HEADER_1;
    packet[idx++] = R307_HEADER_2;
    packet[idx++] = 0xFF; packet[idx++] = 0xFF;
    packet[idx++] = 0xFF; packet[idx++] = 0xFF;
    packet[idx++] = 0x01; // Command packet
    uint16_t len = 1 + data_len + 2;
    packet[idx++] = (len >> 8) & 0xFF;
    packet[idx++] = len & 0xFF;
    packet[idx++] = cmd;
    for (uint16_t i = 0; i < data_len; i++) packet[idx++] = data[i];
    uint16_t chk = calc_checksum(packet, idx);
    packet[idx++] = (chk >> 8) & 0xFF;
    packet[idx++] = chk & 0xFF;
    
    uart_flush(FP_UART);
    int written = uart_write_bytes(FP_UART, (char*)packet, idx);
    return (written == idx);
}

static uint8_t fp_read_resp(uint8_t *data_buf, uint16_t exp_len, uint16_t *out_len) {
    uint8_t resp[64];
    int len = uart_read_bytes(FP_UART, resp, 12 + exp_len, pdMS_TO_TICKS(1000));
    if (out_len) *out_len = len;
    
    if (len < 9) {
        ESP_LOGE(TAG_FP, "Response too short: %d bytes", len);
        return 0xFF;
    }
    if (resp[0] != R307_HEADER_1 || resp[1] != R307_HEADER_2) {
        ESP_LOGE(TAG_FP, "Invalid header: %02X %02X", resp[0], resp[1]);
        return 0xFF;
    }
    if (data_buf && exp_len > 0 && len >= 12) {
        for (uint16_t i = 0; i < exp_len && (10 + i) < len; i++) {
            data_buf[i] = resp[10 + i];
        }
    }
    return resp[9]; // Confirmation code
}

static bool fp_init(void) {
    ESP_LOGI(TAG_FP, "Initializing R307S...");
    
    // Verify password (default 0x00000000)
    uint8_t pwd_data[] = {0x00, 0x00, 0x00, 0x00};
    if (!fp_send_cmd(CMD_VER_PASSWORD, pwd_data, 4)) return false;
    
    uint8_t resp[16];
    uint16_t len;
    uint8_t cc = fp_read_resp(resp, 0, &len);
    
    if (cc != CC_SUCCESS) {
        ESP_LOGE(TAG_FP, "Password verification failed: 0x%02X", cc);
        return false;
    }
    
    ESP_LOGI(TAG_FP, "R307S initialized successfully");
    return true;
}

static bool fp_get_template_count(uint16_t *count) {
    if (!fp_send_cmd(CMD_TEMPLATE_NUM, NULL, 0)) return false;
    
    uint8_t resp[8];
    uint16_t len;
    uint8_t cc = fp_read_resp(resp, 4, &len);
    
    if (cc == CC_SUCCESS && len >= 12) {
        *count = (resp[2] << 8) | resp[3];
        ESP_LOGI(TAG_FP, "Valid templates in library: %d", *count);
        return true;
    }
    return false;
}

// GR_Identify - Automatic fingerprint identification (0x34)
static int8_t fp_identify(uint16_t *matched_id, uint16_t *score) {
    ESP_LOGI(TAG_FP, "Starting GR_Identify...");
    
    // Send GR_Identify command
    if (!fp_send_cmd(CMD_GR_IDENTIFY, NULL, 0)) {
        ESP_LOGE(TAG_FP, "Failed to send GR_Identify");
        return -1;
    }
    
    uint8_t resp[8];
    uint16_t len;
    uint8_t cc = fp_read_resp(resp, 5, &len);
    
    ESP_LOGI(TAG_FP, "GR_Identify response: CC=0x%02X", cc);
    
    if (cc == CC_SUCCESS && len >= 14) {
        *matched_id = (resp[1] << 8) | resp[2];
        *score = (resp[3] << 8) | resp[4];
        ESP_LOGI(TAG_FP, "Match found: ID=%d, Score=%d", *matched_id, *score);
        return 0; // Success
    } else if (cc == CC_NO_FINGER) {
        ESP_LOGD(TAG_FP, "No finger detected");
        return -2; // No finger
    } else if (cc == CC_NO_MATCH) {
        ESP_LOGW(TAG_FP, "No match found in library");
        *matched_id = 0;
        *score = 0;
        return -3; // No match
    } else {
        ESP_LOGE(TAG_FP, "Identification error: 0x%02X", cc);
        return -1; // Error
    }
}

// ==================== HARDWARE INIT ====================

static void init_gpio(void) {
    gpio_config_t io = {};
    
    // Outputs
    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = (1ULL << RELAY_PIN) | (1ULL << BUZZER_PIN) | (1ULL << LED_PIN);
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level(RELAY_PIN, 0);
    gpio_set_level(BUZZER_PIN, 0);
    gpio_set_level(LED_PIN, 0);
    
    // Keypad rows
    io.pin_bit_mask = (1ULL << KEYPAD_ROW1) | (1ULL << KEYPAD_ROW2) | 
                      (1ULL << KEYPAD_ROW3) | (1ULL << KEYPAD_ROW4);
    gpio_config(&io);
    gpio_set_level(KEYPAD_ROW1, 1);
    gpio_set_level(KEYPAD_ROW2, 1);
    gpio_set_level(KEYPAD_ROW3, 1);
    gpio_set_level(KEYPAD_ROW4, 1);
    
    // Keypad columns with pull-ups
    io.mode = GPIO_MODE_INPUT;
    io.pin_bit_mask = (1ULL << KEYPAD_COL1) | (1ULL << KEYPAD_COL2) | (1ULL << KEYPAD_COL3);
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io);
    
    // Touch interrupt
    io.pin_bit_mask = (1ULL << FP_TOUCH_PIN);
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io);
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(FP_TOUCH_PIN, [](void* arg) {
        BaseType_t hp = pdFALSE;
        xEventGroupSetBitsFromISR(evt_group, EVT_TOUCH_DETECTED, &hp);
        portYIELD_FROM_ISR(hp);
    }, NULL);
    
    ESP_LOGI(TAG_SYS, "GPIO initialized");
}

static void init_uart(void) {
    uart_config_t cfg = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(FP_UART, 512, 0, 0, NULL, 0);
    uart_param_config(FP_UART, &cfg);
    uart_set_pin(FP_UART, FP_TX_PIN, FP_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG_SYS, "UART initialized at 57600 baud");
}

static void init_i2c(void) {
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_PORT, &cfg);
    i2c_driver_install(I2C_PORT, cfg.mode, 0, 0, 0);
    ESP_LOGI(TAG_SYS, "I2C initialized");
}

static void init_buzzer_pwm(void) {
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
    ESP_LOGI(TAG_SYS, "Buzzer PWM initialized");
}

// ==================== ACTUATOR CONTROL ====================

static void beep(uint8_t count, uint16_t ms) {
    for (uint8_t i = 0; i < count; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(ms));
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        if (i < count - 1) vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

static void relay_on(void) {
    gpio_set_level(RELAY_PIN, 1);
    gpio_set_level(LED_PIN, 1);
    ESP_LOGI(TAG_RELAY, "RELAY ON - Door unlocked");
}

static void relay_off(void) {
    gpio_set_level(RELAY_PIN, 0);
    gpio_set_level(LED_PIN, 0);
    ESP_LOGI(TAG_RELAY, "RELAY OFF - Door locked");
}

// ==================== LCD TASK ====================

static void task_lcd(void *pv) {
    LCDCommand_t cmd;
    lcd_init();
    
    lcd_clear();
    lcd_cursor(0, 0); lcd_print("Smart Lock");
    lcd_cursor(0, 1); lcd_print("System Ready");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    while (1) {
        if (xQueueReceive(lcd_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (cmd.type) {
                case LCD_CMD_CLEAR:
                    lcd_clear();
                    break;
                case LCD_CMD_LINE1:
                    lcd_cursor(0, 0);
                    lcd_print(cmd.text);
                    break;
                case LCD_CMD_LINE2:
                    lcd_cursor(0, 1);
                    lcd_print(cmd.text);
                    break;
                case LCD_CMD_BACKLIGHT:
                    // Handled in i2c write
                    break;
            }
        }
    }
}

static void lcd_msg(LCDCommandType_t line, const char *msg) {
    LCDCommand_t cmd = {.type = line};
    strncpy(cmd.text, msg, 16);
    cmd.text[16] = '\0';
    xQueueSend(lcd_queue, &cmd, portMAX_DELAY);
}

// ==================== KEYPAD TASK ====================

static void task_keypad(void *pv) {
    ESP_LOGI(TAG_KEYPAD, "Keypad task started");
    uint32_t last_scan = 0;
    
    while (1) {
        uint32_t now = xTaskGetTickCount();
        if ((now - last_scan) < pdMS_TO_TICKS(KEYPAD_SCAN_MS)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        last_scan = now;
        
        char key = '\0';
        for (int r = 0; r < 4; r++) {
            for (int i = 0; i < 4; i++) gpio_set_level(row_pins[i], (i == r) ? 0 : 1);
            vTaskDelay(pdMS_TO_TICKS(5));
            
            for (int c = 0; c < 3; c++) {
                if (gpio_get_level(col_pins[c]) == 0) {
                    key = keypad_map[r][c];
                    while (gpio_get_level(col_pins[c]) == 0) vTaskDelay(pdMS_TO_TICKS(10));
                    vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
                    r = 4; break;
                }
            }
        }
        for (int i = 0; i < 4; i++) gpio_set_level(row_pins[i], 1);
        
        if (key != '\0') {
            ESP_LOGI(TAG_KEYPAD, "Key pressed: %c", key);
            KeypadEvent_t evt = {.key = key, .timestamp = now};
            xQueueSend(keypad_queue, &evt, portMAX_DELAY);
            xEventGroupSetBits(evt_group, EVT_KEYPAD_PRESS);
        }
    }
}

// ==================== FINGERPRINT TASK ====================

static void task_fingerprint(void *pv) {
    ESP_LOGI(TAG_FP, "Fingerprint task started");
    uint16_t matched_id = 0, score = 0;
    uint32_t scan_start = 0;
    bool scanning = false;
    
    while (1) {
        SystemState_t state = get_state();
        
        if (state == STATE_FINGERPRINT_SCAN) {
            if (!scanning) {
                scanning = true;
                scan_start = xTaskGetTickCount();
                ESP_LOGI(TAG_FP, "Starting fingerprint scan sequence");
            }
            
            if ((xTaskGetTickCount() - scan_start) > pdMS_TO_TICKS(FINGERPRINT_TIMEOUT_MS)) {
                ESP_LOGW(TAG_FP, "Fingerprint scan timeout");
                xEventGroupSetBits(evt_group, EVT_FP_FAILED);
                scanning = false;
                continue;
            }
            
            int8_t result = fp_identify(&matched_id, &score);
            
            if (result == 0) {
                ESP_LOGI(TAG_FP, "Authentication success: ID=%d, Score=%d", matched_id, score);
                xEventGroupSetBits(evt_group, EVT_FP_MATCHED);
                scanning = false;
            } else if (result == -2) {
                // No finger yet, keep waiting
                vTaskDelay(pdMS_TO_TICKS(100));
            } else {
                ESP_LOGW(TAG_FP, "Authentication failed: result=%d", result);
                xEventGroupSetBits(evt_group, EVT_FP_FAILED);
                scanning = false;
            }
        } else {
            scanning = false;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ==================== STATE MACHINE ====================

static void grant_access(const char *method) {
    ESP_LOGI(TAG_SYS, "ACCESS GRANTED via %s", method);
    
    LCDCommand_t cmd;
    cmd.type = LCD_CMD_LINE1; strcpy(cmd.text, "ACCESS GRANTED");
    xQueueSend(lcd_queue, &cmd, portMAX_DELAY);
    cmd.type = LCD_CMD_LINE2; strncpy(cmd.text, method, 16);
    xQueueSend(lcd_queue, &cmd, portMAX_DELAY);
    
    relay_on();
    beep(1, 200);
    
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    g_failed_attempts = 0;
    g_unlock_start = xTaskGetTickCount();
    xSemaphoreGive(state_mutex);
    
    set_state(STATE_UNLOCKED);
}

static void lock_door(void) {
    ESP_LOGI(TAG_SYS, "Locking door...");
    relay_off();
    
    lcd_msg(LCD_CMD_LINE1, "Door Locked");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    set_state(STATE_IDLE);
    lcd_msg(LCD_CMD_LINE1, "Place Finger");
    lcd_msg(LCD_CMD_LINE2, "Or * for PIN");
}

static void trigger_lockout(void) {
    ESP_LOGE(TAG_SYS, "TRIGGERING LOCKOUT - Too many failed attempts");
    
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    g_lockout_start = xTaskGetTickCount();
    xSemaphoreGive(state_mutex);
    
    set_state(STATE_LOCKED_OUT);
    
    lcd_msg(LCD_CMD_LINE1, "TOO MANY FAILS");
    lcd_msg(LCD_CMD_LINE2, "Wait 30s...");
    beep(5, 500);
}

static void check_failed(void) {
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    uint8_t attempts = g_failed_attempts;
    xSemaphoreGive(state_mutex);
    
    ESP_LOGW(TAG_SYS, "Failed attempts: %d/%d", attempts, MAX_FAILED_ATTEMPTS);
    
    if (attempts >= MAX_FAILED_ATTEMPTS) {
        trigger_lockout();
    } else {
        set_state(STATE_IDLE);
        lcd_msg(LCD_CMD_LINE1, "Place Finger");
        lcd_msg(LCD_CMD_LINE2, "Or * for PIN");
    }
}

static void handle_keypad(char key) {
    SystemState_t state = get_state();
    
    if (state == STATE_IDLE) {
        if (key == '*') {
            // Start PIN entry
            set_state(STATE_PIN_ENTRY);
            memset(g_pin_buffer, 0, sizeof(g_pin_buffer));
            g_pin_count = 0;
            lcd_msg(LCD_CMD_LINE1, "Enter PIN:");
            lcd_msg(LCD_CMD_LINE2, "");
        }
    }
    else if (state == STATE_PIN_ENTRY) {
        if (key == '#') {
            // Submit PIN
            if (strcmp(g_pin_buffer, MASTER_PIN) == 0) {
                grant_access("PIN OK");
            } else {
                ESP_LOGW(TAG_SYS, "Wrong PIN entered");
                lcd_msg(LCD_CMD_LINE1, "Wrong PIN!");
                beep(3, 100);
                vTaskDelay(pdMS_TO_TICKS(1500));
                
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                g_failed_attempts++;
                xSemaphoreGive(state_mutex);
                
                memset(g_pin_buffer, 0, sizeof(g_pin_buffer));
                g_pin_count = 0;
                check_failed();
            }
        }
        else if (key == '*') {
            // Cancel
            set_state(STATE_IDLE);
            lcd_msg(LCD_CMD_LINE1, "Place Finger");
            lcd_msg(LCD_CMD_LINE2, "Or * for PIN");
        }
        else if (g_pin_count < PIN_MAX_DIGITS) {
            g_pin_buffer[g_pin_count++] = key;
            g_pin_buffer[g_pin_count] = '\0';
            
            // Update display with asterisks
            char stars[PIN_MAX_DIGITS + 1] = {0};
            for (int i = 0; i < g_pin_count; i++) stars[i] = '*';
            lcd_msg(LCD_CMD_LINE2, stars);
        }
    }
}

static void task_state_machine(void *pv) {
    ESP_LOGI(TAG_SYS, "State machine started");
    
    lcd_msg(LCD_CMD_LINE1, "Place Finger");
    lcd_msg(LCD_CMD_LINE2, "Or * for PIN");
    
    while (1) {
        EventBits_t evt = xEventGroupWaitBits(
            evt_group,
            EVT_TOUCH_DETECTED | EVT_KEYPAD_PRESS | EVT_FP_MATCHED | 
            EVT_FP_FAILED | EVT_TIMEOUT,
            pdTRUE, pdFALSE, pdMS_TO_TICKS(100)
        );
        
        SystemState_t state = get_state();
        uint32_t now = xTaskGetTickCount();
        
        // Handle unlock timeout
        if (state == STATE_UNLOCKED) {
            uint32_t elapsed = now - g_unlock_start;
            if (elapsed >= pdMS_TO_TICKS(UNLOCK_DURATION_MS)) {
                ESP_LOGI(TAG_SYS, "Auto-lock timeout reached");
                lock_door();
                continue;
            }
            
            uint32_t remaining = (UNLOCK_DURATION_MS - elapsed) / 1000;
            char buf[17];
            snprintf(buf, 17, "Lock in %lus", remaining);
            lcd_msg(LCD_CMD_LINE2, buf);
        }
        
        // Handle lockout timeout
        if (state == STATE_LOCKED_OUT) {
            uint32_t elapsed = now - g_lockout_start;
            if (elapsed >= pdMS_TO_TICKS(LOCKOUT_DURATION_MS)) {
                ESP_LOGI(TAG_SYS, "Lockout period ended");
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                g_failed_attempts = 0;
                xSemaphoreGive(state_mutex);
                set_state(STATE_IDLE);
                lcd_msg(LCD_CMD_LINE1, "Place Finger");
                lcd_msg(LCD_CMD_LINE2, "Or * for PIN");
                continue;
            }
            
            uint32_t remaining = (LOCKOUT_DURATION_MS - elapsed) / 1000;
            char buf[17];
            snprintf(buf, 17, "Wait %lus", remaining);
            lcd_msg(LCD_CMD_LINE2, buf);
        }
        
        // Process events
        if (evt & EVT_TOUCH_DETECTED) {
            if (state == STATE_IDLE) {
                ESP_LOGI(TAG_SYS, "Touch detected, starting FP scan");
                set_state(STATE_FINGERPRINT_SCAN);
                lcd_msg(LCD_CMD_LINE1, "Scanning...");
                lcd_msg(LCD_CMD_LINE2, "Place finger");
            }
        }
        
        if (evt & EVT_KEYPAD_PRESS) {
            KeypadEvent_t key_evt;
            if (xQueueReceive(keypad_queue, &key_evt, 0) == pdTRUE) {
                handle_keypad(key_evt.key);
            }
        }
        
        if (evt & EVT_FP_MATCHED) {
            grant_access("Fingerprint OK");
        }
        
        if (evt & EVT_FP_FAILED) {
            lcd_msg(LCD_CMD_LINE1, "No Match!");
            lcd_msg(LCD_CMD_LINE2, "Try again...");
            beep(3, 100);
            vTaskDelay(pdMS_TO_TICKS(1500));
            
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            g_failed_attempts++;
            xSemaphoreGive(state_mutex);
            
            check_failed();
        }
    }
}

// ==================== MAIN ====================

void app_main(void) {
    ESP_LOGI(TAG_SYS, "Smart Lock Starting...");
    
    // Init NVS (required for LittleFS if used later)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize hardware
    init_gpio();
    init_uart();
    init_i2c();
    init_buzzer_pwm();
    
    // Create FreeRTOS objects
    evt_group = xEventGroupCreate();
    lcd_queue = xQueueCreate(8, sizeof(LCDCommand_t));
    keypad_queue = xQueueCreate(16, sizeof(KeypadEvent_t));
    state_mutex = xSemaphoreCreateMutex();
    i2c_mutex = xSemaphoreCreateMutex();
    
    if (!evt_group || !lcd_queue || !keypad_queue || !state_mutex || !i2c_mutex) {
        ESP_LOGE(TAG_SYS, "Failed to create RTOS objects!");
        return;
    }
    
    // Init LCD early for feedback
    lcd_init();
    lcd_clear();
    lcd_cursor(0, 0); lcd_print("Init FP Sensor");
    
    // Init fingerprint
    vTaskDelay(pdMS_TO_TICKS(200)); // Power-on delay for R307S
    
    if (fp_init()) {
        lcd_cursor(0, 1); lcd_print("FP Sensor OK");
        fp_get_template_count(&g_template_count);
    } else {
        lcd_cursor(0, 1); lcd_print("FP Sensor FAIL");
        ESP_LOGE(TAG_SYS, "Fingerprint sensor not responding!");
    }
    
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    // Create tasks
    xTaskCreatePinnedToCore(task_lcd, "lcd", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(task_keypad, "keypad", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(task_fingerprint, "fp", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_state_machine, "state", 8192, NULL, 6, NULL, 1);
    
    ESP_LOGI(TAG_SYS, "All tasks created, system running");
}