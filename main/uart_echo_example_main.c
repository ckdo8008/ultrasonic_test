#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "sw_serial.h"
#include "esp_task_wdt.h"

#define RX00 3
#define TX00 4
#define RX01 5
#define TX01 6
#define RX02 7
#define TX02 10
#define RX03 20
#define TX03 21

static const char *TAG = "UART_Left";

#define BUF_SIZE (1024)
#define ALPHA 0.3  // 필터 상수 (0과 1 사이)

static volatile int32_t iRange00 = -1;
static volatile int32_t iRange01 = -1;
static volatile int32_t iRange02 = -1;
static volatile int32_t iRange03 = -1;

static volatile int32_t iRange00_lpf = -1;
static volatile int32_t iRange01_lpf = -1;
static volatile int32_t iRange02_lpf = -1;
static volatile int32_t iRange03_lpf = -1;

static volatile bool sensor00_fault = false;
static volatile bool sensor01_fault = false;
static volatile bool sensor02_fault = false;
static volatile bool sensor03_fault = false;

uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

int32_t apply_lpf(int32_t previous_value, int32_t current_value) {
    if (previous_value == -1) {
        return current_value;
    }
    return (int32_t)(ALPHA * current_value + (1 - ALPHA) * previous_value);
}

int32_t soft_range(gpio_num_t Tx, gpio_num_t Rx) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();    
    int32_t ret = -1;
    SwSerial *tmp = sw_new(Tx, Rx, false, 128);
    uint8_t data[4];
    sw_open(tmp, 9600);
    sw_write(tmp, (uint8_t)0x00);
    sw_flush(tmp);
    sw_write(tmp, (uint8_t)0x55);
    // vTaskDelay(61 / portTICK_PERIOD_MS);
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(61));
    int len = sw_any(tmp);
    if (len == 4) {
        for (size_t i = 0; i < len; i++)
        {
            data[i] = sw_read(tmp);
        }
        if (len == 4 && data[0] == 0xff ) {
            if (data[3] == ((0xff + data[1] + data[2]) & 0xFF)) {
                ret = (data[1] << 8) | data[2];
                if (ret == 0) ret = 6000;
            }
            else {
                ret = -1;
            }
        }
        else {
            ret = -1;
        }
    }
    else {
        ret = -1;
    }

    sw_del(tmp);
    return ret;
}

int32_t hw_range(gpio_num_t Tx, gpio_num_t Rx) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    int32_t ret = -1;
    int intr_alloc_flags = 0;
    uint8_t data[10];
    uint8_t senddataNull[1];
    uint8_t senddata[1];
    senddataNull[0] = 0x00;
    senddata[0] = 0x55;
    int len = 0;
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, Tx, Rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_write_bytes(UART_NUM_1, (char *)senddataNull, 1);
    uart_flush(UART_NUM_1);
    uart_write_bytes(UART_NUM_1, (char *)senddata, 1);
    // vTaskDelay(61 / portTICK_PERIOD_MS);
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(61));
    len = uart_read_bytes(UART_NUM_1, data, 10, 10 / portTICK_PERIOD_MS);
    if (len == 4 && data[0] == 0xff ) {
        if (data[3] == ((0xff + data[1] + data[2]) & 0xFF)) {
            ret = (data[1] << 8) | data[2];
            if (ret == 0) ret = 6000;
        }
        else {
            ret = -1;
        }
    }
    else {
        ret = -1;
    }
    uart_driver_delete(UART_NUM_1);
    gpio_reset_pin(Tx);
    // gpio_reset_pin(Rx);    
    return ret;
}

static void ultrasonic_soft(void *arg) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 1000,        // 타임아웃 1초 설정
        .idle_core_mask = 0x3,     
        .trigger_panic = true      // 타임아웃 시 시스템 패닉 트리거
    };
    esp_task_wdt_init(&wdt_config);

    // uint8_t data[10];
    esp_task_wdt_add(NULL);
    while (true)
    {
        esp_task_wdt_reset();
        // iRange00 = soft_range(TX00, RX00);
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(65));
        int32_t raw_range00 = soft_range(TX00, RX00);
        // ESP_LOGI(TAG, "raw_range00 : %ld", raw_range00);
        if (raw_range00 == -1) {
            if (sensor00_fault) {
                iRange00_lpf = -1;
                iRange00 = iRange00_lpf;
            }
            sensor00_fault = true;            
        } else {
            iRange00_lpf = apply_lpf(iRange00_lpf, raw_range00);
            iRange00 = iRange00_lpf;
            sensor00_fault = false; 
        }
        // ESP_LOGI(TAG, "%ld,%ld", iRange00, raw_range00);

        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(65));
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(65));
        // iRange01 = soft_range(TX01, RX01);
        int32_t raw_range01 = soft_range(TX01, RX01);
        // ESP_LOGI(TAG, "raw_range01 : %ld", raw_range01);
        if (raw_range01 == -1) {
            if (sensor01_fault) {
                iRange01_lpf = -1;
                iRange01 = iRange01_lpf;
            }
            sensor01_fault = true;            
        } else {
            iRange01_lpf = apply_lpf(iRange01_lpf, raw_range01);
            iRange01 = iRange01_lpf;
            sensor01_fault = false;
        }
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(65));
        // ESP_LOGI(TAG, "111111111111111111111111111111111");
    }
}

static void ultrasonic_hw(void *arg){
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 1000,        // 타임아웃 1초 설정
        .idle_core_mask = 0x3,     
        .trigger_panic = true      // 타임아웃 시 시스템 패닉 트리거
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(10));
    while (1) {
        esp_task_wdt_reset();
        // iRange02 = hw_range(TX02, RX02);
        int32_t raw_range02 = hw_range(TX02, RX02);
        // ESP_LOGI(TAG, "raw_range02 : %ld", raw_range02);
        if (raw_range02 == -1) {
            if (sensor02_fault) {
                iRange02_lpf = -1;
                iRange02 = iRange02_lpf;
            }
            sensor02_fault = true;            
        } else {
            iRange02_lpf = apply_lpf(iRange02_lpf, raw_range02);
            iRange02 = iRange02_lpf;
            sensor02_fault = false;
        }
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(65));
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(65));
        // iRange03 = hw_range(TX03, RX03);
        int32_t raw_range03 = hw_range(TX03, RX03);
        // ESP_LOGI(TAG, "raw_range03 : %ld", raw_range03);
        if (raw_range03 == -1) {
            if (sensor03_fault) {
                iRange03_lpf = -1;
                iRange03 = iRange03_lpf;
            }
            sensor03_fault = true;            
        } else {
            iRange03_lpf = apply_lpf(iRange03_lpf, raw_range03);
            iRange03 = iRange03_lpf;
            sensor03_fault = false;
        }        
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(65));
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(65));
    }
}

static void send_data(void *arg) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 1000,        // 타임아웃 1초 설정
        .idle_core_mask = 0x3,     
        .trigger_panic = true      // 타임아웃 시 시스템 패닉 트리거
    };
    esp_task_wdt_init(&wdt_config);
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(500));
    esp_task_wdt_add(NULL);
    while (true)
    {
        esp_task_wdt_reset();
        ESP_LOGI(TAG, "%ld,%ld,%ld,%ld", iRange00, iRange01, iRange02, iRange03);
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(130));
    }
}

void app_main(void)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(100));
    esp_log_level_set("gpio", ESP_LOG_WARN);
    xTaskCreate(ultrasonic_soft, "uart_echo_task1", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(ultrasonic_hw, "uart_echo_task2", 1024 * 2, NULL, configMAX_PRIORITIES - 3, NULL);
    xTaskCreate(send_data, "send_data", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
}
