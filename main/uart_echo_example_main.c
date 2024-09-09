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

static volatile int32_t iRange00 = -1;
static volatile int32_t iRange01 = -1;
static volatile int32_t iRange02 = -1;
static volatile int32_t iRange03 = -1;

uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

int32_t soft_range(gpio_num_t Tx, gpio_num_t Rx) {
    int32_t ret = -1;
    SwSerial *tmp = sw_new(Tx, Rx, false, 128);
    uint8_t data[4];
    sw_open(tmp, 9600);
    sw_flush(tmp);
    sw_write(tmp, (uint8_t)0x55);
    vTaskDelay(60 / portTICK_PERIOD_MS);
    int len = sw_any(tmp);
    if (len == 4) {
        for (size_t i = 0; i < len; i++)
        {
            data[i] = sw_read(tmp);
        }
        if (len == 4 && data[0] == 0xff ) {
            if (data[3] == ((0xff + data[1] + data[2]) & 0xFF)) {
                ret = (data[1] << 8) | data[2];
                if (ret == 0) ret = -1;
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
    int32_t ret = -1;
    int intr_alloc_flags = 0;
    uint8_t data[10];
    uint8_t senddata[1];
    senddata[0] = 0x55;
    int len = 0;

    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, Tx, Rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_flush(UART_NUM_1);
    uart_write_bytes(UART_NUM_1, (char *)senddata, 1);
    vTaskDelay(60 / portTICK_PERIOD_MS);
    len = uart_read_bytes(UART_NUM_1, data, 10, 10 / portTICK_PERIOD_MS);
    if (len == 4 && data[0] == 0xff ) {
        if (data[3] == ((0xff + data[1] + data[2]) & 0xFF)) {
            ret = (data[1] << 8) | data[2];
            if (ret == 0) ret = -1;
        }
        else {
            ret = -1;
        }
    }
    else {
        ret = -1;
    }
    uart_driver_delete(UART_NUM_1);
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
        iRange00 = soft_range(TX00, RX00);
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(70));
        iRange01 = soft_range(TX01, RX01);
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(70));
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
    while (1) {
        esp_task_wdt_reset();
        iRange02 = hw_range(TX02, RX02);
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(70));
        iRange03 = hw_range(TX03, RX03);
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(70));
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
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(140));
    }
}

void app_main(void)
{
    xTaskCreate(ultrasonic_soft, "uart_echo_task1", 1024 * 2, NULL, configMAX_PRIORITIES - 3, NULL);
    xTaskCreate(ultrasonic_hw, "uart_echo_task2", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(send_data, "send_data", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
}
