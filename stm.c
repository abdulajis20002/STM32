#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#define LED1 GPIO_NUM_2
#define LED2 GPIO_NUM_4

#define UART_PORT UART_NUM_1
#define BUF_SIZE 1024

static QueueHandle_t cmd_queue;
static const char *TAG = "HOME_AUTO";

/* ---------- UART TASK ---------- */
void uart_task(void *arg)
{
    uint8_t data[BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "Received: %s", data);
            xQueueSend(cmd_queue, data, portMAX_DELAY);
        }
    }
}

/* ---------- DEVICE CONTROL TASK ---------- */
void device_task(void *arg)
{
    char cmd[20];

    while (1) {
        if (xQueueReceive(cmd_queue, &cmd, portMAX_DELAY)) {

            if (strcmp(cmd, "ON1") == 0)
                gpio_set_level(LED1, 1);
            else if (strcmp(cmd, "OFF1") == 0)
                gpio_set_level(LED1, 0);
            else if (strcmp(cmd, "ON2") == 0)
                gpio_set_level(LED2, 1);
            else if (strcmp(cmd, "OFF2") == 0)
                gpio_set_level(LED2, 0);

            ESP_LOGI(TAG, "Command Executed");
        }
    }
}

/* ---------- MAIN ---------- */
void app_main(void)
{
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    cmd_queue = xQueueCreate(5, sizeof(char[20]));

    xTaskCreate(uart_task, "UART_Task", 4096, NULL, 2, NULL);
    xTaskCreate(device_task, "Device_Task", 4096, NULL, 2, NULL);
}
