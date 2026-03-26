#ifndef PROJECT_SETTINGS_H
#define PROJECT_SETTINGS_H

// Global settings
#define ESP_NODE       "esp32_imu_node"
#define IMU_TOPIC      "imu/data_raw"
#define MAG_TOPIC      "imu/mag"
#define MICROROS_UART_BAUDRATE 921600

// Serial configuration
#include <driver/uart.h>
#define UART_TXD  1
#define UART_RXD  3
#define UART_RTS  UART_PIN_NO_CHANGE
#define UART_CTS  UART_PIN_NO_CHANGE

// imu_task.c config
#define UROS_SYNC_TIMEOUT_MS    1000
#define UROS_RESYNC_PERIOD_MS   10000
#define IMU_TASK_PERIOD_MS 20

#endif
