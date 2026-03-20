#include <uxr/client/transport.h>
#include <driver/uart.h>
#include <driver/gpio.h>

#include "project_settings.h"

#define UART_TXD  1
#define UART_RXD  3
#define UART_RTS  UART_PIN_NO_CHANGE
#define UART_CTS  UART_PIN_NO_CHANGE
#define UART_BUFFER_SIZE (512)

bool esp32_serial_open(struct uxrCustomTransport * transport){
    size_t * uart_port = (size_t*) transport->args;

    uart_config_t uart_config = {
        .baud_rate = MICROROS_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    if (uart_param_config(*uart_port, &uart_config) != ESP_OK) {
        return false;
    }
    if (uart_set_pin(*uart_port, UART_TXD, UART_RXD, UART_RTS, UART_CTS) != ESP_OK) {
        return false;
    }
    if (uart_driver_install(*uart_port, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0) != ESP_OK) {
        return false;
    }

    return true;
}

bool esp32_serial_close(struct uxrCustomTransport * transport){
    size_t * uart_port = (size_t*) transport->args;

    return uart_driver_delete(*uart_port) == ESP_OK;
}

size_t esp32_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    size_t * uart_port = (size_t*) transport->args;
    const int txBytes = uart_write_bytes(*uart_port, (const char*) buf, len);
    return txBytes;
}

size_t esp32_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    size_t * uart_port = (size_t*) transport->args;
    const int rxBytes = uart_read_bytes(*uart_port, buf, len, timeout / portTICK_PERIOD_MS);
    return rxBytes;
}
