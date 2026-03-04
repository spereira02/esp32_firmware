#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_PORT I2C_NUM_0
#define ICM_ADDR 0x68  // Based on your Arduino AD0_VAL 0

// The "Secret" from the Arduino Library: Wake up the chip
esp_err_t icm20948_wakeup() {
    uint8_t data[] = {0x06, 0x01}; // Register 0x06 (PWR_MGMT_1), Value 0x01 (Auto-select clock)
    return i2c_master_write_to_device(I2C_MASTER_PORT, ICM_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
}

void init_i2c() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = { .clk_speed = 400000 }
    };
    i2c_param_config(I2C_MASTER_PORT, &conf);
    i2c_driver_install(I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0);
}
esp_err_t icm20948_select_bank(uint8_t bank) {
    // Register 0x7F is the "Bank Select" register on all banks
    uint8_t data[] = {0x7F, (uint8_t)(bank << 4)}; 
    return i2c_master_write_to_device(I2C_MASTER_PORT, ICM_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
}

extern "C" void app_main(void) {
    init_i2c();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    icm20948_select_bank(0);
    
    // Attempt to wake up the sensor
    if (icm20948_wakeup() == ESP_OK) {
        printf("Sensor Woken Up Successfully!\n");
    } else {
        printf("Failed to communicate with Sensor at 0x68\n");
    }

    while (1) {
        uint8_t reg = 0x2D; // ACCEL_XOUT_H register
        uint8_t data[6];    // Space for X, Y, Z (2 bytes each)
        
        // Read 6 bytes starting from Accel X
        esp_err_t err = i2c_master_write_read_device(I2C_MASTER_PORT, ICM_ADDR, &reg, 1, data, 6, 100 / portTICK_PERIOD_MS);

        if (err == ESP_OK) {
            // Combine high and low bytes (Reverse engineering the Arduino logic)
            int16_t ax = (data[0] << 8) | data[1];
            int16_t ay = (data[2] << 8) | data[3];
            int16_t az = (data[4] << 8) | data[5];
            printf("Accel Raw -> X: %d | Y: %d | Z: %d\n", ax, ay, az);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
