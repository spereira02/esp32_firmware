#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icm20948.h"

void imu_task(void *pvParameters) {
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 22, //  SCL Pin (serial clock pin)
        .sda_io_num = 21, //  SDA Pin (serial data pin)
        .glitch_ignore_cnt = 7,
    };
    /**  Initialize new master bus, i2c_new_master_bus init a new bus driver using the bus_cfg 
    * config at bus_handle. With ESP_ERROR_CHECK we chekcs if init was successfull
    * if not successfull halt program*/
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    i2c_master_dev_handle_t icm_h, mag_h;
    icm20948_init(bus_handle, &icm_h, &mag_h);

    imu_reading_t imu_data;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        if (imu_read_all(icm_h, mag_h, &imu_data) == ESP_OK) {
            printf("Accel: %.2f %.2f %.2f | Gyro: %.2f %.2f %.2f | Mag: %.7f %.7f %.7f\n", 
            imu_data.ax, imu_data.ay, imu_data.az,
            imu_data.gx, imu_data.gy, imu_data.gz,
            imu_data.mx, imu_data.my, imu_data.mz);
        }
        // Run exactly at 50Hz (20ms)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

void app_main(void) {
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}
