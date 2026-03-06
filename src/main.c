#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icm20948.h"
#include "project_settings.h"

void imu_task(void *pvParameters) {
    i2c_master_dev_handle_t icm_h, mag_h;
    
    // Initialize IMU, this handles the I2C bus creation internally
    if (icm20948_init(&icm_h, &mag_h) != ESP_OK) {
        printf("Failed to initialize IMU. Suspending task.\n");
        vTaskSuspend(NULL);
    }

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

void micro_ros_task(void *pvParameters) {
    // Placeholder for micro-ROS logic.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    // Hardware Task (Producer) Priority 5
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
    
    // micro-ROS Task (Consumer) Priority 4
    xTaskCreate(micro_ros_task, "micro_ros_task", 8192, NULL, 4, NULL);
}
