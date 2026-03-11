#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_task.h"
#include "icm20948.h"
#include "project_settings.h"

QueueHandle_t imu_queue;

void imu_task(void *pvParameters) {
    (void) pvParameters;
    i2c_master_dev_handle_t icm_h, mag_h;
    imu_reading_t imu_data;
    
    // Initialize IMU, this handles the I2C bus creation internally
    if (icm20948_init(&icm_h, &mag_h) != ESP_OK) {
        printf("Failed to initialize IMU. Suspending task.\n");
        vTaskSuspend(NULL);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        if (imu_read_all(icm_h, mag_h, &imu_data) == ESP_OK) {
            xQueueOverwrite(imu_queue, &imu_data);
        }
        // Run exactly at 50Hz (20ms)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}
