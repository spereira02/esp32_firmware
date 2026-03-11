#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icm20948.h"

#include "imu_task.h"
#include "micro_ros_task.h"

void app_main(void) {
    imu_queue = xQueueCreate(1, sizeof(imu_reading_t));
    configASSERT(imu_queue != NULL);

    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
    xTaskCreate(micro_ros_task, "micro_ros_task", 8192, NULL, 4, NULL);
}
