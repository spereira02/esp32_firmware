#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "icm20948.h"

extern QueueHandle_t imu_queue;

void imu_task(void *pvParameters);
