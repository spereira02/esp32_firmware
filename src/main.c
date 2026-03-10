#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "icm20948.h"
#include "project_settings.h"

#include "esp_netif.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rmw_microros/custom_transport.h>
#include <rmw_microxrcedds_c/config.h>
#include "esp32_serial_transport.h"
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;

rcl_publisher_t imu_pub;
rcl_publisher_t mag_pub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;

QueueHandle_t imu_mailbox;

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
            /*printf("Accel: %.2f %.2f %.2f | Gyro: %.2f %.2f %.2f | Mag: %.7f %.7f %.7f\n", 
            imu_data.ax, imu_data.ay, imu_data.az,
            imu_data.gx, imu_data.gy, imu_data.gz,
            imu_data.mx, imu_data.my, imu_data.mz);*/
            xQueueOverwrite(imu_mailbox, &imu_data);
        }
        // Run exactly at 50Hz (20ms)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    imu_reading_t local_data;

    if (xQueueReceive(imu_mailbox, &local_data, 0) == pdTRUE) {
        
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        
        /* Now the sensor_msgs/msg/Imu is populated with the measured values
        *  Note: since the IMU doesnt produce values for the orientation we set the quat values x=y=z=0
        *  and w = 1 to fullfill the quat arg. The 0th entry of the covarinace matrix is set to -1 to signal that the orientaion 
        *  is unkknown. The orientation will be later calculated with the measurements of the IMU
        */
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 1.0;

        imu_msg.orientation_covariance[0] = -1.0;

        imu_msg.header.stamp.sec = ts.tv_sec;
        imu_msg.header.stamp.nanosec = ts.tv_nsec;
        imu_msg.header.frame_id.data = "imu_link";
        imu_msg.header.frame_id.size = strlen("imu_link");
        imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

        imu_msg.linear_acceleration.x = local_data.ax;
        imu_msg.linear_acceleration.y = local_data.ay;
        imu_msg.linear_acceleration.z = local_data.az;
        imu_msg.angular_velocity.x = local_data.gx;
        imu_msg.angular_velocity.y = local_data.gy;
        imu_msg.angular_velocity.z = local_data.gz;

        mag_msg.header = imu_msg.header; // Use same timestamp
        mag_msg.magnetic_field.x = local_data.mx;
        mag_msg.magnetic_field.y = local_data.my;
        mag_msg.magnetic_field.z = local_data.mz;

        // publish topics
        rcl_publish(&imu_pub, &imu_msg, NULL);
        rcl_publish(&mag_pub, &mag_msg, NULL);
    }
}

#define RCCHECK(fn) { rcl_ret_t _rc = fn; if(_rc != RCL_RET_OK) { printf("RCL error %d at line %d\n",(int)_rc,__LINE__); goto cleanup; } }

void micro_ros_task(void *pvParameters) {
    // Register the custom serial transport (UART0)
    static size_t uart_port = UART_NUM_0;
    rmw_uros_set_custom_transport(
        true,
        (void *) &uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read
    );

    // Keep retrying until the agent connects
    while (1) {
        allocator = rcl_get_default_allocator();

        printf("Waiting for micro-ROS agent...\n");
        RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
        RCCHECK(rclc_node_init_default(&node, "esp32_imu_node", "", &support));
        RCCHECK(rclc_publisher_init_default(
            &imu_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "imu/data_raw"));
        RCCHECK(rclc_publisher_init_default(
            &mag_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
            "imu/mag"));
        RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(20), timer_callback, true));
        RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
        RCCHECK(rclc_executor_add_timer(&executor, &timer));

        printf("micro-ROS agent connected!\n");

        while (1) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        cleanup:
            printf("Agent connection failed, retrying in 2s...\n");
            // Clean up before retrying
            rcl_publisher_fini(&imu_pub, &node);
            rcl_publisher_fini(&mag_pub, &node);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
            vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void) {
    sensor_msgs__msg__Imu__init(&imu_msg);
    sensor_msgs__msg__MagneticField__init(&mag_msg);

    ESP_ERROR_CHECK(esp_netif_init());
    imu_mailbox = xQueueCreate(1, sizeof(imu_reading_t));
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
    xTaskCreate(micro_ros_task, "micro_ros_task", 8192, NULL, 4, NULL);
}
