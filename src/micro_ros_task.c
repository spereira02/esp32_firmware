#include "micro_ros_task.h"

#include <string.h>
#include <time.h>
#include <stdio.h>

#include "project_settings.h"
#include "imu_task.h"
#include "esp32_serial_transport.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"

#include "sensor_msgs/msg/imu.h"
#include "sensor_msgs/msg/magnetic_field.h"

#include "rmw_microros/custom_transport.h"
#include "rmw_microros/rmw_microros.h"
#include "driver/uart.h"


rcl_publisher_t imu_pub;
rcl_publisher_t mag_pub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;

static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__MagneticField mag_msg;

#define RCCHECK(fn) { rcl_ret_t _rc = fn; if(_rc != RCL_RET_OK) { printf("RCL error %d at line %d\n",(int)_rc,__LINE__); goto cleanup; } }
#define RCSOFTCHECK(fn) { rcl_ret_t _rc = fn; if(_rc != RCL_RET_OK) { printf("RCL soft error %d at line %d\n",(int)_rc,__LINE__); } }

/* Timer callback: publishes IMU data */
static void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) timer;
    (void) last_call_time;
    imu_reading_t local_data;

    if (xQueuePeek(imu_queue, &local_data, 0) == pdTRUE) {
        
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

        mag_msg.header = imu_msg.header; 
        mag_msg.magnetic_field.x = local_data.mx;
        mag_msg.magnetic_field.y = local_data.my;
        mag_msg.magnetic_field.z = local_data.mz;

        // publish topics
        RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
        RCSOFTCHECK(rcl_publish(&mag_pub, &mag_msg, NULL));
    }
}

/* micro-ROS task */
void micro_ros_task(void *pvParameters) {
    (void) pvParameters;
    sensor_msgs__msg__Imu__init(&imu_msg);
    sensor_msgs__msg__MagneticField__init(&mag_msg);

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
    // First loop to establish connection with micro-ROS agent, reconnects to agent if something fails
    while (1) {
        allocator = rcl_get_default_allocator();

        printf("Waiting for micro-ROS agent...\n");

        RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
        RCCHECK(rclc_node_init_default(&node, ESP_NODE, "", &support));
        RCCHECK(rclc_publisher_init_default(
            &imu_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            IMU_TOPIC));
        RCCHECK(rclc_publisher_init_default(
            &mag_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
            MAG_TOPIC));
        RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(IMU_TASK_PERIOD_MS), timer_callback, true));
        RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
        RCCHECK(rclc_executor_add_timer(&executor, &timer));

        printf("micro-ROS agent connected!\n");

        /* Check the ROS executor for pending callbacks (timers, subscriptions).
        * Wait up to 10 ms for events, then return to the task loop.
        * The timer triggers timer_callback() every IMU_TASK_PERIOD_MS (20 ms).
        */
        while (1) {
            rcl_ret_t rc =  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
                printf("Executor spin failed: %d\n", (int) rc);
                goto cleanup;
            }
            if (rmw_uros_ping_agent(20, 1) != RMW_RET_OK) {
                printf("micro-ROS agent disconnected\n");
                goto cleanup;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        cleanup:
            printf("Agent connection failed, retrying in 2s...\n");
            // Clean up before retrying
            RCSOFTCHECK(rcl_publisher_fini(&imu_pub, &node));
            RCSOFTCHECK(rcl_publisher_fini(&mag_pub, &node));
            RCSOFTCHECK(rcl_node_fini(&node));
            RCSOFTCHECK(rclc_support_fini(&support));
            vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
