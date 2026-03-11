/* header file that initializes all necessary registers of the chip to enable data readings the first time its called */
#ifndef ICM20948_H
#define ICM20948_H

#include "driver/i2c_master.h"

// Define the I2C pins for encapsulation
#define ICM20948_SCL_PIN    22
#define ICM20948_SDA_PIN    21

#define SCL_SPEED_HZ        400000  // clock speed

// I2C Addresses. Accel&Gyro live on the icm, magnetometer lives on external chip, AK09916
#define ICM20948_ADDR       0x68
#define MAG_AK09916_ADDR    0x0C

// ICM-20948 Registers (Bank 0) this is used to first wake up the chip from idle mode
#define REG_PWR_MGMT_1      0x06  // Wake up
#define WRT_PWR_CLK         0x01  // Write 0x01 to the register to seelect auto clock mode

#define REG_ACCEL_CONFIG    0x14  // Accelerometer config (DLPF in bits 2:0, register to set cuttoff freq for Lowpass filter)
#define WRT_ACCEL_MODE      0x04  ///* Configure Low-Pass Filters for cleaner data, using a lowpass filter built into chip
                                  // DLPF Modes: 0=196.6Hz, 1=151.8Hz, 2=119.5Hz, 3=51.2Hz, 4=23.9Hz, 5=11.6Hz, 6=5.7Hz
                                  // Mode 4 (23.9Hz) gives good noise filtering at 50Hz sampling rate */
#define REG_GYRO_CONFIG     0x01  // Gyroscope config (DLPF in bits 2:0, we use this register to set the cutoff freq on the built in Lowpass filter)
#define WRT_GYRO_MODE       0x04

#define REG_INT_PIN_CFG     0x0F  // Bypass enable
#define WRT_BYPASS          0x02
#define REG_ACCEL_XOUT_H    0x2D  // Start of 12-byte burst

// Magnetometer Registers (AK09916)
#define REG_MAG_ST1         0x10  // Data Ready bit
#define REG_MAG_CNTL2       0x31  // Mode setting og mag, setting it to mode 4 means cont data measurements at 100 Hz
#define WRT_MAG_MODE        0x08

/* Datastruct to store all 9 IMU measurements*/
typedef struct {
    float ax, ay, az; // m/s^2
    float gx, gy, gz; // rad/s
    float mx, my, mz; // T
} imu_reading_t;

// Modified signature: it creates its own bus internally or uses an existing one
esp_err_t icm20948_init(
    i2c_master_dev_handle_t *icm_handle, 
    i2c_master_dev_handle_t *mag_handle);

esp_err_t imu_read_all(
    i2c_master_dev_handle_t icm_h, 
    i2c_master_dev_handle_t mag_h, 
    imu_reading_t *data);

#endif
