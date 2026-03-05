/* header file that initializes all necessary registers of the chip to enable data readings the first time its called */
#ifndef ICM20948_H
#define ICM20948_H

#include "driver/i2c_master.h"

// I2C Addresses. Accel&Gyro live on the icm, magnetometer lives on external chip, AK09916
#define ICM20948_ADDR       0x68
#define MAG_AK09916_ADDR    0x0C

// ICM-20948 Registers (Bank 0) this is used to first wake up the chip from idle mode
#define REG_PWR_MGMT_1      0x06  // Wake up
#define REG_INT_PIN_CFG     0x0F  // Bypass enable
#define REG_GYRO_CONFIG     0x01  // Gyroscope config (DLPF in bits 2:0, we use this register to set the cutoff freq on the built in Lowpass filter)
#define REG_ACCEL_CONFIG    0x14  // Accelerometer config (DLPF in bits 2:0, register to set cuttoff freq for Lowpass filter)
#define REG_ACCEL_XOUT_H    0x2D  // Start of 12-byte burst

// Magnetometer Registers (AK09916)
#define REG_MAG_ST1         0x10  // Data Ready bit
#define REG_MAG_CNTL2       0x31  // Mode setting og mag, setting it to mode 4 means cont data measurements at 100 Hz

/* Datastruct to store all 9 IMU measurements*/
typedef struct {
    float ax, ay, az; // m/s^2
    float gx, gy, gz; // rad/s
    float mx, my, mz; // T
} imu_reading_t;


esp_err_t icm20948_init(
    i2c_master_bus_handle_t bus_handle, 
    i2c_master_dev_handle_t *icm_handle, 
    i2c_master_dev_handle_t *mag_handle);
esp_err_t imu_read_all(
    i2c_master_dev_handle_t icm_h, 
    i2c_master_dev_handle_t mag_h, 
    imu_reading_t *data);

#endif
