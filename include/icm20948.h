#ifndef ICM20948_H
#define ICM20948_H

#include "driver/i2c_master.h"

// I2C Addresses
#define ICM20948_ADDR       0x68
#define MAG_AK09916_ADDR    0x0C

// ICM-20948 Registers (Bank 0)
#define REG_WHO_AM_I        0x00  // Should return 0xEA
#define REG_USER_CTRL       0x03  
#define REG_PWR_MGMT_1      0x06  // Wake up
#define REG_INT_PIN_CFG     0x0F  // Bypass enable
#define REG_ACCEL_XOUT_H    0x2D  // Start of 12-byte burst
#define REG_BANK_SEL        0x7F  // Bank switching

// Magnetometer Registers (AK09916)
#define REG_MAG_WIA2        0x01  // Should return 0x48
#define REG_MAG_ST1         0x10  // Data Ready bit
#define REG_MAG_HXL         0x11  // Start of 9-byte burst
#define REG_MAG_ST2         0x18  // Unlock register
#define REG_MAG_CNTL2       0x31  // Mode setting

typedef struct {
    float ax, ay, az; // m/s^2
    float gx, gy, gz; // rad/s
    float mx, my, mz; // uT
} imu_reading_t;

// Function Prototypes
esp_err_t icm20948_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *icm_handle, i2c_master_dev_handle_t *mag_handle);
esp_err_t imu_read_all(i2c_master_dev_handle_t icm_h, i2c_master_dev_handle_t mag_h, imu_reading_t *data);

#endif
