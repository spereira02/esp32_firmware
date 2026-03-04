#include "icm20948.h"
#include "esp_log.h"

static const char *TAG = "IMU_DRIVER";

esp_err_t icm20948_init(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *icm_handle, i2c_master_dev_handle_t *mag_handle) {
    // 1. Add ICM-20948 to the bus
    i2c_device_config_t icm_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = ICM20948_ADDR, .scl_speed_hz = 400000 };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &icm_cfg, icm_handle));

    // 2. Wake up & Reset Bank 0
    uint8_t wake_cmd[] = {REG_PWR_MGMT_1, 0x01}; // Bit 0: Auto-select clock
    i2c_master_transmit(*icm_handle, wake_cmd, 2, -1);

    // 3. Enable Bypass Mode so we can see the Magnetometer
    uint8_t bypass_cmd[] = {REG_INT_PIN_CFG, 0x02}; 
    i2c_master_transmit(*icm_handle, bypass_cmd, 2, -1);

    // 4. Add Magnetometer to the bus
    i2c_device_config_t mag_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = MAG_AK09916_ADDR, .scl_speed_hz = 400000 };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mag_cfg, mag_handle));

    // 5. Set Magnetometer to 100Hz Continuous Mode
    uint8_t mag_mode_cmd[] = {REG_MAG_CNTL2, 0x08}; // Mode 4: 100Hz
    i2c_master_transmit(*mag_handle, mag_mode_cmd, 2, -1);

    ESP_LOGI(TAG, "9-Axis IMU Initialized");
    return ESP_OK;
}

esp_err_t imu_read_all(i2c_master_dev_handle_t icm_h, i2c_master_dev_handle_t mag_h, imu_reading_t *data) {
    /*The ICM-20948 chip uses a MEMS structure to pick up physical acceleration as an analog voltage. That signal gets
     reformated by internal 16-bit ADCs into Two's Complement signed numbers so we can handle both positive and negative Gs. 
     Because $I^2C$ can only move 8 bits at a time, the chip splits that 16-bit value into High and Low registers. Since it’s a 
     Big-Endian device, it sends the "important" High byte first.Once the ESP32 grabs those bytes into an array, we "stitch" 
     them back together. We take the High byte, shift it left by 8 bits to put it in the top half of a 16-bit variable, and then 
     drop the Low byte into the bottom half using a bitwise OR. This gives us our Raw Counts, which we scale by 16,384 (for the  +-2g setting) 
     and then multiply by 9.80665 to get the m/s^2 values. 

     For the gyroscope, the chip detects rotation by measuring the Coriolis force acting on internal vibrating silicon structures. 
     These vibrations are converted by a 16-bit ADC into signed counts, which we reconstruct by shifting the High byte left and appending the 
     Low byte. We use the rescaling factor of 131 LSB/dps because, at a +-250°/s range, the ADC maps 131 digital steps to every 1 degree per 
     second of physical rotation. To convert to SI units we divide the raw counts by 131 and multiply by 0.01745 to convert the output into radians 
     per second.The magnetometer operates as a separate Little-Endian logic block that detects magnetic flux density via the Hall Effect. Unlike 
     the other sensors, we stitch these bytes by shifting the second byte received, as the Low byte arrives first. We rescale the data using a fixed 
     factor of 0.15 to convert raw counts into Microtesla ($\mu T$), which represents the strength of the local magnetic field. For the final micro-ROS
    transfer, this value is scaled down to Tesla to ensure the logic aligns with standard SI units.
    */
    uint8_t icm_buf[12];
    uint8_t mag_buf[9];
    uint8_t reg;

    // Burst Read Accel/Gyro (12 bytes) Accel: 3 axes, Gyro: 3 axes 16bits into 2 * 8bits = registers
    reg = REG_ACCEL_XOUT_H;
    i2c_master_transmit_receive(icm_h, &reg, 1, icm_buf, 12, -1);

    // Burst Read Mag (9 bytes: ST1 + 6 Data + ST2)
    reg = REG_MAG_ST1;
    i2c_master_transmit_receive(mag_h, &reg, 1, mag_buf, 9, -1);

    int16_t raw_ax = (int16_t)((icm_buf[0] << 8) | icm_buf[1]);
    int16_t raw_ay = (int16_t)((icm_buf[2] << 8) | icm_buf[3]);
    int16_t raw_az = (int16_t)((icm_buf[4] << 8) | icm_buf[5]);

    int16_t raw_gx = (int16_t)((icm_buf[6] << 8) | icm_buf[7]);
    int16_t raw_gy = (int16_t)((icm_buf[8] << 8) | icm_buf[9]);
    int16_t raw_gz = (int16_t)((icm_buf[10] << 8) | icm_buf[11]);

    float accel_scale = 9.80665f / 16384.0f;
    data->ax =  (float)raw_ax * accel_scale;
    data->ay =  (float)raw_ay * accel_scale;
    data->az =  (float)raw_az * accel_scale;

    float gyro_scale = 0.01745329f / 131.0f;
    data->gx =  (float)raw_gx * gyro_scale;
    data->gy =  (float)raw_gy * gyro_scale;
    data->gz =  (float)raw_gz * gyro_scale;

    if (mag_buf[0] & 0x01) {
        // Little-Endian stitch (High byte is buf[index+1])
        int16_t raw_mx = (int16_t)((mag_buf[2] << 8) | mag_buf[1]);
        int16_t raw_my = (int16_t)((mag_buf[4] << 8) | mag_buf[3]);
        int16_t raw_mz = (int16_t)((mag_buf[6] << 8) | mag_buf[5]);

        float magn_scale = 0.15e-6f; 
        data->mx = (float)raw_mx * magn_scale;
        data->my = (float)raw_my * magn_scale;
        data->mz = (float)raw_mz * magn_scale;
    }

    return ESP_OK;
}
