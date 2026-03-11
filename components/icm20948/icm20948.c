#include "icm20948.h"
#include "esp_log.h"

static const char *TAG = "IMU_DRIVER";

esp_err_t icm20948_init(
    i2c_master_dev_handle_t *icm_handle, 
    i2c_master_dev_handle_t *mag_handle) {
    
    // Create the I2C bus internally
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = ICM20948_SCL_PIN,
        .sda_io_num = ICM20948_SDA_PIN,
        .glitch_ignore_cnt = 7,
    };
    
    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master bus");
        return ret;
    }
    
    // Add ICM-20948 to the bus
    i2c_device_config_t icm_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = ICM20948_ADDR, .scl_speed_hz = SCL_SPEED_HZ };
    ret = i2c_master_bus_add_device(bus_handle, &icm_cfg, icm_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ICM20948 device to bus");
        return ret;
    }

    // Wake up chip from idle mode & Reset Bank 0
    uint8_t wake_cmd[] = {REG_PWR_MGMT_1, WRT_PWR_CLK};
    ret = i2c_master_transmit(*icm_handle, wake_cmd, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write power management register");
        return ret;
    }

    uint8_t accel_filter[] = {REG_ACCEL_CONFIG, WRT_ACCEL_MODE};  
    ret = i2c_master_transmit(*icm_handle, accel_filter, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer filter");
        return ret;
    }

    uint8_t gyro_filter[] = {REG_GYRO_CONFIG, WRT_GYRO_MODE};   
    ret = i2c_master_transmit(*icm_handle, gyro_filter, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope filter");
        return ret;
    }

    // Enable Bypass Mode so we can see the Magnetometer
    uint8_t bypass_cmd[] = {REG_INT_PIN_CFG, WRT_BYPASS}; 
    ret = i2c_master_transmit(*icm_handle, bypass_cmd, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable bypass mode");
        return ret;
    }

    // Add Magnetometer to the bus
    i2c_device_config_t mag_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = MAG_AK09916_ADDR, .scl_speed_hz = SCL_SPEED_HZ };
    ret = i2c_master_bus_add_device(bus_handle, &mag_cfg, mag_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add AK09916 magnetometer to bus");
        return ret;
    }

    // Set Magnetometer to 100Hz Continuous Mode
    uint8_t mag_mode_cmd[] = {REG_MAG_CNTL2, WRT_MAG_MODE}; // Mode 4: 100Hz
    ret = i2c_master_transmit(*mag_handle, mag_mode_cmd, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure magnetometer mode");
        return ret;
    }

    ESP_LOGI(TAG, "9-Axis IMU Initialized");
    return ESP_OK;
}

esp_err_t imu_read_all(i2c_master_dev_handle_t icm_h, i2c_master_dev_handle_t mag_h, imu_reading_t *data) {
    uint8_t icm_buf[12];
    uint8_t mag_buf[9];
    uint8_t reg;
    esp_err_t ret;

    // Burst Read Accel/Gyro (12 bytes) Accel: 3 axes, Gyro: 3 axes. Each reading contains 16bits, to store its split into 2 * 8bits = 2 registers per reading
    reg = REG_ACCEL_XOUT_H;
    ret = i2c_master_transmit_receive(icm_h, &reg, 1, icm_buf, 12, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer/gyroscope data");
        return ret;
    }

    // Burst Read Mag (9 bytes: ST1 + 6 Data + ST2) Big endian notation for gyro and acc. Since each mesurement is split into 2 registers, we have to stich them back together
    reg = REG_MAG_ST1;
    ret = i2c_master_transmit_receive(mag_h, &reg, 1, mag_buf, 9, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read magnetometer data");
        return ret;
    }

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
