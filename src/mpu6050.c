#include "mpu6050.h"
#include "i2c_manager.h"
#include "system_config.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MPU6050";

#define MPU6050_REG_WHO_AM_I      0x75
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_ACCEL_XOUT_H  0x3B

#define ACCEL_SENSITIVITY 16384.0f
#define GYRO_SENSITIVITY    131.0f

// ✅ FACTOR DE ESCALA PARA CORREGIR EL ROLL
// Si el sensor marca 40° cuando debería marcar 45°:
// factor = 45 / 40 = 1.125
#define ROLL_SCALE_FACTOR       1.125f   // ⚠️ AJUSTAR SEGÚN TU MEDICIÓN

esp_err_t mpu6050_who_am_i(uint8_t *id)
{
    return i2c_read_register(MPU6050_ADDRESS, MPU6050_REG_WHO_AM_I, id, 1);
}

esp_err_t mpu6050_init(void)
{
    uint8_t data = 0x00;
    esp_err_t ret = i2c_write_register(MPU6050_ADDRESS, MPU6050_REG_PWR_MGMT_1, &data, 1);
    
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "No fue posible despertar el MPU6050");
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 inicializado");
    ESP_LOGI(TAG, "Roll Scale Factor: %.3f", ROLL_SCALE_FACTOR);
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_t *data)
{
    uint8_t buffer[14];
    esp_err_t ret = i2c_read_register(MPU6050_ADDRESS, MPU6050_REG_ACCEL_XOUT_H, buffer, 14);

    if(ret != ESP_OK)
        return ret;

    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    int16_t temp_raw = (int16_t)((buffer[6] << 8) | buffer[7]);

    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

    data->temperature = (temp_raw / 340.0f) + 36.53f;

    data->accel_x_g = (float)data->accel_x / ACCEL_SENSITIVITY;
    data->accel_y_g = (float)data->accel_y / ACCEL_SENSITIVITY;
    data->accel_z_g = (float)data->accel_z / ACCEL_SENSITIVITY;

    // Calcular roll
    data->roll = atan2f(data->accel_y_g, data->accel_z_g) * 180.0f / (float)M_PI;
    
    // Normalizar a -180 a 180
    if (data->roll > 180.0f) data->roll -= 360.0f;
    if (data->roll < -180.0f) data->roll += 360.0f;

    // ✅ APLICAR FACTOR DE ESCALA PARA CORREGIR LOS EXTREMOS
    data->roll *= ROLL_SCALE_FACTOR;

    // Limitar a ±45° físicos
    if (data->roll > ANGLE_MAX) data->roll = ANGLE_MAX;
    if (data->roll < ANGLE_MIN) data->roll = ANGLE_MIN;

    data->pitch = atan2f(-data->accel_x_g,
            sqrtf(data->accel_y_g * data->accel_y_g + data->accel_z_g * data->accel_z_g)) *
            180.0f / (float)M_PI;

    // ✅ También aplicar escala al pitch
    data->pitch *= ROLL_SCALE_FACTOR;

    if (data->pitch > ANGLE_MAX) data->pitch = ANGLE_MAX;
    if (data->pitch < ANGLE_MIN) data->pitch = ANGLE_MIN;

    data->gyro_x_dps = (float)data->gyro_x / GYRO_SENSITIVITY;
    data->gyro_y_dps = (float)data->gyro_y / GYRO_SENSITIVITY;
    data->gyro_z_dps = (float)data->gyro_z / GYRO_SENSITIVITY;

    return ESP_OK;
}