#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"
#include <stdint.h>

typedef struct
{
    /* Datos crudos */
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    /* Datos convertidos */
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    /* Temperatura */
    float temperature;

    /* NUEVO */
    float roll;
    float pitch;

} mpu6050_data_t;

esp_err_t mpu6050_init(void);

esp_err_t mpu6050_who_am_i(uint8_t *id);

esp_err_t mpu6050_read(mpu6050_data_t *data);

#endif