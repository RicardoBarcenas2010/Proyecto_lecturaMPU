#ifndef IMU_H
#define IMU_H

#include "esp_err.h"

typedef struct
{
    float roll;
    float pitch;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float accel_x;
    float accel_y;
    float accel_z;

    float temperature;

    /* NUEVOS CAMPOS PARA VALORES MAPEADOS */
    int16_t roll_mapped;    // Rango -128 a 128
    int16_t pitch_mapped;   // Rango -128 a 128

} imu_data_t;

esp_err_t imu_init(void);
esp_err_t imu_calibrate_gyro(void);
void imu_get_gyro_bias(float *gx, float *gy, float *gz);
esp_err_t imu_get_data(imu_data_t *data);

/* FUNCIONES DE CALIBRACIÓN */
void imu_set_equilibrium_angle(float angle);
void imu_set_max_angle(float angle);
float imu_get_equilibrium_angle(void);
float imu_get_max_angle(void);

#endif