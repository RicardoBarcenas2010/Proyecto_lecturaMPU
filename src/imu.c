#include "imu.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include <math.h>

#include "mpu6050.h"
#include "system_config.h"

static const char *TAG = "IMU";

static TaskHandle_t xIMUTaskHandle = NULL;

static imu_data_t imu_data;

static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

// Variables para calibración dinámica
static float equilibrium_angle = 175.5f;
static float max_angle_deg = 45.0f;

// Control de impresión
static bool imu_print_enabled = false;

/* Función para mapear el ángulo a rango -128 a 128 */
static int16_t map_angle_to_range(float angle_degrees)
{
    float deviation;
    
    if (isnan(angle_degrees) || isinf(angle_degrees)) {
        return 0;
    }
    
    if (max_angle_deg <= 0.0f) {
        max_angle_deg = 45.0f;
    }
    
    if (angle_degrees < 0.0f)
    {
        float normalized_angle = angle_degrees + 360.0f;
        deviation = normalized_angle - equilibrium_angle;
    }
    else
    {
        deviation = angle_degrees - equilibrium_angle;
    }
    
    int16_t mapped = (int16_t)((deviation / max_angle_deg) * 128.0f);
    
    if (mapped > 128) mapped = 128;
    if (mapped < -128) mapped = -128;
    
    return mapped;
}

static float scale_to_degrees(int16_t mapped_value)
{
    if (max_angle_deg <= 0.0f) {
        max_angle_deg = 45.0f;
    }
    return (mapped_value * max_angle_deg) / 128.0f;
}

static void imu_task(void *pvParameters)
{
    (void)pvParameters;
    mpu6050_data_t imu;
    uint32_t counter = 0;

    while (1)
    {
        if (mpu6050_read(&imu) == ESP_OK)
        {
            if (isnan(imu.roll) || isinf(imu.roll)) {
                imu.roll = 0.0f;
            }
            if (isnan(imu.pitch) || isinf(imu.pitch)) {
                imu.pitch = 0.0f;
            }
            
            imu_data.roll = imu.roll;
            imu_data.pitch = imu.pitch;
            imu_data.gyro_x = imu.gyro_x_dps;
            imu_data.gyro_y = imu.gyro_y_dps;
            imu_data.gyro_z = imu.gyro_z_dps;
            imu_data.accel_x = imu.accel_x_g;
            imu_data.accel_y = imu.accel_y_g;
            imu_data.accel_z = imu.accel_z_g;
            imu_data.temperature = imu.temperature;

            imu_data.roll_mapped = map_angle_to_range(imu.roll);
            imu_data.pitch_mapped = map_angle_to_range(imu.pitch);

            if (imu_print_enabled && (counter % 10 == 0))
            {
                float roll_scaled = scale_to_degrees(imu_data.roll_mapped);
                ESP_LOGI(TAG, "Roll: %6.2f° (mapped: %d)", roll_scaled, imu_data.roll_mapped);
            }
            counter++;
        }
        else
        {
            ESP_LOGE(TAG, "Error leyendo MPU6050");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t imu_init(void)
{
    ESP_ERROR_CHECK(imu_calibrate_gyro());

    if (xTaskCreate(
            imu_task,
            "IMU_Task",
            IMU_TASK_STACK_SIZE,    // ✅ Ahora definido en system_config.h
            NULL,
            IMU_TASK_PRIORITY,      // ✅ Ahora definido en system_config.h
            &xIMUTaskHandle) != pdPASS)
    {
        ESP_LOGE(TAG, "No fue posible crear la tarea");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Tarea IMU creada");
    return ESP_OK;
}

esp_err_t imu_calibrate_gyro(void)
{
    mpu6050_data_t imu;
    const uint16_t samples = 500;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "Calibrando giroscopio...");
    ESP_LOGI(TAG, "NO muevas el sensor.");
    ESP_LOGI(TAG, "=================================");

    vTaskDelay(pdMS_TO_TICKS(2000));

    for (uint16_t i = 0; i < samples; i++)
    {
        if (mpu6050_read(&imu) != ESP_OK)
        {
            ESP_LOGE(TAG, "Error durante calibración");
            return ESP_FAIL;
        }

        sum_x += imu.gyro_x_dps;
        sum_y += imu.gyro_y_dps;
        sum_z += imu.gyro_z_dps;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    gyro_bias_x = sum_x / samples;
    gyro_bias_y = sum_y / samples;
    gyro_bias_z = sum_z / samples;

    ESP_LOGI(TAG, "Calibracion terminada.");
    ESP_LOGI(TAG, "Bias X = %.4f", gyro_bias_x);
    ESP_LOGI(TAG, "Bias Y = %.4f", gyro_bias_y);
    ESP_LOGI(TAG, "Bias Z = %.4f", gyro_bias_z);

    return ESP_OK;
}

void imu_get_gyro_bias(float *gx, float *gy, float *gz)
{
    if (gx != NULL) *gx = gyro_bias_x;
    if (gy != NULL) *gy = gyro_bias_y;
    if (gz != NULL) *gz = gyro_bias_z;
}

esp_err_t imu_get_data(imu_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *data = imu_data;
    return ESP_OK;
}

void imu_set_equilibrium_angle(float angle)
{
    equilibrium_angle = angle;
    ESP_LOGI(TAG, "Ángulo de equilibrio ajustado a: %.1f°", equilibrium_angle);
}

void imu_set_max_angle(float angle)
{
    if (angle <= 0.0f) {
        max_angle_deg = 45.0f;
    } else {
        max_angle_deg = angle;
    }
    ESP_LOGI(TAG, "Ángulo máximo ajustado a: %.1f°", max_angle_deg);
}

float imu_get_equilibrium_angle(void)
{
    return equilibrium_angle;
}

float imu_get_max_angle(void)
{
    return max_angle_deg;
}

void imu_set_print_enabled(bool enabled)
{
    imu_print_enabled = enabled;
}