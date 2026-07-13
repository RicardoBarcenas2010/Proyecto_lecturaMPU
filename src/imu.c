#include "imu.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "mpu6050.h"
#include "system_config.h"

static const char *TAG = "IMU";

static TaskHandle_t xIMUTaskHandle = NULL;

static imu_data_t imu_data;

static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

// Variables para calibración dinámica
static float equilibrium_angle = 175.5f;  // Ángulo de equilibrio
static float max_angle_deg = 45.0f;       // Ángulo máximo que quieres (puedes ajustarlo)

// Control de impresión
static bool imu_print_enabled = true;     // Por defecto imprime

/* Función para mapear el ángulo a rango -128 a 128 con manejo de cruce por 180° */
static int16_t map_angle_to_range(float angle_degrees)
{
    float deviation;
    
    // Manejar ángulos negativos (cruce por 180°)
    if (angle_degrees < 0.0f)
    {
        // Normalizar a 0-360
        float normalized_angle = angle_degrees + 360.0f;
        deviation = normalized_angle - equilibrium_angle;
    }
    else
    {
        deviation = angle_degrees - equilibrium_angle;
    }
    
    // Mapear a rango -128 a 128 usando el ángulo máximo configurable
    int16_t mapped = (int16_t)((deviation / max_angle_deg) * 128.0f);
    
    // Limitar a rango -128 a 128 (permitiendo 128)
    if (mapped > 128) mapped = 128;
    if (mapped < -128) mapped = -128;
    
    return mapped;
}

/* Función para escalar de -128..128 a -max_angle_deg..max_angle_deg */
static float scale_to_degrees(int16_t mapped_value)
{
    return (mapped_value * max_angle_deg) / 128.0f;
}

static void imu_task(void *pvParameters)
{
    (void)pvParameters;

    mpu6050_data_t imu;

    while (1)
    {
        if (mpu6050_read(&imu) == ESP_OK)
        {
            imu_data.roll = imu.roll;
            imu_data.pitch = imu.pitch;

            imu_data.gyro_x = imu.gyro_x_dps;
            imu_data.gyro_y = imu.gyro_y_dps;
            imu_data.gyro_z = imu.gyro_z_dps;

            imu_data.accel_x = imu.accel_x_g;
            imu_data.accel_y = imu.accel_y_g;
            imu_data.accel_z = imu.accel_z_g;

            imu_data.temperature = imu.temperature;

            // Calcular valores mapeados (-128 a 128)
            imu_data.roll_mapped = map_angle_to_range(imu.roll);
            imu_data.pitch_mapped = map_angle_to_range(imu.pitch);

            // Escalar a grados
            float roll_scaled = scale_to_degrees(imu_data.roll_mapped);

            // Imprimir solo si está habilitado
            if (imu_print_enabled)
            {
                ESP_LOGI(TAG, "Roll: %6.2f°", roll_scaled);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t imu_init(void)
{
    ESP_ERROR_CHECK(imu_calibrate_gyro());

    if (xTaskCreate(
            imu_task,
            "IMU_Task",
            MPU_TASK_STACK_SIZE,
            NULL,
            MPU_TASK_PRIORITY,
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

    /* Dar tiempo a que el usuario suelte el sensor */
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (uint16_t i = 0; i < samples; i++)
    {
        if (mpu6050_read(&imu) != ESP_OK)
        {
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


void imu_get_gyro_bias(float *gx,
                       float *gy,
                       float *gz)
{
    if (gx != NULL)
        *gx = gyro_bias_x;

    if (gy != NULL)
        *gy = gyro_bias_y;

    if (gz != NULL)
        *gz = gyro_bias_z;
}


esp_err_t imu_get_data(imu_data_t *data)
{
    if (data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *data = imu_data;

    return ESP_OK;
}

/* FUNCIONES DE CALIBRACIÓN */
void imu_set_equilibrium_angle(float angle)
{
    equilibrium_angle = angle;
    ESP_LOGI(TAG, "Ángulo de equilibrio ajustado a: %.1f°", equilibrium_angle);
}

void imu_set_max_angle(float angle)
{
    max_angle_deg = angle;
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

/* NUEVA FUNCIÓN PARA CONTROLAR LA IMPRESIÓN */
void imu_set_print_enabled(bool enabled)
{
    imu_print_enabled = enabled;
}