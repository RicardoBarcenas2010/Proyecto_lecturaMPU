#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "system_config.h"
#include "i2c_manager.h"
#include "mpu6050.h"
#include "imu.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    uint8_t id = 0;

    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Inicializando proyecto...");
    ESP_LOGI(TAG, "================================");

    /* Inicializar bus I2C */
    ESP_ERROR_CHECK(i2c_manager_init());

    /* Inicializar MPU6050 */
    ESP_ERROR_CHECK(mpu6050_init());

    /* Leer WHO_AM_I */
    ESP_ERROR_CHECK(mpu6050_who_am_i(&id));

    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", id);

    if (id != 0x68)
    {
        ESP_LOGE(TAG, "MPU6050 no detectado");

        while (1)
        {
            vTaskDelay(portMAX_DELAY);
        }
    }

    ESP_LOGI(TAG, "MPU6050 detectado correctamente.");
    ESP_LOGI(TAG, "Inicializando módulo IMU...");

    ESP_ERROR_CHECK(imu_init());

    ESP_LOGI(TAG, "Sistema iniciado correctamente.");

    while (1)
    {
        vTaskDelay(portMAX_DELAY);
    }
}