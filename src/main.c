#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "esp_log.h"
#include "esp_system.h"

#include "system_config.h"
#include "i2c_manager.h"
#include "mpu6050.h"
#include "imu.h"
#include "brushless.h"

#define ESC_GPIO_PIN_1   GPIO_NUM_13   // Motor Maestro (Izquierdo)
#define ESC_GPIO_PIN_2   GPIO_NUM_14   // Motor Esclavo (Derecho)

// Tarea para leer teclas desde consola
static void console_task(void *pvParameters)
{
    (void)pvParameters;
    
    char c;
    
    printf("\n========================================\n");
    printf("CARACTERIZACIÓN - ZONA DE INTERÉS\n");
    printf("========================================\n");
    printf("  'x' - Detener caracterización\n");
    printf("  'q' - Reiniciar sistema\n");
    printf("========================================\n\n");
    
    while (1) {
        if (scanf("%c", &c) == 1) {
            switch(c) {
                case 'x':
                case 'X':
                    if (brushless_is_sweep_running()) {
                        brushless_stop_sweep_char();
                    }
                    break;
                    
                case 'q':
                case 'Q':
                    if (brushless_is_sweep_running()) {
                        brushless_stop_sweep_char();
                    }
                    vTaskDelay(pdMS_TO_TICKS(500));
                    esp_restart();
                    break;
                    
                default:
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    uint8_t id = 0;
    
    // ====== DESACTIVAR TODOS LOS LOGS ======
    esp_log_level_set("*", ESP_LOG_NONE);
    
    // ====== INICIALIZACIÓN ======
    ESP_ERROR_CHECK(i2c_manager_init());
    ESP_ERROR_CHECK(mpu6050_init());
    ESP_ERROR_CHECK(mpu6050_who_am_i(&id));

    if (id != 0x68)
    {
        printf("MPU6050_ERROR\n");
        while (1) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    ESP_ERROR_CHECK(imu_init());
    ESP_ERROR_CHECK(brushless_init(ESC_GPIO_PIN_1, ESC_GPIO_PIN_2));
    brushless_arm();
    
    // Crear tarea para leer la consola
    xTaskCreate(console_task, "Console", 4096, NULL, 1, NULL);
    
    // ====== INICIAR CARACTERIZACIÓN EN ZONA DE INTERÉS ======
    // Parámetros:
    // - PWM Maestro: 55% → 70%
    // - Paso: 0.2% (más fino)
    // - Delay: 500ms (tiempo para estabilizar)
    brushless_start_sweep_char(55.0f, 70.0f, 0.2f, 500);

    while (1)
    {
        vTaskDelay(portMAX_DELAY);
    }
}