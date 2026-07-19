#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "system_config.h"
#include "i2c_manager.h"
#include "mpu6050.h"
#include "brushless.h"
#include "controller.h"

// ====== VARIABLES ESTÁTICAS (SOLO PARA ESTE MÓDULO) ======
static float s_current_roll = 0.0f;
static float s_correction = 0.0f;
static uint16_t s_motor1_us = PWM_BASE_M1;
static uint16_t s_motor2_us = PWM_BASE_M2;

// ====== TAREA DE CONTROL ======
static void control_task(void *pvParameters)
{
    mpu6050_data_t imu_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    while (1)
    {
        if (mpu6050_read(&imu_data) == ESP_OK) {
            s_current_roll = imu_data.roll;
        } else {
            ESP_LOGE("MAIN", "Error leyendo MPU6050");
            vTaskDelay(period);
            continue;
        }

        s_correction = controller_update(s_current_roll);

        s_motor1_us = controller_get_pwm_motor1(s_correction);
        s_motor2_us = controller_get_pwm_motor2(s_correction);

        brushless_set_both_us(s_motor1_us, s_motor2_us);

        vTaskDelayUntil(&last_wake_time, period);
    }
}

// ====== TAREA DE MONITOREO ======
static void monitor_task(void *pvParameters)
{
    const TickType_t period = pdMS_TO_TICKS(MONITOR_PERIOD_MS);

    while (1)
    {
        printf("Roll: %6.2f° | Corr: %6.2fµs | M1: %4dµs | M2: %4dµs\n",
               s_current_roll, s_correction, s_motor1_us, s_motor2_us);

        vTaskDelay(period);
    }
}

// ====== MAIN ======
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║      SISTEMA DE CONTROL - PWM BASE INDIVIDUAL           ║\n");
    printf("║           PD + KI = 0 (SINTONIZACIÓN INICIAL)           ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n\n");

    // ====== 1. INICIALIZAR I2C ======
    printf("📌 Inicializando I2C...\n");
    ESP_ERROR_CHECK(i2c_manager_init());
    printf("   ✅ I2C OK\n\n");

    // ====== 2. INICIALIZAR MPU6050 ======
    printf("📌 Inicializando MPU6050...\n");
    ESP_ERROR_CHECK(mpu6050_init());

    uint8_t id = 0;
    ESP_ERROR_CHECK(mpu6050_who_am_i(&id));
    printf("   ✅ WHO_AM_I: 0x%02X %s\n\n", id, (id == 0x68) ? "(DETECTADO)" : "⚠️ ID inesperado!");

    // ====== 3. INICIALIZAR MOTORES ======
    printf("📌 Inicializando motores brushless...\n");
    ESP_ERROR_CHECK(brushless_init(MOTOR1_GPIO, MOTOR2_GPIO));
    printf("   ✅ Motor1: GPIO%d, Motor2: GPIO%d\n", MOTOR1_GPIO, MOTOR2_GPIO);

    // ====== 4. ARMAR ESC ======
    printf("📌 Armando ESC...\n");
    ESP_ERROR_CHECK(brushless_arm());
    printf("   ✅ Motores armados\n\n");

    // ====== 5. INICIALIZAR CONTROLADOR ======
    printf("📌 Inicializando controlador PID...\n");
    controller_init();
    controller_set_setpoint(0.0f);

    printf("\n📊 CONFIGURACIÓN DEL CONTROLADOR:\n");
    printf("   PWM Base M1: %d µs\n", PWM_BASE_M1);
    printf("   PWM Base M2: %d µs\n", PWM_BASE_M2);
    printf("   Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
           controller_get_kp(), controller_get_ki(), controller_get_kd());
    printf("   Slew Rate: %.0f µs/ciclo\n", (float)SLEW_RATE);
    printf("   Tolerancia: ±%.1f°\n\n", ERROR_TOLERANCE);

    // ====== 6. CREAR TAREAS ======
    printf("📌 Creando tareas...\n");

    xTaskCreate(control_task, "Control", CONTROL_TASK_STACK, NULL,
                CONTROL_TASK_PRIORITY, NULL);

    xTaskCreate(monitor_task, "Monitor", MONITOR_TASK_STACK, NULL,
                MONITOR_TASK_PRIORITY, NULL);

    printf("   ✅ Tareas creadas\n\n");

    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║           SISTEMA EN EJECUCIÓN                          ║\n");
    printf("║   Roll  |  Corr   |  M1    |  M2                         ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n");

    vTaskDelete(NULL);
}