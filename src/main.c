#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "system_config.h"
#include "i2c_manager.h"
#include "mpu6050.h"
#include "brushless.h"
#include "controller.h"

// ============================================================
//  ESTRUCTURA PARA DATOS SINCRONIZADOS
// ============================================================
typedef struct
{
    float angle;
    float error;
    float gyro;
    float correction;
    float raw_correction;
    float deadband_gain;
    uint16_t pwm1;
    uint16_t pwm2;
    bool motors_running;
    bool deadband_active;
} debug_data_t;

// ====== VARIABLES ======
static debug_data_t s_debug_data = {0};
static bool s_motors_running = true;

// ============================================================
//  TAREA DE CONSOLA
// ============================================================
static void console_task(void *pvParameters)
{
    char input[16];
    
    printf("\n📌 COMANDOS DISPONIBLES:\n");
    printf("   'x'  = Detener motores (PWM = 1000µs)\n");
    printf("   'r'  = Reanudar motores (PWM base)\n");
    printf("   's'  = Mostrar estado actual\n");
    printf("   'h'  = Mostrar esta ayuda\n\n");
    
    while (1)
    {
        if (fgets(input, sizeof(input), stdin) != NULL)
        {
            input[strcspn(input, "\n")] = '\0';
            char cmd = input[0];
            
            switch (cmd)
            {
                case 'x':
                case 'X':
                    if (s_motors_running)
                    {
                        brushless_set_both_us(ESC_MIN_US, ESC_MIN_US);
                        s_motors_running = false;
                        s_debug_data.motors_running = false;
                        printf("⏹️  Motores DETENIDOS\n");
                        printf("   Presiona 'r' para reanudar\n\n");
                    }
                    else
                    {
                        printf("⚠️  Los motores ya están detenidos\n\n");
                    }
                    break;
                    
                case 'r':
                case 'R':
                    if (!s_motors_running)
                    {
                        brushless_set_both_us(s_debug_data.pwm1, s_debug_data.pwm2);
                        s_motors_running = true;
                        s_debug_data.motors_running = true;
                        printf("▶️  Motores REANUDADOS\n");
                        printf("   M1: %d µs | M2: %d µs\n\n", s_debug_data.pwm1, s_debug_data.pwm2);
                    }
                    else
                    {
                        printf("⚠️  Los motores ya están corriendo\n\n");
                    }
                    break;
                    
                case 's':
                case 'S':
                {
                    debug_data_t local_copy = s_debug_data;
                    
                    printf("\n📊 ESTADO ACTUAL:\n");
                    printf("   Roll: %.2f°\n", local_copy.angle);
                    printf("   Error: %.2f°\n", local_copy.error);
                    printf("   Velocidad Angular: %.2f °/s\n", local_copy.gyro);
                    printf("   Corrección cruda: %.2f µs\n", local_copy.raw_correction);
                    printf("   Ganancia aplicada: %.0f%%\n", local_copy.deadband_gain * 100.0f);
                    printf("   Corrección final: %.2f µs\n", local_copy.correction);
                    printf("   Banda muerta activa: %s\n", local_copy.deadband_active ? "✅ SÍ (ganancia reducida)" : "❌ NO (ganancia completa)");
                    printf("   Motor 1: %d µs\n", local_copy.pwm1);
                    printf("   Motor 2: %d µs\n", local_copy.pwm2);
                    printf("   Estado: %s\n", local_copy.motors_running ? "🟢 CORRIENDO" : "🔴 DETENIDO");
                    printf("   PWM Base M1: %d µs\n", PWM_BASE_M1);
                    printf("   PWM Base M2: %d µs\n", PWM_BASE_M2);
                    printf("   Kp=%.2f, Ki=%.2f, Kd=%.2f\n\n",
                           controller_get_kp(), controller_get_ki(), controller_get_kd());
                    break;
                }
                    
                case 'h':
                case 'H':
                    printf("\n📌 COMANDOS DISPONIBLES:\n");
                    printf("   'x'  = Detener motores\n");
                    printf("   'r'  = Reanudar motores\n");
                    printf("   's'  = Mostrar estado\n");
                    printf("   'h'  = Ayuda\n\n");
                    break;
                    
                default:
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================
//  TAREA DE CONTROL
// ============================================================
static void control_task(void *pvParameters)
{
    mpu6050_data_t imu_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    float correction;
    uint16_t pwm1, pwm2;

    while (1)
    {
        if (mpu6050_read(&imu_data) == ESP_OK) {
            
            controller_set_gyro_rate(imu_data.gyro_x_dps);
            
            correction = controller_update(imu_data.roll);
            
            pwm1 = controller_get_pwm_motor1(correction);
            pwm2 = controller_get_pwm_motor2(correction);
            
            if (s_motors_running) {
                brushless_set_both_us(pwm1, pwm2);
            }
            
            // ACTUALIZAR ESTRUCTURA COMPLETA
            s_debug_data.angle = imu_data.roll;
            s_debug_data.error = controller_get_setpoint() - imu_data.roll;
            s_debug_data.gyro = imu_data.gyro_x_dps;
            s_debug_data.correction = correction;
            s_debug_data.raw_correction = controller_get_raw_correction();
            s_debug_data.deadband_gain = controller_get_deadband_gain();
            s_debug_data.pwm1 = pwm1;
            s_debug_data.pwm2 = pwm2;
            s_debug_data.motors_running = s_motors_running;
            s_debug_data.deadband_active = controller_is_in_deadband();
            
        } else {
            ESP_LOGE("MAIN", "Error leyendo MPU6050");
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

// ============================================================
//  TAREA DE MONITOREO
// ============================================================
static void monitor_task(void *pvParameters)
{
    const TickType_t period = pdMS_TO_TICKS(MONITOR_PERIOD_MS);
    debug_data_t local_data;
    float debug_p, debug_i, debug_d, debug_correction, debug_angle;

    while (1)
    {
        local_data = s_debug_data;
        
        controller_get_debug_values(&debug_angle, NULL, &debug_p, &debug_i, &debug_d, 
                                    &debug_correction, NULL, NULL);
        
        if (fabsf(local_data.angle) < DEBUG_ANGLE_THRESHOLD)
        {
            printf("\n");
            printf("╔══════════════════════════════════════════════════════════════════╗\n");
            printf("║  📊 DEPURACIÓN DEL CONTROLADOR (|Roll| < %.1f°)                  ║\n", DEBUG_ANGLE_THRESHOLD);
            printf("╠══════════════════════════════════════════════════════════════════╣\n");
            printf("║  Ángulo:      %8.2f°                                            ║\n", local_data.angle);
            printf("║  Error:       %8.2f°                                            ║\n", local_data.error);
            printf("║  Setpoint:    %8.2f°                                            ║\n", controller_get_setpoint());
            printf("║  Vel Angular: %8.2f °/s                                         ║\n", local_data.gyro);
            printf("╠══════════════════════════════════════════════════════════════════╣\n");
            printf("║  TÉRMINOS DEL PID:                                              ║\n");
            printf("║    P = Kp × Error      = %8.2f  (Kp=%.2f)                      ║\n", debug_p, KP_INITIAL);
            printf("║    I = Ki × Integral   = %8.2f  (Ki=%.2f)                      ║\n", debug_i, KI_INITIAL);
            printf("║    D = Kd × VelAngular = %8.2f  (Kd=%.2f)                      ║\n", debug_d, KD_INITIAL);
            printf("╠══════════════════════════════════════════════════════════════════╣\n");
            printf("║  Corrección cruda:     %8.2f µs                                  ║\n", local_data.raw_correction);
            printf("║  Ganancia en banda:    %8.0f%%                                   ║\n", local_data.deadband_gain * 100.0f);
            printf("║  Banda muerta activa:  %s                                  ║\n", 
                   local_data.deadband_active ? "✅ SÍ (25%)" : "❌ NO (100%)");
            printf("║  Corrección final:     %8.2f µs                                  ║\n", local_data.correction);
            printf("╠══════════════════════════════════════════════════════════════════╣\n");
            printf("║  PWM M1 (Base+Corr):  %8d µs                                   ║\n", local_data.pwm1);
            printf("║  PWM M2 (Base-Corr):  %8d µs                                   ║\n", local_data.pwm2);
            printf("║  Diferencia M1-M2:    %8d µs                                   ║\n", 
                   abs((int)local_data.pwm1 - (int)local_data.pwm2));
            printf("╚══════════════════════════════════════════════════════════════════╝\n");
            
            if (local_data.deadband_active) {
                printf("  🟢 BANDA MUERTA ACTIVA: Ganancia reducida al %.0f%%\n", 
                       local_data.deadband_gain * 100.0f);
                printf("     → Corrección suave para evitar micro-oscilaciones\n");
            } else if (fabsf(local_data.angle) < 1.0f) {
                printf("  🟡 Dentro de ±1° pero ganancia completa (transición)\n");
            } else if (fabsf(local_data.angle) < 3.0f) {
                printf("  ⚠️  Desviándose ligeramente (%.1f°)\n", local_data.angle);
            } else {
                printf("  🔴 La barra está cayendo (%.1f°)\n", local_data.angle);
            }
            
            printf("\n");
        }
        else
        {
            printf("Roll: %6.2f° | Error: %6.2f° | Gyro: %6.2f°/s | Corr: %6.2fµs | Gan: %3.0f%% | M1: %4dµs | M2: %4dµs | %s\n",
                   local_data.angle, local_data.error, local_data.gyro, 
                   local_data.correction,
                   local_data.deadband_gain * 100.0f,
                   local_data.pwm1, local_data.pwm2,
                   local_data.motors_running ? "🟢" : "🔴");
        }

        vTaskDelay(period);
    }
}

// ============================================================
//  MAIN
// ============================================================
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║   PID + BANDA MUERTA SUAVE (±1°)                        ║\n");
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
    printf("📌 Inicializando controlador PID con banda muerta suave...\n");
    controller_init();
    controller_set_setpoint(SETPOINT_DEFAULT);

    printf("\n📊 CONFIGURACIÓN DEL CONTROLADOR:\n");
    printf("   PWM Base M1: %d µs\n", PWM_BASE_M1);
    printf("   PWM Base M2: %d µs\n", PWM_BASE_M2);
    printf("   Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
           controller_get_kp(), controller_get_ki(), controller_get_kd());
    printf("   Banda muerta: ±%.1f° (ganancia al %.0f%%)\n", 1.0f, 25.0f);
    printf("   Slew Rate: %.0f µs/ciclo\n", (float)SLEW_RATE);
    printf("   Setpoint: %.1f°\n", SETPOINT_DEFAULT);
    printf("   ✅ Derivada desde GIROSCOPIO\n\n");

    // ====== 6. CREAR TAREAS ======
    printf("📌 Creando tareas...\n");

    xTaskCreate(console_task, "Console", CONSOLE_TASK_STACK, NULL,
                CONSOLE_TASK_PRIORITY, NULL);

    xTaskCreate(control_task, "Control", CONTROL_TASK_STACK, NULL,
                CONTROL_TASK_PRIORITY, NULL);

    xTaskCreate(monitor_task, "Monitor", MONITOR_TASK_STACK, NULL,
                MONITOR_TASK_PRIORITY, NULL);

    printf("   ✅ Tareas creadas\n\n");

    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║  📊 BANDA MUERTA SUAVE: ±1°                             ║\n");
    printf("║  Cuando |Roll| < 1° → ganancia reducida al 25%%         ║\n");
    printf("║  Cuando |Roll| ≥ 1° → ganancia completa (100%%)         ║\n");
    printf("║  El PID SIEMPRE calcula, nunca se apaga                 ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n\n");

    vTaskDelete(NULL);
}