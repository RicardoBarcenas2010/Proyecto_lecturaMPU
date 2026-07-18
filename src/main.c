#include <math.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#include "driver/uart.h"

#include "system_config.h"
#include "i2c_manager.h"
#include "mpu6050.h"
#include "imu.h"
#include "brushless.h"
#include "controller.h"

#define ESC_GPIO_PIN_1 GPIO_NUM_13
#define ESC_GPIO_PIN_2 GPIO_NUM_14

// ============================================
// CONFIGURACIÓN - THROTTLE_BASE = 17%
// ============================================
#define THROTTLE_BASE      17.0f   // VALOR ÓPTIMO ENCONTRADO
#define PWM_MIN_OUT        12.0f   // Mínimo absoluto
#define PWM_MAX_OUT        30.0f   // Máximo seguro

#define PWM_STEP           0.2f
#define PID_FILTER         0.35f
#define MAX_ANGLE_DEG      45.0f
#define EQUILIBRIUM_ANGLE  177.6f
#define MAX_CORRECTION     8.0f

#define PRINT_INTERVAL     10

// ============================================
// VARIABLES GLOBALES
// ============================================
static bool emergency_stop = false;
static bool system_running = false;
static float current_setpoint = 0.0f;

// ============================================
// FUNCIÓN PARA LEER CARÁCTER DEL UART
// ============================================
char uart_read_char(void)
{
    char c = 0;
    uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(10));
    return c;
}

// ============================================
// FUNCIÓN DE ARMADO
// ============================================
void arm_motors(void)
{
    printf("\n============================================\n");
    printf(" ARMANDO MOTORES\n");
    printf("============================================\n");
    printf(" Enviando señal de armado (1000 us)...\n");
    
    brushless_set_motor1_us(1000);
    brushless_set_motor2_us(1000);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    printf(" Motores armados\n");
    printf("============================================\n\n");
}

// ============================================
// FUNCIÓN DE RAMPA SUAVE
// ============================================
void ramp_motors_smooth(uint16_t target_us)
{
    printf(" Rampa desde 1000 us hasta %d us...\n", target_us);
    
    for(uint16_t us = 1000; us <= target_us; us += 2) {
        brushless_set_motor1_us(us);
        brushless_set_motor2_us(us);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    
    printf(" Rampa completada en %d us\n", target_us);
}

// ============================================
// FUNCIÓN PARA MOSTRAR AYUDA
// ============================================
void show_help(void)
{
    printf("\n--- COMANDOS ---\n");
    printf("[X] EMERGENCIA\n");
    printf("[R] REANUDAR\n");
    printf("[S] ESTADO\n");
    printf("[0] SETPOINT 0°\n");
    printf("[1] SETPOINT 10°\n");
    printf("[2] SETPOINT 20°\n");
    printf("[3] SETPOINT 30°\n");
    printf("[4] SETPOINT 40°\n");
    printf("[-] SETPOINT -10°\n");
    printf("[=] SETPOINT -20°\n");
    printf("[A] RESETEAR PID\n");
    printf("[H] AYUDA\n");
    printf("-----------------\n");
}

// ============================================
// TAREA DE CONTROL CON PID
// ============================================
static void control_task(void *pvParameters)
{
    imu_data_t imu_data;
    
    float angle = 0.0f;
    float setpoint = 0.0f;
    float error = 0.0f;
    float correction = 0.0f;
    float correction_filtered = 0.0f;
    float motor1 = THROTTLE_BASE;
    float motor2 = THROTTLE_BASE;
    float motor1_cmd = THROTTLE_BASE;
    float motor2_cmd = THROTTLE_BASE;
    uint16_t motor1_us = 0;
    uint16_t motor2_us = 0;
    uint32_t loop_counter = 0;
    int consecutive_errors = 0;
    char input_char = 0;
    int anti_windup_counter = 0;
    
    // Inicializar PID
    controller_init();
    current_setpoint = 0.0f;
    controller_set_setpoint(current_setpoint);
    
    // Configurar IMU
    imu_set_equilibrium_angle(EQUILIBRIUM_ANGLE);
    imu_set_max_angle(MAX_ANGLE_DEG);
    imu_set_print_enabled(false);
    
    printf("\n============================================\n");
    printf(" CONTROL PID ACTIVADO\n");
    printf("============================================\n");
    printf(" THROTTLE_BASE: %.1f%% (%d us)\n", THROTTLE_BASE, brushless_percent_to_us(THROTTLE_BASE));
    printf(" Setpoint actual: %.1f°\n", current_setpoint);
    printf(" Corrección máxima: ±%.1f°\n", MAX_CORRECTION);
    printf("============================================\n");
    printf(" Ganancias PID:\n");
    printf("   KP: %.1f\n", KP_INITIAL);
    printf("   KI: %.1f\n", KI_INITIAL);
    printf("   KD: %.2f\n", KD_INITIAL);
    printf("============================================\n\n");
    
    printf("COMANDOS:\n");
    printf("  [0-4] Setpoint positivo (0° a 40°)\n");
    printf("  [-=]  Setpoint negativo (-10° a -20°)\n");
    printf("  [X]   EMERGENCIA\n");
    printf("  [R]   REANUDAR\n");
    printf("  [A]   RESETEAR PID\n");
    printf("  [S]   ESTADO\n");
    printf("  [H]   AYUDA\n");
    printf("\n============================================\n");
    printf(" SISTEMA EN MARCHA\n");
    printf(" SETP  ANG   ERR   CORR   M1   M2\n");
    
    system_running = true;
    TickType_t last_wake_time = xTaskGetTickCount();

    while(1)
    {
        input_char = uart_read_char();
        if(input_char != 0)
        {
            if(input_char >= 'a' && input_char <= 'z') {
                input_char = input_char - 'a' + 'A';
            }
            
            switch(input_char)
            {
                case 'X':
                    printf("\n");
                    printf("============================================\n");
                    printf(" !!! EMERGENCIA ACTIVADA !!!\n");
                    printf("============================================\n\n");
                    brushless_stop();
                    emergency_stop = true;
                    system_running = false;
                    break;
                    
                case 'R':
                    if(emergency_stop) {
                        printf("\n============================================\n");
                        printf(" REANUDANDO OPERACIÓN\n");
                        printf("============================================\n");
                        
                        arm_motors();
                        
                        uint16_t target_us = brushless_percent_to_us(THROTTLE_BASE);
                        ramp_motors_smooth(target_us);
                        
                        controller_reset();
                        motor1_cmd = THROTTLE_BASE;
                        motor2_cmd = THROTTLE_BASE;
                        correction_filtered = 0.0f;
                        
                        emergency_stop = false;
                        system_running = true;
                        
                        printf(" SISTEMA REANUDADO\n");
                        printf("============================================\n\n");
                        printf(" SETP  ANG   ERR   CORR   M1   M2\n");
                    } else {
                        printf(">>> No hay emergencia activa.\n");
                    }
                    break;
                    
                case '0':
                    current_setpoint = 0.0f;
                    controller_set_setpoint(current_setpoint);
                    controller_reset();
                    printf(">>> Setpoint: 0.0°\n");
                    break;
                    
                case '1':
                    current_setpoint = 10.0f;
                    controller_set_setpoint(current_setpoint);
                    controller_reset();
                    printf(">>> Setpoint: 10.0°\n");
                    break;
                    
                case '2':
                    current_setpoint = 20.0f;
                    controller_set_setpoint(current_setpoint);
                    controller_reset();
                    printf(">>> Setpoint: 20.0°\n");
                    break;
                    
                case '3':
                    current_setpoint = 30.0f;
                    controller_set_setpoint(current_setpoint);
                    controller_reset();
                    printf(">>> Setpoint: 30.0°\n");
                    break;
                    
                case '4':
                    current_setpoint = 40.0f;
                    controller_set_setpoint(current_setpoint);
                    controller_reset();
                    printf(">>> Setpoint: 40.0°\n");
                    break;
                    
                case '-':
                    current_setpoint = -10.0f;
                    controller_set_setpoint(current_setpoint);
                    controller_reset();
                    printf(">>> Setpoint: -10.0°\n");
                    break;
                    
                case '=':
                    current_setpoint = -20.0f;
                    controller_set_setpoint(current_setpoint);
                    controller_reset();
                    printf(">>> Setpoint: -20.0°\n");
                    break;
                    
                case 'A':
                    printf(">>> Reseteando PID manualmente\n");
                    controller_reset();
                    correction_filtered = 0.0f;
                    break;
                    
                case 'S':
                    printf("\n--- ESTADO ---\n");
                    printf("Setpoint: %.1f°\n", current_setpoint);
                    printf("Ángulo actual: %.1f°\n", angle);
                    printf("Error: %.1f°\n", error);
                    printf("Corrección: %.1f°\n", correction_filtered);
                    printf("Motor1: %.1f%% (%d us)\n", motor1_cmd, brushless_percent_to_us(motor1_cmd));
                    printf("Motor2: %.1f%% (%d us)\n", motor2_cmd, brushless_percent_to_us(motor2_cmd));
                    printf("Throttle Base: %.1f%%\n", THROTTLE_BASE);
                    printf("--------------\n");
                    break;
                    
                case 'H':
                    show_help();
                    break;
            }
        }

        if(system_running && !emergency_stop)
        {
            esp_err_t err = imu_get_data(&imu_data);
            if(err == ESP_OK)
            {
                consecutive_errors = 0;
                
                if(imu_data.roll_mapped >= -128 && imu_data.roll_mapped <= 128)
                {
                    angle = (float)imu_data.roll_mapped * MAX_ANGLE_DEG / 128.0f;
                    if (isnan(angle) || isinf(angle)) angle = 0.0f;
                }
                else
                {
                    angle = 0.0f;
                }
            }
            else
            {
                consecutive_errors++;
                angle = 0.0f;
                
                if(consecutive_errors > 10) {
                    printf("ERROR: Demasiados errores IMU - Deteniendo motores\n");
                    brushless_stop();
                    emergency_stop = true;
                    system_running = false;
                }
            }

            correction = controller_update(angle);
            if (isnan(correction) || isinf(correction)) correction = 0.0f;
            
            // Anti-windup
            error = current_setpoint - angle;
            if (fabs(error) > 15.0f && fabs(correction) >= MAX_CORRECTION * 0.9f) {
                anti_windup_counter++;
                if (anti_windup_counter > 30) {
                    controller_reset();
                    anti_windup_counter = 0;
                    printf(">>> Anti-windup: Reseteando PID\n");
                }
            } else {
                if (anti_windup_counter > 0) anti_windup_counter--;
            }
            
            // Aplicar límite de corrección
            if(correction > MAX_CORRECTION) correction = MAX_CORRECTION;
            if(correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

            correction_filtered = correction_filtered * (1.0f - PID_FILTER) + correction * PID_FILTER;
            if (isnan(correction_filtered) || isinf(correction_filtered)) correction_filtered = 0.0f;

            // Calcular PWM
            motor1 = THROTTLE_BASE + correction_filtered;
            motor2 = THROTTLE_BASE - correction_filtered;

            if(motor1 < PWM_MIN_OUT) motor1 = PWM_MIN_OUT;
            if(motor2 < PWM_MIN_OUT) motor2 = PWM_MIN_OUT;
            if(motor1 > PWM_MAX_OUT) motor1 = PWM_MAX_OUT;
            if(motor2 > PWM_MAX_OUT) motor2 = PWM_MAX_OUT;

            // Rampa suave
            if(motor1 > motor1_cmd + PWM_STEP) motor1 = motor1_cmd + PWM_STEP;
            if(motor1 < motor1_cmd - PWM_STEP) motor1 = motor1_cmd - PWM_STEP;
            if(motor2 > motor2_cmd + PWM_STEP) motor2 = motor2_cmd + PWM_STEP;
            if(motor2 < motor2_cmd - PWM_STEP) motor2 = motor2_cmd - PWM_STEP;

            motor1_cmd = motor1;
            motor2_cmd = motor2;

            motor1_us = brushless_percent_to_us(motor1_cmd);
            motor2_us = brushless_percent_to_us(motor2_cmd);
            
            if(motor1_us < 1000) motor1_us = 1000;
            if(motor2_us < 1000) motor2_us = 1000;
            if(motor1_us > 2000) motor1_us = 2000;
            if(motor2_us > 2000) motor2_us = 2000;
            
            brushless_set_motor1_us(motor1_us);
            brushless_set_motor2_us(motor2_us);

            // Mostrar datos
            loop_counter++;
            if(loop_counter >= PRINT_INTERVAL)
            {
                loop_counter = 0;
                error = current_setpoint - angle;
                
                printf("%5.1f  %5.1f  %5.1f  %5.1f  %5.1f  %5.1f\n", 
                       current_setpoint, angle, error, correction_filtered, motor1_cmd, motor2_cmd);
            }
        }
        else if(emergency_stop)
        {
            loop_counter++;
            if(loop_counter >= 50)
            {
                loop_counter = 0;
                printf(">>> !!! EMERGENCIA ACTIVA - Presiona [R] para reanudar !!! <<<\n");
            }
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}

// ============================================
// APP MAIN
// ============================================
void app_main(void)
{
    uint8_t id = 0;

    esp_log_level_set("*", ESP_LOG_NONE);
    esp_task_wdt_deinit();

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    printf("\n============================================\n");
    printf(" SISTEMA DE CONTROL CON PID\n");
    printf("============================================\n");
    printf(" THROTTLE_BASE: 17%% (ÓPTIMO)\n");
    printf(" ¡ATENCIÓN! MANTÉN DISTANCIA DE SEGURIDAD\n");
    printf("============================================\n\n");

    printf("Inicializando I2C...\n");
    ESP_ERROR_CHECK(i2c_manager_init());
    
    printf("Inicializando MPU6050...\n");
    ESP_ERROR_CHECK(mpu6050_init());
    
    printf("Verificando MPU6050...\n");
    ESP_ERROR_CHECK(mpu6050_who_am_i(&id));
    if(id != 0x68)
    {
        printf("ERROR: MPU6050 no detectado (ID: 0x%02X)\n", id);
        while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    printf("MPU6050 detectado (ID: 0x%02X)\n", id);

    printf("Inicializando IMU...\n");
    ESP_ERROR_CHECK(imu_init());
    
    printf("Inicializando ESC...\n");
    ESP_ERROR_CHECK(brushless_init(ESC_GPIO_PIN_1, ESC_GPIO_PIN_2));

    // ARMADO
    arm_motors();

    // RAMPA SUAVE A 17%
    uint16_t target_us = brushless_percent_to_us(THROTTLE_BASE);
    ramp_motors_smooth(target_us);

    printf("\n============================================\n");
    printf(" SISTEMA LISTO\n");
    printf("============================================\n");
    printf(" Throttle Base: %.1f%% (%d us)\n", THROTTLE_BASE, target_us);
    printf(" Setpoint actual: 0.0°\n");
    printf("============================================\n");
    printf(" Presiona [0-4] para cambiar setpoint\n");
    printf(" Presiona [X] para EMERGENCIA\n");
    printf(" Presiona [H] para ayuda\n");
    printf("============================================\n\n");

    if (xTaskCreatePinnedToCore(
        control_task,
        "control",
        8192,
        NULL,
        2,
        NULL,
        1) != pdPASS)
    {
        printf("ERROR: No se pudo crear la tarea\n");
        while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}