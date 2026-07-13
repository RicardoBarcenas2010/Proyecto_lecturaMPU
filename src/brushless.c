#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"

#include "brushless.h"
#include "imu.h"

// ====== CONFIGURACIÓN PWM ======
#define PWM_FREQUENCY     50

// Motor 1 (Maestro - GPIO 13)
#define PWM_TIMER_1       LEDC_TIMER_0
#define PWM_CHANNEL_1     LEDC_CHANNEL_0
#define PWM_MODE_1        LEDC_LOW_SPEED_MODE
#define PWM_RESOLUTION_1  LEDC_TIMER_16_BIT

// Motor 2 (Esclavo - GPIO 14)
#define PWM_TIMER_2       LEDC_TIMER_0
#define PWM_CHANNEL_2     LEDC_CHANNEL_1
#define PWM_MODE_2        LEDC_LOW_SPEED_MODE
#define PWM_RESOLUTION_2  LEDC_TIMER_16_BIT

// Rango de pulsos en microsegundos
#define PULSE_STOP        1000
#define PULSE_ARM         1100
#define PULSE_MIN         1200
#define PULSE_MAX         2000

// Variables estáticas
static int motor1_gpio = -1;
static int motor2_gpio = -1;
static float current_pwm_1 = 0.0f;
static float current_pwm_2 = 0.0f;
static bool is_armed = false;
static bool sweep_running = false;
static TaskHandle_t sweep_task_handle = NULL;

// Función interna para convertir porcentaje a microsegundos
static int pwm_percent_to_micros(float percent)
{
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    int micros = PULSE_MIN + (int)((percent / 100.0f) * (PULSE_MAX - PULSE_MIN));
    return micros;
}

// Función para configurar PWM en un motor específico
static void brushless_set_microseconds(int gpio, int timer, int channel, int mode, int micros)
{
    if (gpio < 0) return;
    
    uint32_t duty = (uint32_t)((micros / 20000.0f) * 65535);
    
    ESP_ERROR_CHECK(ledc_set_duty(mode, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(mode, channel));
}

// Inicialización
esp_err_t brushless_init(int gpio_pin_1, int gpio_pin_2)
{
    if (gpio_pin_1 < 0 || gpio_pin_2 < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    motor1_gpio = gpio_pin_1;
    motor2_gpio = gpio_pin_2;
    
    ledc_timer_config_t timer_config = {
        .speed_mode = PWM_MODE_1,
        .duty_resolution = PWM_RESOLUTION_1,
        .timer_num = PWM_TIMER_1,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
    
    ledc_channel_config_t channel_config_1 = {
        .speed_mode = PWM_MODE_1,
        .channel = PWM_CHANNEL_1,
        .timer_sel = PWM_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor1_gpio,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config_1));
    
    ledc_channel_config_t channel_config_2 = {
        .speed_mode = PWM_MODE_2,
        .channel = PWM_CHANNEL_2,
        .timer_sel = PWM_TIMER_2,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor2_gpio,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config_2));
    
    return ESP_OK;
}

// Armado
esp_err_t brushless_arm(void)
{
    if (motor1_gpio < 0 || motor2_gpio < 0) {
        return ESP_ERR_INVALID_STATE;
    }
    
    for (int i = 5; i > 0; i--) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    brushless_set_microseconds(motor1_gpio, PWM_TIMER_1, PWM_CHANNEL_1, PWM_MODE_1, PULSE_STOP);
    brushless_set_microseconds(motor2_gpio, PWM_TIMER_2, PWM_CHANNEL_2, PWM_MODE_2, PULSE_STOP);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    brushless_set_microseconds(motor1_gpio, PWM_TIMER_1, PWM_CHANNEL_1, PWM_MODE_1, PULSE_ARM);
    brushless_set_microseconds(motor2_gpio, PWM_TIMER_2, PWM_CHANNEL_2, PWM_MODE_2, PULSE_ARM);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    is_armed = true;
    current_pwm_1 = 0.0f;
    current_pwm_2 = 0.0f;
    
    return ESP_OK;
}

// Establecer PWM Motor 1
void brushless_set_pwm_motor1(float percent)
{
    if (!is_armed) return;
    
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    current_pwm_1 = percent;
    int micros = pwm_percent_to_micros(percent);
    brushless_set_microseconds(motor1_gpio, PWM_TIMER_1, PWM_CHANNEL_1, PWM_MODE_1, micros);
}

// Establecer PWM Motor 2
void brushless_set_pwm_motor2(float percent)
{
    if (!is_armed) return;
    
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    
    current_pwm_2 = percent;
    int micros = pwm_percent_to_micros(percent);
    brushless_set_microseconds(motor2_gpio, PWM_TIMER_2, PWM_CHANNEL_2, PWM_MODE_2, micros);
}

// Obtener PWM actual del Motor 1
float brushless_get_pwm_motor1(void)
{
    return current_pwm_1;
}

// Obtener PWM actual del Motor 2
float brushless_get_pwm_motor2(void)
{
    return current_pwm_2;
}

// Tarea de caracterización - SOLO ZONA DE INTERÉS
static void sweep_char_task(void *pvParameters)
{
    (void)pvParameters;
    
    float *params = (float *)pvParameters;
    float pwm_start = params[0];
    float pwm_end = params[1];
    float pwm_step = params[2];
    int delay_ms = (int)params[3];
    
    imu_data_t imu_data;
    float angle = 0.0f;
    float angle_filtered = 0.0f;
    float pwm_master = pwm_start;
    float pwm_slave = 0.0f;
    const float alpha = 0.3f;
    
    // Desactivar impresión del IMU
    imu_set_print_enabled(false);
    
    // Posición inicial
    brushless_set_pwm_motor1(pwm_start);
    brushless_set_pwm_motor2(100.0f - pwm_start);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Leer ángulo inicial
    if (imu_get_data(&imu_data) == ESP_OK) {
        angle = (float)imu_data.roll_mapped * 45.0f / 128.0f;
        angle_filtered = angle;
    }
    
    // ====== SOLO DATOS NUMÉRICOS PARA MATLAB ======
    // Encabezado (comentario para MATLAB)
    printf("%% PWM_MASTER  PWM_SLAVE  ANGLE\n");
    
    // Primer punto
    printf("%.2f       %.2f       %.2f\n", pwm_start, 100.0f - pwm_start, angle_filtered);
    
    // Barrido de PWM en zona de interés
    while (pwm_master <= pwm_end && sweep_running) {
        // Avanzar al siguiente paso
        pwm_master += pwm_step;
        if (pwm_master > pwm_end) break;
        
        // Calcular PWM esclavo (complementario)
        pwm_slave = 100.0f - pwm_master;
        if (pwm_slave < 0.0f) pwm_slave = 0.0f;
        if (pwm_slave > 100.0f) pwm_slave = 100.0f;
        
        // Aplicar PWM
        brushless_set_pwm_motor1(pwm_master);
        brushless_set_pwm_motor2(pwm_slave);
        
        // Esperar a que se estabilice
        for (int i = 0; i < delay_ms / 50 && sweep_running; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        if (!sweep_running) break;
        
        // Leer ángulo con filtro
        if (imu_get_data(&imu_data) == ESP_OK) {
            angle = (float)imu_data.roll_mapped * 45.0f / 128.0f;
            angle_filtered = angle_filtered + alpha * (angle - angle_filtered);
        }
        
        // ====== IMPRIMIR SOLO DATOS ======
        printf("%.2f       %.2f       %.2f\n", pwm_master, pwm_slave, angle_filtered);
    }
    
    // Detener motores
    brushless_stop();
    imu_set_print_enabled(true);
    
    // ====== INDICADOR FINAL (comentario para MATLAB) =====
    printf("%% FIN\n");
    
    sweep_running = false;
    sweep_task_handle = NULL;
    free(params);
    vTaskDelete(NULL);
}

// Iniciar caracterización
void brushless_start_sweep_char(float pwm_start, float pwm_end, float pwm_step, int delay_ms)
{
    if (!is_armed) {
        printf("ERROR: ESC no armado\n");
        return;
    }
    
    if (sweep_running) {
        printf("ERROR: Caracterización ya en curso\n");
        return;
    }
    
    sweep_running = true;
    
    float *params = malloc(4 * sizeof(float));
    params[0] = pwm_start;
    params[1] = pwm_end;
    params[2] = pwm_step;
    params[3] = (float)delay_ms;
    
    xTaskCreate(sweep_char_task, "SweepChar", 4096, params, 5, &sweep_task_handle);
}

// Detener caracterización
void brushless_stop_sweep_char(void)
{
    if (sweep_running) {
        sweep_running = false;
        if (sweep_task_handle != NULL) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// Verificar si caracterización está en curso
bool brushless_is_sweep_running(void)
{
    return sweep_running;
}

// Detener ambos motores
void brushless_stop(void)
{
    if (is_armed) {
        brushless_set_microseconds(motor1_gpio, PWM_TIMER_1, PWM_CHANNEL_1, PWM_MODE_1, PULSE_STOP);
        brushless_set_microseconds(motor2_gpio, PWM_TIMER_2, PWM_CHANNEL_2, PWM_MODE_2, PULSE_STOP);
        current_pwm_1 = 0.0f;
        current_pwm_2 = 0.0f;
    }
}