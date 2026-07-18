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

// ====== CONFIGURACIÓN PWM ======
#define PWM_FREQUENCY       50
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_MODE            LEDC_LOW_SPEED_MODE
#define PWM_RESOLUTION      LEDC_TIMER_16_BIT
#define PWM_DUTY_MAX        65535

#define PWM_CHANNEL_1       LEDC_CHANNEL_0
#define PWM_CHANNEL_2       LEDC_CHANNEL_1

// ====== VARIABLES ESTÁTICAS ======
static int motor1_gpio = -1;
static int motor2_gpio = -1;
static bool is_armed = false;

// ====== FUNCIÓN INTERNA: Calcular duty ======
static uint32_t us_to_duty(uint16_t micros)
{
    return (uint32_t)((micros / 20000.0f) * PWM_DUTY_MAX);
}

// ====== FUNCIÓN INTERNA: Enviar pulso PWM ======
static void pwm_set_pulse(int gpio, int channel, uint16_t micros)
{
    if (gpio < 0) return;
    if (micros < 1000) micros = 1000;
    if (micros > 2000) micros = 2000;
    
    uint32_t duty = us_to_duty(micros);
    ledc_set_duty(PWM_MODE, channel, duty);
    ledc_update_duty(PWM_MODE, channel);
}

// ====== INICIALIZACIÓN ======
esp_err_t brushless_init(int gpio_pin_1, int gpio_pin_2)
{
    if (gpio_pin_1 < 0 || gpio_pin_2 < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    motor1_gpio = gpio_pin_1;
    motor2_gpio = gpio_pin_2;
    
    ledc_timer_config_t timer_config = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
    
    ledc_channel_config_t channel_config_1 = {
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL_1,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor1_gpio,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config_1));
    
    ledc_channel_config_t channel_config_2 = {
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL_2,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = motor2_gpio,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config_2));
    
    printf("PWM inicializado\n");
    printf("Motor 1: GPIO %d\n", motor1_gpio);
    printf("Motor 2: GPIO %d\n", motor2_gpio);
    
    return ESP_OK;
}

// =====================================================
// ARMADO - SOLO ARMA, NO CALIBRA
// =====================================================
esp_err_t brushless_arm(void)
{
    if (motor1_gpio < 0 || motor2_gpio < 0) {
        return ESP_ERR_INVALID_STATE;
    }
    
    printf("\n============================================================\n");
    printf(" ARMANDO ESC (USANDO CALIBRACIÓN PREVIA)\n");
    printf("============================================================\n");
    printf(" Los ESC ya están calibrados, solo se arman\n");
    printf("============================================================\n\n");
    
    // Enviar señal de armado (1000µs) por 2 segundos
    printf(" Enviando señal de ARMADO (1000 us)...\n");
    pwm_set_pulse(motor1_gpio, PWM_CHANNEL_1, 1000);
    pwm_set_pulse(motor2_gpio, PWM_CHANNEL_2, 1000);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    printf("\n============================================================\n");
    printf(" ESC ARMADOS CORRECTAMENTE\n");
    printf("============================================================\n\n");
    
    is_armed = true;
    return ESP_OK;
}

// =====================================================
// FUNCIONES DE CONTROL
// =====================================================
void brushless_set_motor1_us(uint16_t us)
{
    if (motor1_gpio < 0) return;
    pwm_set_pulse(motor1_gpio, PWM_CHANNEL_1, us);
}

void brushless_set_motor2_us(uint16_t us)
{
    if (motor2_gpio < 0) return;
    pwm_set_pulse(motor2_gpio, PWM_CHANNEL_2, us);
}

void brushless_set_both_us(uint16_t motor1_us, uint16_t motor2_us)
{
    brushless_set_motor1_us(motor1_us);
    brushless_set_motor2_us(motor2_us);
}

uint16_t brushless_get_motor1_us(void)
{
    return 1300;
}

uint16_t brushless_get_motor2_us(void)
{
    return 1300;
}

uint16_t brushless_percent_to_us(float percent)
{
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    return (uint16_t)(1000 + (percent / 100.0f) * 1000);
}

float brushless_us_to_percent(uint16_t us)
{
    if (us <= 1000) return 0.0f;
    if (us >= 2000) return 100.0f;
    return ((float)(us - 1000) / 1000.0f) * 100.0f;
}

void brushless_stop(void)
{
    if (is_armed) {
        pwm_set_pulse(motor1_gpio, PWM_CHANNEL_1, 1000);
        pwm_set_pulse(motor2_gpio, PWM_CHANNEL_2, 1000);
    }
}

void brushless_emergency_stop(void)
{
    brushless_stop();
}

bool brushless_is_armed(void)
{
    return is_armed;
}