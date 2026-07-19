#include "brushless.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "system_config.h"

static const char *TAG = "BRUSHLESS";

// ====== CONFIGURACIÓN PWM ======
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_MODE            LEDC_LOW_SPEED_MODE
#define PWM_RESOLUTION      LEDC_TIMER_16_BIT
#define PWM_DUTY_MAX        65535
#define PWM_FREQUENCY       50

#define PWM_CHANNEL_1       LEDC_CHANNEL_0
#define PWM_CHANNEL_2       LEDC_CHANNEL_1

// ====== VARIABLES ESTÁTICAS ======
static int s_motor1_gpio = -1;
static int s_motor2_gpio = -1;
static bool s_is_armed = false;
static bool s_is_calibrated = false;

// ====== FUNCIONES INTERNAS ======

static uint32_t us_to_duty(uint16_t micros)
{
    return (uint32_t)((micros / 20000.0f) * PWM_DUTY_MAX);
}

static void pwm_set_pulse(int gpio, int channel, uint16_t micros)
{
    if (gpio < 0) return;
    
    if (micros < ESC_MIN_US) micros = ESC_MIN_US;
    if (micros > ESC_MAX_US) micros = ESC_MAX_US;
    
    uint32_t duty = us_to_duty(micros);
    ledc_set_duty(PWM_MODE, channel, duty);
    ledc_update_duty(PWM_MODE, channel);
}

// ====== FUNCIONES PÚBLICAS ======

esp_err_t brushless_init(int gpio_pin_1, int gpio_pin_2)
{
    if (gpio_pin_1 < 0 || gpio_pin_2 < 0) {
        ESP_LOGE(TAG, "GPIO inválido");
        return ESP_ERR_INVALID_ARG;
    }
    
    s_motor1_gpio = gpio_pin_1;
    s_motor2_gpio = gpio_pin_2;
    
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
        .gpio_num = s_motor1_gpio,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config_1));
    
    ledc_channel_config_t channel_config_2 = {
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL_2,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = s_motor2_gpio,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config_2));
    
    // Enviar 1000µs al inicio (motor detenido)
    pwm_set_pulse(s_motor1_gpio, PWM_CHANNEL_1, ESC_MIN_US);
    pwm_set_pulse(s_motor2_gpio, PWM_CHANNEL_2, ESC_MIN_US);
    
    ESP_LOGI(TAG, "Motores inicializados en GPIO%d y GPIO%d", 
             s_motor1_gpio, s_motor2_gpio);
    ESP_LOGI(TAG, "Rango: %d-%d µs (50Hz)", ESC_MIN_US, ESC_MAX_US);
    
    return ESP_OK;
}

/**
 * @brief CALIBRACIÓN AUTOMÁTICA DEL ESC
 * 
 * Secuencia según datasheet:
 * 1. Enviar 2000µs (máximo)
 * 2. Esperar 5 segundos (self-test + beeps)
 * 3. Enviar 1000µs (mínimo)
 * 4. Esperar 3 segundos (BEEP largo de confirmación)
 */
esp_err_t brushless_calibrate(void)
{
    if (s_motor1_gpio < 0 || s_motor2_gpio < 0) {
        ESP_LOGE(TAG, "Motores no inicializados");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "  CALIBRACIÓN AUTOMÁTICA DEL ESC");
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "⚠️  CONECTA LA BATERÍA AHORA");
    ESP_LOGI(TAG, "   El ESP32 controlará el throttle automáticamente");
    ESP_LOGI(TAG, "   ESCUCHA los tonos del ESC");
    ESP_LOGI(TAG, "   ⏱️  Duración: ~10 segundos");
    ESP_LOGI(TAG, "");
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // PASO 1: Throttle a MÁXIMO (2000µs)
    ESP_LOGI(TAG, "📌 PASO 1: Throttle a MÁXIMO (2000µs)");
    ESP_LOGI(TAG, "   ESCUCHA: '123' → beeps por celdas → 'Beep-Beep'");
    ESP_LOGI(TAG, "   ⏱️  Esperando 5 segundos...");
    ESP_LOGI(TAG, "");
    
    pwm_set_pulse(s_motor1_gpio, PWM_CHANNEL_1, ESC_MAX_US);
    pwm_set_pulse(s_motor2_gpio, PWM_CHANNEL_2, ESC_MAX_US);
    
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // PASO 2: Throttle a MÍNIMO (1000µs)
    ESP_LOGI(TAG, "📌 PASO 2: Throttle a MÍNIMO (1000µs)");
    ESP_LOGI(TAG, "   ESCUCHA: BEEP LARGO (calibración exitosa)");
    ESP_LOGI(TAG, "   ⏱️  Esperando 3 segundos...");
    ESP_LOGI(TAG, "");
    
    pwm_set_pulse(s_motor1_gpio, PWM_CHANNEL_1, ESC_MIN_US);
    pwm_set_pulse(s_motor2_gpio, PWM_CHANNEL_2, ESC_MIN_US);
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    s_is_calibrated = true;
    
    ESP_LOGI(TAG, "✅ CALIBRACIÓN COMPLETADA");
    ESP_LOGI(TAG, "   ⏱️  Tiempo total: ~11 segundos");
    ESP_LOGI(TAG, "   El ESC ya está calibrado permanentemente");
    ESP_LOGI(TAG, "   AHORA: Ejecuta 'brushless_arm()' para armar");
    ESP_LOGI(TAG, "=========================================");
    
    return ESP_OK;
}

esp_err_t brushless_arm(void)
{
    if (s_motor1_gpio < 0 || s_motor2_gpio < 0) {
        ESP_LOGE(TAG, "Motores no inicializados");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Armando ESC...");
    ESP_LOGI(TAG, "  Enviando 1000µs (0% throttle) por 2 segundos");
    
    pwm_set_pulse(s_motor1_gpio, PWM_CHANNEL_1, ESC_MIN_US);
    pwm_set_pulse(s_motor2_gpio, PWM_CHANNEL_2, ESC_MIN_US);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    s_is_armed = true;
    ESP_LOGI(TAG, "✅ ESC ARMADOS");
    ESP_LOGI(TAG, "  Los motores ya no deberían pitar");
    
    return ESP_OK;
}

void brushless_set_motor1_us(uint16_t us)
{
    if (s_motor1_gpio < 0 || !s_is_armed) return;
    pwm_set_pulse(s_motor1_gpio, PWM_CHANNEL_1, us);
}

void brushless_set_motor2_us(uint16_t us)
{
    if (s_motor2_gpio < 0 || !s_is_armed) return;
    pwm_set_pulse(s_motor2_gpio, PWM_CHANNEL_2, us);
}

void brushless_set_both_us(uint16_t motor1_us, uint16_t motor2_us)
{
    brushless_set_motor1_us(motor1_us);
    brushless_set_motor2_us(motor2_us);
}

void brushless_stop(void)
{
    if (s_is_armed) {
        pwm_set_pulse(s_motor1_gpio, PWM_CHANNEL_1, ESC_MIN_US);
        pwm_set_pulse(s_motor2_gpio, PWM_CHANNEL_2, ESC_MIN_US);
        ESP_LOGI(TAG, "Motores detenidos");
    }
}

void brushless_emergency_stop(void)
{
    brushless_stop();
}

bool brushless_is_armed(void)
{
    return s_is_armed;
}