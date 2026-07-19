#include "controller.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "CONTROLLER";

// ====== INSTANCIA DEL CONTROLADOR ======
static pid_controller_t s_pid = {
    .kp = KP_INITIAL,
    .ki = KI_INITIAL,
    .kd = KD_INITIAL,
    .setpoint = 0.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .derivative_filtered = 0.0f,
    .prev_output = 0.0f
};

// ====== FUNCIONES INTERNAS ======

/**
 * @brief Valida que un número no sea NaN o infinito
 */
static float validate_float(float value)
{
    if (isnan(value) || isinf(value)) {
        return 0.0f;
    }
    return value;
}

/**
 * @brief Aplica límite a un valor
 */
static float clamp(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/**
 * @brief Aplica slew rate (limitador de velocidad)
 */
static float apply_slew_rate(float desired, float current, float max_change)
{
    float diff = desired - current;
    if (diff > max_change) return current + max_change;
    if (diff < -max_change) return current - max_change;
    return desired;
}

// ====== FUNCIONES PÚBLICAS ======

void controller_init(void)
{
    s_pid.kp = KP_INITIAL;
    s_pid.ki = KI_INITIAL;
    s_pid.kd = KD_INITIAL;
    s_pid.setpoint = 0.0f;
    s_pid.integral = 0.0f;
    s_pid.prev_error = 0.0f;
    s_pid.derivative_filtered = 0.0f;
    s_pid.prev_output = 0.0f;
    
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "  CONTROLADOR PID CON PWM BASE INDIVIDUAL");
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "  PWM Base M1: %d µs", PWM_BASE_M1);
    ESP_LOGI(TAG, "  PWM Base M2: %d µs", PWM_BASE_M2);
    ESP_LOGI(TAG, "  Kp=%.2f, Ki=%.2f, Kd=%.2f", s_pid.kp, s_pid.ki, s_pid.kd);
    ESP_LOGI(TAG, "  Slew Rate: %.0f µs/ciclo", (float)SLEW_RATE);
    ESP_LOGI(TAG, "  Tolerancia: ±%.1f°", ERROR_TOLERANCE);
    ESP_LOGI(TAG, "=========================================");
}

void controller_reset(void)
{
    s_pid.integral = 0.0f;
    s_pid.prev_error = 0.0f;
    s_pid.derivative_filtered = 0.0f;
    s_pid.prev_output = 0.0f;
    ESP_LOGI(TAG, "Controlador reseteado");
}

void controller_set_setpoint(float angle)
{
    s_pid.setpoint = validate_float(angle);
    controller_reset();
    ESP_LOGI(TAG, "Setpoint: %.2f°", s_pid.setpoint);
}

float controller_get_setpoint(void)
{
    return s_pid.setpoint;
}

float controller_update(float current_angle)
{
    // 1. Validar entrada
    current_angle = validate_float(current_angle);
    
    // 2. Calcular error
    float error = s_pid.setpoint - current_angle;
    error = validate_float(error);
    
    // 3. ZONA MUERTA: Si el error es menor a ±0.5°, no corregir
    if (fabsf(error) < ERROR_TOLERANCE) {
        // Mantener la integral viva con decaimiento
        if (s_pid.integral != 0.0f) {
            s_pid.integral *= 0.99f;
        }
        // Si Ki = 0, la integral no se usa de todos modos
        return 0.0f;
    }
    
    // 4. TÉRMINO INTEGRAL CON ANTI-WINDUP MEJORADO
    // Solo integrar si Ki > 0
    if (s_pid.ki > 0.001f) {
        if (fabsf(error) < SATURATION_LIMIT * 2.0f) {
            s_pid.integral += error * DT;
        } else {
            s_pid.integral += error * DT * 0.5f;
        }
        s_pid.integral = clamp(s_pid.integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    } else {
        // Si Ki = 0, mantener integral en cero
        s_pid.integral = 0.0f;
    }
    
    // 5. TÉRMINO DERIVATIVO CON FILTRO
    float derivative = (error - s_pid.prev_error) / DT;
    derivative = validate_float(derivative);
    
    s_pid.derivative_filtered = (DERIVATIVE_ALPHA * s_pid.derivative_filtered) + 
                                ((1.0f - DERIVATIVE_ALPHA) * derivative);
    s_pid.derivative_filtered = validate_float(s_pid.derivative_filtered);
    
    // 6. Guardar error para la próxima iteración
    s_pid.prev_error = error;
    
    // 7. Calcular salida del PID (corrección en µs)
    float correction = (s_pid.kp * error) + 
                       (s_pid.ki * s_pid.integral) + 
                       (s_pid.kd * s_pid.derivative_filtered);
    correction = validate_float(correction);
    
    // 8. Limitar SOLO la corrección (NO el PWM final)
    correction = clamp(correction, -MAX_PID_CORRECTION, MAX_PID_CORRECTION);
    
    // 9. APLICAR SLEW RATE (limitador de velocidad)
    correction = apply_slew_rate(correction, s_pid.prev_output, SLEW_RATE);
    s_pid.prev_output = correction;
    
    // 10. Logging en modo debug
    // ESP_LOGD(TAG, "err=%.2f°, P=%.2f, I=%.2f, D=%.2f, corr=%.2fµs", 
    //          error, s_pid.kp*error, s_pid.ki*s_pid.integral, 
    //          s_pid.kd*s_pid.derivative_filtered, correction);
    
    return correction;
}

uint16_t controller_get_pwm_motor1(float correction)
{
    // Motor 1: PWM_BASE_M1 + corrección
    float pwm_float = (float)PWM_BASE_M1 + correction;
    
    // Limitar solo al rango físico del ESC
    if (pwm_float < ESC_MIN_US) pwm_float = ESC_MIN_US;
    if (pwm_float > ESC_MAX_US) pwm_float = ESC_MAX_US;
    
    return (uint16_t)pwm_float;
}

uint16_t controller_get_pwm_motor2(float correction)
{
    // Motor 2: PWM_BASE_M2 - corrección (contra-rotación)
    float pwm_float = (float)PWM_BASE_M2 - correction;
    
    // Limitar solo al rango físico del ESC
    if (pwm_float < ESC_MIN_US) pwm_float = ESC_MIN_US;
    if (pwm_float > ESC_MAX_US) pwm_float = ESC_MAX_US;
    
    return (uint16_t)pwm_float;
}

// ====== GETTERS Y SETTERS ======

void controller_set_kp(float kp)
{
    s_pid.kp = validate_float(kp);
    if (s_pid.kp < 0.0f) s_pid.kp = 0.0f;
    ESP_LOGI(TAG, "Kp = %.2f", s_pid.kp);
}

void controller_set_ki(float ki)
{
    s_pid.ki = validate_float(ki);
    if (s_pid.ki < 0.0f) s_pid.ki = 0.0f;
    ESP_LOGI(TAG, "Ki = %.2f", s_pid.ki);
    if (s_pid.ki > 0.001f) {
        ESP_LOGW(TAG, "⚠️  Ki activado - asegurar estabilidad primero");
    }
}

void controller_set_kd(float kd)
{
    s_pid.kd = validate_float(kd);
    if (s_pid.kd < 0.0f) s_pid.kd = 0.0f;
    ESP_LOGI(TAG, "Kd = %.2f", s_pid.kd);
}

float controller_get_kp(void) { return s_pid.kp; }
float controller_get_ki(void) { return s_pid.ki; }
float controller_get_kd(void) { return s_pid.kd; }