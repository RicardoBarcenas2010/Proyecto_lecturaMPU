#include "controller.h"
#include "system_config.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "CONTROLLER";

// ====== ESTRUCTURA DEL CONTROLADOR ======
typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float prev_error;
    float derivative_filtered;
    float prev_output;
    bool in_deadband;         // Indica si está dentro de la banda de ±1°
} pid_t;

// ====== INSTANCIA ======
static pid_t s_pid = {
    .kp = KP_INITIAL,
    .ki = KI_INITIAL,
    .kd = KD_INITIAL,
    .setpoint = 0.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .derivative_filtered = 0.0f,
    .prev_output = 0.0f,
    .in_deadband = false
};

// ====== VARIABLE PARA GIROSCOPIO ======
static float s_gyro_rate = 0.0f;

// ====== VARIABLES PARA DEPURACIÓN ======
static float s_debug_p = 0.0f;
static float s_debug_i = 0.0f;
static float s_debug_d = 0.0f;
static float s_debug_error = 0.0f;
static float s_debug_angle = 0.0f;
static float s_debug_correction = 0.0f;
static float s_debug_raw_correction = 0.0f;  // Corrección antes de la reducción
static float s_debug_gain_reduction = 1.0f;  // Factor de reducción aplicado
static bool s_debug_deadband_active = false;

// ====== CONSTANTES DE BANDA MUERTA ======
#define DEADBAND_THRESHOLD      1.0f    // ±1° de banda
#define DEADBAND_GAIN           0.25f   // Factor de reducción dentro de la banda

// ============================================================
//  FUNCIONES AUXILIARES
// ============================================================

static float clamp_float(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static float validate_float(float value)
{
    if (isnan(value) || isinf(value)) {
        return 0.0f;
    }
    return value;
}

static float apply_slew_rate(float desired, float current, float max_change)
{
    float diff = desired - current;
    if (diff > max_change) return current + max_change;
    if (diff < -max_change) return current - max_change;
    return desired;
}

// ============================================================
//  FUNCIÓN DE GANANCIA REDUCIDA EN BANDA MUERTA
// ============================================================
static float apply_deadband_gain(float error, float correction)
{
    float abs_error = fabsf(error);
    
    if (abs_error < DEADBAND_THRESHOLD) {
        // Dentro de la banda: reducir ganancia
        s_pid.in_deadband = true;
        s_debug_deadband_active = true;
        
        // Factor de reducción: 0.25 (25% de la corrección original)
        float reduction = DEADBAND_GAIN;
        s_debug_gain_reduction = reduction;
        
        // También reducimos la integral para evitar windup
        s_pid.integral *= 0.95f;  // Decaimiento suave de la integral
        
        return correction * reduction;
        
    } else {
        // Fuera de la banda: ganancia completa (100%)
        s_pid.in_deadband = false;
        s_debug_deadband_active = false;
        s_debug_gain_reduction = 1.0f;
        return correction;
    }
}

// ============================================================
//  FUNCIONES PÚBLICAS
// ============================================================

void controller_init(void)
{
    s_pid.kp = KP_INITIAL;
    s_pid.ki = KI_INITIAL;
    s_pid.kd = KD_INITIAL;
    s_pid.setpoint = SETPOINT_DEFAULT;
    s_pid.integral = 0.0f;
    s_pid.prev_error = 0.0f;
    s_pid.derivative_filtered = 0.0f;
    s_pid.prev_output = 0.0f;
    s_pid.in_deadband = false;
    s_gyro_rate = 0.0f;
    
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "  PID + BANDA MUERTA SUAVE (±1°)");
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "  PWM Base M1: %d µs", PWM_BASE_M1);
    ESP_LOGI(TAG, "  PWM Base M2: %d µs", PWM_BASE_M2);
    ESP_LOGI(TAG, "  Kp = %.2f", s_pid.kp);
    ESP_LOGI(TAG, "  Ki = %.2f", s_pid.ki);
    ESP_LOGI(TAG, "  Kd = %.2f", s_pid.kd);
    ESP_LOGI(TAG, "  Slew Rate: %.0f µs/ciclo", (float)SLEW_RATE);
    ESP_LOGI(TAG, "  Banda muerta: ±%.1f°", DEADBAND_THRESHOLD);
    ESP_LOGI(TAG, "  Ganancia en banda: %.0f%%", DEADBAND_GAIN * 100.0f);
    ESP_LOGI(TAG, "  Max Correction: %.0f µs", (float)MAX_PID_CORRECTION);
    ESP_LOGI(TAG, "  ✅ Derivada desde GIROSCOPIO");
    ESP_LOGI(TAG, "=========================================");
}

void controller_reset(void)
{
    s_pid.integral = 0.0f;
    s_pid.prev_error = 0.0f;
    s_pid.derivative_filtered = 0.0f;
    s_pid.prev_output = 0.0f;
    s_pid.in_deadband = false;
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

void controller_set_gyro_rate(float gyro_rate_dps)
{
    s_gyro_rate = validate_float(gyro_rate_dps);
}

float controller_update(float current_angle)
{
    // Validar entrada
    current_angle = validate_float(current_angle);
    s_debug_angle = current_angle;
    
    // ============================================================
    //  1. CALCULAR ERROR
    // ============================================================
    float error = s_pid.setpoint - current_angle;
    error = validate_float(error);
    s_debug_error = error;
    
    // ============================================================
    //  2. TÉRMINO INTEGRAL (siempre calculando)
    // ============================================================
    if (s_pid.ki > 0.001f) {
        s_pid.integral += error * DT;
        s_pid.integral = clamp_float(s_pid.integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    } else {
        s_pid.integral = 0.0f;
    }
    
    // ============================================================
    //  3. TÉRMINO DERIVATIVO (VELOCIDAD ANGULAR DEL GIROSCOPIO)
    // ============================================================
    float derivative = -s_gyro_rate;
    derivative = validate_float(derivative);
    
    s_pid.derivative_filtered = (DERIVATIVE_ALPHA * s_pid.derivative_filtered) + 
                                ((1.0f - DERIVATIVE_ALPHA) * derivative);
    s_pid.derivative_filtered = validate_float(s_pid.derivative_filtered);
    
    s_pid.prev_error = error;
    
    // ============================================================
    //  4. CALCULAR TÉRMINOS DEL PID (siempre)
    // ============================================================
    float p_term = s_pid.kp * error;
    float i_term = s_pid.ki * s_pid.integral;
    float d_term = s_pid.kd * s_pid.derivative_filtered;
    
    s_debug_p = p_term;
    s_debug_i = i_term;
    s_debug_d = d_term;
    
    // ============================================================
    //  5. CALCULAR CORRECCIÓN BRUTA (sin reducción)
    // ============================================================
    float raw_correction = p_term + i_term + d_term;
    raw_correction = validate_float(raw_correction);
    s_debug_raw_correction = raw_correction;
    
    // ============================================================
    //  6. APLICAR BANDA MUERTA SUAVE (reducción de ganancia)
    // ============================================================
    float correction = apply_deadband_gain(error, raw_correction);
    correction = validate_float(correction);
    s_debug_correction = correction;
    
    // ============================================================
    //  7. SATURAR CORRECCIÓN
    // ============================================================
    correction = clamp_float(correction, -MAX_PID_CORRECTION, MAX_PID_CORRECTION);
    
    // ============================================================
    //  8. APLICAR SLEW RATE
    // ============================================================
    correction = apply_slew_rate(correction, s_pid.prev_output, SLEW_RATE);
    s_pid.prev_output = correction;
    
    return correction;
}

// ============================================================
//  CÁLCULO DE PWM
// ============================================================

uint16_t controller_get_pwm_motor1(float correction)
{
    float pwm = (float)PWM_BASE_M1 + correction;
    pwm = clamp_float(pwm, (float)ESC_MIN_US, (float)ESC_MAX_US);
    return (uint16_t)pwm;
}

uint16_t controller_get_pwm_motor2(float correction)
{
    float pwm = (float)PWM_BASE_M2 - correction;
    pwm = clamp_float(pwm, (float)ESC_MIN_US, (float)ESC_MAX_US);
    return (uint16_t)pwm;
}

// ============================================================
//  GETTERS Y SETTERS PARA GANANCIAS
// ============================================================

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
}

void controller_set_kd(float kd)
{
    s_pid.kd = validate_float(kd);
    if (s_pid.kd < 0.0f) s_pid.kd = 0.0f;
    ESP_LOGI(TAG, "Kd = %.2f", s_pid.kd);
}

float controller_get_kp(void)
{
    return s_pid.kp;
}

float controller_get_ki(void)
{
    return s_pid.ki;
}

float controller_get_kd(void)
{
    return s_pid.kd;
}

// ============================================================
//  DEPURACIÓN
// ============================================================

void controller_get_debug_values(float *angle, float *error, float *p, float *i, float *d, 
                                  float *correction, float *pwm1, float *pwm2)
{
    if (angle) *angle = s_debug_angle;
    if (error) *error = s_debug_error;
    if (p) *p = s_debug_p;
    if (i) *i = s_debug_i;
    if (d) *d = s_debug_d;
    if (correction) *correction = s_debug_correction;
    (void)pwm1;
    (void)pwm2;
}

// NUEVAS FUNCIONES PARA DEPURACIÓN DE BANDA MUERTA
bool controller_is_in_deadband(void)
{
    return s_debug_deadband_active;
}

float controller_get_deadband_gain(void)
{
    return s_debug_gain_reduction;
}

float controller_get_raw_correction(void)
{
    return s_debug_raw_correction;
}