#include "controller.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "CONTROLLER";

static float setpoint = 0.0f;
static float integral = 0.0f;
static float previous_error = 0.0f;
static float derivative_filtered = 0.0f;

static float Kp = KP_INITIAL;
static float Ki = KI_INITIAL;
static float Kd = KD_INITIAL;

#define DT                 0.01f
#define INTEGRAL_LIMIT     30.0f
#define DERIVATIVE_ALPHA   0.80f
#define SATURATION_LIMIT   12.0f   // Si la corrección supera esto, resetear integral

void controller_init(void)
{
    setpoint = 0.0f;
    integral = 0.0f;
    previous_error = 0.0f;
    derivative_filtered = 0.0f;
    Kp = KP_INITIAL;
    Ki = KI_INITIAL;
    Kd = KD_INITIAL;
}

void controller_reset(void)
{
    integral = 0.0f;
    previous_error = 0.0f;
    derivative_filtered = 0.0f;
}

void controller_set_setpoint(float angle)
{
    setpoint = angle;
    integral = 0.0f;
    previous_error = 0.0f;
    derivative_filtered = 0.0f;
}

float controller_get_setpoint(void)
{
    return setpoint;
}

float controller_update(float current_angle)
{
    if (isnan(current_angle) || isinf(current_angle)) {
        current_angle = 0.0f;
    }
    
    float error = setpoint - current_angle;
    
    if (isnan(error) || isinf(error)) {
        error = 0.0f;
    }

    // ============================================
    // ANTI-WINDUP: Si el error es muy grande, resetear integral
    // ============================================
    if (fabs(error) > 20.0f) {
        // Si el error es mayor a 20°, el balancín está muy inclinado
        // Resetear integral para evitar saturación
        integral = 0.0f;
        ESP_LOGI(TAG, "Anti-windup: Error grande (%.1f°), integral reset", error);
    } else {
        integral += error * DT;
    }

    if(integral > INTEGRAL_LIMIT)
        integral = INTEGRAL_LIMIT;
    if(integral < -INTEGRAL_LIMIT)
        integral = -INTEGRAL_LIMIT;

    float derivative = (error - previous_error) / DT;
    
    if (isnan(derivative) || isinf(derivative)) {
        derivative = 0.0f;
    }

    derivative_filtered = DERIVATIVE_ALPHA * derivative_filtered + (1.0f - DERIVATIVE_ALPHA) * derivative;
    
    if (isnan(derivative_filtered) || isinf(derivative_filtered)) {
        derivative_filtered = 0.0f;
    }

    previous_error = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative_filtered);
    
    if (isnan(output) || isinf(output)) {
        output = 0.0f;
    }

    if(output > MAX_PID_OUTPUT)
        output = MAX_PID_OUTPUT;
    if(output < -MAX_PID_OUTPUT)
        output = -MAX_PID_OUTPUT;

    return output;
}

void controller_set_kp(float kp) { Kp = kp; }
void controller_set_ki(float ki) { Ki = ki; }
void controller_set_kd(float kd) { Kd = kd; }

float controller_get_kp(void) { return Kp; }
float controller_get_ki(void) { return Ki; }
float controller_get_kd(void) { return Kd; }