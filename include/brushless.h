#ifndef BRUSHLESS_H
#define BRUSHLESS_H

#include "esp_err.h"
#include <stdbool.h>

// ====== PARÁMETROS ======
#define PWM_MIN_1         5.0f
#define PWM_MAX_1         100.0f
#define PWM_MIN_2         0.0f
#define PWM_MAX_2         100.0f
#define ANGLE_MIN         -45.0f
#define ANGLE_MAX         45.0f

// ====== FUNCIONES ======
esp_err_t brushless_init(int gpio_pin_1, int gpio_pin_2);
esp_err_t brushless_arm(void);
void brushless_set_pwm_motor1(float percent);
void brushless_set_pwm_motor2(float percent);
float brushless_get_pwm_motor1(void);
float brushless_get_pwm_motor2(void);
void brushless_start_sweep_char(float pwm_start, float pwm_end, float pwm_step, int delay_ms);
void brushless_stop_sweep_char(void);
bool brushless_is_sweep_running(void);
void brushless_stop(void);

#endif