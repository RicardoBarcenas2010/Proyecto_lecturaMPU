#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdbool.h>
#include "system_config.h"

/*******************************************************
 * FUNCIONES PÚBLICAS
 *******************************************************/

void controller_init(void);
void controller_reset(void);
void controller_set_setpoint(float angle);
float controller_get_setpoint(void);
float controller_update(float current_angle);
void controller_set_gyro_rate(float gyro_rate_dps);

uint16_t controller_get_pwm_motor1(float correction);
uint16_t controller_get_pwm_motor2(float correction);

void controller_set_kp(float kp);
void controller_set_ki(float ki);
void controller_set_kd(float kd);

float controller_get_kp(void);
float controller_get_ki(void);
float controller_get_kd(void);

// DEPURACIÓN
void controller_get_debug_values(float *angle, float *error, float *p, float *i, float *d, 
                                  float *correction, float *pwm1, float *pwm2);

// NUEVAS: Funciones para depuración de la banda muerta
bool controller_is_in_deadband(void);
float controller_get_deadband_gain(void);
float controller_get_raw_correction(void);

#endif