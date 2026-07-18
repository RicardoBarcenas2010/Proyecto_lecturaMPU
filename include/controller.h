#ifndef CONTROLLER_H
#define CONTROLLER_H

// GANANCIAS PID OPTIMIZADAS PARA ESTABILIDAD
#define KP_INITIAL        1.0f    // Proporcional - Principal fuerza de corrección
#define KI_INITIAL        0.15f   // Integral - Elimina error en estado estable
#define KD_INITIAL        0.08f   // Derivativo - Amortigua oscilaciones

#define MAX_PID_OUTPUT    150.0f

#define ANGLE_MIN        -45.0f
#define ANGLE_MAX         45.0f

// Funciones
void controller_init(void);
void controller_reset(void);
void controller_set_setpoint(float angle);
float controller_get_setpoint(void);
float controller_update(float current_angle);

void controller_set_kp(float kp);
void controller_set_ki(float ki);
void controller_set_kd(float kd);

float controller_get_kp(void);
float controller_get_ki(void);
float controller_get_kd(void);

#endif