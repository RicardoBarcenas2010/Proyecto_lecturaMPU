#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdbool.h>
#include "system_config.h"

/*******************************************************
 * ESTRUCTURA PARA PARÁMETROS DEL CONTROLADOR
 *******************************************************/

typedef struct {
    float kp;           // Ganancia proporcional
    float ki;           // Ganancia integral
    float kd;           // Ganancia derivativa
    float setpoint;     // Ángulo deseado
    float integral;     // Término integral acumulado
    float prev_error;   // Error anterior
    float derivative_filtered;  // Derivada filtrada
    float prev_output;  // Salida anterior (para slew rate)
} pid_controller_t;

/*******************************************************
 * FUNCIONES PÚBLICAS
 *******************************************************/

/**
 * @brief Inicializa el controlador PID
 */
void controller_init(void);

/**
 * @brief Reinicia el controlador (resetea integral y derivada)
 */
void controller_reset(void);

/**
 * @brief Establece el setpoint (ángulo deseado)
 */
void controller_set_setpoint(float angle);

/**
 * @brief Obtiene el setpoint actual
 */
float controller_get_setpoint(void);

/**
 * @brief Actualiza el controlador y retorna la corrección en µs
 * @param current_angle Ángulo actual en grados
 * @return Corrección en microsegundos (para sumar al PWM base)
 */
float controller_update(float current_angle);

/**
 * @brief Obtiene el PWM final para el Motor 1
 * @param correction Corrección del PID en µs
 * @return PWM en microsegundos (limitado a ESC_MIN_US - ESC_MAX_US)
 */
uint16_t controller_get_pwm_motor1(float correction);

/**
 * @brief Obtiene el PWM final para el Motor 2
 * @param correction Corrección del PID en µs
 * @return PWM en microsegundos (limitado a ESC_MIN_US - ESC_MAX_US)
 */
uint16_t controller_get_pwm_motor2(float correction);

/**
 * @brief Setters para ganancias (en tiempo real)
 */
void controller_set_kp(float kp);
void controller_set_ki(float ki);
void controller_set_kd(float kd);

/**
 * @brief Getters para ganancias
 */
float controller_get_kp(void);
float controller_get_ki(void);
float controller_get_kd(void);

#endif