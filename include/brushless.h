#ifndef BRUSHLESS_H
#define BRUSHLESS_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/*******************************************************
 * CONFIGURACIÓN DEL ESC
 *******************************************************/

// Frecuencia PWM
#define ESC_PWM_FREQUENCY      50

// Pulsos estándar
#define ESC_MIN_US             1000
#define ESC_MAX_US             2000
#define ESC_ARM_US             1000

// Valor inicial para ambos motores
#define ESC_CENTER_US          1500

// Límite de cambio por actualización (µs)
#define ESC_RAMP_US            5

/*******************************************************
 * INICIALIZACIÓN
 *******************************************************/

esp_err_t brushless_init(int gpio_motor1,
                         int gpio_motor2);

esp_err_t brushless_arm(void);

/*******************************************************
 * CONTROL EN MICROSEGUNDOS
 *******************************************************/

void brushless_set_motor1_us(uint16_t us);

void brushless_set_motor2_us(uint16_t us);

void brushless_set_both_us(uint16_t motor1_us,
                           uint16_t motor2_us);

/*******************************************************
 * GETTERS
 *******************************************************/

uint16_t brushless_get_motor1_us(void);

uint16_t brushless_get_motor2_us(void);

/*******************************************************
 * UTILIDADES
 *******************************************************/

// Convierte porcentaje (0-100) a µs.
// Solo para pruebas y depuración.
uint16_t brushless_percent_to_us(float percent);

// Convierte µs a porcentaje.
float brushless_us_to_percent(uint16_t us);

/*******************************************************
 * SEGURIDAD
 *******************************************************/

void brushless_stop(void);

void brushless_emergency_stop(void);

bool brushless_is_armed(void);

#endif