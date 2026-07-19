#ifndef BRUSHLESS_H
#define BRUSHLESS_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"

/*******************************************************
 * FUNCIONES PÚBLICAS
 *******************************************************/

esp_err_t brushless_init(int gpio_motor1, int gpio_motor2);

/**
 * @brief CALIBRA el rango de throttle del ESC (AUTOMÁTICO)
 * Secuencia: 2000µs → 1000µs
 * Duración: ~10 segundos
 */
esp_err_t brushless_calibrate(void);

/**
 * @brief ARMA el ESC (envía 1000µs por 2 segundos)
 */
esp_err_t brushless_arm(void);

/**
 * @brief Establece el pulso del motor 1
 * @param us Valor en microsegundos (1000-2000)
 */
void brushless_set_motor1_us(uint16_t us);

/**
 * @brief Establece el pulso del motor 2
 * @param us Valor en microsegundos (1000-2000)
 */
void brushless_set_motor2_us(uint16_t us);

/**
 * @brief Establece pulsos para ambos motores
 */
void brushless_set_both_us(uint16_t motor1_us, uint16_t motor2_us);

/**
 * @brief Detiene ambos motores (envía 1000µs)
 */
void brushless_stop(void);

/**
 * @brief Parada de emergencia (igual que stop)
 */
void brushless_emergency_stop(void);

/**
 * @brief Verifica si los motores están armados
 */
bool brushless_is_armed(void);

#endif