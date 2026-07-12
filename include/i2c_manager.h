#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Inicializa el bus I2C.
 *
 * @return ESP_OK si todo salió correctamente.
 */
esp_err_t i2c_manager_init(void);

/**
 * @brief Escribe uno o varios bytes en un registro.
 *
 * @param device_addr Dirección I2C del dispositivo.
 * @param reg_addr Registro donde escribir.
 * @param data Datos a escribir.
 * @param len Número de bytes.
 *
 * @return ESP_OK si la operación fue exitosa.
 */
esp_err_t i2c_write_register(uint8_t device_addr,
                             uint8_t reg_addr,
                             uint8_t *data,
                             size_t len);

/**
 * @brief Lee uno o varios bytes de un registro.
 *
 * @param device_addr Dirección I2C.
 * @param reg_addr Registro.
 * @param data Buffer donde almacenar.
 * @param len Número de bytes.
 *
 * @return ESP_OK si la lectura fue correcta.
 */
esp_err_t i2c_read_register(uint8_t device_addr,
                            uint8_t reg_addr,
                            uint8_t *data,
                            size_t len);

#endif