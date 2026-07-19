#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

esp_err_t i2c_manager_init(void);
esp_err_t i2c_write_register(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_read_register(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len);

#endif