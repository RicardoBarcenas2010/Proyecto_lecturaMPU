#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "driver/gpio.h"
#include "driver/i2c_master.h"

/*=========================
    CONFIGURACIÓN I2C
=========================*/

#define I2C_PORT                I2C_NUM_0

#define I2C_SDA_PIN             GPIO_NUM_21
#define I2C_SCL_PIN             GPIO_NUM_22

#define I2C_FREQUENCY           400000

/*=========================
    MPU6050
=========================*/

#define MPU6050_ADDRESS         0x68

/*=========================
    FREERTOS
=========================*/

#define MPU_TASK_STACK_SIZE     4096
#define MPU_TASK_PRIORITY       5

#endif