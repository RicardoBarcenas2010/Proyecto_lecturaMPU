#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "driver/gpio.h"
#include "driver/i2c.h"

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

// LÍMITES FÍSICOS DEL SISTEMA
#define ANGLE_MIN              -45.0f
#define ANGLE_MAX               45.0f

/*=========================
    MOTORES BRUSHLESS
=========================*/

#define MOTOR1_GPIO             GPIO_NUM_13
#define MOTOR2_GPIO             GPIO_NUM_14

// RANGOS DEL ESC (1000-2000µs)
#define ESC_MIN_US              1000
#define ESC_MAX_US              2000

// PWM BASE INDIVIDUAL PARA CADA MOTOR
#define PWM_BASE_M1             1176
#define PWM_BASE_M2             1180

// LÍMITES DE CORRECCIÓN DEL PID
#define MAX_PID_CORRECTION      100.0f

/*=========================
    CONTROLADOR PID
=========================*/

// GANANCIAS INICIALES (PD - Ki = 0)
#define KP_INITIAL              1.5f
#define KI_INITIAL              0.0f
#define KD_INITIAL              0.8f

// LÍMITES Y PROTECCIONES
#define INTEGRAL_LIMIT          30.0f
#define SATURATION_LIMIT        5.0f

// CONSTANTES DE TIEMPO
#define DT                      0.01f
#define DERIVATIVE_ALPHA        0.85f
#define SLEW_RATE               10.0f

// TOLERANCIA DE ERROR
#define ERROR_TOLERANCE         0.5f

/*=========================
    TAREAS
=========================*/

#define CONTROL_TASK_STACK      4096
#define CONTROL_TASK_PRIORITY   10
#define MONITOR_TASK_STACK      2048
#define MONITOR_TASK_PRIORITY   5

// ✅ TAREA IMU (para compatibilidad con imu.c)
#define IMU_TASK_STACK_SIZE     4096
#define IMU_TASK_PRIORITY       5

/*=========================
    PERIODOS
=========================*/

#define CONTROL_PERIOD_MS       10      // 10ms (100Hz)
#define MONITOR_PERIOD_MS       100     // 100ms

#endif