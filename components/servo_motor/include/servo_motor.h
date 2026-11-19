#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>
#include "driver/ledc.h"
#include "esp_err.h"

// Configuración
#define SERVO_SPEED_MODE         LEDC_LOW_SPEED_MODE
#define SERVO_TIMER              LEDC_TIMER_2
#define SERVO_DUTY_RESOLUTION    LEDC_TIMER_13_BIT
#define SERVO_FREQUENCY          50

#define SERVO_MIN_PULSEWIDTH_US  500
#define SERVO_MAX_PULSEWIDTH_US  2500
#define SERVO_MAX_DEGREE         180

// Handle
typedef struct servo_dev_t *servo_handle_t;

// Crear servo
esp_err_t servo_create(servo_handle_t *handle, uint8_t gpio, ledc_channel_t channel);

// Setear ángulo
esp_err_t servo_set_angle(servo_handle_t handle, int angle);

// Setear pulso directo
esp_err_t servo_set_pulse_us(servo_handle_t handle, uint32_t pulsewidth);

// Eliminar servo
esp_err_t servo_delete(servo_handle_t handle);

#endif