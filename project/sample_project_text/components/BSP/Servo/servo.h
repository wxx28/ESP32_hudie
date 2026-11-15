#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef enum {
    SERVO_CHANNEL_PWM0 = 0,
    SERVO_CHANNEL_PWM1 = 1,
} servo_channel_t;

#define SERVO_MIN_PULSE_US 1000U
#define SERVO_MAX_PULSE_US 2000U

esp_err_t servo_init(void);
esp_err_t servo_set_pulse_us(servo_channel_t channel, uint32_t pulse_us);
