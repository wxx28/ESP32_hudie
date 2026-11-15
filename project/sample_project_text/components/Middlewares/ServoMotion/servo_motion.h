#pragma once

#include <stdbool.h>

#include "esp_err.h"

/*
 * 双路舵机差速扑翼控制。
 *
 *   s = sin(2πft + φ)
 *   PWM0(GPIO4) = k + (A - D) * s
 *   PWM1(GPIO5) = k - (A + D) * s
 *
 * 合法条件：A + |D| <= k 且 A + |D| <= 180 - k。
 */
typedef struct {
    float amplitude_deg;    /* A，40..90 */
    float differential_deg; /* D，差动量，默认 0 */
    float frequency_hz;     /* f，1.56..3.13 */
    float phase_deg;        /* φ，0..360 */
    float center_deg;       /* k，40..140 */
    bool running;
    bool joystick_active;   /* 只读：由摇杆接口维护，set_params 会将其清为 false */
} servo_motion_params_t;

#define SERVO_MOTION_MIN_AMPLITUDE_DEG 40.0f
#define SERVO_MOTION_MAX_AMPLITUDE_DEG 90.0f
#define SERVO_MOTION_MAX_DIFFERENTIAL_DEG 20.0f
#define SERVO_MOTION_MIN_FREQUENCY_HZ 1.56f
#define SERVO_MOTION_MAX_FREQUENCY_HZ 3.13f
#define SERVO_MOTION_MIN_PHASE_DEG 0.0f
#define SERVO_MOTION_MAX_PHASE_DEG 360.0f
#define SERVO_MOTION_MIN_CENTER_DEG 40.0f
#define SERVO_MOTION_MAX_CENTER_DEG 140.0f

/* 摇杆 Y -> 振幅：A = 65 + 25 * y。 */
#define SERVO_MOTION_JOYSTICK_AMPLITUDE_MID_DEG 65.0f
#define SERVO_MOTION_JOYSTICK_AMPLITUDE_SPAN_DEG 25.0f

/* 初始化 PWM 并启动运动任务，可重复调用。 */
esp_err_t servo_motion_init(void);

/* 获取当前参数的线程安全快照。 */
servo_motion_params_t servo_motion_get_params(void);

/* 校验并应用全量参数（非摇杆来源），非法参数返回 ESP_ERR_INVALID_ARG。 */
esp_err_t servo_motion_set_params(const servo_motion_params_t *params);

/* 仅更新 f/φ/k，并按边界重新夹取 A 和 D。 */
esp_err_t servo_motion_set_config(float frequency_hz, float phase_deg, float center_deg);

/* 摇杆输入：active=true 时按 x/y 更新 A/D 并运行，active=false 时停止。 */
esp_err_t servo_motion_set_joystick(float x, float y, bool active);

/* 参数是否在允许范围内（含 NaN/Inf 与混控边界检查）。 */
bool servo_motion_params_valid(const servo_motion_params_t *params);
