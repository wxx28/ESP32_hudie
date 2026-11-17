#pragma once

/*
 * 双路舵机输出（LEDC 硬件 PWM）
 *
 *   SERVO_LEFT  -> IO1 / PWM2 -> U9  -> PWM_L -> J3
 *   SERVO_RIGHT -> IO8 / PWM1 -> U10 -> PWM_R -> J2
 *
 * 光耦反相在这里处理（LEDC output_invert），上层只给逻辑角度 0~180°。
 * 机构安装方向（左右镜像）不在这里处理，由 algo_linkage 的 servo_direction 负责。
 */

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SERVO_LEFT = 0,
    SERVO_RIGHT = 1,
    SERVO_COUNT = 2
} servo_id_t;

esp_err_t servo_init(void);

/* 逻辑角度 0~180°，超出范围会被限幅。 */
esp_err_t servo_set_angle(servo_id_t id, float angle_deg);

/* 直接写脉宽（us），用于台架测试。 */
esp_err_t servo_set_pulse_us(servo_id_t id, uint32_t pulse_us);

#ifdef __cplusplus
}
#endif
