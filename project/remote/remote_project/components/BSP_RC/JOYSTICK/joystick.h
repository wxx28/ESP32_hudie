#pragma once

/*
 * 两个摇杆（4 路 ADC1）+ 遥控器电池（ADC1_CH0）
 *
 * 上电时自动记录俯仰/滚转/偏航的中位（开机时不要碰摇杆），油门不校准中位。
 * 输出：油门 0..1000，其余 -1000..1000，已做死区和方向处理。
 */

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t throttle; /* 0..1000 */
    int16_t roll;     /* -1000..1000，正 = 右 */
    int16_t pitch;    /* -1000..1000 */
    int16_t yaw;      /* -1000..1000 */
} joystick_values_t;

/* 初始化 ADC 并采集中位（约 0.2 s）。 */
esp_err_t joystick_init(void);

/* 采样一次（带滑动平均），约 0.3 ms。 */
void joystick_read(joystick_values_t *out);

/* 遥控器电池（mV，已滤波），每次调用采样一次。 */
uint32_t joystick_read_battery_mv(void);

#ifdef __cplusplus
}
#endif
