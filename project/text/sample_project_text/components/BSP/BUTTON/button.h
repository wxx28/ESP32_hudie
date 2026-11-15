#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 按下回调，在 esp_timer 任务上下文执行（不可长时间阻塞） */
typedef void (*button_press_cb_t)(void);

/**
 * @brief 初始化按键（任意边沿中断 + 软消抖）
 *
 * 电平需稳定保持 20 ms 才被认定为新的按键状态，只有「松开 -> 按下」的状态跃迁
 * 才会触发一次 on_press；按住不放期间的电平抖动不会重复触发。
 *
 * @param gpio_num    按键引脚
 * @param active_low  true 表示低电平为按下（使用内部上拉）
 * @param on_press    按下回调，可为 NULL
 */
esp_err_t button_init(int gpio_num, bool active_low, button_press_cb_t on_press);

#ifdef __cplusplus
}
#endif
