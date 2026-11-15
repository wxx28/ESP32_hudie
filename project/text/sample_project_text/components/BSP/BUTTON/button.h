#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*button_press_cb_t)(void);

/** 初始化按键，active_low 为 true 时按下为低电平。回调在 esp_timer 任务上下文执行 */
esp_err_t button_init(int gpio_num, bool active_low, button_press_cb_t on_press);

#ifdef __cplusplus
}
#endif
