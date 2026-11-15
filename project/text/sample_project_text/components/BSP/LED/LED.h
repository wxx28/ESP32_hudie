#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化指示灯，active_low 为 true 时低电平点亮 */
esp_err_t led_init(int gpio_num, bool active_low);

void led_set(bool on);

void led_toggle(void);

bool led_get(void);

#ifdef __cplusplus
}
#endif
