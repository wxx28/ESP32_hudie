#pragma once

/*
 * 状态灯（IO10 -> Q3 -> LED2）。闪烁节奏由上层用 led_set_pattern() 选择，
 * 内部用 esp_timer 以 50 ms 为节拍驱动，不占用飞控任务。
 */

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_PATTERN_OFF = 0,
    LED_PATTERN_ON,          /* 常亮：已解锁 */
    LED_PATTERN_SLOW,        /* 1 Hz：待机，遥控在线 */
    LED_PATTERN_DOUBLE,      /* 双闪：待机，没有遥控 / 等待绑定 */
    LED_PATTERN_FAST,        /* 5 Hz：失控保护 / 低电 */
    LED_PATTERN_RAPID,       /* 10 Hz：自检 / 校准中 */
    LED_PATTERN_SOS          /* 三长三短：急停 / 故障 */
} led_pattern_t;

esp_err_t led_init(void);

void led_set(bool on);

void led_set_pattern(led_pattern_t pattern);

#ifdef __cplusplus
}
#endif
