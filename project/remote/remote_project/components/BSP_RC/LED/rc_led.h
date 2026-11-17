#pragma once

/* 遥控器绿灯 DS1（IO10，低电平点亮），节奏由 esp_timer 50 ms 驱动。 */

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RC_LED_OFF = 0,
    RC_LED_ON,       /* 常亮：已解锁 */
    RC_LED_SLOW,     /* 1 Hz：上锁，链路正常 */
    RC_LED_FAST,     /* 5 Hz：飞控没有回传（失联） */
    RC_LED_RAPID,    /* 10 Hz：绑定中 */
    RC_LED_DOUBLE,   /* 双闪：电量低（遥控器或飞控） */
    RC_LED_BLIP      /* 短闪一下后回到原节奏：按键确认 */
} rc_led_pattern_t;

esp_err_t rc_led_init(void);

void rc_led_set(rc_led_pattern_t p);

/* 叠加一次 150 ms 的闪烁确认。 */
void rc_led_blip(void);

#ifdef __cplusplus
}
#endif
