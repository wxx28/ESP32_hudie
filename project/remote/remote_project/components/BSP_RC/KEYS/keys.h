#pragma once

/*
 * 8 个按键（低电平按下，内部上拉），10 ms 轮询，20 ms 消抖。
 *
 * 事件只报一次，由 keys_poll() 的调用者取走：
 *   CLICK   ：短按松开（如果还在双击窗口内，先等窗口结束）
 *   DOUBLE  ：400 ms 内按两下（不会再报 CLICK）
 *   LONG    ：按住超过 800 ms（按住期间只报一次，松开后不再报 CLICK）
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    KEY_STOP = 0,
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_UP_LEFT,
    KEY_UP_RIGHT,
    KEY_COUNT
} key_id_t;

typedef enum {
    KEY_EVT_NONE = 0,
    KEY_EVT_CLICK,
    KEY_EVT_DOUBLE,
    KEY_EVT_LONG
} key_event_t;

esp_err_t keys_init(void);

/* 每 10 ms 调用一次，更新状态机。 */
void keys_poll(void);

/* 取走某个键的事件（取后清零）。 */
key_event_t keys_take_event(key_id_t id);

/* 当前是否按住（消抖后）。 */
bool keys_is_down(key_id_t id);

/* 不消抖直接读，用于开机检测。 */
bool keys_raw_down(key_id_t id);

#ifdef __cplusplus
}
#endif
