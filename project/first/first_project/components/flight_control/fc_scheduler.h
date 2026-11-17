#ifndef FC_SCHEDULER_H
#define FC_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 400 Hz 周期调度
 *
 * esp_timer 每 2.5 ms 发一次任务通知，飞控任务（高优先级）醒来执行一拍。
 * 定时器回调里不做 SPI，也不做计算。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 每拍回调，在飞控任务上下文执行。now_us 为本拍时间，dt_s 为与上一拍的真实间隔。 */
typedef void (*fc_scheduler_tick_cb_t)(uint64_t now_us, float dt_s);

bool fc_scheduler_init(uint32_t period_us, fc_scheduler_tick_cb_t cb);

bool fc_scheduler_start(void);

void fc_scheduler_stop(void);

bool fc_scheduler_is_running(void);

/* 统计：错过的节拍数（任务没来得及处理就来了下一个通知）。 */
uint32_t fc_scheduler_overrun_count(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_SCHEDULER_H */
