#pragma once

/*
 * 1S 电池电压（IO0 / ADC1_CH0，100k/100k 分压）
 *
 * 读数带一阶低通（时间常数约 0.5 s），避免舵机电流尖峰造成误报低电。
 * ESP32-C3 的 ADC 有个体差异，使用芯片 eFuse 校准；仍建议用万用表核一次。
 */

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t battery_init(void);

/* 采样一次并更新滤波值，返回滤波后的电压（mV）。建议 20~100 Hz 调用。 */
uint32_t battery_update_mv(void);

/* 最近一次滤波后的电压（mV），不采样。 */
uint32_t battery_get_mv(void);

#ifdef __cplusplus
}
#endif
