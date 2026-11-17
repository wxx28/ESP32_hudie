#pragma once

/*
 * 绑定信息存取：把对端 MAC 存进 NVS（命名空间 "hudie_bind"）。
 * 必须在 espnow_link_init() 之后调用（NVS 由其初始化）。
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 读取已绑定的对端 MAC，没有绑定时返回 false。 */
bool hudie_bind_load(uint8_t mac[6]);

esp_err_t hudie_bind_save(const uint8_t mac[6]);

esp_err_t hudie_bind_clear(void);

#ifdef __cplusplus
}
#endif
