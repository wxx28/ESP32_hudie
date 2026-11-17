#pragma once
/* 仿真用：替代 ESP-IDF 的 esp_err.h，只保留 BSP 头文件需要的定义 */
typedef int esp_err_t;
#define ESP_OK                0
#define ESP_FAIL             -1
#define ESP_ERR_INVALID_ARG   0x102
#define ESP_ERR_INVALID_STATE 0x103
