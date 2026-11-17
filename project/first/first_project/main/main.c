/*
 * 仿生蝴蝶 V1 飞控入口
 *
 * main 只负责组装配置、初始化和启动飞控，不写业务逻辑。
 * 硬件引脚：components/BSP/board_v1.h
 * 可调参数：idf.py menuconfig -> "蝴蝶飞控参数" / "蝴蝶 V1 飞控板 BSP"
 */
#include "esp_log.h"
#include "fc_config.h"
#include "flight_control.h"

static const char *TAG = "main";

void app_main(void)
{
    static algo_config_t config;
    fc_config_v1(&config);

    fc_status_t st = flight_control_init(&config);
    if (st != FC_STATUS_OK) {
        ESP_LOGE(TAG, "飞控初始化失败: %d（舵机保持中位）", st);
        return;
    }
    st = flight_control_start();
    if (st != FC_STATUS_OK) {
        ESP_LOGE(TAG, "飞控启动失败: %d", st);
    }
}
