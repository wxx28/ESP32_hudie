/*
 * 仿生蝴蝶 EDU 遥控器入口（ESP32-C3，c3mini V2.19）
 *
 * main 只负责启动应用；操作说明见 components/rc_app/include/rc_app.h。
 */
#include "esp_log.h"
#include "rc_app.h"

void app_main(void)
{
    if (rc_app_start() != ESP_OK) {
        ESP_LOGE("main", "遥控器启动失败");
    }
}
