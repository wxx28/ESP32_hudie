#include "esp_err.h"
#include "nvs_flash.h"
#include "servo_motion.h"
#include "servo_web_server.h"
#include "wifi_ap.h"

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(servo_motion_init());

    wifi_ap_start();
    servo_web_server_start();
}
