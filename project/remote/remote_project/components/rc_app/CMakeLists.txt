# 遥控器应用层：摇杆/按键 -> 控制帧，绑定，遥测显示
idf_component_register(
    SRCS "rc_app.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES BSP_RC hudie_link nvs_flash esp_timer freertos log
)
