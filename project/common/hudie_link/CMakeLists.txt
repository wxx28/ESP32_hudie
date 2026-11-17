# 飞控与遥控器共用的无线链路组件：
#   hudie_protocol：帧格式 + CRC16（两端必须同一份）
#   espnow_link   ：ESP-NOW 收发（广播 + 单播对端切换）
#   hudie_bind    ：对端 MAC 绑定信息的 NVS 存取
idf_component_register(
    SRCS
        "hudie_protocol.c"
        "espnow_link.c"
        "hudie_bind.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
)
