# 中间件组件：新增源文件目录时，把目录加进 SRC_DIRS / INCLUDE_DIRS
set(src_dirs
            ESP_NOW)

set(include_dirs
            ESP_NOW)

set(requires
            esp_wifi
            nvs_flash
            esp_netif
            esp_event)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})