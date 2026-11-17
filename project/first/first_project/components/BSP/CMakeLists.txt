# 蝴蝶 V1 飞控板硬件层：引脚只在 board_v1.h 里定义
set(src_dirs
            LED
            IMU
            SERVO
            BATTERY)

set(include_dirs
            .
            LED
            IMU
            SERVO
            BATTERY)

set(requires
            esp_driver_gpio
            esp_driver_spi
            esp_driver_ledc
            esp_adc
            esp_timer)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3)
