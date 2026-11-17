# EDU 遥控器硬件层：引脚只在 board_rc.h 里定义
set(src_dirs
            JOYSTICK
            KEYS
            LED)

set(include_dirs
            .
            JOYSTICK
            KEYS
            LED)

set(requires
            esp_driver_gpio
            esp_adc
            esp_timer)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})
