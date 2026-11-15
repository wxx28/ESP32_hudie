set(src_dirs
            LED
            BUTTON)

set(include_dirs
            LED
            BUTTON)

set(requires
            esp_driver_gpio
            esp_timer)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3)
