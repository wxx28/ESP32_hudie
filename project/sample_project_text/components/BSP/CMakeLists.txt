set(src_dirs
            LED
            Servo)

set(include_dirs
            LED
            Servo)

set(requires
            driver
            esp_driver_gpio
            esp_driver_ledc)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3)
