# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# NOTE: components/ 目录下的组件会被 ESP-IDF 自动扫描，不需要再写 EXTRA_COMPONENT_DIRS。
# 飞控与遥控器共用的无线链路组件放在 project/common 下：
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../common/hudie_link")
add_compile_options(-fdiagnostics-color=always)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hudie_flight)
