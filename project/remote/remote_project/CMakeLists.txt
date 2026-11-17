cmake_minimum_required(VERSION 3.22)

# 飞控与遥控器共用的无线链路组件（协议 + ESP-NOW + 绑定）
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../common/hudie_link")
add_compile_options(-fdiagnostics-color=always)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hudie_remote)
