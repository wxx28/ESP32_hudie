# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# NOTE: components/ 目录下的组件会被 ESP-IDF 自动扫描，不需要再写 EXTRA_COMPONENT_DIRS。
# 如果要加项目目录之外的组件目录，语法是：
#   set(EXTRA_COMPONENT_DIRS "<绝对路径>")
add_compile_options(-fdiagnostics-color=always)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sample_project_text)
