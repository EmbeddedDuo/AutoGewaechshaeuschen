# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)
set(COMPONENT_REQUIRES "driver" "esp32-i2c-lcd1602" "esp32-smbus")

idf_component_register(SRCS ${app_sources})
