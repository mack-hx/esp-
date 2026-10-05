cmake_minimum_required(VERSION 3.16)

# PlatformIO sets IDF_PATH automatically when framework = espidf

# 注册 hardware/ 下的所有外设组件
# 注意: EXTRA_COMPONENT_DIRS 必须在 project() 之前设置
# 因为 ESP-IDF 在 project() 内部 include project.cmake 时就立即扫描组件目录
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/mpu9250"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/hc595"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/tft_display"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/servo"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/motor_driver"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/pcf8574"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/k230_uart"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/voice_module"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/encoder"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/led"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/magnet"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/lcd"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/i2c_scanner"
    "${CMAKE_CURRENT_SOURCE_DIR}/hardware/wifi"
    "${CMAKE_CURRENT_SOURCE_DIR}/user/param"
    "${CMAKE_CURRENT_SOURCE_DIR}/user/control"
    "${CMAKE_CURRENT_SOURCE_DIR}/user/web"
    "${CMAKE_CURRENT_SOURCE_DIR}/user/ui"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(balanced_dog)