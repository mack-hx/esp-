idf_component_register(
    SRCS "mpu9250.c"
    INCLUDE_DIRS "."
    # esp_timer: 互补滤波的 dt 用 esp_timer_get_time() 实测, 不再硬编码 10ms
    # nvs_flash: 陀螺零偏的断电保存 (namespace "imu", key "gof")
    REQUIRES driver esp_timer nvs_flash
)
