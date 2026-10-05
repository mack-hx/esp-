idf_component_register(
    SRCS "pid.c"
         "balance.c"
    INCLUDE_DIRS "."
    REQUIRES param legmap mpu9250 encoder motor_driver servo freertos esp_timer
)
