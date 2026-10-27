idf_component_register(
    SRCS "balance.c"
    INCLUDE_DIRS "."
    REQUIRES param trim mpu9250 encoder motor_driver servo freertos onenet_client pid
)
