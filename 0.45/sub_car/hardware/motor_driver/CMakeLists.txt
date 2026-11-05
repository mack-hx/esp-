idf_component_register(
    SRCS "motor_driver.c"
    INCLUDE_DIRS "."
    REQUIRES driver hc595
)