idf_component_register(
    SRCS "rolltest.c"
    INCLUDE_DIRS "."
    REQUIRES servo roll_compensator control hal freertos
)
