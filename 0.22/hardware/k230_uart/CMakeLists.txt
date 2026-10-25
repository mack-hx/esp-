idf_component_register(
    SRCS "k230_uart.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos
)