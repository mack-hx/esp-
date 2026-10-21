idf_component_register(
    SRCS "onenet_client.c"
    INCLUDE_DIRS "."
    REQUIRES mqtt control param mbedtls led
)
