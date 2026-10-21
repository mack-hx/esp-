idf_component_register(
    SRCS "onenet_client.c"
    INCLUDE_DIRS "."
    REQUIRES mqtt control param mbedtls led servo magnet encoder pcf8574 esp_netif
)
