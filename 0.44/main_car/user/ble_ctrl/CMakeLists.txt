idf_component_register(
    SRCS "ble_ctrl.c"
    INCLUDE_DIRS "."
    REQUIRES bt control onenet_client esp_timer
)
