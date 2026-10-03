idf_component_register(
    SRCS "ui_home.c" "ui_menu.c" "ui_servo.c"
         "ui_encoder.c" "ui_mpu9250.c" "ui_wifi_bt.c"
    INCLUDE_DIRS "."
    REQUIRES driver lcd tft_display pcf8574 servo encoder motor_driver mpu9250 esp_hw_support
)