idf_component_register(
    SRCS "ui_home.c" "ui_menu.c" "ui_balance.c" "ui_servo.c"
         "ui_encoder.c" "ui_mpu9250.c" "ui_wifi.c"
         "ui_hc595_test.c"
    INCLUDE_DIRS "."
    REQUIRES driver lcd tft_display pcf8574 servo encoder motor_driver mpu9250 hc595 wifi param control
)