idf_component_register(
    SRCS "ui_home.c" "ui_menu.c" "ui_balance.c" "ui_servo.c"
         "ui_encoder.c" "ui_mpu9250.c" "ui_netstat.c" "ui_led_magnet.c"
    INCLUDE_DIRS "."
    REQUIRES driver lcd tft_display pcf8574 servo encoder motor_driver mpu9250 hc595
             wifi_sta param control onenet_client led magnet esp_app_format)
