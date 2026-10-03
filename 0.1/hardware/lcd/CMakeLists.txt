idf_component_register(
    SRCS "lcd_ui.c" "font6x8.c"
         "pages/ui_home.c"
         "pages/ui_menu.c"
         "pages/ui_servo.c"
         "pages/ui_encoder.c"
         "pages/ui_mpu9250.c"
         "pages/ui_wifi_bt.c"
    INCLUDE_DIRS "." "pages"
    REQUIRES driver tft_display pcf8574 servo encoder motor_driver mpu9250 esp_hw_support
)