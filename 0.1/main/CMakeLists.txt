idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES lcd_ui tft_display led hc595
                             servo motor_driver encoder mpu9250 pcf8574
                             freertos nvs_flash)