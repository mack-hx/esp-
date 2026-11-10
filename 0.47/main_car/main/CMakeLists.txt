idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES lcd ui tft_display led hc595
                             servo motor_driver encoder mpu9250 pcf8574
                             wifi_sta param control onenet_client ble_ctrl freertos nvs_flash odom
                             roll_compensator status_led rolltest)
