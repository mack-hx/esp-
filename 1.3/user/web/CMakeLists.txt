idf_component_register(SRCS "web_server.c" "web_page.c"
                    INCLUDE_DIRS "."
    REQUIRES esp_http_server esp_timer wifi param control
             motor_driver encoder mpu9250 servo)
