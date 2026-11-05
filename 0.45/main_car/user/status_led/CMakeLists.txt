idf_component_register(SRCS "status_led.c"
                    INCLUDE_DIRS "."
                    REQUIRES led servo magnet pcf8574 control freertos)
