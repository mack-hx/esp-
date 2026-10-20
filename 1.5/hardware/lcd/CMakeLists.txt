idf_component_register(
    SRCS "lcd_ui.c" "font6x8.c"
    INCLUDE_DIRS "."
    REQUIRES driver tft_display pcf8574
)