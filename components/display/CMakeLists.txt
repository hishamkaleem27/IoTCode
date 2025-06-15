idf_component_register(
    SRCS "toronto.c" "display.c"
    INCLUDE_DIRS "."
    REQUIRES lvgl esp_lcd esp_lcd_ili9341 driver esp_timer
)
