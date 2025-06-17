#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

static void lvgl_tick_inc_cb(void *arg);
lv_display_t *init_full_display(void);
extern const lv_image_dsc_t toronto_img;
void sib_create_image();
spi_device_interface_config_t LCD_SPI();

#ifdef __cplusplus
}
#endif
