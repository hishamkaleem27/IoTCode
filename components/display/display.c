#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>

#include "esp_lcd_ili9341.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"

#include "toronto.h"

#define TAG "display"

#define LCD_H_RES 320
#define LCD_V_RES 240
#define CLOCK_HZ (40 * 1000 * 1000)

#define LCD_DC     GPIO_NUM_15
#define LCD_RST    GPIO_NUM_16
#define LCD_CS     GPIO_NUM_17
#define BK_LIGHT   GPIO_NUM_7

static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    lv_draw_sw_rgb565_swap(px_map, (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1));
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
    lv_display_flush_ready(disp);
}

const lv_image_dsc_t toronto_img = {
    .header.w = 320,
    .header.h = 240,
    .data_size = sizeof(torontoArr),
    .header.cf = LV_COLOR_FORMAT_RGB565,
    .data = (const uint8_t *)torontoArr
};

void sib_create_image(void) {
    lv_obj_t *img = lv_image_create(lv_screen_active());
    lv_image_set_src(img, &toronto_img);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
}


lv_display_t *init_full_display(void) {
    ESP_LOGI(TAG, "Initializing display...");

    gpio_config_t bk_gpio_config = {
        .pin_bit_mask = 1ULL << BK_LIGHT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&bk_gpio_config);
    gpio_set_level(BK_LIGHT, 0);

    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = LCD_CS,
        .dc_gpio_num = LCD_DC,
        .spi_mode = 0,
        .pclk_hz = CLOCK_HZ,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .flags = {}
    };
    esp_err_t ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init panel IO: %s", esp_err_to_name(ret));
    }

    esp_lcd_panel_handle_t panel_handle;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
        .vendor_config = NULL,
    };
    ret = esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ILI9341 panel: %s", esp_err_to_name(ret));
    }

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_mirror(panel_handle, false, false);
    esp_lcd_panel_swap_xy(panel_handle, true);

    lv_init();
    lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    size_t buf_size = LCD_H_RES * 40 * sizeof(lv_color16_t);
    void *buf1 = heap_caps_malloc(buf_size, MALLOC_CAP_DMA);
    void *buf2 = heap_caps_malloc(buf_size, MALLOC_CAP_DMA);
    memset(buf1, 0, buf_size);
    memset(buf2, 0, buf_size);

    lv_display_set_buffers(disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_user_data(disp, panel_handle);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(disp, lvgl_flush_cb);

    sib_create_image();
    lv_refr_now(NULL);

    esp_lcd_panel_disp_on_off(panel_handle, true);
    gpio_set_level(BK_LIGHT, 1);

    return disp;
}
spi_device_interface_config_t LCD_SPI() {
    spi_device_interface_config_t LCDcfg = {
        .clock_speed_hz = 40000000,
        .mode = 0,
        .spics_io_num = LCD_CS,
        .queue_size = 7,
    };
    return LCDcfg;
}

