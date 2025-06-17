extern "C" {
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"

#include "SPI_Conf.h"
#include "RFID.h"
#include "display.h"

static const char *TAG = "main";
static _lock_t lvgl_api_lock;

static void lvgl_tick_inc_cb(void *arg) {
    static int count = 0;
    lv_tick_inc(2);
    if (++count % 100 == 0) {
        ESP_LOGI(TAG, "LVGL Tick: %d", count);
    }
}

static void lvgl_task(void *arg) {
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting app_main");

    spi_init();

    spi_device_interface_config_t RFIDspi = RFID_SPI();
    spi_device_handle_t rfid_handle;
    esp_err_t ret = spi_bus_add_device(SPI_HOST, &RFIDspi, &rfid_handle);
  
    lv_display_t *disp = init_full_display();
    if (disp == NULL) {
        ESP_LOGE(TAG, "Display initialization failed");
        return;
    }
    esp_timer_handle_t tick_timer;
    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_inc_cb,
        .name = "lvgl_tick"
    };
    ret = esp_timer_create(&tick_args, &tick_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create tick timer: %s", esp_err_to_name(ret));
    }
    esp_timer_start_periodic(tick_timer, 2 * 1000);

    xTaskCreate(lvgl_task, "lvgl_task", 4 * 1024, NULL, 1, NULL);

    ESP_LOGI(TAG, "Creating LVGL label");

    _lock_acquire(&lvgl_api_lock);
    sib_create_image();
    _lock_release(&lvgl_api_lock);

    RFID_InitRead(rfid_handle, TAG);
} 
}
