#include "SPI_Conf.h"
#include "esp_log.h"

#define TAG "SPI_Conf"

esp_err_t spi_init(void) {
    static bool initialized = false;
    if (initialized) return ESP_OK;

    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 51200
    };

    esp_err_t ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_OK) {
        initialized = true;
    } else {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
    }

    return ret;
}
