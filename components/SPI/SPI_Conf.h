#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define SPI_HOST SPI2_HOST
#define SPI_MOSI GPIO_NUM_14
#define SPI_MISO GPIO_NUM_12
#define SPI_CLK GPIO_NUM_13

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t spi_init(void);

#ifdef __cplusplus
}
#endif
