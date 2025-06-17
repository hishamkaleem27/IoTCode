#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "RFID.h"

#define RFID_CS  GPIO_NUM_10
#define RFID_RST GPIO_NUM_9

void PCD_Version(spi_device_handle_t spi) {
    uint8_t ver = PCD_ReadRegister(spi, VersionReg);
    if (ver == 0x92) {
        printf("MFRC522 Version 2 detected.\n");
    } else if (ver == 0x91) {
        printf("MFRC522 Version 1 detected.\n");
    } else {
        printf("Is connected device MFRC522? If yes, check the wiring again.\n");
        for (int i = 5; i >= 0; i--) {
            printf("Restarting in %d seconds...\n", i);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        printf("Restarting Now.\n");
        fflush(stdout);
        esp_restart();
    }
}

void PCD_WriteRegister(spi_device_handle_t spi, uint8_t reg, uint8_t val) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 16; 
    t.tx_data[0] = reg;
    t.tx_data[1] = val;
    t.rx_buffer = NULL; 
    ret = spi_device_transmit(spi, &t);
    assert(ret == ESP_OK);
}

void PCD_WriteRegisterMany(spi_device_handle_t spi, uint8_t reg, uint8_t count, uint8_t *values) {
    esp_err_t ret;
    uint8_t total[count + 1];
    total[0] = reg & 0x7E;
    memcpy(&total[1], values, count);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * (count + 1);
    t.tx_buffer = total;

    ret = spi_device_transmit(spi, &t);
    assert(ret == ESP_OK);
}

uint8_t PCD_ReadRegister(spi_device_handle_t spi, uint8_t reg) {
    esp_err_t ret;
    uint8_t tx_data[2];
    uint8_t rx_data[2];

    tx_data[0] = reg | 0x80; // MSB=1 for read per MFRC522 datasheet
    tx_data[1] = 0x00;       // Dummy byte to receive data

    spi_transaction_t t = {
        .length = 8 * 2,                 // total 16 bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    ret = spi_device_transmit(spi, &t);
    assert(ret == ESP_OK);

    return rx_data[1];  // Second byte is the response from MFRC522
}


void PCD_ReadRegisterMany(spi_device_handle_t spi, uint8_t reg, uint8_t count, uint8_t *values, uint8_t rxAlign) {
    if (count == 0) return;

    esp_err_t ret;
    uint8_t tx[count + 1];
    uint8_t rx[count + 1];

    tx[0] = 0x80 | reg;  // Read command
    memset(&tx[1], 0, count);
    memset(rx, 0, sizeof(rx));

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * (count + 1);
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    ret = spi_device_transmit(spi, &t);
    assert(ret == ESP_OK);

    memcpy(values, &rx[1], count);  // Skip address byte
}

void PCD_ClearRegisterBitMask(spi_device_handle_t spi, uint8_t reg, uint8_t mask) {
    uint8_t tmp = PCD_ReadRegister(spi, reg);
    PCD_WriteRegister(spi, reg, tmp & (~mask));
}

void PCD_SetRegisterBitMask(spi_device_handle_t spi, uint8_t reg, uint8_t mask) {
    uint8_t tmp = PCD_ReadRegister(spi, reg);
    PCD_WriteRegister(spi, reg, tmp | mask);
}

void PCD_Init(spi_device_handle_t spi) {
    PCD_Version(spi);
    gpio_set_direction(RFID_RST, GPIO_MODE_OUTPUT);

    gpio_set_level(RFID_RST, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(RFID_RST, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    PCD_WriteRegister(spi, TxModeReg, 0x00);
    PCD_WriteRegister(spi, RxModeReg, 0x00);
    PCD_WriteRegister(spi, ModWidthReg, 0x26);
    PCD_WriteRegister(spi, TModeReg, 0x80);
    PCD_WriteRegister(spi, TPrescalerReg, 0xA9);
    PCD_WriteRegister(spi, TReloadRegH, 0x03);
    PCD_WriteRegister(spi, TReloadRegL, 0xE8);
    PCD_WriteRegister(spi, TxASKReg, 0x40);
    PCD_WriteRegister(spi, ModeReg, 0x3D);
    PCD_AntennaOn(spi);

    printf("Initialization successful.\n");
}

void PCD_AntennaOn(spi_device_handle_t spi) {
    uint8_t value = PCD_ReadRegister(spi, TxControlReg);
    if ((value & 0x03) != 0x03) {
        PCD_WriteRegister(spi, TxControlReg, value | 0x03);
    }
    printf("Antenna turned on.\n");
}

bool PICC_IsNewCardPresent(spi_device_handle_t spi) {
    uint8_t bufferATQA[3] = { 0 };
    uint8_t bufferSize = sizeof(bufferATQA);

    PCD_WriteRegister(spi, TxModeReg, 0x00);
    PCD_WriteRegister(spi, RxModeReg, 0x00);
    PCD_WriteRegister(spi, ModWidthReg, 0x26);

    uint8_t result = PICC_RequestA(spi, bufferATQA, &bufferSize);
    return (result == STATUS_OK || result == STATUS_COLLISION);
}

uint8_t PICC_RequestA(spi_device_handle_t spi, uint8_t *bufferATQA, uint8_t *bufferSize) {
    return PICC_REQA_or_WUPA(spi, PICC_CMD_REQA, bufferATQA, bufferSize);
}

uint8_t PICC_REQA_or_WUPA(spi_device_handle_t spi, uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize) {
    if (bufferATQA == NULL || *bufferSize < 2) return STATUS_NO_ROOM;

    PCD_ClearRegisterBitMask(spi, CollReg, 0x80);
    uint8_t validBits = 7;

    uint8_t result = PCD_TransceiveData(spi, &command, 1, bufferATQA, bufferSize, &validBits, 0, false);
    if (result != STATUS_OK) return result;
    if (*bufferSize != 2 || validBits != 0) return STATUS_ERROR;

    return STATUS_OK;
}

uint8_t PCD_TransceiveData(spi_device_handle_t spi, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, bool checkCRC) {
    return PCD_CommunicateWithPICC(spi, PCD_Transceive, 0x30, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

uint8_t PCD_CommunicateWithPICC(spi_device_handle_t spi, uint8_t command, uint8_t waitIRq, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, bool checkCRC) {
    uint8_t txLastBits = validBits ? *validBits : 0;
    uint8_t bitFraming = (rxAlign << 4) + txLastBits;

    PCD_WriteRegister(spi, CommandReg, PCD_Idle);
    PCD_WriteRegister(spi, ComIrqReg, 0x7F);
    PCD_WriteRegister(spi, FIFOLevelReg, 0x80);
    PCD_WriteRegisterMany(spi, FIFODataReg, sendLen, sendData);
    PCD_WriteRegister(spi, BitFramingReg, bitFraming);
    PCD_WriteRegister(spi, CommandReg, command);

    if (command == PCD_Transceive) {
        PCD_SetRegisterBitMask(spi, BitFramingReg, 0x80);
    }

    for (uint16_t i = 20000; i > 0; i--) {
        uint8_t n = PCD_ReadRegister(spi, ComIrqReg);
        if (n & waitIRq) break;
        if (n & 0x01) return STATUS_TIMEOUT;
    }

    uint8_t errorRegValue = PCD_ReadRegister(spi, ErrorReg);
    if (errorRegValue & 0x13) return STATUS_ERROR;

    uint8_t _validBits = 0;
    if (backData && backLen) {
        uint8_t n = PCD_ReadRegister(spi, FIFOLevelReg);
        if (n > *backLen) return STATUS_NO_ROOM;

        *backLen = n;
        PCD_ReadRegisterMany(spi, FIFODataReg, n, backData, rxAlign);
        _validBits = PCD_ReadRegister(spi, ControlReg) & 0x07;
        if (validBits) *validBits = _validBits;
    }

    if (errorRegValue & 0x08) return STATUS_COLLISION;
    return STATUS_OK;
}

spi_device_interface_config_t RFID_SPI() {
    spi_device_interface_config_t RFIDcfg = {
        .clock_speed_hz = 5000000,
        .mode = 0,
        .spics_io_num = RFID_CS,
        .queue_size = 7,
    };
    return RFIDcfg;
}

uint8_t PICC_SelectUID(spi_device_handle_t spi, uint8_t *uid, uint8_t *uidSize) {
    uint8_t backData[10];
    uint8_t backLen = sizeof(backData);
    uint8_t validBits = 0;

    uint8_t buffer[] = { 0x93, 0x20 };
    uint8_t result = PCD_TransceiveData(spi, buffer, 2, backData, &backLen, &validBits, 0, false);

    if (result == STATUS_OK && backLen >= 5) {
        memcpy(uid, backData, backLen);
        *uidSize = backLen;
        return STATUS_OK;
    }

    return STATUS_ERROR;
}

void RFID_InitRead(spi_device_handle_t spi, const char *TAG){
    PCD_Init(spi);
    ESP_LOGI(TAG, "RFID initialized");
    while (1) {
        if (PICC_IsNewCardPresent(spi)) {
            uint8_t uid[10];
            uint8_t uid_len = 0;

            if (PICC_SelectUID(spi, uid, &uid_len) == STATUS_OK) {
                char uid_str[32] = {0};
                for (int i = 0; i < uid_len; i++) {
                    char byte_str[4];
                    sprintf(byte_str, "%02X", uid[i]);
                    strcat(uid_str, byte_str);
                }

                ESP_LOGI(TAG, "Card UID: %s", uid_str);

            } else {
                ESP_LOGW(TAG, "Failed to read UID");
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
}