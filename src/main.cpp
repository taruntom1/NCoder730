#include <cmath>
#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define SPI_HOST SPI2_HOST // SPI host to use
#define TAG "NCoder730"

// Commands and registers
#define READ_REG_COMMAND    (0b010 << 13)
#define WRITE_REG_COMMAND   (0b100 << 13)

// Registers of MA730
#define ZERO_SETTING0_REG 0x0
#define ZERO_SETTING1_REG 0x1
#define BCT_REG 0x2
#define TRIMMING_REG 0x3
#define PPT0_REG 0x4
#define PPT1_REG 0x5
#define ROT_DIR_REG 0x9

class NCoder730 {
private:
    spi_device_handle_t spi;
    gpio_num_t cs_pin;

    void writeRegister(uint8_t reg, uint8_t value) {
        uint16_t command = WRITE_REG_COMMAND | ((reg & 0x1F) << 8) | value;
        transmitSPI(command);
    }

    uint8_t readRegister(uint8_t reg) {
        uint16_t command = READ_REG_COMMAND | ((reg & 0x1F) << 8);
        uint16_t response = transmitSPI(command);
        return response & 0xFF;
    }

    uint16_t transmitSPI(uint16_t data) {
        uint16_t rx_data = 0;
        spi_transaction_t trans = {
            .length = 16,          // Transaction length in bits
            .tx_buffer = &data,    // Transmit buffer
            .rx_buffer = &rx_data, // Receive buffer
        };
        ESP_ERROR_CHECK(spi_device_transmit(spi, &trans));
        return rx_data;
    }

public:
    NCoder730(gpio_num_t cs) :  spi(nullptr), cs_pin(cs) {}

    void initSPI() {
        spi_bus_config_t buscfg = {
            .mosi_io_num = GPIO_NUM_11,
            .miso_io_num = GPIO_NUM_13,
            .sclk_io_num = GPIO_NUM_12,
            .max_transfer_sz = 32,
        };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

        spi_device_interface_config_t devcfg = {
            .mode = 0,
            .clock_speed_hz = 1 * 1000 * 1000,
            .spics_io_num = cs_pin,
            .queue_size = 1,
        };
        ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi));
    }

    void deinitSPI() {
        if (spi) {
            ESP_ERROR_CHECK(spi_bus_remove_device(spi));
            spi = nullptr;
        }
        ESP_ERROR_CHECK(spi_bus_free(SPI_HOST));
    }

    void writeDefaultConfigurations() {
        writeRegister(ZERO_SETTING0_REG, 0x00);
        writeRegister(ZERO_SETTING1_REG, 0x00);
        writeRegister(BCT_REG, 0x00);
        writeRegister(TRIMMING_REG, 0x00);
        writeRegister(PPT0_REG, 0xC0);
        writeRegister(PPT1_REG, 0xFF);
        writeRegister(ROT_DIR_REG, 0x00);
    }

    uint16_t readAbsoluteAngleRaw16() {
        return transmitSPI(0x0000);
    }

    uint8_t readAbsoluteAngleRaw8() {
        return transmitSPI(0x0000) & 0xFF;
    }

    void setZeroPosition(float angle) {
        uint16_t zero_pos = pow(2, 16) * (1 - (angle / 360.0f));
        writeRegister(ZERO_SETTING0_REG, uint8_t(zero_pos));
        writeRegister(ZERO_SETTING1_REG, uint8_t(zero_pos >> 8));
    }

    float getZeroPosition() {
        uint16_t zero_pos = (readRegister(ZERO_SETTING1_REG) << 8) | readRegister(ZERO_SETTING0_REG);
        return 360.0f - ((zero_pos / float(pow(2, 16))) * 360.0f);
    }

    void setBCTValue(uint8_t bct_value) {
        writeRegister(BCT_REG, bct_value);
    }

    uint8_t getBCTValue() {
        return readRegister(BCT_REG);
    }

    void setETX(bool val) {
        bool temp = getETY();
        writeRegister(TRIMMING_REG, (temp << 1) | val);
    }

    void setETY(bool val) {
        bool temp = getETX();
        writeRegister(TRIMMING_REG, (val << 1) | temp);
    }

    bool getETX() {
        return readRegister(TRIMMING_REG) & 0x1;
    }

    bool getETY() {
        return (readRegister(TRIMMING_REG) >> 1) & 0x1;
    }

    void setPulsePerTurn(uint16_t ppr) {
        uint16_t val = ppr - 1;
        uint8_t reg_val = readRegister(PPT0_REG);
        writeRegister(PPT0_REG, uint8_t((val & 0x03) << 6) | (reg_val & 0x3F));
        writeRegister(PPT1_REG, uint8_t(val >> 2));
    }

    uint16_t getPulsePerTurn() {
        uint16_t val = (readRegister(PPT1_REG) << 2) | ((readRegister(PPT0_REG) >> 6) & 0x03);
        return (val + 1);
    }
};


extern "C" void app_main() {
    NCoder730 encoder(GPIO_NUM_10); // Replace with your CS pin
    encoder.initSPI();
    encoder.writeDefaultConfigurations();

    while (true) {
        uint16_t angle = encoder.readAbsoluteAngleRaw16();
        ESP_LOGI(TAG, "Angle: %u", angle);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Clean up resources
    encoder.deinitSPI();
}
