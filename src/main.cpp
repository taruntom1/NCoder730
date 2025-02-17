#include <iostream>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SPI_READ";

struct spi_config
{
    spi_host_device_t spi_host;
    gpio_num_t miso_pin;
    gpio_num_t mosi_pin;
    gpio_num_t clk_pin;
    gpio_num_t cs_pin;
};

class SensorSPI
{
private:
    spi_device_handle_t spi;
    spi_config config;

    void spi_init()
    {
        spi_bus_config_t buscfg = {
            .mosi_io_num = config.mosi_pin,
            .miso_io_num = config.miso_pin,
            .sclk_io_num = config.clk_pin,
            .max_transfer_sz = 16};
        ESP_ERROR_CHECK(spi_bus_initialize(config.spi_host, &buscfg, SPI_DMA_CH_AUTO));
        spi_device_interface_config_t devcfg = {
            .mode = 0,
            .clock_speed_hz = 1 * 1000 * 1000,
            .spics_io_num = config.cs_pin,
            .queue_size = 1,
        };

        ESP_ERROR_CHECK(spi_bus_add_device(config.spi_host, &devcfg, &spi));
    }

    uint8_t spi_read_register(uint8_t reg)
    {
        uint8_t tx_buffer[2] = {static_cast<uint8_t>((0x02 << 5) | reg), 0x00};
        uint8_t rx_buffer[2] = {0};

        spi_transaction_t trans = {};
        trans.length = 16;
        trans.tx_buffer = tx_buffer;
        trans.rx_buffer = rx_buffer;

        ESP_ERROR_CHECK(spi_device_transmit(spi, &trans));

        tx_buffer[0] = 0x00;
        tx_buffer[1] = 0x00;
        ESP_ERROR_CHECK(spi_device_transmit(spi, &trans));

        return rx_buffer[0];
    }

public:
    uint8_t registers[9] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x09, 0x1B};
    SensorSPI(spi_config cfg) : config(cfg)
    {
        spi_init();
    }

    void GetRegisterDump(uint8_t values[9])
    {
        for (size_t i = 0; i < 9; i++)
        {
            values[i] = spi_read_register(registers[i]);
        }
    }

    void WriteRegister(uint8_t reg, uint8_t value)
    {
        uint8_t tx_buffer[2] = {static_cast<uint8_t>((0x04 << 5) | (reg & 0x1F)), value};
        spi_transaction_t trans = {};
        trans.length = 16;
        trans.tx_buffer = tx_buffer;
        trans.rx_buffer = nullptr;

        ESP_ERROR_CHECK(spi_device_transmit(spi, &trans));

        vTaskDelay(pdMS_TO_TICKS(20));

        uint8_t rx_buffer[2] = {0};
        trans.tx_buffer = nullptr;
        trans.rx_buffer = rx_buffer;
        ESP_ERROR_CHECK(spi_device_transmit(spi, &trans));
    }
};

extern "C" void app_main()
{
    std::cout << "Initializing SPI..." << std::endl;
    spi_config config = {SPI2_HOST, GPIO_NUM_13, GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_10};
    SensorSPI sensor(config);

    vTaskDelay(pdMS_TO_TICKS(10000));

    std::cout << "Writing to sensor register 0x09..." << std::endl;
    sensor.WriteRegister(0x05, 0xfe);

    std::cout << "Reading sensor registers:" << std::endl;
    uint8_t values[9];
    sensor.GetRegisterDump(values);
    for (size_t i = 0; i < 9; i++)
    {
        std::cout << "Register 0x" << std::hex << int(sensor.registers[i]) << " = 0x" << std::hex << int(values[i]) << std::endl;
    }
}
