#include <iostream>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "NCoder730.h"
#include <bitset>

static const char *TAG = "SPI_READ";

extern "C" void app_main()
{
    std::cout << "Initializing SPI..." << std::endl;
    spi_config config = {SPI2_HOST, GPIO_NUM_13, GPIO_NUM_11, GPIO_NUM_12, GPIO_NUM_10};
    NCoder730 sensor(config);

    vTaskDelay(pdMS_TO_TICKS(10000));

    std::cout << "Reading sensor registers:" << std::endl;
    uint8_t values[9];
    sensor.GetRegisterDump(values);
    for (size_t i = 0; i < 9; i++)
    {
        std::cout << "Register 0x" << std::hex
                  << int(sensor.registers[i])
                  << " = 0x"
                  << std::hex
                  << int(values[i])
                  << std::endl;
    }

    while (true)
    {
        std::cout << "Reading sensor position: "
                  << sensor.readAbsoluteAngle()
                  << std::endl;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
