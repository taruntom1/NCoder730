#include "NCoder730.h"

NCoder730::NCoder730(spi_config cfg) : config(cfg)
{
    spi_init();
}

void NCoder730::GetRegisterDump(uint8_t values[9])
{
    for (size_t i = 0; i < 9; i++)
    {
        values[i] = readRegister(registers[i]);
    }
}

void NCoder730::spi_init()
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

uint8_t NCoder730::readRegister(uint8_t reg)
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

uint8_t NCoder730::writeRegister(uint8_t reg, uint8_t value)
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
    return rx_buffer[0];
}

double NCoder730::convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle)
{
    double angleInDegree;
    angleInDegree = (rawAngle * 360.0) / ((double)pow(2, rawAngleDataBitLength));
    return angleInDegree;
}

void NCoder730::writeDefaultConfigurations()
{
    writeRegister(ZERO_SETTING0_REG, 0x00);
    writeRegister(ZERO_SETTING1_REG, 0x00);
    writeRegister(BCT_REG, 0x00);
    writeRegister(TRIMMING_REG, 0x00);
    writeRegister(PPT0_REG, 0xC0);
    writeRegister(PPT1_REG, 0xFF);
    writeRegister(ROT_DIR_REG, 0x00);
}

double NCoder730::readAbsoluteAngle()
{
    uint16_t angle;
    double angleInDegree;
    angle = readAbsoluteAngleRaw16();
    angleInDegree = (angle * 360.0) / 65536.0;
    return angleInDegree;
}


uint16_t NCoder730::readAbsoluteAngleRaw16()
{
    uint16_t rx_buffer = 0;
    spi_transaction_t trans = {
        .length = 16,
        .rx_buffer = &rx_buffer};

    ESP_ERROR_CHECK(spi_device_transmit(spi, &trans));

    return (rx_buffer >> 8) | (rx_buffer << 8);
}

void NCoder730::setZeroPosition(float angle)
{
    uint16_t zero_pos = pow(2, 16) * (1 - (angle / 360.0f));
    writeRegister(ZERO_SETTING0_REG, uint8_t(zero_pos));
    writeRegister(ZERO_SETTING1_REG, uint8_t(zero_pos >> 8));
}

float NCoder730::getZeroPosition()
{
    uint16_t zero_pos = readRegister(ZERO_SETTING1_REG) << 8 | readRegister(ZERO_SETTING0_REG);
    float angle = convertRawAngleToDegree(16, pow(2, 16) - zero_pos);
    return angle;
}

void NCoder730::setBCTValue(uint8_t bct_value)
{
    writeRegister(BCT_REG, bct_value);
}

uint8_t NCoder730::getBCTValue()
{
    return readRegister(BCT_REG);
}

void NCoder730::setETX(bool val)
{
    bool temp = getETY();
    writeRegister(TRIMMING_REG, temp << 1 | val);
}

void NCoder730::setETY(bool val)
{
    bool temp = getETX();
    writeRegister(TRIMMING_REG, val << 1 | temp);
}

bool NCoder730::getETX()
{
    return readRegister(TRIMMING_REG) & 0x1;
}

bool NCoder730::getETY()
{
    return (readRegister(TRIMMING_REG) >> 1) & 0x1;
}

void NCoder730::setPulsePerTurn(uint16_t ppr)
{
    uint16_t val = ppr - 1;
    uint8_t reg_val = readRegister(PPT0_REG);
    writeRegister(PPT0_REG, uint8_t(uint8_t(val & 0x03) << 6) | (reg_val & 0x3F));
    writeRegister(PPT1_REG, uint8_t(uint8_t(val >> 2)));
}

uint16_t NCoder730::getPulsePerTurn()
{
    uint16_t val = (readRegister(PPT1_REG) << 2) | ((readRegister(PPT0_REG) >> 6) & 0x03);
    return (val + 1);
}

void NCoder730::setRotationDirection(bool dir)
{
    writeRegister(ROT_DIR_REG, uint8_t(dir) << 7);
}

bool NCoder730::getRotationDirection()
{
    return readRegister(ROT_DIR_REG);
}

void NCoder730::setMagneticFieldLowThreshold(uint8_t MGLT)
{
    uint8_t reg_val = readRegister(MAG_FIELD_THRESHOLD_REG);
    uint8_t MGLT_val = MGLT;
    writeRegister(MAG_FIELD_THRESHOLD_REG, uint8_t((reg_val & 0x1F) | (MGLT_val << 5)));
}

void NCoder730::setMagneticFieldHighThreshold(uint8_t MGHT)
{
    uint8_t reg_val = readRegister(MAG_FIELD_THRESHOLD_REG);
    uint8_t MGHT_val = MGHT;
    writeRegister(MAG_FIELD_THRESHOLD_REG, uint8_t((reg_val & 0xE3) | (MGHT_val << 2)));
}

uint8_t NCoder730::getMagneticFieldLowThreshold()
{
    return (readRegister(MAG_FIELD_THRESHOLD_REG) >> 5) & 0x07;
}

uint8_t NCoder730::getMagneticFieldHighThreshold()
{
    return (readRegister(MAG_FIELD_THRESHOLD_REG) >> 2) & 0x07;
}

bool NCoder730::getMagneticFieldLowLevelStatus()
{
    return ((readRegister(MAG_FIELD_LEVEL_REG) >> 6) & 0x1);
}

bool NCoder730::getMagneticFieldHighLevelStatus()
{
    return ((readRegister(MAG_FIELD_LEVEL_REG) >> 7) & 0x1);
}

void NCoder730::setIndexLength(float length)
{
    uint8_t val = length * 2 - 1;
    if (val < 0 && val > 3)
        val = 0;
    uint8_t reg_val = readRegister(ILIP_REG);
    writeRegister(ILIP_REG, (reg_val & 0xCF) | (val << 4));
}

float NCoder730::getIndexLength()
{
    return (((readRegister(ILIP_REG) >> 4) & 0x3) + 1.0f) * 0.50f;
}

void NCoder730::setIndexPosition(uint8_t position)
{
    uint8_t val = -1;
    uint8_t reg_val = readRegister(ILIP_REG);
    val = (((reg_val >> 4) & 0x03) + position) & 0x03;
    writeRegister(ILIP_REG, (reg_val & 0xF3) | (val << 2));
}

uint8_t NCoder730::getIndexPosition()
{
    uint8_t reg_val = readRegister(ILIP_REG);
    uint8_t length_reg_val = (reg_val >> 4) & 0x3;
    uint8_t pos_reg_val = (reg_val >> 2) & 0x3;
    if (pos_reg_val < length_reg_val)
        pos_reg_val = pos_reg_val | 0x4;
    return (pos_reg_val - length_reg_val) & 0x3;
}

void NCoder730::setFilterWindow(uint8_t filter_window)
{
    writeRegister(FW_REG, filter_window);
}

uint8_t NCoder730::getFilterWindow()
{
    uint8_t reg_val = readRegister(FW_REG);
    uint8_t filter_window = reg_val;
    return filter_window;
}