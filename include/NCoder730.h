#ifndef NCODER730_H
#define NCODER730_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "math.h"

// Registers of MA730
#define ZERO_SETTING0_REG 0x0
#define ZERO_SETTING1_REG 0x1
#define BCT_REG 0x2
#define TRIMMING_REG 0x3
#define PPT0_REG 0x4
#define ILIP_REG 0x4
#define PPT1_REG 0x5
#define MAG_FIELD_THRESHOLD_REG 0x6
#define ROT_DIR_REG 0x9
#define MAG_FIELD_LEVEL_REG 0x1B
#define FW_REG 0x0E

struct spi_config
{
    spi_host_device_t spi_host;
    gpio_num_t miso_pin;
    gpio_num_t mosi_pin;
    gpio_num_t clk_pin;
    gpio_num_t cs_pin;
};

class NCoder730
{
private:
    spi_device_handle_t spi;
    spi_config config;

    void spi_init();

    /**
     * @brief Reads the SPI registers of NCoder730
     *
     * @param address address whose value is to be read
     * @return returns the value written to the address
     */
    uint8_t readRegister(uint8_t address);
    /**
     * @brief Writes the SPI registors of NCoder730
     *
     * @param address address into which value is to be written
     * @param value value that is to be written
     * @return returns the value in the registor
     */
    uint8_t writeRegister(uint8_t address, uint8_t value);

public:
    uint8_t registers[9] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x09, 0x1B};

    NCoder730(spi_config cfg);

    void GetRegisterDump(uint8_t values[9]);

    /**
     * @brief Function to write default configurations to the registors
     *
     */
    void writeDefaultConfigurations();
    /**
     * @brief Reads absolute encoder angle
     *
     * @return return absolute_angle in degrees
     */
    double readAbsoluteAngle();
    /**
     * @brief Reads the 16 bit Absolute Raw Angle Value
     *
     * @return returns 16 bit raw absolute angle value
     */
    uint16_t readAbsoluteAngleRaw16();
    /**
     * @brief Reads raw value of absolute angle with error check
     *
     * @param error retreive error
     * @return return raw absolute value
     */
    uint16_t readAbsoluteAngleRaw(bool *error);
    /**
     * @brief Reads raw value of absolute angle with 8bit resolution
     *
     * @return returns 8 bit raw absolute value
     */
    uint8_t readAbsoluteAngleRaw8();
    /**
     * @brief Converts raw absolute angle to degrees
     *
     * @param rawAngleDataBitLength data length of raw angle
     * @param rawAngle 16-bit / 8bit raw absolute angle value
     * @return returns absolute angle in degrees
     */
    double convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle);
    /**
     * @brief Set the Zero Position of the NCoder730
     *
     * @param angle offset angle in degrees for zero positioning
     */
    void setZeroPosition(float angle);
    /**
     * @brief Get the Zero Position Offset Angle of the NCoder730
     *
     * @return returns zero position offset angle in degrees
     */
    float getZeroPosition();
    /**
     * @brief Set the value of BCT Settings the NCoder730
     *
     * @param bct_value BCT value
     */
    void setBCTValue(uint8_t bct_value);
    /**
     * @brief Get the BCT settings of the NCoder730
     *
     * @return returns value of the BCT Register
     */
    uint8_t getBCTValue();
    /**
     * @brief Enable/Disable the Enable Trimming of X axis of the NCoder730
     *
     * @param val status of ETX
     */
    void setETX(bool val);
    /**
     * @brief Enable/Disable the Enable Trimming of Y axis of the NCoder730
     *
     * @param val status of ETY
     */
    void setETY(bool val);
    /**
     * @brief Get the Enable Trimming of X axis of the NCoder730
     *
     * @return returns status of ETX
     */
    bool getETX();
    /**
     * @brief Get the Enable Trimming of Y axis of the NCoder730
     *
     * @return returns status of ETX
     */
    bool getETY();
    /**
     * @brief Set the Pulse Per Turn for NCoder730
     *
     * @param ppr Pulse Per Revolution value of the Encoder in Incremental Mode
     * @n CPR is 4 times PPR
     */
    void setPulsePerTurn(uint16_t ppr);
    /**
     * @brief Get the Pulse Per Turn value of Incremental Encoder for NCoder730
     *
     * @return returns the PPR value
     */
    uint16_t getPulsePerTurn();
    /**
     * @brief Set the Index Length
     *
     * @param length is the length of index pulse
     * Four Possible values accepted: Index lenght is
     * 0.5 times the A or B pulse length
     * 1 times the A or B pulse length
     * 1.5 times the A or B pulse length
     * 2 times the A or B pulse length
     * Refer Fig 26 Page No. 23 of MA730 IC datasheet for more details
     */
    void setIndexLength(float length);
    /**
     * @brief Get the Index Length
     *
     * @return returns the length of index pulse with respect to A or B pulse length
     */
    float getIndexLength();
    /**
     * @brief Set the Index Position with respect to channel A or B
     *
     * @param pos is value for setting rising edge of Index pulse with respect to A or B pulse
     * 0 : index rising edge is aligned with the channel B falling edge
     * 1 : index rising edge is aligned with the channel A rising edge
     * 2 : index rising edge is aligned with the channel B rising edge
     * 3 : index rising edge is aligned with the channel A falling edge
     */
    void setIndexPosition(uint8_t pos);
    /**
     * @brief Get the Index Position with respect to channel A or B
     *
     * @return returns the value of pos which is as follows:
     * 0 : index rising edge is aligned with the channel B falling edge
     * 1 : index rising edge is aligned with the channel A rising edge
     * 2 : index rising edge is aligned with the channel B rising edge
     * 3 : index rising edge is aligned with the channel A falling edge
     */
    uint8_t getIndexPosition();
    /**
     * @brief Set the Magnetic Field Low Threshold value for NCoder730
     * Following values can be set: 26mT, 41mT, 56mT, 70mT ,84mT ,98mT ,112mT ,126mT.
     */
    void setMagneticFieldLowThreshold(uint8_t MAGLT);
    /**
     * @brief Set the Magnetic Field High Threshold value for NCoder730
     * Following values can be set: 20mT, 35mT, 50mT, 64mT ,78mT ,92mT ,106mT ,120mT.
     */
    void setMagneticFieldHighThreshold(uint8_t MAGHT);
    /**
     * @brief Get the Magnetic Field Low Threshold of NCoder730
     *
     * @return returns the MGLT value (refer datasheet for more details)
     */
    uint8_t getMagneticFieldLowThreshold();
    /**
     * @brief Get the Magnetic Field High Threshold of NCoder730
     *
     * @return returns the MGHT value (refer datasheet for more details)
     */
    uint8_t getMagneticFieldHighThreshold();
    /**
     * @brief Set the Rotation Direction for the NCoder730
     *
     * @param dir direction value
     * @n true -> clockwise
     * @n false-> anticlockwise
     */
    void setRotationDirection(bool dir);
    /**
     * @brief Get the Rotation Direction of the NCoder730
     *
     * @return returns true if the rotation direction is clockwise
     * @return returns false if the rotation direction is anticlockwise
     */
    bool getRotationDirection();
    /**
     * @brief Set the Filter Window
     * @param filter_window value of the filter window
     */
    void setFilterWindow(uint8_t filter_window);
    /**
     * @brief Get the value of the Filter Window settings of the NCoder730
     *
     * @return returns the value of filter window
     */
    uint8_t getFilterWindow();
    /**
     * @brief Get the Magnetic Field Low Level Status of NCoder730
     *
     * @return returns true if the magnetic field is below the low threshold.
     * @return returns false if the magnetic field is above the low threshold
     */
    bool getMagneticFieldLowLevelStatus();
    /**
     * @brief Get the Magnetic Field High Level Status of NCoder730
     *
     * @return returns true if the magnetic field is above the high threshold.
     * @return returns false if the magnetic field is below the high threshold
     */
    bool getMagneticFieldHighLevelStatus();
};

#endif // NCODER730_H
