#include "ADNS9800.h"

ADNS9800::ADNS9800(int cs_pin)
{
    _cs_pin = cs_pin;
    _initialized = false;
}

void ADNS9800::begin()
{
    // Initialize the CS pin
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);

    // Start SPI
    SPI.begin();
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(8); // SPI speed

    // Initialize the sensor
    performStartup();

    // Set initialization flag
    _initialized = true;

    Serial.println("ADNS9800 sensor initialized");
}

bool ADNS9800::readMotion(int16_t *dx, int16_t *dy)
{
    if (!_initialized)
        return false;

    // Check if motion occurred
    uint8_t motion = readRegister(REG_Motion);
    if (!(motion & 0x80))
    {
        *dx = 0;
        *dy = 0;
        return false; // No motion detected
    }

    // Read X and Y movement data
    int16_t rawX = (int16_t)((readRegister(REG_Delta_X_H) << 8) | readRegister(REG_Delta_X_L));
    int16_t rawY = (int16_t)((readRegister(REG_Delta_Y_H) << 8) | readRegister(REG_Delta_Y_L));

    // Invert X-axis for more natural movement
    *dx = -rawX;
    *dy = rawY;

    // 添加更严格的噪声过滤
    if (abs(*dx) < 3 && abs(*dy) < 3)
    {
        *dx = 0;
        *dy = 0;
    }

    return true;
}

uint8_t ADNS9800::readRegister(uint8_t reg_addr)
{
    com_begin();

    // Send register address with MSB = 0 for read
    SPI.transfer(reg_addr & 0x7F);
    delayMicroseconds(200); // tSRAD delay

    // Read data
    uint8_t data = SPI.transfer(0);

    delayMicroseconds(2); // tSCLK-NCS for read (min 120ns)
    com_end();
    delayMicroseconds(38); // tSRW/tSRR (=20us) minus tSCLK-NCS

    return data;
}

void ADNS9800::writeRegister(uint8_t reg_addr, uint8_t data)
{
    com_begin();

    // Send register address with MSB = 1 for write
    SPI.transfer(reg_addr | 0x80);

    // Send data
    SPI.transfer(data);

    delayMicroseconds(20); // tSCLK-NCS for write (min 120ns)
    com_end();
    delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS
}

void ADNS9800::displayRegisters()
{
    Serial.println("ADNS9800 Registers:");

    // Product ID
    Serial.print("Product_ID (0x00): 0x");
    Serial.println(readRegister(REG_Product_ID), HEX);

    // Inverse Product ID
    Serial.print("Inverse_Product_ID (0x3F): 0x");
    Serial.println(readRegister(REG_Inverse_Product_ID), HEX);

    // SROM Version
    Serial.print("SROM_Version (0x2A): 0x");
    Serial.println(readRegister(REG_SROM_ID), HEX);

    // Motion register
    Serial.print("Motion (0x02): 0b");
    Serial.println(readRegister(REG_Motion), BIN);

    // Surface quality
    Serial.print("SQUAL (0x07): 0x");
    Serial.println(readRegister(REG_SQUAL), HEX);
}

bool ADNS9800::isMotionDetected()
{
    uint8_t motion = readRegister(REG_Motion);
    return (motion & 0x80) != 0; // Bit 7 indicates motion
}

void ADNS9800::com_begin()
{
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(5); // add short delay to ensure signal stability
}

void ADNS9800::com_end()
{
    delayMicroseconds(5); // add short delay to ensure signal stability
    digitalWrite(_cs_pin, HIGH);
}

void ADNS9800::uploadFirmware()
{
    Serial.println("Uploading firmware...");

    // Set the Configuration_IV register in 3k firmware mode
    writeRegister(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode

    // Initialize SROM download
    writeRegister(REG_SROM_Enable, 0x1D);

    // Wait for more than one frame period
    delay(10); // Assume frame rate is never below 100fps

    // Start SROM download
    writeRegister(REG_SROM_Enable, 0x18);

    // Write the firmware data
    com_begin();
    SPI.transfer(REG_SROM_Load_Burst | 0x80);
    delayMicroseconds(15);

    // Send all bytes of the firmware
    for (int i = 0; i < firmware_length; i++)
    {
        unsigned char c = pgm_read_byte(firmware_data + i);
        SPI.transfer(c);
        delayMicroseconds(15);
    }

    com_end();
    Serial.println("Firmware upload complete");
}

void ADNS9800::performStartup()
{
    // Reset the sensor
    com_end();
    com_begin();
    com_end();

    // Force reset
    writeRegister(REG_Power_Up_Reset, 0x5A);
    delay(50); // Wait for reboot

    // Clear motion data registers
    readRegister(REG_Motion);
    readRegister(REG_Delta_X_L);
    readRegister(REG_Delta_X_H);
    readRegister(REG_Delta_Y_L);
    readRegister(REG_Delta_Y_H);

    // Upload the firmware
    uploadFirmware();
    delay(10);

    // Enable laser in normal mode
    // Important: read the current value and only modify bit 0
    uint8_t laser_ctrl0 = readRegister(REG_LASER_CTRL0);
    writeRegister(REG_LASER_CTRL0, laser_ctrl0 & 0xF0);

    delay(1);
}

void ADNS9800::resetAccumulation()
{
    // read and discard current motion data, force clear internal accumulation
    readRegister(REG_Motion);
    readRegister(REG_Delta_X_L);
    readRegister(REG_Delta_X_H);
    readRegister(REG_Delta_Y_L);
    readRegister(REG_Delta_Y_H);
}