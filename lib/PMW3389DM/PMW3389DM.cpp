#include "PMW3389DM.h"
#include <pgmspace.h>
#include "SROM.h"

// // External firmware data from SROM.cpp
// extern const unsigned short firmware_length;
// extern const unsigned char firmware_data[];

PMW3389DM::PMW3389DM(int chipSelectPin, int motionPin) {
    _ncs = chipSelectPin;
    _motionPin = motionPin;
    _initComplete = 0;
    _xydat[0] = 0;
    _xydat[1] = 0;
    _movementFlag = 0;
    _inBurst = false;
}

void PMW3389DM::begin() {
    pinMode(_ncs, OUTPUT);

    if (_motionPin >= 0) {
        pinMode(_motionPin, INPUT);
        digitalWrite(_motionPin, HIGH);
    }

    SPI.begin();
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV128);

    performStartup();

    delay(5000);

    displayRegisters();
    _initComplete = 9;
}

void PMW3389DM::comBegin() {
    digitalWrite(_ncs, LOW);
}

void PMW3389DM::comEnd() {
    digitalWrite(_ncs, HIGH);
}

byte PMW3389DM::readRegister(byte reg_addr) {
    comBegin();

    // Send address of the register, with MSBit = 0 to indicate it's a read
    SPI.transfer(reg_addr & 0x7f);
    delayMicroseconds(100); // tSRAD
    // Read data
    byte data = SPI.transfer(0);

    delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
    comEnd();
    delayMicroseconds(19); // tSRW/tSRR (=20us) minus tSCLK-NCS

    return data;
}

void PMW3389DM::writeRegister(byte reg_addr, byte data) {
    comBegin();

    // Send address of the register, with MSBit = 1 to indicate it's a write
    SPI.transfer(reg_addr | 0x80);
    // Send data
    SPI.transfer(data);

    delayMicroseconds(20); // tSCLK-NCS for write operation
    comEnd();
    delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS
}

void PMW3389DM::uploadFirmware() {
    // Send the firmware to the chip, cf p.18 of the datasheet
    Serial.println("Uploading firmware...");

    // Write 0 to Rest_En bit of Config2 register to disable Rest mode.
    writeRegister(Config2, 0x20);

    // Write 0x1d in SROM_enable reg for initializing
    writeRegister(SROM_Enable, 0x1d);

    // Wait for more than one frame period
    delay(10); // Assume that the frame rate is as low as 100fps

    // Write 0x18 to SROM_enable to start SROM download
    writeRegister(SROM_Enable, 0x18);

    // Write the SROM file (=firmware data)
    comBegin();
    SPI.transfer(SROM_Load_Burst | 0x80); // Write burst destination address
    delayMicroseconds(15);

    // Send all bytes of the firmware
    unsigned char c;
    for(int i = 0; i < firmware_length; i++) {
        c = (unsigned char)pgm_read_byte(firmware_data + i);
        SPI.transfer(c);
        delayMicroseconds(15);
    }

    // Read the SROM_ID register to verify the ID before any other register reads or writes.
    readRegister(SROM_ID);

    // Write 0x00 to Config2 register for wired mouse or 0x20 for wireless mouse design.
    writeRegister(Config2, 0x00);

    // Set initial CPI resolution
    writeRegister(Config1, 0x15);

    comEnd();
}

void PMW3389DM::performStartup() {
    comEnd();   // Ensure that the serial port is reset
    comBegin(); // Ensure that the serial port is reset
    comEnd();   // Ensure that the serial port is reset
    writeRegister(Power_Up_Reset, 0x5a); // Force reset
    delay(50); // Wait for it to reboot

    // Read registers 0x02 to 0x06 (and discard the data)
    readRegister(Motion);
    readRegister(Delta_X_L);
    readRegister(Delta_X_H);
    readRegister(Delta_Y_L);
    readRegister(Delta_Y_H);

    // Upload the firmware
    uploadFirmware();
    delay(10);
    Serial.println("Optical Chip Initialized");
}

void PMW3389DM::updateMotion() {
    if(_initComplete == 9) {
        // Write 0x01 to Motion register and read from it to freeze the motion values and make them available
        writeRegister(Motion, 0x01);
        readRegister(Motion);

        // Read both low and high bytes for full 16-bit signed values
        int16_t xLow = readRegister(Delta_X_L);
        int16_t xHigh = readRegister(Delta_X_H);
        int16_t yLow = readRegister(Delta_Y_L);
        int16_t yHigh = readRegister(Delta_Y_H);

        // Combine low and high bytes into 16-bit signed integers
        _xydat[0] = (xHigh << 8) | xLow;
        _xydat[1] = (yHigh << 8) | yLow;

        _movementFlag = 1;
    }
}

void PMW3389DM::handleInterrupt() {
    updateMotion();
}

void PMW3389DM::displayRegisters() {
    int oreg[4] = {0x00, 0x3F, 0x2A, 0x02};
    const char* oregname[] = {"Product_ID", "Inverse_Product_ID", "SROM_Version", "Motion"};
    byte regres;

    digitalWrite(_ncs, LOW);

    for(int rctr = 0; rctr < 4; rctr++) {
        SPI.transfer(oreg[rctr]);
        delay(1);
        Serial.println("---");
        Serial.println(oregname[rctr]);
        Serial.println(oreg[rctr], HEX);
        regres = SPI.transfer(0);
        Serial.println(regres, BIN);
        Serial.println(regres, HEX);
        delay(1);
    }
    digitalWrite(_ncs, HIGH);
}

int16_t PMW3389DM::convTwosComp(int16_t b) {
    // Convert from 2's complement for 16-bit values
    // Note: For int16_t, the sign bit is already handled by the type itself
    // So we just return the value as-is since it's already a signed type
    return b;
}

int PMW3389DM::getX() {
    return convTwosComp(_xydat[0]);
}

int PMW3389DM::getY() {
    return convTwosComp(_xydat[1]);
}

bool PMW3389DM::hasMotion() {
    if(_movementFlag) {
        _movementFlag = 0;
        return true;
    }
    return false;
}

void PMW3389DM::readBurstData(BurstData& data) {
    byte burstBuffer[12];

    comBegin();
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));

    SPI.transfer(Motion_Burst);
    delayMicroseconds(35); // tSRAD - wait for data to be ready

    SPI.transfer(burstBuffer, 12); // Read 12 bytes in one burst
    delayMicroseconds(1); // tSCLK-NCS for read operation

    SPI.endTransaction();
    comEnd();

    // Parse the burst buffer into the data structure
    /*
    BYTE[00] = Motion       - bit 7: motion detected, bit 3: on surface (0=on, 1=off)
    BYTE[01] = Observation
    BYTE[02] = Delta_X_L    - X movement low byte
    BYTE[03] = Delta_X_H    - X movement high byte
    BYTE[04] = Delta_Y_L    - Y movement low byte
    BYTE[05] = Delta_Y_H    - Y movement high byte
    BYTE[06] = SQUAL        - Surface quality (max 0x80)
    BYTE[07] = Raw_Data_Sum
    BYTE[08] = Maximum_Raw_Data
    BYTE[09] = Minimum_Raw_Data
    BYTE[10] = Shutter_Upper
    BYTE[11] = Shutter_Lower
    */

    data.motion = burstBuffer[0];
    data.observation = burstBuffer[1];
    data.deltaX = (int16_t)((burstBuffer[3] << 8) | burstBuffer[2]);
    data.deltaY = (int16_t)((burstBuffer[5] << 8) | burstBuffer[4]);
    data.squal = burstBuffer[6];
    data.rawDataSum = burstBuffer[7];
    data.maxRawData = burstBuffer[8];
    data.minRawData = burstBuffer[9];
    data.shutterUpper = burstBuffer[10];
    data.shutterLower = burstBuffer[11];
}

void PMW3389DM::updateMotionBurst() {
    if(_initComplete != 9) {
        return;
    }

    // Initialize burst mode if not already started
    if(!_inBurst) {
        writeRegister(Motion_Burst, 0x00);
        _inBurst = true;
    }

    // Read burst data
    readBurstData(_burstData);

    // Update motion data
    _xydat[0] = _burstData.deltaX;
    _xydat[1] = _burstData.deltaY;

    // Set movement flag if motion detected
    if(_burstData.motion & 0x80) {
        _movementFlag = 1;
    }
}

bool PMW3389DM::isOnSurface() {
    // Bit 3 of motion register: 0 = on surface, 1 = off surface
    return (_burstData.motion & 0x08) == 0;
}

byte PMW3389DM::getSurfaceQuality() {
    return _burstData.squal;
}
