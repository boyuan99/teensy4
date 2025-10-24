#ifndef PMW3389DM_H
#define PMW3389DM_H

#include <Arduino.h>
#include <SPI.h>

// SPI Settings for Teensy 4.0 compatibility (2MHz like ref/Arduino ADNS)
#define PMW3389_SPI_SPEED 2000000
#define PMW3389_SPI_MODE SPI_MODE3
#define PMW3389_SPI_ORDER MSBFIRST

// PMW3389DM Register Addresses
#define Product_ID  0x00
#define Revision_ID 0x01
#define Motion  0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum  0x08
#define Maximum_Raw_data  0x09
#define Minimum_Raw_data  0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Config1 0x0F
#define Config2 0x10
#define Angle_Tune  0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower  0x15
#define Rest1_Rate_Upper  0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower  0x18
#define Rest2_Rate_Upper  0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower  0x1B
#define Rest3_Rate_Upper  0x1C
#define Observation 0x24
#define Data_Out_Lower  0x25
#define Data_Out_Upper  0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run  0x2B
#define Raw_Data_Threshold  0x2C
#define Config5 0x2F
#define Power_Up_Reset  0x3A
#define Shutdown  0x3B
#define Inverse_Product_ID  0x3F
#define LiftCutoff_Tune3  0x41
#define Angle_Snap  0x42
#define LiftCutoff_Tune1  0x4A
#define Motion_Burst  0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length  0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst  0x64
#define LiftCutoff_Tune2  0x65

// Burst mode data structure
struct BurstData {
    byte motion;         // Motion register (bit 7: motion detected, bit 3: on surface)
    byte observation;    // Observation register
    int16_t deltaX;      // X movement (16-bit signed)
    int16_t deltaY;      // Y movement (16-bit signed)
    byte squal;          // Surface quality (0-0x80)
    byte rawDataSum;     // Raw data sum
    byte maxRawData;     // Maximum raw data
    byte minRawData;     // Minimum raw data
    byte shutterUpper;   // Shutter upper byte
    byte shutterLower;   // Shutter lower byte
};

class PMW3389DM {
public:
    // Constructor
    PMW3389DM(int chipSelectPin, int motionPin = -1);

    // Initialization
    void begin();
    void performStartup();
    void uploadFirmware();
    void displayRegisters();

    // SPI Communication
    byte readRegister(byte reg_addr);
    void writeRegister(byte reg_addr, byte data);

    // Motion Reading
    void updateMotion();              // Legacy mode (individual register reads)
    void updateMotionBurst();         // Burst mode (recommended)
    void readBurstData(BurstData& data);  // Read burst buffer into struct

    int getX();
    int getY();
    bool hasMotion();
    bool isOnSurface();
    byte getSurfaceQuality();

    // Interrupt callback (must be public for attachInterrupt)
    void handleInterrupt();

private:
    int _ncs;           // Chip select pin
    int _motionPin;     // Motion interrupt pin
    byte _initComplete;
    volatile int16_t _xydat[2];  // 16-bit signed integers for full delta range
    volatile byte _movementFlag;
    bool _inBurst;      // Burst mode state
    BurstData _burstData;  // Store last burst read

    // Helper functions
    void comBegin();
    void comEnd();
    int16_t convTwosComp(int16_t b);
};

#endif // PMW3389DM_H
