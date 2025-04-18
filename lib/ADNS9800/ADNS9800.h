#ifndef ADNS9800_H
#define ADNS9800_H

#include <Arduino.h>
#include <SPI.h>
#include "ADNS9800_SROM_A4.h"

// Result calculation methods
#define CALC_MEDIAN 0
#define CALC_AVG 1

// ADNS9800 Register Map
#define REG_Product_ID 0x00
#define REG_Revision_ID 0x01
#define REG_Motion 0x02
#define REG_Delta_X_L 0x03
#define REG_Delta_X_H 0x04
#define REG_Delta_Y_L 0x05
#define REG_Delta_Y_H 0x06
#define REG_SQUAL 0x07
#define REG_Pixel_Sum 0x08
#define REG_Maximum_Pixel 0x09
#define REG_Minimum_Pixel 0x0a
#define REG_Shutter_Lower 0x0b
#define REG_Shutter_Upper 0x0c
#define REG_Frame_Period_Lower 0x0d
#define REG_Frame_Period_Upper 0x0e
#define REG_Configuration_I 0x0f
#define REG_Configuration_II 0x10
#define REG_Frame_Capture 0x12
#define REG_SROM_Enable 0x13
#define REG_Run_Downshift 0x14
#define REG_Rest1_Rate 0x15
#define REG_Rest1_Downshift 0x16
#define REG_Rest2_Rate 0x17
#define REG_Rest2_Downshift 0x18
#define REG_Rest3_Rate 0x19
#define REG_Frame_Period_Max_Bound_Lower 0x1a
#define REG_Frame_Period_Max_Bound_Upper 0x1b
#define REG_Frame_Period_Min_Bound_Lower 0x1c
#define REG_Frame_Period_Min_Bound_Upper 0x1d
#define REG_Shutter_Max_Bound_Lower 0x1e
#define REG_Shutter_Max_Bound_Upper 0x1f
#define REG_LASER_CTRL0 0x20
#define REG_Observation 0x24
#define REG_Data_Out_Lower 0x25
#define REG_Data_Out_Upper 0x26
#define REG_SROM_ID 0x2a
#define REG_Lift_Detection_Thr 0x2e
#define REG_Configuration_V 0x2f
#define REG_Configuration_IV 0x39
#define REG_Power_Up_Reset 0x3a
#define REG_Shutdown 0x3b
#define REG_Inverse_Product_ID 0x3f
#define REG_Motion_Burst 0x50
#define REG_SROM_Load_Burst 0x62
#define REG_Pixel_Burst 0x64

class ADNS9800
{
public:
    // Constructor
    ADNS9800(int cs_pin);

    // Initialize the sensor
    void begin();

    // Read motion data
    bool readMotion(int16_t *dx, int16_t *dy);

    // Read a register
    uint8_t readRegister(uint8_t reg_addr);

    // Write to a register
    void writeRegister(uint8_t reg_addr, uint8_t data);

    // Display sensor information
    void displayRegisters();

    // Check if motion is detected
    bool isMotionDetected();

    // Reset accumulation
    void resetAccumulation();

    // Read motion data with filtering and optional calculation method
    bool readMotionFiltered(float *dx, float *dy, int samples = 5, uint8_t calcMethod = CALC_MEDIAN);

private:
    int _cs_pin;       // Chip select pin
    bool _initialized; // Flag to track initialization status

    // SPI communication helpers
    void com_begin();
    void com_end();

    // Firmware upload
    void uploadFirmware();

    // Startup procedure
    void performStartup();

    // Process readings using the selected calculation method
    float processReadings(int16_t samples[], int count, uint8_t calcMethod);
};

#endif // ADNS9800_H