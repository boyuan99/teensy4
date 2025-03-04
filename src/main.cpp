#include <Arduino.h>
#include "ADNS9800.h"

// Pin definitions for two sensors
#define CS_PIN_1 8  // First sensor chip select pin
#define CS_PIN_2 10 // Second sensor chip select pin

// Create two ADNS9800 sensor objects
ADNS9800 sensor1(CS_PIN_1);
ADNS9800 sensor2(CS_PIN_2);

// Variables to store motion data for both sensors
int16_t deltaX1 = 0, deltaY1 = 0;
int16_t deltaX2 = 0, deltaY2 = 0;

// Timer for 20Hz update rate
unsigned long lastOutputTime = 0;
const int outputInterval = 50; // 50ms = 20Hz

// reset sensor every 10 seconds
unsigned long lastResetTime = 0;
const int resetInterval = 5000;

unsigned long lastCheckTime = 0;
const int checkInterval = 30000;

void checkSensors()
{
    // check sensor 1
    Serial.println("\n=== Checking sensor 1 registers ===");
    digitalWrite(CS_PIN_1, LOW);
    delayMicroseconds(50);
    sensor1.displayRegisters();
    digitalWrite(CS_PIN_1, HIGH);
    delayMicroseconds(200);

    // check sensor 2
    Serial.println("\n=== Checking sensor 2 registers ===");
    digitalWrite(CS_PIN_2, LOW);
    delayMicroseconds(50);
    sensor2.displayRegisters();
    digitalWrite(CS_PIN_2, HIGH);
    delayMicroseconds(200);
}

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);
    delay(1000); // Give serial time to initialize

    Serial.println("\n\n=================================");
    Serial.println("Dual ADNS9800 Optical Sensor Test");
    Serial.println("=================================");

    // Initialize both sensors
    Serial.println("Initializing sensor 1...");
    sensor1.begin();
    sensor1.displayRegisters();

    delay(500); // Short delay between sensor initializations

    Serial.println("\nInitializing sensor 2...");
    sensor2.begin();
    sensor2.displayRegisters();

    Serial.println("\nBoth sensors initialized. Starting main loop.");
    Serial.println("Outputting at 20Hz (every 50ms)");

    checkSensors();
}

void loop()
{
    unsigned long currentTime = millis();

    // Check if it's time to output readings (20Hz)
    if (currentTime - lastOutputTime >= outputInterval)
    {
        digitalWrite(CS_PIN_1, LOW); // activate sensor 1
        delayMicroseconds(50);       // stable signal
        bool motion1 = sensor1.readMotion(&deltaX1, &deltaY1);
        digitalWrite(CS_PIN_1, HIGH); // ensure CS1 high
        delayMicroseconds(200);       // add buffer

        digitalWrite(CS_PIN_2, LOW); // activate sensor 2
        delayMicroseconds(50);       // stable signal
        bool motion2 = sensor2.readMotion(&deltaX2, &deltaY2);
        digitalWrite(CS_PIN_2, HIGH); // ensure CS2 high
        delayMicroseconds(200);       // add buffer

        // Always print data for both sensors at 20Hz with timestamp
        Serial.print("[T:");
        Serial.print(currentTime);
        Serial.print("ms] Sensor1 [");
        if (motion1)
        {
            Serial.print("X: ");
            Serial.print(deltaX1);
            Serial.print(" Y: ");
            Serial.print(deltaY1);
        }
        else
        {
            Serial.print("X: 0 Y: 0");
        }

        Serial.print("] Sensor2 [");
        if (motion2)
        {
            Serial.print("X: ");
            Serial.print(deltaX2);
            Serial.print(" Y: ");
            Serial.print(deltaY2);
        }
        else
        {
            Serial.print("X: 0 Y: 0");
        }
        Serial.println("]");

        // Update the timer
        lastOutputTime = currentTime;
    }

    // reset sensor to prevent accumulation
    if (currentTime - lastResetTime > resetInterval)
    {
        // clean accumulation
        sensor1.resetAccumulation();
        sensor2.resetAccumulation();
        lastResetTime = currentTime;
    }

    // Small delay to prevent CPU hogging
    delayMicroseconds(100);

    if (millis() - lastCheckTime > checkInterval)
    {
        checkSensors();
        lastCheckTime = millis();
    }
}

