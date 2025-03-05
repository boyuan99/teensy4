#include <Arduino.h>
#include "ADNS9800.h"

// Pin definitions for two sensors
#define CS_PIN_1 9  // First sensor chip select pin
#define CS_PIN_2 10 // Second sensor chip select pin

// Create two ADNS9800 sensor objects
ADNS9800 sensor1(CS_PIN_1);
ADNS9800 sensor2(CS_PIN_2);

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
    Serial.println("ADNS9800 Sensor Test Utility");
    Serial.println("=================================");

    // Initialize both sensors
    Serial.println("Initializing sensor 1...");
    sensor1.begin();

    delay(500); // Short delay between sensor initializations

    Serial.println("\nInitializing sensor 2...");
    sensor2.begin();

    Serial.println("\nBoth sensors initialized. Running diagnostic check.");

    // Run a single check on startup
    checkSensors();

    // Run periodic checks every 5 seconds
    while (true)
    {
        delay(5000);
        Serial.println("\nPerforming periodic sensor check...");
        checkSensors();
    }
}

void loop()
{
    // Empty - not used for test
}