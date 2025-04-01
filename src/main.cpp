#include <Arduino.h>
#include <ADNS9800.h>

// Pin definitions for two sensors
#define CS_PIN_1 9  // First sensor chip select pin
#define CS_PIN_2 10 // Second sensor chip select pin

// Create two ADNS9800 sensor objects
ADNS9800 sensor1(CS_PIN_1);
ADNS9800 sensor2(CS_PIN_2);

// Variables to store motion data for both sensors
int16_t deltaX1 = 0, deltaY1 = 0;
int16_t deltaX2 = 0, deltaY2 = 0;

// Variables to store accumulated positions for both sensors
int32_t accX1 = 0, accY1 = 0;
int32_t accX2 = 0, accY2 = 0;

// Timer for 20Hz update rate
unsigned long lastOutputTime = 0;
const int outputInterval = 50; // 50ms = 20Hz

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
}

void loop()
{
    unsigned long currentTime = millis();

    // Check if it's time to output readings (20Hz)
    if (currentTime - lastOutputTime >= outputInterval)
    {
        digitalWrite(CS_PIN_1, LOW);
        delayMicroseconds(50);
        bool motion1 = sensor1.readMotionFiltered(&deltaX1, &deltaY1);
        digitalWrite(CS_PIN_1, HIGH);
        delayMicroseconds(500);

        digitalWrite(CS_PIN_2, LOW);
        delayMicroseconds(50);
        bool motion2 = sensor2.readMotionFiltered(&deltaX2, &deltaY2);
        digitalWrite(CS_PIN_2, HIGH);

        // Update accumulated positions
        if (motion1)
        {
            accX1 += deltaX1;
            accY1 += deltaY1;
        }
        if (motion2)
        {
            accX2 += deltaX2;
            accY2 += deltaY2;
        }

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
            Serial.print(" AccX: ");
            Serial.print(accX1);
            Serial.print(" AccY: ");
            Serial.print(accY1);
        }
        else
        {
            Serial.print("X: 0 Y: 0");
            Serial.print(" AccX: ");
            Serial.print(accX1);
            Serial.print(" AccY: ");
            Serial.print(accY1);
        }

        Serial.print("] Sensor2 [");
        if (motion2)
        {
            Serial.print("X: ");
            Serial.print(deltaX2);
            Serial.print(" Y: ");
            Serial.print(deltaY2);
            Serial.print(" AccX: ");
            Serial.print(accX2);
            Serial.print(" AccY: ");
            Serial.print(accY2);
        }
        else
        {
            Serial.print("X: 0 Y: 0");
            Serial.print(" AccX: ");
            Serial.print(accX2);
            Serial.print(" AccY: ");
            Serial.print(accY2);
        }
        Serial.println("]");

        // Update the timer
        lastOutputTime = currentTime;
    }

    // Small delay to prevent CPU hogging
    delayMicroseconds(100);
}
