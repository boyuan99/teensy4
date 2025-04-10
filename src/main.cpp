#include <Arduino.h>
#include <ADNS9800.h>

// Pin definitions for two sensors
#define CS_PIN_1 20 // First sensor chip select pin
#define CS_PIN_2 21 // Second sensor chip select pin

// Sample size for motion readings
#define MOTION_SAMPLE_SIZE 5

// Output mode flag - true: print all samples, false: print summary only
bool printAllSamples = false;

// Selected calculation method - CALC_MEDIAN or CALC_AVG
uint8_t selectedCalcMethod = CALC_AVG;

// Create two ADNS9800 sensor objects
ADNS9800 sensor1(CS_PIN_1);
ADNS9800 sensor2(CS_PIN_2);

// Variables to store motion data for both sensors
float deltaX1 = 0, deltaY1 = 0;
float deltaX2 = 0, deltaY2 = 0;

// Variables to store accumulated positions for both sensors
float accX1 = 0, accY1 = 0;
float accX2 = 0, accY2 = 0;

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
        int16_t samplesX1[MOTION_SAMPLE_SIZE], samplesY1[MOTION_SAMPLE_SIZE];
        bool motionDetected = false;

        // Get samples from sensor1
        for (int i = 0; i < MOTION_SAMPLE_SIZE; i++)
        {
            digitalWrite(CS_PIN_1, LOW);
            delayMicroseconds(50);
            bool motion = sensor1.readMotion(&samplesX1[i], &samplesY1[i]);
            digitalWrite(CS_PIN_1, HIGH);
            if (motion)
                motionDetected = true;
            delayMicroseconds(1000);
        }

        // Process sensor1 data using our selected method
        if (motionDetected)
        {
            sensor1.readMotionFiltered(&deltaX1, &deltaY1, MOTION_SAMPLE_SIZE, selectedCalcMethod);
            accX1 += deltaX1;
            accY1 += deltaY1;
        }

        // Read sensor2 data but don't output it
        digitalWrite(CS_PIN_2, LOW);
        delayMicroseconds(50);
        bool motion2 = sensor2.readMotionFiltered(&deltaX2, &deltaY2, MOTION_SAMPLE_SIZE, selectedCalcMethod);
        digitalWrite(CS_PIN_2, HIGH);

        // Update accumulated positions for sensor2
        if (motion2)
        {
            accX2 += deltaX2;
            accY2 += deltaY2;
        }

        // Only print data for sensor1 with all samples
        Serial.print("[T:");
        Serial.print(currentTime);
        Serial.print("ms] Sensor1 [");
        if (motionDetected)
        {
            if (printAllSamples)
            {
                // Print all sample data
                Serial.print("X samples: ");
                for (int i = 0; i < MOTION_SAMPLE_SIZE; i++)
                {
                    Serial.print(samplesX1[i]);
                    if (i < MOTION_SAMPLE_SIZE - 1)
                        Serial.print(", ");
                }
                Serial.print(" | Filtered X: ");
                Serial.print(deltaX1, 2); // Print with 2 decimal places
                Serial.print(" | AccX: ");
                Serial.print(accX1, 2);
            }
            else
            {
                // Print summary information
                Serial.print("X: ");
                Serial.print(deltaX1, 2);
                Serial.print(" Y: ");
                Serial.print(deltaY1, 2);
                Serial.print(" AccX: ");
                Serial.print(accX1, 2);
                Serial.print(" AccY: ");
                Serial.print(accY1, 2);
            }
        }
        else
        {
            if (printAllSamples)
            {
                Serial.print("No motion detected | AccX: ");
                Serial.print(accX1, 2);
            }
            else
            {
                Serial.print("X: 0.00 Y: 0.00");
                Serial.print(" AccX: ");
                Serial.print(accX1, 2);
                Serial.print(" AccY: ");
                Serial.print(accY1, 2);
            }
        }
        Serial.print("] Sensor2 [");
        if (motion2)
        {
            if (printAllSamples)
            {
                Serial.print("X: ");
                Serial.print(deltaX2, 2);
                Serial.print(" | AccX: ");
                Serial.print(accX2, 2);
            }
            else
            {
                Serial.print("X: ");
                Serial.print(deltaX2, 2);
                Serial.print(" Y: ");
                Serial.print(deltaY2, 2);
                Serial.print(" AccX: ");
                Serial.print(accX2, 2);
                Serial.print(" AccY: ");
                Serial.print(accY2, 2);
            }
        }
        else
        {
            if (printAllSamples)
            {
                Serial.print("No motion detected | AccX: ");
                Serial.print(accX2, 2);
            }
            else
            {
                Serial.print("X: 0.00 Y: 0.00");
                Serial.print(" AccX: ");
                Serial.print(accX2, 2);
                Serial.print(" AccY: ");
                Serial.print(accY2, 2);
            }
        }
        Serial.println("]");

        // Update the timer
        lastOutputTime = currentTime;
    }

    // Small delay to prevent CPU hogging
    delayMicroseconds(100);
}
