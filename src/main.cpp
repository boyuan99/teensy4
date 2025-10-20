/*
 * PMW3389DM Optical Sensor Example - Burst Mode Polling
 *
 * This example uses burst mode to efficiently read motion data from the sensor
 * at regular intervals. Burst mode reads all sensor data in a single SPI transaction.
 */

#include <Arduino.h>
#include <PMW3389DM.h>

// Pin definitions
const int CHIP_SELECT_PIN = 10;  // SPI slave select pin

// Create sensor object
PMW3389DM sensor(CHIP_SELECT_PIN);

// Timing variables
unsigned long lastTS = 0;
unsigned long timer = 0;
byte testCounter = 0;

long totalX = 0;
long totalY = 0;
bool firstReadDone = false;  // Flag to skip first read after initialization

void setup() {
    Serial.begin(115200);

    // Initialize the sensor
    sensor.begin();

    Serial.println("Setup complete!");

    // Perform a few dummy reads to clear any accumulated data in the sensor
    Serial.println("Clearing sensor buffer...");
    for(int i = 0; i < 10; i++) {
        sensor.updateMotionBurst();
        delay(10);
    }
    Serial.println("Buffer cleared!");

    lastTS = micros();
}

void loop() {
    unsigned long currTime = micros();
    unsigned long elapsed = currTime - lastTS;

    // Print a counter every 2 seconds for debugging
    if((currTime - timer) >= 2000000) {  // 2 seconds in microseconds
        Serial.println(testCounter++);
        timer = currTime;
    }

    // Poll sensor every 1ms (1000 microseconds) using burst mode
    if(elapsed >= 1000) {
        sensor.updateMotionBurst();

        // Skip the very first read to ensure clean data
        if(!firstReadDone) {
            firstReadDone = true;
            lastTS = currTime;
            return;
        }

        // Only print if motion is detected
        if(sensor.hasMotion()) {
            int x = sensor.getX();
            int y = sensor.getY();
            byte squal = sensor.getSurfaceQuality();

            totalX += x;
            totalY += y;

            Serial.print("x = ");
            Serial.print(x);
            Serial.print(" | y = ");
            Serial.print(y);
            Serial.print(" | totalX = ");
            Serial.print(totalX);
            Serial.print(" | totalY = ");
            Serial.print(totalY);
            Serial.print(" | squal = ");
            Serial.print(squal);
            Serial.print(" | surface = ");
            Serial.println(sensor.isOnSurface() ? "ON" : "OFF");
        }

        lastTS = currTime;
    }
}
