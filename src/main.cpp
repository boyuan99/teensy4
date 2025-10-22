/*

 * Dual PMW3389DM Optical Sensor System
   *
 * This system uses two PMW3389DM optical sensors to track ball movement
 * in a dual-axis configuration. The sensors are read in burst mode for
 * efficient data acquisition.
   *
 * Pin Configuration:
 * - Sensor A (Left):  CS = Pin 10
 * - Sensor B (Right): CS = Pin 9
     */

#include <Arduino.h>
#include <PMW3389DM.h>

// ============================================================================
//                           Hardware Configuration
// ============================================================================

// Pin definitions
const int CS_PIN_A = 10;  // Left sensor chip select
const int CS_PIN_B = 9;   // Right sensor chip select

// Create sensor objects
PMW3389DM sensorA(CS_PIN_A);
PMW3389DM sensorB(CS_PIN_B);

// ============================================================================
//                           Timing Variables
// ============================================================================

unsigned long lastTS = 0;
unsigned long timer = 0;
byte testCounter = 0;

// Movement tracking
long totalX_A = 0;
long totalY_A = 0;
long totalX_B = 0;
long totalY_B = 0;
bool firstReadDone = false;  // Flag to skip first read after initialization

// ============================================================================
//                           Calibration Parameters
// ============================================================================

// Sensor positioning (angle between sensors in degrees)
const float SENSOR_ANGLE_DEGREES = 78.0;
const float SENSOR_ANGLE_RADIANS = SENSOR_ANGLE_DEGREES * PI / 180.0;

// Sensor calibration factors
const float SENSOR_A_CALIBRATION = 1.00;  // Left sensor calibration
const float SENSOR_B_CALIBRATION = 1.00;  // Right sensor calibration

// Combined movement (calculated from both sensors)
float combinedX = 0;
float combinedY = 0;
float combinedTheta = 0;

// ============================================================================
//                              Setup & Loop
// ============================================================================

void setup() {
    Serial.begin(115200);

    // Wait for serial connection
    delay(100);

    Serial.println("Dual PMW3389DM Sensor System");
    Serial.println("Initializing sensors...");

    // Initialize sensor A (left)
    Serial.println("Initializing Sensor A (Left)...");
    sensorA.begin();
    delay(50);

    // Initialize sensor B (right)
    Serial.println("Initializing Sensor B (Right)...");
    sensorB.begin();
    delay(50);

    Serial.println("Setup complete!");

    // Clear sensor buffers
    Serial.println("Clearing sensor buffers...");
    for(int i = 0; i < 10; i++) {
        sensorA.updateMotionBurst();
        sensorB.updateMotionBurst();
        delay(10);
    }
    Serial.println("Buffers cleared!");

    lastTS = micros();

    // Print header
    Serial.println("Time(us) | A_X | A_Y | A_SQUAL | A_Surface | B_X | B_Y | B_SQUAL | B_Surface | TotalA_X | TotalA_Y | TotalB_X | TotalB_Y");
}

void loop() {
    unsigned long currTime = micros();
    unsigned long elapsed = currTime - lastTS;

    // Print a counter every 2 seconds for debugging
    if((currTime - timer) >= 2000000) {  // 2 seconds in microseconds
        Serial.print("Heartbeat: ");
        Serial.println(testCounter++);
        timer = currTime;
    }

    // Poll sensors every 1ms (1000 microseconds) using burst mode
    if(elapsed >= 1000) {
        // Update both sensors
        sensorA.updateMotionBurst();
        sensorB.updateMotionBurst();

        // Skip the very first read to ensure clean data
        if(!firstReadDone) {
            firstReadDone = true;
            lastTS = currTime;
            return;
        }

        // Check if either sensor has motion
        bool motionA = sensorA.hasMotion();
        bool motionB = sensorB.hasMotion();

        if(motionA || motionB) {
            // Read sensor A data
            int x_a = sensorA.getX();
            int y_a = sensorA.getY();
            byte squal_a = sensorA.getSurfaceQuality();
            bool surface_a = sensorA.isOnSurface();

            // Read sensor B data
            int x_b = sensorB.getX();
            int y_b = sensorB.getY();
            byte squal_b = sensorB.getSurfaceQuality();
            bool surface_b = sensorB.isOnSurface();

            // Update totals
            totalX_A += x_a;
            totalY_A += y_a;
            totalX_B += x_b;
            totalY_B += y_b;

            // Print data
            Serial.print(currTime);
            Serial.print(" | ");

            // Sensor A data
            Serial.print(x_a);
            Serial.print(" | ");
            Serial.print(y_a);
            Serial.print(" | ");
            Serial.print(squal_a);
            Serial.print(" | ");
            Serial.print(surface_a ? "ON" : "OFF");
            Serial.print(" | ");

            // Sensor B data
            Serial.print(x_b);
            Serial.print(" | ");
            Serial.print(y_b);
            Serial.print(" | ");
            Serial.print(squal_b);
            Serial.print(" | ");
            Serial.print(surface_b ? "ON" : "OFF");
            Serial.print(" | ");

            // Cumulative data
            Serial.print(totalX_A);
            Serial.print(" | ");
            Serial.print(totalY_A);
            Serial.print(" | ");
            Serial.print(totalX_B);
            Serial.print(" | ");
            Serial.println(totalY_B);
        }

        lastTS = currTime;
    }
}
