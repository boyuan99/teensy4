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
#include <SPI.h>
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
//                           Timing Variables & Control
// ============================================================================

unsigned long lastSampleTS = 0;

// Sampling interval: 50ms = 20 Hz (sample and print together)
const unsigned int SAMPLE_INTERVAL_US = 50000;

// Acquisition control
volatile bool isRunning = false;
unsigned long acquisitionStartTime = 0;
unsigned long acquisitionDuration = 0;  // 0 = continuous mode (seconds)

// Movement tracking
long totalX_A = 0;
long totalY_A = 0;
long totalX_B = 0;
long totalY_B = 0;
bool firstReadDone = false;

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
//                         Function Declarations
// ============================================================================

void parseSerialCommand();
void beginAcquisition(float samplingIntervalMs, float durationSeconds);
void endAcquisition();

// ============================================================================
//                         Movement Processing
// ============================================================================

void processDualSensorMovement(int dx_a, int dy_a, int dx_b, int dy_b) {
    // Apply calibration factors
    float dxA = dx_a * SENSOR_A_CALIBRATION;
    float dyA = dy_a * SENSOR_A_CALIBRATION;
    float dxB = dx_b * SENSOR_B_CALIBRATION;
    float dyB = dy_b * SENSOR_B_CALIBRATION;

    // Calculate combined movement using sensor geometry
    // Method 1: Simple averaging (basic fusion)
    combinedX = (dxA + dxB) / 2.0;
    combinedY = (dyA + dyB) / 2.0;

    // Method 2: Geometric fusion (more accurate for angled sensors)
    // Uncomment below for more sophisticated processing similar to ref/Arduino
    /*
    float effectiveAngle = SENSOR_ANGLE_RADIANS;
    combinedY = dyB;  // Primary forward movement from right sensor
    combinedX = (dyA - dyB * cos(effectiveAngle)) / sin(effectiveAngle);
    */

    // Calculate rotation (if ball is rotating)
    combinedTheta = (dxA + dxB) / 2.0;  // Average rotation from both sensors
}

// ============================================================================
//                              Setup & Loop
// ============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for serial connection (like ref/Arduino)
    }
    delay(100);

    Serial.println("Dual PMW3389DM Sensor System");
    Serial.println("Initializing sensors...");

    // Initialize SPI once before initializing sensors
    // This prevents each sensor from re-initializing the SPI bus
    // Use modern SPISettings for Teensy 4.0 (2MHz like ref/Arduino's ADNS)
    SPI.begin();
    delay(100);

    // Initialize sensor A (left)
    sensorA.begin();
    delay(100);

    // Initialize sensor B (right)
    sensorB.begin();

    Serial.println("Setup complete!");

    // Wait for START command (don't clear buffers yet - wait for command like ref/Arduino)
    Serial.println("\n==============================================");
    Serial.println("System initialized and ready!");
    Serial.println("==============================================");
    Serial.println("Send 'START,<interval_ms>,<duration_s>' to begin");
    Serial.println("Example: START,50,60  (50ms interval, 60 seconds)");
    Serial.println("Example: START,50,0   (50ms interval, continuous)");
    Serial.println("Example: START        (default: 50ms, continuous)");
    Serial.println("Send 'STOP' to stop acquisition");
    Serial.println("==============================================\n");

    // Clear any existing serial data
    while (Serial.available()) {
        Serial.read();
    }
}

void loop() {
    // Check for serial commands
    if (Serial.available() > 0) {
        parseSerialCommand();
    }

    // Only run acquisition if started
    if (!isRunning) {
        return;
    }

    // Check if acquisition duration has elapsed (if duration is set)
    if (acquisitionDuration > 0) {
        unsigned long elapsedSeconds = (millis() - acquisitionStartTime) / 1000;
        if (elapsedSeconds >= acquisitionDuration) {
            Serial.println("\n[INFO] Acquisition duration completed");
            endAcquisition();
            return;
        }
    }

    unsigned long currTime = micros();
    unsigned long elapsed = currTime - lastSampleTS;

    // Sample and print at configured interval
    if (elapsed >= SAMPLE_INTERVAL_US) {
        // Update both sensors
        sensorA.updateMotionBurst();
        sensorB.updateMotionBurst();

        // Skip the very first read to ensure clean data
        if (!firstReadDone) {
            firstReadDone = true;
            lastSampleTS = currTime;
            return;
        }

        // Always read sensor data to keep them synchronized
        int x_a = sensorA.getX();
        int y_a = sensorA.getY();
        byte squal_a = sensorA.getSurfaceQuality();
        bool surface_a = sensorA.isOnSurface();

        int x_b = sensorB.getX();
        int y_b = sensorB.getY();
        byte squal_b = sensorB.getSurfaceQuality();
        bool surface_b = sensorB.isOnSurface();

        // Always update totals (accumulate all displacement)
        totalX_A += x_a;
        totalY_A += y_a;
        totalX_B += x_b;
        totalY_B += y_b;

        // Calculate combined movement
        processDualSensorMovement(x_a, y_a, x_b, y_b);

        // Print data in CSV format (sample and print together)
        Serial.print(currTime);
        Serial.print(",");
        Serial.print(x_a);
        Serial.print(",");
        Serial.print(y_a);
        Serial.print(",");
        Serial.print(squal_a);
        Serial.print(",");
        Serial.print(surface_a ? "1" : "0");
        Serial.print(",");
        Serial.print(x_b);
        Serial.print(",");
        Serial.print(y_b);
        Serial.print(",");
        Serial.print(squal_b);
        Serial.print(",");
        Serial.print(surface_b ? "1" : "0");
        Serial.print(",");
        Serial.print(totalX_A);
        Serial.print(",");
        Serial.print(totalY_A);
        Serial.print(",");
        Serial.print(totalX_B);
        Serial.print(",");
        Serial.println(totalY_B);

        lastSampleTS = currTime;
    }
}

// ============================================================================
//                         Serial Command Processing
// ============================================================================

void parseSerialCommand() {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() == 0) {
        return;
    }

    Serial.print("[CMD] Received: ");
    Serial.println(command);

    // Parse START command
    if (command.startsWith("START")) {
        // Default values
        float intervalMs = 50.0;
        float durationS = 0.0;

        // Parse parameters if provided
        int firstComma = command.indexOf(',');
        if (firstComma > 0) {
            int secondComma = command.indexOf(',', firstComma + 1);

            String intervalStr = command.substring(firstComma + 1,
                                                   secondComma > 0 ? secondComma : command.length());
            intervalMs = intervalStr.toFloat();

            if (secondComma > 0) {
                String durationStr = command.substring(secondComma + 1);
                durationS = durationStr.toFloat();
            }
        }

        // Validate and set parameters
        if (intervalMs < 1.0) intervalMs = 50.0;     // Minimum 1ms
        if (intervalMs > 10000) intervalMs = 10000;  // Maximum 10 seconds

        Serial.println("[CONFIG] Parameters:");
        Serial.print("  - Sampling interval: ");
        Serial.print(intervalMs);
        Serial.println(" ms");
        Serial.print("  - Duration: ");
        if (durationS == 0) {
            Serial.println("Continuous");
        } else {
            Serial.print(durationS);
            Serial.println(" seconds");
        }

        beginAcquisition(intervalMs, durationS);
    }
    // Parse STOP command
    else if (command.equals("STOP")) {
        if (isRunning) {
            Serial.println("[CMD] Stopping acquisition...");
            endAcquisition();
        } else {
            Serial.println("[INFO] Not running, nothing to stop");
        }
    }
    else {
        Serial.println("[ERROR] Unknown command");
        Serial.println("Valid commands: START, STOP");
    }
}

void beginAcquisition(float samplingIntervalMs, float durationSeconds) {
    Serial.println("\n[START] Beginning data acquisition...");

    // Reset tracking variables
    totalX_A = 0;
    totalY_A = 0;
    totalX_B = 0;
    totalY_B = 0;
    firstReadDone = false;

    // Clear sensor buffers
    for (int i = 0; i < 5; i++) {
        sensorA.updateMotionBurst();
        sensorB.updateMotionBurst();
        delay(5);
    }

    // Set acquisition parameters
    acquisitionDuration = (unsigned long)durationSeconds;

    // Send data header
    Serial.println("\n--- Data Header ---");
    Serial.println("Time(us),A_X,A_Y,A_SQUAL,A_Surface,B_X,B_Y,B_SQUAL,B_Surface,TotalA_X,TotalA_Y,TotalB_X,TotalB_Y");
    Serial.println("--- Data Start ---");

    // Start timing
    acquisitionStartTime = millis();
    lastSampleTS = micros();
    isRunning = true;

    Serial.println("[RUNNING] Acquisition started\n");
}

void endAcquisition() {
    isRunning = false;

    Serial.println("\n[STOP] Acquisition ended");
    Serial.println("==============================================");
    Serial.println("Send 'START' to begin new acquisition");
    Serial.println("==============================================\n");
}
