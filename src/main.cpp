#include <Arduino.h>

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
}

void loop() {
    // Main loop
    Serial.println("Running...");
    delay(1000);
}
