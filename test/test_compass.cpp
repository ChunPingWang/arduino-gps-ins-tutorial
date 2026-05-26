/**
 * test_compass.cpp — QMC5883L Compass smoke test  (T-024)
 * =========================================================
 * WHAT TO EXPECT:
 *   Heading printed at 5 Hz.
 *   Point the board's positive-X axis toward magnetic North.
 *   Heading should read ~0° (or ~360°).
 *   Rotate 90° clockwise → heading increases toward 90°.
 *
 * NOTE: Before trusting the heading, run compass_calibrate() once.
 *       See compass.cpp for instructions.
 *
 * WIRING (QMC5883L → Arduino Mega):
 *   VCC → 3.3 V
 *   GND → GND
 *   SDA → Pin 20
 *   SCL → Pin 21
 */

// #define RUN_TEST_COMPASS

#ifdef RUN_TEST_COMPASS

#include <Arduino.h>
#include <Wire.h>
#include "src/sensors/compass.h"
#include "src/task_defs.h"

SemaphoreHandle_t xI2CMutex = nullptr;
QueueHandle_t xQueueIMU = nullptr;
QueueHandle_t xQueueMAG = nullptr;
QueueHandle_t xQueueGPS = nullptr;
SemaphoreHandle_t xNavMutex = nullptr;
NavOut_t navOut = {};

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    Wire.begin();
    Wire.setClock(400000UL);

    Serial.println(F("=== QMC5883L Compass Smoke Test ==="));
    compass_init();

    // Uncomment to run calibration:
    // compass_calibrate();

    Serial.println(F("mx, my, mz, Heading(°)"));
}

void loop() {
    MagData_t data;
    compass_read(&data);

    Serial.print(data.mx,      1); Serial.print(F(", "));
    Serial.print(data.my,      1); Serial.print(F(", "));
    Serial.print(data.mz,      1); Serial.print(F(", "));
    Serial.println(data.heading, 1);

    delay(200);  // 5 Hz
}

#endif  // RUN_TEST_COMPASS
