/**
 * test_imu.cpp — Standalone MPU-6050 smoke test  (T-013)
 * =======================================================
 * Flash this sketch independently (comment out the main.cpp tasks).
 *
 * WHAT TO EXPECT:
 *   Open Serial Monitor at 115200 baud.
 *   Place the board flat on a desk.
 *   You should see Roll and Pitch within ±5° of 0.
 *   Tilt the board left/right — Roll should change.
 *   Tilt the board nose-up/down — Pitch should change.
 *
 * WIRING (MPU-6050 → Arduino Mega):
 *   VCC  → 3.3 V  (some modules have onboard regulator — check yours!)
 *   GND  → GND
 *   SDA  → Pin 20 (SDA)
 *   SCL  → Pin 21 (SCL)
 *   AD0  → GND    (sets I²C address to 0x68)
 *   INT  → not connected for this test
 */

// ── Un-comment the next line to enable this test sketch ─────────────────────
// #define RUN_TEST_IMU

#ifdef RUN_TEST_IMU

#include <Arduino.h>
#include <Wire.h>
#include "src/sensors/imu.h"
#include "src/task_defs.h"

// Dummy mutex required by imu_read (not really used in single-task test)
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

    Serial.println(F("=== MPU-6050 IMU Smoke Test ==="));
    if (!imu_init()) {
        Serial.println(F("ERROR: MPU-6050 not found!"));
        for (;;) {}
    }
    Serial.println(F("Sensor OK. Reading at 10 Hz..."));
    Serial.println(F("ax(g), ay(g), az(g), gx(r/s), gy(r/s), gz(r/s), Roll(°), Pitch(°)"));
}

void loop() {
    IMUData_t data;
    imu_read(&data);

    float roll, pitch;
    imu_accel_angles(data, roll, pitch);

    Serial.print(data.ax,  3); Serial.print(F(", "));
    Serial.print(data.ay,  3); Serial.print(F(", "));
    Serial.print(data.az,  3); Serial.print(F(", "));
    Serial.print(data.gx,  4); Serial.print(F(", "));
    Serial.print(data.gy,  4); Serial.print(F(", "));
    Serial.print(data.gz,  4); Serial.print(F(", "));
    Serial.print(roll,     2); Serial.print(F(", "));
    Serial.println(pitch,  2);

    delay(100);  // 10 Hz
}

#endif  // RUN_TEST_IMU
