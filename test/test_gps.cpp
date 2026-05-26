/**
 * test_gps.cpp — NEO-6M GPS smoke test  (T-033)
 * ===============================================
 * WHAT TO EXPECT:
 *   Take the board outdoors with clear sky view.
 *   LED on NEO-6M module blinks every second when a fix is acquired.
 *   Lat/Lon/Alt/Satellites printed at 1 Hz.
 *   Valid fix should be obtained within 3 minutes (cold start).
 *
 * WIRING (NEO-6M → Arduino Mega):
 *   VCC → 5 V (the module has an onboard 3.3 V regulator)
 *   GND → GND
 *   TX  → Pin 19 (RX1 on Mega)
 *   RX  → Pin 18 (TX1 on Mega) — optional, only for sending NMEA commands
 */

// #define RUN_TEST_GPS

#ifdef RUN_TEST_GPS

#include <Arduino.h>
#include "src/sensors/gps.h"
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

    Serial.println(F("=== NEO-6M GPS Smoke Test ==="));
    gps_init();
    Serial.println(F("Waiting for fix... (go outdoors!)"));
    Serial.println(F("Lat, Lon, Alt(m), Speed(kmh), Sats, Valid"));
}

void loop() {
    GPSData_t data = {};
    // Poll frequently to feed the NMEA parser
    for (int i = 0; i < 50; i++) {
        gps_poll(&data);
        delay(20);
    }

    Serial.print(data.lat,  6); Serial.print(F(", "));
    Serial.print(data.lon,  6); Serial.print(F(", "));
    Serial.print(data.alt,  1); Serial.print(F(", "));
    Serial.print(data.spd,  1); Serial.print(F(", "));
    Serial.print(data.sats);   Serial.print(F(", "));
    Serial.println(data.valid ? F("YES") : F("NO"));
}

#endif  // RUN_TEST_GPS
