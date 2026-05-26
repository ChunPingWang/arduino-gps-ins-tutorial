/**
 * gps.cpp — NEO-6M GPS driver implementation.
 */

#include "gps.h"

// ── Module-level TinyGPSPlus parser object ──────────────────────────────────
static TinyGPSPlus gps;

// ===========================================================================
void gps_init()
// ===========================================================================
{
    // NEO-6M default baud rate is 9600
    Serial1.begin(9600);

#ifdef DEBUG_SERIAL
    Serial.println(F("[GPS] Serial1 opened at 9600 baud"));
    Serial.println(F("[GPS] Waiting for fix — go outdoors!"));
#endif
}

// ===========================================================================
void gps_poll(GPSData_t *out)
// ===========================================================================
{
    // Feed every available byte to the NMEA parser
    while (Serial1.available() > 0) {
        gps.encode(Serial1.read());
    }

    // Report the latest fix data whenever TinyGPS signals an update
    if (gps.location.isUpdated()) {
        out->lat    = gps.location.lat();
        out->lon    = gps.location.lng();
        out->alt    = (float)gps.altitude.meters();
        out->spd    = (float)gps.speed.kmph();
        out->course = (float)gps.course.deg();
        out->sats   = (uint8_t)gps.satellites.value();
        out->valid  = gps.location.isValid();
    } else {
        out->valid  = false;
    }
}
