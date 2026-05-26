/**
 * compass.cpp — QMC5883L magnetometer driver implementation.
 *
 * Library used: mprograms/QMC5883LCompass
 *
 * HARD-IRON CALIBRATION:
 *   Run compass_calibrate() once and note the printed offsets.
 *   Then update OFFSET_X/Y/Z below with those values and recompile.
 *   Without calibration, heading accuracy will be poor.
 */

#include "compass.h"
#include <QMC5883LCompass.h>

// ── Hard-iron offset constants — update after running compass_calibrate() ──
static constexpr float OFFSET_X = 0.0f;  // ← paste calibrated value here
static constexpr float OFFSET_Y = 0.0f;  // ← paste calibrated value here
static constexpr float OFFSET_Z = 0.0f;  // ← paste calibrated value here

// ── Module-level sensor object ──────────────────────────────────────────────
static QMC5883LCompass compass;

// ===========================================================================
bool compass_init()
// ===========================================================================
{
    compass.init();
    // Mode register: Continuous | 200 Hz ODR | 8 G range | OSR 512  (T-021)
    compass.setMode(0x01, 0x0C, 0x10, 0x00);

#ifdef DEBUG_SERIAL
    Serial.println(F("[MAG] QMC5883L initialised"));
#endif
    return true;  // Library does not expose a connection-test method
}

// ===========================================================================
void compass_read(MagData_t *out)
// ===========================================================================
{
    compass.read();

    // Apply hard-iron correction
    out->mx = (float)compass.getX() - OFFSET_X;
    out->my = (float)compass.getY() - OFFSET_Y;
    out->mz = (float)compass.getZ() - OFFSET_Z;

    // Compute magnetic heading (2-D, tilt not compensated here; Madgwick
    // provides tilt-compensated yaw for navigation use).
    float heading = atan2f(out->my, out->mx) * RAD_TO_DEG;

    // Apply magnetic declination correction (T-022)
    heading += MAG_DECLINATION_DEG;

    // Normalise to [0, 360)
    if (heading < 0.0f)   heading += 360.0f;
    if (heading >= 360.f) heading -= 360.0f;

    out->heading = heading;
}

// ===========================================================================
void compass_calibrate()
// ===========================================================================
{
    Serial.println(F("[MAG] Calibration — rotate device 360° in 10 seconds..."));

    float min_x =  1e9f, min_y =  1e9f;
    float max_x = -1e9f, max_y = -1e9f;

    uint32_t start = millis();
    while (millis() - start < 10000UL) {
        compass.read();
        float x = (float)compass.getX();
        float y = (float)compass.getY();
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
        delay(50);
    }

    float off_x = (max_x + min_x) / 2.0f;
    float off_y = (max_y + min_y) / 2.0f;

    Serial.println(F("[MAG] Calibration complete. Paste into compass.cpp:"));
    Serial.print(F("  OFFSET_X = ")); Serial.println(off_x, 2);
    Serial.print(F("  OFFSET_Y = ")); Serial.println(off_y, 2);
}
