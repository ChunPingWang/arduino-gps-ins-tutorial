/**
 * imu.cpp — MPU-6050 driver implementation.
 *
 * Library used: electroniccats/MPU6050
 * Datasheet sensitivity values (T-012):
 *   Accel ±2 g  → 16 384 LSB/g
 *   Gyro  ±250°/s → 131 LSB/(°/s)
 */

#include "imu.h"
#include <MPU6050.h>

// ── Module-level sensor object ──────────────────────────────────────────────
static MPU6050 mpu;

// ===========================================================================
bool imu_init()
// ===========================================================================
{
    mpu.initialize();

    // Set measurement ranges (T-011)
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // ±2 g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   // ±250 °/s

    bool ok = mpu.testConnection();
#ifdef DEBUG_SERIAL
    Serial.print(F("[IMU] testConnection → "));
    Serial.println(ok ? F("OK") : F("FAIL — check wiring!"));
#endif
    return ok;
}

// ===========================================================================
void imu_read(IMUData_t *out)
// ===========================================================================
{
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;

    mpu.getMotion6(&raw_ax, &raw_ay, &raw_az,
                   &raw_gx, &raw_gy, &raw_gz);

    // Convert raw counts → physical units (T-012)
    constexpr float ACCEL_SCALE = 1.0f / 16384.0f;        // g per LSB
    constexpr float GYRO_SCALE  = (1.0f / 131.0f)         // °/s per LSB
                                 * (float)DEG_TO_RAD;      // → rad/s

    out->ax = (float)raw_ax * ACCEL_SCALE;
    out->ay = (float)raw_ay * ACCEL_SCALE;
    out->az = (float)raw_az * ACCEL_SCALE;

    out->gx = (float)raw_gx * GYRO_SCALE;
    out->gy = (float)raw_gy * GYRO_SCALE;
    out->gz = (float)raw_gz * GYRO_SCALE;

    out->ts = millis();
}

// ===========================================================================
void imu_accel_angles(const IMUData_t &d, float &roll, float &pitch)
// ===========================================================================
{
    // Simple tilt sensing from accelerometer only (no filter).
    // Good enough to verify the sensor is mounted correctly.
    roll  = atan2f(d.ay, d.az) * RAD_TO_DEG;
    pitch = atan2f(-d.ax, sqrtf(d.ay * d.ay + d.az * d.az)) * RAD_TO_DEG;
}
