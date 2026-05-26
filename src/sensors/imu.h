/**
 * imu.h
 * -----
 * MPU-6050 6-axis IMU driver interface.
 *
 * BEGINNER NOTE:
 *   The MPU-6050 combines a 3-axis accelerometer and a 3-axis gyroscope
 *   on a single chip, communicating via I²C (address 0x68).
 *
 *   Accelerometer — measures force (g).  At rest on a flat desk:
 *     ax ≈ 0 g,  ay ≈ 0 g,  az ≈ +1 g  (gravity pointing down).
 *
 *   Gyroscope — measures rotational speed (rad/s).  At rest:
 *     gx ≈ gy ≈ gz ≈ 0  (no rotation).
 */

#pragma once

#include "../task_defs.h"

/**
 * Initialise the MPU-6050.
 * Must be called after Wire.begin().
 * @return true on success, false if the sensor is not found on the bus.
 */
bool imu_init();

/**
 * Read one sample from the MPU-6050 and populate *out.
 * Caller must hold xI2CMutex before calling this function.
 *
 * @param out  Pointer to destination struct.  Must not be nullptr.
 */
void imu_read(IMUData_t *out);

/**
 * Compute roll & pitch (degrees) from accelerometer data only.
 * Useful for a quick sanity check without running the full filter.
 *
 * @param d      Filled IMUData_t.
 * @param roll   Output roll  (degrees, positive = right wing down).
 * @param pitch  Output pitch (degrees, positive = nose up).
 */
void imu_accel_angles(const IMUData_t &d, float &roll, float &pitch);
