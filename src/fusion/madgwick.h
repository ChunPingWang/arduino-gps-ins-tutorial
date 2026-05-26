/**
 * madgwick.h
 * ----------
 * Madgwick AHRS (Attitude and Heading Reference System) filter.
 *
 * BEGINNER NOTE — What is an AHRS filter?
 *   Gyroscopes drift over time (integration error accumulates).
 *   Accelerometers are noisy and affected by vibration.
 *   Magnetometers can be disturbed by nearby metal.
 *
 *   The Madgwick filter fuses all three sensors using a mathematical
 *   technique called "gradient descent" on a unit quaternion.
 *   Result: stable Roll/Pitch/Yaw with very low CPU cost.
 *
 * WHAT IS A QUATERNION?
 *   A quaternion (q0, q1, q2, q3) represents a 3-D orientation without
 *   the "gimbal lock" problem that plagues Euler angles.  You don't need
 *   to understand the math — just call madgwick_update() every 20 ms and
 *   read angles with madgwick_get_euler().
 *
 * Reference:
 *   Madgwick, S.O.H. (2010). "An efficient orientation filter for inertial
 *   and inertial/magnetic sensor arrays." Report x-io and University of Bristol.
 */

#pragma once

#include <Arduino.h>

/**
 * Reset the filter quaternion to the identity rotation (no rotation).
 * Call once from setup().
 */
void madgwick_init();

/**
 * Run one filter step.
 * Call at a fixed rate (every dt seconds).
 *
 * @param gx,gy,gz  Gyroscope  (rad/s)
 * @param ax,ay,az  Accelerometer (any unit — will be normalised internally)
 * @param mx,my,mz  Magnetometer (any unit — will be normalised internally)
 * @param dt        Time since last call (seconds), clamped to [0, 0.2] s
 */
void madgwick_update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz,
                     float dt);

/**
 * Extract Euler angles from the current quaternion state.
 *
 * @param roll   Output: roll  (degrees, positive = right wing down)
 * @param pitch  Output: pitch (degrees, positive = nose up)
 * @param yaw    Output: yaw   (degrees, [0, 360), 0 = North)
 */
void madgwick_get_euler(float *roll, float *pitch, float *yaw);

/**
 * Runtime-tune the filter's gain parameter.
 *
 * Higher beta → faster convergence but noisier output.
 * Lower  beta → smoother but slower to correct gyro drift.
 * Recommended starting range: 0.033 (steady flight) – 0.1 (fast dynamics).
 *
 * @param b  New beta value (> 0)
 */
void madgwick_set_beta(float b);

/**
 * Read the current quaternion (for advanced users / logging).
 */
void madgwick_get_quaternion(float *q0, float *q1, float *q2, float *q3);
