/**
 * madgwick.cpp — Full 9-DOF Madgwick AHRS implementation.
 *
 * This is a self-contained C implementation of the algorithm described in:
 *   Madgwick 2010, equations (25)–(46).
 *
 * Key implementation notes:
 *  • Uses the "fast inverse square root" bit-hack for speed on AVR.
 *  • Zero-magnitude guard prevents NaN when sensors return all-zero.
 *  • Beta (gradient descent step size) is tunable at runtime.
 */

#include "madgwick.h"
#include <math.h>

// ── Filter state ─────────────────────────────────────────────────────────────
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float beta = 0.1f;   // Default gain (T-051)

// ── Fast inverse square root (Quake III trick) ───────────────────────────────
static float invSqrt(float x) {
    float   halfx = 0.5f * x;
    float   y     = x;
    int32_t i;
    memcpy(&i, &y, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - halfx * y * y);   // one Newton-Raphson iteration
    return y;
}

// ===========================================================================
void madgwick_init()
// ===========================================================================
{
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
}

// ===========================================================================
void madgwick_set_beta(float b)
// ===========================================================================
{
    if (b > 0.0f) beta = b;
}

// ===========================================================================
void madgwick_get_quaternion(float *o0, float *o1, float *o2, float *o3)
// ===========================================================================
{
    *o0 = q0; *o1 = q1; *o2 = q2; *o3 = q3;
}

// ===========================================================================
void madgwick_get_euler(float *roll, float *pitch, float *yaw)
// ===========================================================================
{
    // Standard ZYX Euler extraction from unit quaternion (T-053)
    *roll  = atan2f(2.0f*(q0*q1 + q2*q3),
                    1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;

    float sinp = 2.0f*(q0*q2 - q3*q1);
    // Clamp to avoid domain error in asinf
    if      (sinp >  1.0f) sinp =  1.0f;
    else if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp) * RAD_TO_DEG;

    float y_raw = atan2f(2.0f*(q0*q3 + q1*q2),
                         1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG;
    // Normalise to [0, 360)
    if (y_raw < 0.0f)   y_raw += 360.0f;
    if (y_raw >= 360.f) y_raw -= 360.0f;
    *yaw = y_raw;
}

// ===========================================================================
void madgwick_update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz,
                     float dt)
// ===========================================================================
{
    // Clamp dt to a safe range (T-045)
    if (dt <= 0.0f) dt = 0.001f;
    if (dt >  0.2f) dt = 0.2f;

    float recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2bx, _2bz;
    float _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // ── Step 1: Rate of change of quaternion from gyroscope ──────────────
    qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    // ── Step 2: Gradient-descent correction (only if sensors are non-zero) ─
    float accelNorm = ax*ax + ay*ay + az*az;
    float magNorm   = mx*mx + my*my + mz*mz;

    if (accelNorm > 0.0f && magNorm > 0.0f) {

        // Normalise accelerometer
        recipNorm = invSqrt(accelNorm);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        // Normalise magnetometer
        recipNorm = invSqrt(magNorm);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        // Pre-compute products
        _2q0  = 2.0f*q0;
        _2q1  = 2.0f*q1;
        _2q2  = 2.0f*q2;
        _2q3  = 2.0f*q3;
        q0q0  = q0*q0; q0q1 = q0*q1; q0q2 = q0*q2; q0q3 = q0*q3;
        q1q1  = q1*q1; q1q2 = q1*q2; q1q3 = q1*q3;
        q2q2  = q2*q2; q2q3 = q2*q3; q3q3 = q3*q3;

        // Reference direction of Earth's magnetic field
        hx = mx*(q0q0 + q1q1 - q2q2 - q3q3)
           + 2.0f*my*(q1q2 - q0q3)
           + 2.0f*mz*(q1q3 + q0q2);
        hy = 2.0f*mx*(q1q2 + q0q3)
           + my*(q0q0 - q1q1 + q2q2 - q3q3)
           + 2.0f*mz*(q2q3 - q0q1);
        _2bx = sqrtf(hx*hx + hy*hy);
        _2bz = -2.0f*mx*(q1q3 - q0q2)
             + 2.0f*my*(q2q3 + q0q1)
             + mz*(q0q0 - q1q1 - q2q2 + q3q3);
        _4bx = 2.0f*_2bx;
        _4bz = 2.0f*_2bz;

        // Gradient descent (objective function Jacobian × objective)
        s0 = -_2q2*(2.0f*(q1q3 - q0q2) - ax)
            + _2q1*(2.0f*(q0q1 + q2q3) - ay)
            - _2bz*q2*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
            + (-_2bx*q3 + _2bz*q1)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
            + _2bx*q2*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        s1 = _2q3*(2.0f*(q1q3 - q0q2) - ax)
           + _2q0*(2.0f*(q0q1 + q2q3) - ay)
           - 4.0f*q1*(1.0f - 2.0f*(q1q1 + q2q2) - az)
           + _2bz*q3*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
           + (_2bx*q2 + _2bz*q0)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
           + (_2bx*q3 - _4bz*q1)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        s2 = -_2q0*(2.0f*(q1q3 - q0q2) - ax)
           + _2q3*(2.0f*(q0q1 + q2q3) - ay)
           - 4.0f*q2*(1.0f - 2.0f*(q1q1 + q2q2) - az)
           + (-_4bx*q2 - _2bz*q0)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
           + (_2bx*q1 + _2bz*q3)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
           + (_2bx*q0 - _4bz*q2)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        s3 = _2q1*(2.0f*(q1q3 - q0q2) - ax)
           + _2q2*(2.0f*(q0q1 + q2q3) - ay)
           + (-_4bx*q3 + _2bz*q1)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
           + (-_2bx*q0 + _2bz*q2)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
           + _2bx*q1*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        // Normalise gradient
        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        // Apply feedback
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // ── Step 3: Integrate to yield quaternion ─────────────────────────────
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Re-normalise quaternion
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm;
    q2 *= recipNorm; q3 *= recipNorm;
}
