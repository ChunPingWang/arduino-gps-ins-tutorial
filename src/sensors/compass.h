/**
 * compass.h
 * ---------
 * QMC5883L 3-axis magnetometer driver interface.
 *
 * BEGINNER NOTE:
 *   The QMC5883L reports the strength of Earth's magnetic field along
 *   three axes (X, Y, Z).  By computing atan2(Y, X) we get a magnetic
 *   heading that points toward magnetic north — NOT geographic north.
 *   The difference is called "magnetic declination" and varies by location.
 *
 *   Hard-iron calibration removes constant offsets caused by nearby
 *   magnets or ferrous materials on your PCB.  Without calibration,
 *   the heading can be off by tens of degrees.
 */

#pragma once

#include "../task_defs.h"

/**
 * Initialise the QMC5883L.
 * Must be called after Wire.begin().
 * @return true on success.
 */
bool compass_init();

/**
 * Read one sample, apply hard-iron offsets, and compute heading.
 * Caller must hold xI2CMutex.
 *
 * @param out  Pointer to destination struct.
 */
void compass_read(MagData_t *out);

/**
 * Interactive calibration routine — rotate the device 360° during the
 * 10-second window.  Prints computed offsets to Serial when done.
 * Call this once, note the values, then paste them into compass.cpp.
 */
void compass_calibrate();
