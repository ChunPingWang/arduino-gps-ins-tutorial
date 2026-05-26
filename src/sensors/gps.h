/**
 * gps.h
 * -----
 * NEO-6M GPS driver interface using TinyGPSPlus.
 *
 * BEGINNER NOTE:
 *   The NEO-6M communicates via UART at 9600 baud.
 *   On the Arduino Mega, Serial1 (pins TX1/RX1 = 18/19) is used so
 *   that the USB Serial port (Serial) remains free for debug output.
 *
 *   GPS fix can take 30 s – 3 min on first power-on (cold start).
 *   Once a fix is acquired, subsequent starts are faster (warm/hot start).
 *   Always test outdoors with a clear sky view.
 */

#pragma once

#include "../task_defs.h"
#include <TinyGPSPlus.h>

/**
 * Initialise Serial1 at 9600 baud for the NEO-6M.
 * Call once from setup().
 */
void gps_init();

/**
 * Feed all pending bytes from Serial1 into TinyGPSPlus.
 * Call this frequently (every 20 ms is fine).
 *
 * @param out  Filled with the latest valid fix when gps.location.isUpdated()
 *             is true.  out->valid is set accordingly.
 */
void gps_poll(GPSData_t *out);
