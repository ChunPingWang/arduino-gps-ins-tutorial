/**
 * task_defs.h
 * -----------
 * Central header shared by EVERY translation unit.
 * Defines all data structures and declares all FreeRTOS
 * inter-task primitives as extern (defined in main.cpp).
 *
 * BEGINNER NOTE:
 *   "extern" means "this variable exists somewhere else —
 *    trust me, the linker will find it".  The actual memory
 *    is allocated once in main.cpp.
 */

#pragma once

#include <Arduino.h>

// ── Pick the right FreeRTOS header for the target MCU ──────────────────────
#if defined(ARDUINO_ARCH_AVR)
  #include <Arduino_FreeRTOS.h>  // feilipu/FreeRTOS library
  #include <queue.h>             // QueueHandle_t, xQueueCreate, xQueueOverwrite …
  #include <semphr.h>            // SemaphoreHandle_t, xSemaphoreCreate* …
#elif defined(ARDUINO_ARCH_ESP32)
  #include <freertos/FreeRTOS.h>
  #include <freertos/task.h>
  #include <freertos/queue.h>
  #include <freertos/semphr.h>
#endif

// ===========================================================================
//  Shared Data Structures  (T-004)
// ===========================================================================

/** Raw IMU data from the MPU-6050. */
struct IMUData_t {
    float    ax, ay, az;   ///< Acceleration in g
    float    gx, gy, gz;   ///< Angular rate in rad/s
    uint32_t ts;           ///< Timestamp — millis()
};

/** Magnetometer data from the QMC5883L. */
struct MagData_t {
    float mx, my, mz;      ///< Magnetic field (raw counts after hard-iron correction)
    float heading;         ///< Tilt-compensated heading [0, 360) degrees
};

/** GPS fix data from the NEO-6M via TinyGPSPlus. */
struct GPSData_t {
    double  lat, lon;      ///< Decimal degrees
    float   alt;           ///< Altitude above sea level (m)
    float   spd;           ///< Ground speed (km/h)
    float   course;        ///< Course over ground (degrees)
    uint8_t sats;          ///< Number of satellites in use
    bool    valid;         ///< True when a fresh fix is available
};

/** Fused navigation output written by Task_Fusion. */
struct NavOut_t {
    float  roll, pitch, yaw;   ///< Euler angles (degrees)
    double lat, lon;           ///< GPS position (decimal degrees)
    float  alt;                ///< Altitude (m)
    float  spd;                ///< Speed (km/h)
    float  heading;            ///< Compass heading (degrees)
};

// ===========================================================================
//  HC-12 Telemetry Packet  (T-063)
// ===========================================================================

/**
 * NavPacket_t — 20-byte packed telemetry frame sent over HC-12.
 *
 *  Offset  Field        Bytes  Description
 *  ------  -----------  -----  -------------------------------------------
 *   0      stx          1      Frame start sentinel = 0xAA
 *   1      len          1      Payload length (bytes after this field, before CRC)
 *   2      type         1      Packet type (0x01 = nav telemetry)
 *   3-4    roll100      2      Roll  × 100  (int16, signed)
 *   5-6    pitch100     2      Pitch × 100  (int16, signed)
 *   7-8    yaw10        2      Yaw   × 10   (uint16)
 *   9-12   lat_e6       4      Latitude  × 1e6 (int32)
 *  13-16   lon_e6       4      Longitude × 1e6 (int32)
 *  17-18   alt_dm       2      Altitude in decimetres (int16)
 *  19      sats         1      Satellite count
 *  20      crc8         1      CRC-8 over bytes [1..19]
 */
#pragma pack(push, 1)
struct NavPacket_t {
    uint8_t  stx;        ///< 0xAA
    uint8_t  len;        ///< payload length
    uint8_t  type;       ///< 0x01 = nav telemetry
    int16_t  roll100;
    int16_t  pitch100;
    uint16_t yaw10;
    int32_t  lat_e6;
    int32_t  lon_e6;
    int16_t  alt_dm;
    uint8_t  sats;
    uint8_t  crc8;
};
#pragma pack(pop)

// ===========================================================================
//  FreeRTOS Inter-Task Primitives  (T-005)
// ===========================================================================

/** One-element queues — always hold the LATEST sample (xQueueOverwrite). */
extern QueueHandle_t xQueueIMU;   ///< IMUData_t — produced by Task_IMU
extern QueueHandle_t xQueueMAG;   ///< MagData_t — produced by Task_Compass
extern QueueHandle_t xQueueGPS;   ///< GPSData_t — produced by Task_GPS

/** Mutual exclusion for shared I²C bus. */
extern SemaphoreHandle_t xI2CMutex;

/** Mutual exclusion for navOut (multi-reader/single-writer). */
extern SemaphoreHandle_t xNavMutex;

/** Live fused navigation state — protected by xNavMutex. */
extern NavOut_t navOut;

// ===========================================================================
//  Compile-time tunables
// ===========================================================================

/** Magnetic declination for Taiwan (negative = West). Adjust for your region. */
static constexpr float MAG_DECLINATION_DEG = -3.5f;
