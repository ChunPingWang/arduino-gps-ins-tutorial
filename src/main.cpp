/**
 * main.cpp — Arduino GPS-INS Tutorial: FreeRTOS Navigation System
 * ================================================================
 * Entry point for the Arduino Mega 2560 target.
 *
 * SYSTEM OVERVIEW (beginner-friendly):
 *
 *   ┌──────────────┐   I²C   ┌─────────────────┐
 *   │  MPU-6050    │─────────│                 │
 *   │  (IMU)       │         │  Arduino         │
 *   ├──────────────┤   I²C   │  Mega 2560       │──USB──▶ Serial Monitor
 *   │  QMC5883L    │─────────│                 │
 *   │  (Compass)   │         │  FreeRTOS Tasks: │
 *   ├──────────────┤  UART1  │  • Task_IMU      │
 *   │  NEO-6M      │─────────│  • Task_Compass  │
 *   │  (GPS)       │         │  • Task_GPS      │
 *   └──────────────┘         │  • Task_Fusion   │
 *                            │  • Task_Display  │
 *                     UART2  │  • Task_TxTele   │──HC-12──▶ Ground Station
 *                   ─────────│  • Task_RxCmd    │
 *                            └─────────────────┘
 *
 * TASK PRIORITY TABLE (higher number = higher priority):
 *   Task_Fusion    5   50 Hz  — must process IMU data fast
 *   Task_IMU       4  100 Hz  — feeds the fusion filter
 *   Task_Compass   3   50 Hz
 *   Task_RxCmd     3   event  — responds to ground-station commands
 *   Task_GPS       2   20 Hz polling
 *   Task_TxTele    2    5 Hz  — sends telemetry to ground
 *   Task_Display   1    2 Hz  — lowest priority, Serial output
 *
 * MEMORY BUDGET (Arduino Mega has 8 KB SRAM):
 *   Each task's stack is sized conservatively.  T-047 verifies that
 *   freeRamBytes() > 1024 at runtime (heap_3 does not export xPortGetFreeHeapSize).
 */

#include <Arduino.h>
#include <Wire.h>

#include "task_defs.h"
#include "sensors/imu.h"
#include "sensors/compass.h"
#include "sensors/gps.h"
#include "fusion/madgwick.h"

#ifdef ENABLE_HC12
  #include "comms/packet.h"
#endif

// ===========================================================================
//  FreeRTOS handle DEFINITIONS (extern declarations are in task_defs.h)
// ===========================================================================
QueueHandle_t    xQueueIMU;
QueueHandle_t    xQueueMAG;
QueueHandle_t    xQueueGPS;
SemaphoreHandle_t xI2CMutex;
SemaphoreHandle_t xNavMutex;
NavOut_t         navOut;

// ===========================================================================
//  Free-RAM helper (AVR)  (T-047)
// ===========================================================================
//  heap_3.c (the feilipu default, wraps malloc/free) does not implement
//  xPortGetFreeHeapSize().  Instead we measure the gap between the heap
//  top and the current stack pointer — which is the true free SRAM.
// ===========================================================================
static uint16_t freeRamBytes() {
#if defined(ARDUINO_ARCH_AVR)
    extern int __heap_start, *__brkval;
    int v;
    return (uint16_t)((int)&v -
           (__brkval == nullptr ? (int)&__heap_start : (int)__brkval));
#else
    return (uint16_t)ESP.getFreeHeap();   // ESP32: use native API
#endif
}

// ===========================================================================
//  Stack Overflow Hook  (T-101)
// ===========================================================================
#if defined(ARDUINO_ARCH_AVR)
extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char *pcTaskName) {
    Serial.print(F("[FATAL] Stack overflow in task: "));
    Serial.println(pcTaskName);
    // Halt — a watchdog reset is preferred in production firmware
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}
#endif

// ===========================================================================
//  Task_IMU — Priority 4, 100 Hz  (T-042)
// ===========================================================================
static void Task_IMU(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    IMUData_t  data;

    for (;;) {
        // Acquire shared I²C bus
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            imu_read(&data);
            xSemaphoreGive(xI2CMutex);
        }

        // Overwrite queue — consumer always gets the freshest sample
        xQueueOverwrite(xQueueIMU, &data);

        // Wait until next 10 ms slot (100 Hz)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

// ===========================================================================
//  Task_Compass — Priority 3, 50 Hz  (T-043)
// ===========================================================================
static void Task_Compass(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    MagData_t  data;

    for (;;) {
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            compass_read(&data);
            xSemaphoreGive(xI2CMutex);
        }

        xQueueOverwrite(xQueueMAG, &data);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

// ===========================================================================
//  Task_GPS — Priority 2, polling at 20 Hz  (T-044)
// ===========================================================================
static void Task_GPS(void *pvParameters) {
    GPSData_t data = {};

    for (;;) {
        gps_poll(&data);

        if (data.valid) {
            xQueueOverwrite(xQueueGPS, &data);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ===========================================================================
//  Task_Fusion — Priority 5, 50 Hz  (T-045)
// ===========================================================================
static void Task_Fusion(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    IMUData_t  imuData = {};
    MagData_t  magData = {};
    GPSData_t  gpsData = {};

    uint32_t   prevMs = millis();

    for (;;) {
        // Peek (non-blocking) — do not remove the items so other tasks can read
        xQueuePeek(xQueueIMU, &imuData, 0);
        xQueuePeek(xQueueMAG, &magData, 0);
        xQueuePeek(xQueueGPS, &gpsData, 0);

        // Compute dt
        uint32_t nowMs = millis();
        float dt = (float)(nowMs - prevMs) * 0.001f;
        prevMs   = nowMs;
        if (dt > 0.2f) dt = 0.2f;
        if (dt <= 0.0f) dt = 0.001f;

        // Run Madgwick filter
        madgwick_update(imuData.gx, imuData.gy, imuData.gz,
                        imuData.ax, imuData.ay, imuData.az,
                        magData.mx, magData.my, magData.mz,
                        dt);

        // Extract Euler angles
        float roll, pitch, yaw;
        madgwick_get_euler(&roll, &pitch, &yaw);

        // Write fused output under mutex
        if (xSemaphoreTake(xNavMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            navOut.roll    = roll;
            navOut.pitch   = pitch;
            navOut.yaw     = yaw;
            navOut.lat     = gpsData.lat;
            navOut.lon     = gpsData.lon;
            navOut.alt     = gpsData.alt;
            navOut.spd     = gpsData.spd;
            navOut.heading = magData.heading;
            xSemaphoreGive(xNavMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

// ===========================================================================
//  Task_Display — Priority 1, 2 Hz  (T-046)
// ===========================================================================
static void Task_Display(void *pvParameters) {
    NavOut_t snap;

    for (;;) {
        if (xSemaphoreTake(xNavMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            snap = navOut;
            xSemaphoreGive(xNavMutex);
        }

        // CSV line: Roll,Pitch,Yaw,Lat,Lon,Alt,Spd,FreeHeap  (T-046)
        Serial.print(snap.roll,    2); Serial.print(',');
        Serial.print(snap.pitch,   2); Serial.print(',');
        Serial.print(snap.yaw,     2); Serial.print(',');
        Serial.print(snap.lat,     6); Serial.print(',');
        Serial.print(snap.lon,     6); Serial.print(',');
        Serial.print(snap.alt,     1); Serial.print(',');
        Serial.print(snap.spd,     1); Serial.print(',');
        Serial.println(freeRamBytes());   // T-047: gap between heap top & SP

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ===========================================================================
//  HC-12 Telemetry Tasks — compiled only when ENABLE_HC12 is defined  (T-065)
// ===========================================================================
#ifdef ENABLE_HC12

static void Task_TxTelemetry(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    NavOut_t   snap;

    for (;;) {
        if (xSemaphoreTake(xNavMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            snap = navOut;
            xSemaphoreGive(xNavMutex);
        }

        NavPacket_t pkt;
        packet_build(&pkt, &snap, 0);   // 0 satellites (update if GPS valid)
        packet_send(&pkt);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));   // 5 Hz
    }
}

static void Task_RxCommand(void *pvParameters) {
    for (;;) {
        if (Serial2.available()) {
            packet_rx_process();
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

#endif  // ENABLE_HC12

// ===========================================================================
//  setup() — runs once before the RTOS scheduler starts
// ===========================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}   // Wait for USB serial (max 3 s)

    Serial.println(F("=== Arduino GPS-INS Navigation System ==="));
    Serial.println(F("Initialising hardware..."));

    // ── I²C bus ─────────────────────────────────────────────────────────────
    Wire.begin();
    Wire.setClock(400000UL);   // Fast Mode — 400 kHz (T-041)

    // ── Sensor init ──────────────────────────────────────────────────────────
    if (!imu_init()) {
        Serial.println(F("[ERROR] MPU-6050 not found! Check SDA/SCL wiring."));
    }
    compass_init();
    gps_init();
    madgwick_init();

    // ── HC-12 UART ───────────────────────────────────────────────────────────
#ifdef ENABLE_HC12
    Serial2.begin(9600);
    hc12_configure();
    Serial.println(F("[HC12] Configured"));
#endif

    // ── FreeRTOS queues (length 1 — always hold latest sample) ──────────────
    xQueueIMU = xQueueCreate(1, sizeof(IMUData_t));
    xQueueMAG = xQueueCreate(1, sizeof(MagData_t));
    xQueueGPS = xQueueCreate(1, sizeof(GPSData_t));

    // ── Mutexes ──────────────────────────────────────────────────────────────
    xI2CMutex = xSemaphoreCreateMutex();
    xNavMutex = xSemaphoreCreateMutex();

    // ── Verify critical resources were created ───────────────────────────────
    if (!xQueueIMU || !xQueueMAG || !xQueueGPS ||
        !xI2CMutex || !xNavMutex) {
        Serial.println(F("[FATAL] FreeRTOS resource creation failed — out of RAM?"));
        for (;;) {}
    }

    // ── Create tasks ─────────────────────────────────────────────────────────
    // Signature: xTaskCreate(function, name, stack_words, params, priority, handle)
    // Stack in WORDS (2 bytes each on AVR), not bytes!
    xTaskCreate(Task_IMU,     "IMU",     80,  NULL, 4, NULL);
    xTaskCreate(Task_Compass, "MAG",     80,  NULL, 3, NULL);
    xTaskCreate(Task_GPS,     "GPS",     110, NULL, 2, NULL);
    xTaskCreate(Task_Fusion,  "Fusion",  110, NULL, 5, NULL);
    xTaskCreate(Task_Display, "Disp",    100, NULL, 1, NULL);

#ifdef ENABLE_HC12
    xTaskCreate(Task_TxTelemetry, "TxTele", 128, NULL, 2, NULL);
    xTaskCreate(Task_RxCommand,   "RxCmd",  100, NULL, 3, NULL);
#endif

    Serial.println(F("Starting FreeRTOS scheduler..."));
    // vTaskStartScheduler() is called automatically by the FreeRTOS Arduino library
    // after setup() returns (for AVR).  On ESP32 the scheduler is already running.
}

// ===========================================================================
//  loop() — intentionally empty; all work is done in FreeRTOS tasks
// ===========================================================================
void loop() {
    // The AVR FreeRTOS port calls vTaskStartScheduler() after setup() returns,
    // so loop() is never reached in normal operation.
    //
    // If you ever land here it means the scheduler failed to start —
    // usually because there is not enough RAM for even one task.
}
