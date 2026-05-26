# Arduino GPS-INS Navigation System — Beginner's Guide

A step-by-step tutorial that builds a real-time **GPS + Inertial Navigation System (INS)** on an Arduino Mega 2560 using FreeRTOS. By the end you will have a running system that fuses accelerometer, gyroscope, magnetometer, and GPS data into stable Roll/Pitch/Yaw angles and transmits live telemetry wirelessly.

---

## Table of Contents

1. [What You Will Build](#1-what-you-will-build)
2. [Hardware Shopping List](#2-hardware-shopping-list)
3. [Wiring Guide](#3-wiring-guide)
4. [Software Setup](#4-software-setup)
5. [Understanding the Architecture](#5-understanding-the-architecture)
6. [Phase 0 — Project Scaffold](#phase-0--project-scaffold)
7. [Phase 1 — Sensor Drivers (Smoke Tests)](#phase-1--sensor-drivers-smoke-tests)
8. [Phase 2 — FreeRTOS Multi-Task System](#phase-2--freertos-multi-task-system)
9. [Phase 3 — Madgwick Sensor Fusion](#phase-3--madgwick-sensor-fusion)
10. [Phase A — HC-12 Wireless Telemetry](#phase-a--hc-12-wireless-telemetry)
11. [Ground Station (Python)](#ground-station-python)
12. [Troubleshooting](#troubleshooting)
13. [Acceptance Criteria](#acceptance-criteria)
14. [Going Further](#going-further)

---

## 1. What You Will Build

```
┌──────────────┐   I²C   ┌──────────────────────────────┐
│  MPU-6050    │─────────│                              │
│  Accel+Gyro  │         │   Arduino Mega 2560          │
├──────────────┤   I²C   │                              │──USB──▶ Serial Monitor
│  QMC5883L    │─────────│   FreeRTOS Tasks             │
│  Magnetometer│         │   ┌────────────────────────┐ │
├──────────────┤  UART1  │   │ Task_IMU   (100 Hz)    │ │
│  NEO-6M GPS  │─────────│   │ Task_Compass (50 Hz)   │ │
└──────────────┘         │   │ Task_GPS    (20 Hz)    │ │
                         │   │ Task_Fusion  (50 Hz)   │ │
                         │   │ Task_Display  (2 Hz)   │ │
         HC-12 ──UART2───│   │ Task_TxTele   (5 Hz)   │ │
         Radio           │   └────────────────────────┘ │
                         └──────────────────────────────┘
                                     ↕ Radio (433 MHz)
                         ┌──────────────────────────────┐
                         │  PC Ground Station           │
                         │  ground_station.py           │
                         └──────────────────────────────┘
```

**Key concepts you will learn:**
- FreeRTOS task scheduling and inter-task communication (queues, mutexes)
- I²C bus sharing between multiple sensors
- NMEA GPS parsing with TinyGPSPlus
- Madgwick AHRS sensor fusion (quaternion math — no prior knowledge needed)
- CRC-protected binary telemetry over 433 MHz radio
- Python serial parsing on the ground station

---

## 2. Hardware Shopping List

| Item | Qty | Notes |
|------|-----|-------|
| Arduino Mega 2560 | 1 | Any clone works; needs 4× hardware UARTs |
| MPU-6050 breakout | 1 | GY-521 module; includes 3.3 V LDO |
| QMC5883L breakout | 1 | GY-271; do NOT buy the older HMC5883L — different register map |
| NEO-6M GPS module | 1 | GY-NEO6MV2; includes ceramic patch antenna |
| HC-12 433 MHz module | 2 | One for Arduino, one connected to your PC via USB-UART adapter |
| USB-UART adapter | 1 | CP2102 or CH340; needed for the ground-station HC-12 |
| Jumper wires | 20+ | Male-to-female for breadboard to module |
| Breadboard | 1 | Full-size recommended |
| Micro-USB cable | 1 | Programming and debug Serial |

**Total estimated cost:** ~$25–40 USD (AliExpress prices)

---

## 3. Wiring Guide

### 3.1 I²C Sensors (MPU-6050 and QMC5883L share the bus)

Both sensors connect to the **same** SDA/SCL pins. They have different I²C addresses (MPU-6050 = 0x68, QMC5883L = 0x0D) so there is no conflict.

| Sensor Pin | Arduino Mega Pin | Notes |
|-----------|-----------------|-------|
| VCC | 3.3 V | Both modules have onboard regulators |
| GND | GND | |
| SDA | Pin 20 (SDA) | Shared between both sensors |
| SCL | Pin 21 (SCL) | Shared between both sensors |
| AD0 (MPU-6050 only) | GND | Sets I²C address to 0x68 |

> ⚠️ **Important:** The Mega's SDA/SCL are on pins 20/21, **not** A4/A5 (those are for Uno).

### 3.2 NEO-6M GPS (UART)

| GPS Pin | Arduino Mega Pin | Notes |
|---------|-----------------|-------|
| VCC | 5 V | Module has onboard 3.3 V regulator |
| GND | GND | |
| TX | Pin 19 (RX1) | GPS transmits → Mega receives |
| RX | Pin 18 (TX1) | Optional; only for sending NMEA config |

### 3.3 HC-12 Radio Module (UART)

| HC-12 Pin | Arduino Mega Pin | Notes |
|-----------|-----------------|-------|
| VCC | 5 V | |
| GND | GND | |
| TX | Pin 17 (RX2) | HC-12 transmits → Mega receives |
| RX | Pin 16 (TX2) | Mega transmits → HC-12 |
| SET | Pin 4 (digital) | Low = AT command mode |

### 3.4 Complete Pin Summary

```
Arduino Mega 2560
─────────────────
Pin 20 (SDA) ─── MPU-6050 SDA ─── QMC5883L SDA
Pin 21 (SCL) ─── MPU-6050 SCL ─── QMC5883L SCL
3.3V         ─── MPU-6050 VCC ─── QMC5883L VCC
GND          ─── MPU-6050 GND ─── QMC5883L GND
GND          ─── MPU-6050 AD0 (I²C address = 0x68)

Pin 19 (RX1) ─── NEO-6M TX
Pin 18 (TX1) ─── NEO-6M RX (optional)
5V           ─── NEO-6M VCC
GND          ─── NEO-6M GND

Pin 17 (RX2) ─── HC-12 TX
Pin 16 (TX2) ─── HC-12 RX
Pin 4        ─── HC-12 SET
5V           ─── HC-12 VCC
GND          ─── HC-12 GND

USB          ─── PC (programming + Serial Monitor at 115200 baud)
```

---

## 4. Software Setup

### 4.1 Install PlatformIO

PlatformIO is a cross-platform build system that handles library management automatically — much simpler than the Arduino IDE for multi-file projects.

1. Install [VS Code](https://code.visualstudio.com/)
2. Open VS Code → Extensions → search **PlatformIO IDE** → Install
3. Restart VS Code

### 4.2 Open the Project

```bash
# Clone or copy the project folder, then:
cd arduino-gps-ins-tutorial
code .          # Open in VS Code
```

PlatformIO will detect `platformio.ini` and offer to install all libraries automatically. Click **Yes**.

### 4.3 Library Dependencies

All libraries are declared in `platformio.ini` and installed automatically:

| Library | Purpose |
|---------|---------|
| `feilipu/FreeRTOS` | Real-Time OS for AVR |
| `electroniccats/MPU6050` | IMU driver |
| `mprograms/QMC5883LCompass` | Magnetometer driver |
| `mikalhart/TinyGPSPlus` | NMEA GPS parser |
| `jgromes/RadioLib` | LoRa (Phase B — not needed for Phase A) |

### 4.4 Build and Flash

```bash
# Build
pio run -e megaatmega2560

# Flash (board connected via USB)
pio run -e megaatmega2560 --target upload

# Open Serial Monitor
pio device monitor --baud 115200
```

---

## 5. Understanding the Architecture

### 5.1 Why FreeRTOS?

Without an RTOS, you write everything in `loop()` and manually time-slice operations using `millis()`. This becomes unmanageable when you have:
- IMU that must be read every **10 ms** (or the filter diverges)
- GPS that sends data every **1 second** (slow, unpredictable)
- Radio that must transmit every **200 ms**
- Serial output that blocks for milliseconds

FreeRTOS lets each concern run as an independent **task** at its own rate. The scheduler ensures higher-priority tasks (like reading the IMU) always preempt lower-priority ones (like printing to Serial).

### 5.2 FreeRTOS Concepts for Beginners

| Concept | Analogy | Our Use |
|---------|---------|---------|
| **Task** | Thread / worker process | One per sensor + fusion + display |
| **Priority** | Job urgency | IMU=4 (urgent) → Display=1 (can wait) |
| **Queue** | Conveyor belt between workers | IMU → Fusion, GPS → Fusion |
| **Mutex** | "Talking stick" | Only one task uses I²C at a time |
| **vTaskDelayUntil** | Alarm clock | Fires task at exact rate |
| **xQueueOverwrite** | Replace with latest | Consumer always gets freshest data |

### 5.3 Task Interaction Diagram

```
Task_IMU (100 Hz) ──xQueueOverwrite──▶ xQueueIMU ──xQueuePeek──▶ Task_Fusion (50 Hz)
Task_Compass(50Hz) ─xQueueOverwrite──▶ xQueueMAG ──xQueuePeek──▶ Task_Fusion
Task_GPS (20 Hz)  ──xQueueOverwrite──▶ xQueueGPS ──xQueuePeek──▶ Task_Fusion
                                                                         │
                                                                  xNavMutex (write)
                                                                         │
                                                                    navOut struct
                                                                   /          \
                                             xNavMutex (read)    /            \
                                         Task_Display (2 Hz) ◀──              ──▶ Task_TxTele (5 Hz)
```

### 5.4 Memory Layout (8 KB SRAM on Mega)

The Mega has only 8192 bytes of SRAM. Stack sizes are specified in **words** (2 bytes each) for the AVR FreeRTOS port:

| Task | Stack (words) | Stack (bytes) | Priority |
|------|--------------|---------------|----------|
| Task_IMU | 80 | 160 | 4 |
| Task_Compass | 80 | 160 | 3 |
| Task_GPS | 110 | 220 | 2 |
| Task_Fusion | 110 | 220 | 5 |
| Task_Display | 100 | 200 | 1 |
| Task_TxTele | 128 | 256 | 2 |
| Task_RxCmd | 100 | 200 | 3 |
| **FreeRTOS kernel** | — | ~500 | — |
| **Queues + mutexes** | — | ~50 | — |
| **Global variables** | — | ~100 | — |
| **Total** | | **~1866** | |
| **Free heap target** | | **> 1024** | |

> Monitor free heap via `Task_Display` which prints `xPortGetFreeHeapSize()`.

---

## Phase 0 — Project Scaffold

**What's here:**
- `platformio.ini` — build configuration
- `src/task_defs.h` — all shared data structures and FreeRTOS handles

**Key data structures in `task_defs.h`:**

```cpp
struct IMUData_t {
    float ax, ay, az;   // Acceleration (g)
    float gx, gy, gz;   // Angular rate (rad/s)
    uint32_t ts;        // Timestamp — millis()
};

struct GPSData_t {
    double lat, lon;    // Decimal degrees
    float  alt, spd;    // Metres, km/h
    uint8_t sats;
    bool valid;         // False until a fix is acquired
};

struct NavOut_t {
    float  roll, pitch, yaw;  // Degrees (Madgwick output)
    double lat, lon;
    float  alt, spd, heading;
};
```

**Why `extern` in the header?**
```cpp
// task_defs.h — declaration (tells the compiler "it exists somewhere")
extern QueueHandle_t xQueueIMU;

// main.cpp — definition (actually allocates memory)
QueueHandle_t xQueueIMU;
```

The `extern` pattern allows every `.cpp` file to `#include "task_defs.h"` and use these handles without the linker complaining about multiple definitions.

---

## Phase 1 — Sensor Drivers (Smoke Tests)

Test each sensor individually **before** integrating into the full system. This is the single most important debugging practice in embedded development.

### Smoke Test: MPU-6050 IMU

1. Open `test/test_imu.cpp`
2. Uncomment `#define RUN_TEST_IMU` at the top
3. Comment out all other `#define RUN_TEST_*` lines
4. Flash and open Serial Monitor at 115200 baud

**Expected output (board flat on desk):**
```
ax(g),  ay(g),  az(g),  gx(r/s), gy(r/s), gz(r/s), Roll(°), Pitch(°)
-0.002,  0.003,  0.998,  0.001,   -0.002,   0.000,   0.17,   -0.11
```
- `az ≈ +1.0 g` → gravity pointing down ✓
- `Roll`, `Pitch` within ±5° of 0° ✓

**If sensor not found:**
- Check SDA/SCL are on pins 20/21 (not A4/A5)
- Check AD0 pin is pulled to GND
- Try `Wire.begin()` before `imu_init()`

### Smoke Test: QMC5883L Compass

1. Uncomment `#define RUN_TEST_COMPASS` in `test/test_compass.cpp`
2. Flash and open Serial Monitor

**Run calibration first (important!):**
Uncomment the `compass_calibrate()` call in the test sketch. Rotate the board slowly through 360° in 10 seconds. Note the printed `OFFSET_X` and `OFFSET_Y` values, then paste them into `src/sensors/compass.cpp`:

```cpp
static constexpr float OFFSET_X = -245.5f;  // ← your values here
static constexpr float OFFSET_Y =  118.2f;
```

**Expected output (after calibration, pointing North):**
```
mx,      my,     mz,    Heading(°)
214.3,   -3.1,  -42.0,   1.4
```

### Smoke Test: NEO-6M GPS

1. Uncomment `#define RUN_TEST_GPS` in `test/test_gps.cpp`
2. Flash, then **take the board outdoors** with clear sky view
3. The blue LED on the module blinks every second when a fix is acquired

**Expected output (after fix):**
```
Lat,          Lon,          Alt(m), Speed(kmh), Sats, Valid
25.040000,    121.565000,   22.3,   0.1,        7,    YES
```

> Cold start (first fix after power-up) can take **30 seconds to 3 minutes**. Subsequent starts are much faster.

---

## Phase 2 — FreeRTOS Multi-Task System

Once all three sensor smoke tests pass, you are ready to run the full RTOS system.

### Understanding `setup()` in `main.cpp`

```cpp
void setup() {
    // 1. Start I²C at 400 kHz (Fast Mode)
    Wire.begin();
    Wire.setClock(400000UL);

    // 2. Init sensors
    imu_init();
    compass_init();
    gps_init();
    madgwick_init();

    // 3. Create queues (length=1, holds ONE item — always the freshest)
    xQueueIMU = xQueueCreate(1, sizeof(IMUData_t));
    xQueueMAG = xQueueCreate(1, sizeof(MagData_t));
    xQueueGPS = xQueueCreate(1, sizeof(GPSData_t));

    // 4. Create mutexes
    xI2CMutex = xSemaphoreCreateMutex();  // guards the I²C bus
    xNavMutex = xSemaphoreCreateMutex();  // guards navOut struct

    // 5. Create tasks
    xTaskCreate(Task_IMU,    "IMU",    80,  NULL, 4, NULL);
    xTaskCreate(Task_Fusion, "Fusion", 110, NULL, 5, NULL);
    // ... etc.
    // Scheduler starts automatically after setup() returns (AVR port)
}
```

### Understanding `xQueueOverwrite` vs `xQueueSend`

```
xQueueSend:      [old] [old] [new]   ← waits if queue full
xQueueOverwrite: [new]               ← replaces old item immediately
```

We use `xQueueOverwrite` because we always want the **latest** sensor reading. Stale data is worse than no data for a navigation filter.

### Understanding the I²C Mutex

```cpp
// Task_IMU
if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    imu_read(&data);          // safe: only one task on I²C
    xSemaphoreGive(xI2CMutex);
}

// Task_Compass (runs concurrently)
if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    compass_read(&data);      // blocks until IMU releases the bus
    xSemaphoreGive(xI2CMutex);
}
```

Without the mutex, both tasks could try to talk to different I²C sensors simultaneously, corrupting the data on the shared SDA/SCL wires.

### Serial Monitor Output

Flash `main.cpp` (all `#define RUN_TEST_*` commented out) and open Serial Monitor at **115200 baud**. You should see CSV lines at 2 Hz:

```
Roll,Pitch,Yaw,Lat,Lon,Alt,Spd,FreeHeap
1.23,-0.45,182.7,25.040001,121.565002,22.3,0.0,4812
```

> **FreeHeap > 1024** means you have enough RAM margin. If it drops below 1024, reduce task stack sizes.

---

## Phase 3 — Madgwick Sensor Fusion

The Madgwick filter takes raw sensor data and produces smooth, accurate Roll/Pitch/Yaw angles.

### Why Not Just Use the Accelerometer?

| Method | Roll/Pitch | Yaw | Problem |
|--------|-----------|-----|---------|
| Accel only | ✓ | ✗ | Noisy under vibration; no yaw |
| Gyro only | ✓ | ✓ | **Drifts** — error accumulates over time |
| Madgwick (9-DOF) | ✓ | ✓ | **Best of both** — corrects drift with accel/mag |

### The Beta Parameter

```cpp
madgwick_set_beta(0.1f);  // default
```

- **Higher beta (0.5–1.0):** Converges to correct attitude faster after power-on, but is noisier
- **Lower beta (0.01–0.05):** Very smooth output, but takes longer to correct gyro drift
- **Recommended starting value:** `0.1` for a stationary/slow-moving platform

You can tune beta at runtime by sending a command from the ground station:
```bash
python ground_station.py --port /dev/ttyUSB0 --beta 0.05
```

### Validating Fusion Output (AC-4, AC-5)

1. Place the board **flat and still** for 30 seconds
2. Roll and Pitch should stabilise within ±1°
3. Yaw drift should be less than 2°/minute

---

## Phase A — HC-12 Wireless Telemetry

The HC-12 is a simple 433 MHz radio module that looks like a serial port to the Arduino.

### HC-12 AT Command Configuration

The code automatically configures the HC-12 at startup via AT commands (in `hc12_configure()`):

| Command | Meaning |
|---------|---------|
| `AT+C010` | Channel 10 (433.4 MHz) |
| `AT+P8` | Maximum TX power (100 mW) |
| `AT+FU3` | FU3 mode (long-range, 9600 effective baud) |
| `AT+B9600` | UART baud rate = 9600 |

Both the Arduino HC-12 and the ground-station HC-12 **must** be on the same channel.

### Telemetry Packet Format

```
Byte  0:  0xAA        ← Start sentinel (always)
Byte  1:  length      ← Payload byte count
Byte  2:  0x01        ← Packet type: navigation telemetry
Byte 3-4: roll×100    ← int16, little-endian  (123 = 1.23°)
Byte 5-6: pitch×100   ← int16
Byte 7-8: yaw×10      ← uint16               (1827 = 182.7°)
Byte 9-12: lat×1e6   ← int32                (25040001 = 25.040001°)
Byte13-16: lon×1e6   ← int32
Byte17-18: alt×10    ← int16, decimetres    (223 = 22.3 m)
Byte  19: sats        ← uint8
Byte  20: CRC-8       ← over bytes [1..19]
```

**Why integers instead of floats?**
- `float` on AVR is 4 bytes; `int16_t` is 2 bytes — saves space
- Integers are endian-safe when encoded byte-by-byte
- CRC can catch single-byte corruption; floats would silently produce wrong values

---

## Ground Station (Python)

### Installation

```bash
pip install pyserial
```

### Usage

Connect the ground-station HC-12 to your PC via a USB-UART adapter, then:

```bash
# Linux / macOS
python ground_station.py --port /dev/ttyUSB0

# Windows
python ground_station.py --port COM3

# Send a beta-tuning command
python ground_station.py --port /dev/ttyUSB0 --beta 0.05
```

**Expected output:**
```
    Roll    Pitch      Yaw          Lat          Lon     Alt  Sats
----------------------------------------------------------------------
    1.23    -0.45   182.70   25.040001  121.565002    22.3     7
    1.24    -0.44   182.71   25.040001  121.565002    22.3     7
```

---

## Troubleshooting

### "MPU-6050 not found"
- Check SDA=Pin20, SCL=Pin21
- Verify AD0 is pulled LOW (address 0x68)
- Add a 4.7 kΩ pull-up resistor on SDA and SCL if bus is long

### Compass heading is 180° off
- Your board's +X axis points South; rotate 180° or negate `mx` in `compass_read()`

### GPS never gets a fix
- Go outdoors — GPS does not work through roofs
- Wait up to 3 minutes on cold start
- Check TX of GPS goes to RX1 (Pin 19) of Mega — it's easy to swap TX/RX

### FreeRTOS won't start / hangs in setup()
- Check `xQueueCreate` and `xSemaphoreCreateMutex` return non-null
- Reduce stack sizes if heap is too small
- Add `Serial.print(xPortGetFreeHeapSize())` to find memory pressure

### Roll/Pitch jitter
- Run `compass_calibrate()` — hard-iron offsets cause Madgwick to fight itself
- Increase `beta` temporarily during startup, then lower it

### HC-12 no data received
- Both modules must be on **same channel** (`AT+C010`)
- SET pin must be HIGH during normal operation (LOW = AT mode)
- Try `AT+RX` on the ground-station HC-12 to verify it receives anything

---

## Acceptance Criteria

Run these tests to confirm your system meets the design requirements:

| ID | Test | How to Check | Pass Condition |
|----|------|-------------|----------------|
| AC-1 | IMU static | Smoke test, board flat | Roll/Pitch within ±1° |
| AC-2 | Compass | After calibration, point North | Heading within ±5° of 0° |
| AC-3 | GPS fix | Outdoors, cold start | Fix within 3 minutes |
| AC-4 | Yaw drift | Main system, stationary, 30 s | Drift < 2°/min |
| AC-5 | HC-12 range | Walk 50 m with ground station open | 0% packet loss |
| AC-8 | Heap margin | Serial Monitor FreeHeap column | > 1024 bytes |

---

## Going Further

Once the PoC is working, the task list includes three more phases:

### Phase B — LoRa SX1276 (2 km range)
Replace HC-12 with a LoRa module for kilometre-scale range. Includes Adaptive Data Rate (ADR) that automatically adjusts spreading factor based on link quality.

### Phase C — MAVLink v2
Switch to the industry-standard drone telemetry protocol. Connect to **QGroundControl** and see your vehicle's attitude on the HUD and position on the map.

### Phase D — ESP32 Port
Port the system to ESP32 for Wi-Fi/Bluetooth connectivity, dual-core task pinning, and significantly more RAM and flash.

---

## Project File Map

```
arduino-gps-ins-tutorial/
├── platformio.ini              ← Build configuration (libraries, flags)
├── README.md                   ← This document
├── ground_station.py           ← Python ground station (run on PC)
└── src/
    ├── task_defs.h             ← ALL shared types + FreeRTOS handles
    ├── main.cpp                ← setup(), tasks, RTOS entry point
    ├── sensors/
    │   ├── imu.h / imu.cpp     ← MPU-6050 driver
    │   ├── compass.h/.cpp      ← QMC5883L driver + calibration
    │   └── gps.h / gps.cpp     ← NEO-6M driver (TinyGPSPlus)
    ├── fusion/
    │   ├── madgwick.h          ← Filter API
    │   └── madgwick.cpp        ← Full 9-DOF Madgwick implementation
    └── comms/
        ├── packet.h/.cpp       ← HC-12 packet build/send/receive
        └── mavlink_tx.h/.cpp   ← MAVLink stubs (Phase C)
test/
    ├── test_imu.cpp            ← MPU-6050 standalone smoke test
    ├── test_compass.cpp        ← QMC5883L standalone smoke test
    └── test_gps.cpp            ← NEO-6M standalone smoke test
```

---

*Built with FreeRTOS · MPU-6050 · QMC5883L · NEO-6M · HC-12 · Arduino Mega 2560*
