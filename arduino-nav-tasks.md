# Arduino FreeRTOS Navigation System — Task List

> **Target Platform:** Arduino Mega 2560 (primary) / ESP32 (advanced)
> **RTOS:** FreeRTOS via `Arduino_FreeRTOS` library (AVR) or native (ESP32)
> **Sensors:** MPU-6050 · QMC5883L · NEO-6M GPS
> **Wireless:** HC-12 (Phase A) → LoRa SX1276 (Phase B)
> **Protocol:** Custom lightweight packet (Phase A) → MAVLink v2 (Phase B)

---

## Phase 0 — Project Scaffold

- [ ] **T-001** Create PlatformIO project for `megaatmega2560`
- [ ] **T-002** Add all required library dependencies to `platformio.ini`:
  - `feilipu/FreeRTOS`
  - `electroniccats/MPU6050`
  - `mprograms/QMC5883LCompass`
  - `mikalhart/TinyGPSPlus`
  - `jgromes/RadioLib`
  - `mavlink/mavlink` (c_library_v2)
- [ ] **T-003** Create directory structure:
  ```
  src/
    main.cpp
    sensors/   imu.h imu.cpp
    sensors/   compass.h compass.cpp
    sensors/   gps.h gps.cpp
    fusion/    madgwick.h madgwick.cpp
    comms/     packet.h packet.cpp
    comms/     mavlink_tx.h mavlink_tx.cpp
    tasks/     task_defs.h
  ```
- [ ] **T-004** Define all shared data structures in `task_defs.h`:
  - `IMUData_t`  — `float ax, ay, az, gx, gy, gz; uint32_t ts`
  - `MagData_t`  — `float mx, my, mz, heading`
  - `GPSData_t`  — `double lat, lon; float alt, spd, course; uint8_t sats; bool valid`
  - `NavOut_t`   — `float roll, pitch, yaw; double lat, lon; float alt, spd, heading`
- [ ] **T-005** Declare all FreeRTOS handles as `extern` in `task_defs.h`:
  - Queues: `xQueueIMU`, `xQueueMAG`, `xQueueGPS`
  - Mutexes: `xI2CMutex`, `xNavMutex`
  - Shared state: `NavOut_t navOut`

---

## Phase 1 — Individual Sensor Drivers

### 1-A  MPU-6050 IMU

- [ ] **T-011** Implement `imu_init()` — call `mpu.initialize()`, set `FullScaleAccelRange(0)` (±2 g) and `FullScaleGyroRange(0)` (±250 °/s), assert `testConnection()`
- [ ] **T-012** Implement `imu_read(IMUData_t *out)`:
  - Read raw `int16_t` via `mpu.getMotion6()`
  - Convert accel: `ax = raw / 16384.0f` (g)
  - Convert gyro:  `gx = raw / 131.0f * DEG_TO_RAD` (rad/s)
  - Stamp `out->ts = millis()`
- [ ] **T-013** Write standalone smoke-test sketch `test_imu.cpp`:
  - Print `Roll` and `Pitch` computed from accelerometer only at 10 Hz via `Serial`
  - Verify values are within ±5° when board is flat

### 1-B  QMC5883L Compass

- [ ] **T-021** Implement `compass_init()` — call `compass.init()`, set mode `(0x01, 0x0C, 0x10, 0x00)` (Continuous, 200 Hz, 8 G, OSR 512)
- [ ] **T-022** Implement `compass_read(MagData_t *out)`:
  - Read raw X/Y/Z via `compass.read()` + `getX/Y/Z()`
  - Apply hard-iron offsets `offset_x/y/z` (configurable constants)
  - Compute `heading = atan2(my, mx) * RAD_TO_DEG`
  - Apply magnetic declination correction (default `−3.5°` for Taiwan)
  - Normalise to `[0, 360)`
- [ ] **T-023** Implement `compass_calibrate()` — rotate device 360° for 10 s, record min/max X/Y, compute and print offsets
- [ ] **T-024** Write standalone smoke-test sketch `test_compass.cpp` — print `Heading` at 5 Hz; verify North ≈ 0°/360°

### 1-C  NEO-6M GPS

- [ ] **T-031** Configure `Serial1.begin(9600)` for GPS UART
- [ ] **T-032** Implement `gps_poll(GPSData_t *out)`:
  - Feed all available bytes from `Serial1` to `TinyGPSPlus::encode()`
  - On `gps.location.isUpdated()`, populate `GPSData_t` fields
  - Set `out->valid = gps.location.isValid()`
- [ ] **T-033** Write standalone smoke-test sketch `test_gps.cpp` — print lat/lon/alt/satellites at 1 Hz; confirm fix acquired outdoors

---

## Phase 2 — FreeRTOS Multi-Task Architecture

- [ ] **T-041** In `main.cpp` `setup()`:
  - Init `Wire.begin()` + `Wire.setClock(400000)` (I2C Fast Mode)
  - Call all sensor `_init()` functions
  - Create queues (length = 1, `sizeof` matching struct)
  - Create mutexes `xI2CMutex`, `xNavMutex`
  - Create all tasks (see T-042 → T-046)
  - Call `vTaskStartScheduler()` — `loop()` left empty
- [ ] **T-042** Implement `Task_IMU` (Priority 4, Stack 160 B, 100 Hz):
  - Take `xI2CMutex` (timeout 5 ms) → call `imu_read()` → give mutex
  - `xQueueOverwrite(xQueueIMU, &data)`
  - `vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(10))`
- [ ] **T-043** Implement `Task_Compass` (Priority 3, Stack 160 B, 50 Hz):
  - Take `xI2CMutex` → call `compass_read()` → give mutex
  - `xQueueOverwrite(xQueueMAG, &data)`
  - `vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(20))`
- [ ] **T-044** Implement `Task_GPS` (Priority 2, Stack 220 B, polling 20 Hz):
  - Call `gps_poll()` in tight loop; `xQueueOverwrite(xQueueGPS, &data)` on valid update
  - `vTaskDelay(pdMS_TO_TICKS(20))` at end of loop body
- [ ] **T-045** Implement `Task_Fusion` (Priority 5, Stack 220 B, 50 Hz):
  - `xQueuePeek` IMU and MAG queues (no-wait)
  - Compute `dt` from `millis()` delta; clamp to `[0, 0.2]` s
  - Call `madgwick_update()` (see T-052)
  - Convert quaternion to Roll/Pitch/Yaw; store in `navOut` under `xNavMutex`
  - `vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(20))`
- [ ] **T-046** Implement `Task_Display` (Priority 1, Stack 200 B, 2 Hz):
  - `xQueuePeek` all three queues
  - Take `xNavMutex`, snapshot `navOut`, give mutex
  - Print CSV line: `Roll,Pitch,Yaw,Lat,Lon,Alt,Spd,FreeHeap`
  - `vTaskDelay(pdMS_TO_TICKS(500))`
- [ ] **T-047** Validate heap margin — add `Serial.print(xPortGetFreeHeapSize())` in `Task_Display`; confirm > 1 KB at runtime

---

## Phase 3 — Madgwick AHRS Sensor Fusion

- [ ] **T-051** Implement `madgwick.h` — declare module-level quaternion state `q0=1, q1=q2=q3=0` and tunable `beta = 0.1f`
- [ ] **T-052** Implement `madgwick_update(float gx, gy, gz, ax, ay, az, mx, my, mz, float dt)` — full 9-DOF gradient-descent update per Madgwick 2010:
  - Guard against zero-magnitude accel or mag vectors (skip gradient step)
  - Use fast inverse square root `invSqrt()` (0x5f3759df bit-hack)
  - Integrate quaternion derivative; renormalise after each step
- [ ] **T-053** Implement `madgwick_get_euler(float *roll, float *pitch, float *yaw)`:
  - `roll  = atan2f(2(q0q1+q2q3), 1−2(q1²+q2²))`
  - `pitch = asinf (2(q0q2−q3q1))`
  - `yaw   = atan2f(2(q0q3+q1q2), 1−2(q2²+q3²))`; normalise to `[0, 360)`
- [ ] **T-054** Implement `madgwick_set_beta(float b)` — runtime tuning via Serial command
- [ ] **T-055** Validate fusion output — with board stationary, confirm Roll/Pitch stable within ±1°, Yaw drift < 2°/min after 30 s warm-up

---

## Phase A — HC-12 Wireless Telemetry

- [ ] **T-061** Configure `Serial2.begin(9600)` for HC-12 module on TX2/RX2 (Pins 16/17)
- [ ] **T-062** Implement `hc12_configure()` — pull SET pin low, send AT commands (`AT+C010`, `AT+P8`, `AT+FU3`, `AT+B9600`), release SET pin
- [ ] **T-063** Define `NavPacket_t` (packed struct, 20 bytes):
  - `uint8_t stx(0xAA), len, type`
  - `int16_t roll100, pitch100; uint16_t yaw10`
  - `int32_t lat_e6, lon_e6; int16_t alt_dm`
  - `uint8_t sats, crc8`
- [ ] **T-064** Implement `crc8(uint8_t *data, uint8_t len)` — polynomial `0x07`
- [ ] **T-065** Implement `Task_TxTelemetry` (Priority 2, Stack 256 B, 5 Hz):
  - Snapshot `navOut` under `xNavMutex`
  - Pack fields into `NavPacket_t`; append CRC
  - `Serial2.write()` raw bytes
  - `vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(200))`
- [ ] **T-066** Implement `Task_RxCommand` (Priority 3, Stack 200 B, event-driven):
  - Read bytes from `Serial2` into ring buffer
  - Parse frame: `STX(0xFC) | LEN | CMD | PAYLOAD | CRC`
  - Dispatch command codes (e.g. `0x01` = set beta, `0x02` = arm/disarm)
  - `vTaskDelay(pdMS_TO_TICKS(10))` when buffer empty
- [ ] **T-067** Write Python ground-station script `ground_station.py`:
  - Open serial port (configurable `PORT`, `BAUDRATE`)
  - Implement same `crc8` and frame parser
  - Print decoded telemetry to console at each received packet
  - Expose `send_command(cmd, payload)` helper function

---

## Phase B — LoRa SX1276 Upgrade

- [ ] **T-071** Wire SX1276 to Mega SPI bus (MOSI 51, MISO 50, SCK 52, NSS 10, DIO0 2, RST 9)
- [ ] **T-072** Implement `lora_init()` using RadioLib `SX1276::begin(433.5, 125.0, 9, 7, 0x12, 17, 8)`:
  - Enable hardware CRC: `radio.setCRC(true)`
  - Attach ISR: `radio.setDio0Action(onRadioIRQ, RISING)`
  - Start in receive mode: `radio.startReceive()`
- [ ] **T-073** Define `LoRaNav_t` (packed struct, 22 bytes) — same fields as `NavPacket_t` plus `uint8_t bat_pct, status`
- [ ] **T-074** Implement `Task_LoRaTxRx` (Priority 2, Stack 300 B, 3 Hz TX):
  - On `rxDone` flag: call `radio.readData()`, dispatch uplink command, restart receive
  - Pack `LoRaNav_t`, call `radio.startTransmit()`
  - Wait for `txDone` flag (max 500 ms timeout), then `radio.startReceive()`
  - `vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(333))`
- [ ] **T-075** Implement `lora_get_link_quality(float *rssi, float *snr)` — read from RadioLib after each receive
- [ ] **T-076** Implement Adaptive Data Rate (ADR) logic inside `Task_LoRaTxRx`:
  - `SNR < −5 dB` → `setSpreadingFactor(SF + 1)` (max SF12)
  - `SNR > +5 dB` && `SF > 7` → `setSpreadingFactor(SF − 1)`
  - Log ADR change to `Serial`

---

## Phase C — MAVLink v2 Integration

- [ ] **T-081** Include MAVLink c_library_v2 headers; define `SYSTEM_ID 1`, `COMPONENT_ID 1`
- [ ] **T-082** Implement `mavlink_send_heartbeat()` — `MAV_TYPE_QUADROTOR`, `MAV_AUTOPILOT_GENERIC`, `MAV_STATE_ACTIVE`
- [ ] **T-083** Implement `mavlink_send_attitude(float roll, pitch, yaw, rollRate, pitchRate, yawRate)` — rates in rad/s, angles in rad
- [ ] **T-084** Implement `mavlink_send_global_position(double lat, lon, float alt, spd)` — encode lat/lon as `×1e7 int32`, alt as mm `int32`
- [ ] **T-085** Implement `mavlink_send_sys_status(uint8_t bat_pct)` — voltage/current fields set to `UINT16_MAX` if not measured
- [ ] **T-086** Implement `Task_MAVLink` (Priority 2, Stack 256 B, 2 Hz):
  - Send `HEARTBEAT` every 1 s (use internal counter)
  - Send `ATTITUDE` + `GLOBAL_POSITION_INT` + `SYS_STATUS` at 2 Hz
  - Write to `Serial2` (HC-12) **or** `Task_LoRaTxRx` queue depending on active transport
- [ ] **T-087** Verify with QGroundControl — connect GCS HC-12 via USB-TTL; confirm HUD displays Roll/Pitch/Yaw and map shows GPS position

---

## Phase D — ESP32 Port (Optional Advanced)

- [ ] **T-091** Create separate PlatformIO env `[env:esp32dev]` in same `platformio.ini`
- [ ] **T-092** Replace `#include <Arduino_FreeRTOS.h>` guards with `#ifdef ARDUINO_ARCH_AVR` / `#ifdef ARDUINO_ARCH_ESP32`
- [ ] **T-093** Remap GPS serial to `Serial2.begin(9600, SERIAL_8N1, 16, 17)`
- [ ] **T-094** Increase all task stack sizes to ≥ 4096 bytes (ESP32 uses bytes not words)
- [ ] **T-095** Pin `Task_Fusion` and `Task_MAVLink` to Core 1 using `xTaskCreatePinnedToCore(..., 1)`; leave Core 0 for WiFi stack
- [ ] **T-096** Implement `Task_WiFiBackup` (Core 0) — connect to AP, publish `NavOut_t` as JSON to MQTT broker at 1 Hz as secondary downlink

---

## Cross-Cutting Concerns

- [ ] **T-101** Add `configASSERT` macro and stack overflow hook `vApplicationStackOverflowHook()` — print offending task name to Serial then halt
- [ ] **T-102** Add runtime stats task (debug build only) — call `vTaskGetRunTimeStats()` and print CPU usage every 10 s
- [ ] **T-103** Define `#define ENABLE_LORA` / `#define ENABLE_HC12` compile-time flags; guard transport code accordingly
- [ ] **T-104** Implement non-volatile calibration storage via `EEPROM.put/get` for compass offsets and LoRa channel/SF settings
- [ ] **T-105** Write `README.md` — wiring table, library versions, AT command setup steps, QGC connection instructions

---

## Acceptance Criteria

| ID | Test | Pass Condition |
|---|---|---|
| AC-1 | IMU static | Roll/Pitch within ±1° on flat surface |
| AC-2 | Compass | Heading within ±5° of known North |
| AC-3 | GPS fix | Valid fix acquired within 3 min outdoors |
| AC-4 | Madgwick Yaw drift | < 2°/min after 30 s warm-up |
| AC-5 | HC-12 link | 0% packet loss at 50 m, < 5% at 500 m |
| AC-6 | LoRa link | RSSI > −120 dBm at 2 km line-of-sight |
| AC-7 | MAVLink / QGC | Attitude and GPS displayed without errors |
| AC-8 | Heap margin | `xPortGetFreeHeapSize()` > 1024 B at runtime |
| AC-9 | Task timing | `Task_IMU` jitter < ±2 ms measured by logic analyser |
