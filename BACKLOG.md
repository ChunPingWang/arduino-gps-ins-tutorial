# Arduino GPS-INS Tutorial — Backlog

> Last updated: 2026-05-26  
> Repo: https://github.com/ChunPingWang/arduino-gps-ins-tutorial  
> Branch: `main`  
> Build status: ✅ Compiles clean (RAM 18.2 % / Flash 14.0 % on Mega 2560)

---

## ✅ Done (PoC — Phase 0 through Phase A)

### Phase 0 — Project Scaffold
- [x] **T-001** PlatformIO project for `megaatmega2560`
- [x] **T-002** All library deps in `platformio.ini` (FreeRTOS, MPU6050, QMC5883LCompass, TinyGPSPlus, RadioLib)
- [x] **T-003** Full directory structure (`src/sensors/`, `src/fusion/`, `src/comms/`, `test/`)
- [x] **T-004** Shared data structs: `IMUData_t`, `MagData_t`, `GPSData_t`, `NavOut_t`, `NavPacket_t`
- [x] **T-005** FreeRTOS handles as `extern` in `task_defs.h` (queues, mutexes, `navOut`)

### Phase 1 — Sensor Drivers
- [x] **T-011** `imu_init()` — MPU-6050, ±2 g / ±250 °/s ranges
- [x] **T-012** `imu_read()` — raw→g and raw→rad/s conversion
- [x] **T-013** `test/test_imu.cpp` — accel-only Roll/Pitch at 10 Hz (guarded by `#define`)
- [x] **T-021** `compass_init()` — QMC5883L, Continuous/200 Hz/8 G/OSR 512
- [x] **T-022** `compass_read()` — hard-iron correction, heading, declination, normalise [0,360)
- [x] **T-023** `compass_calibrate()` — 10 s 360° rotation, prints offsets
- [x] **T-024** `test/test_compass.cpp` — heading at 5 Hz (guarded by `#define`)
- [x] **T-031** `Serial1.begin(9600)` for NEO-6M in `gps_init()`
- [x] **T-032** `gps_poll()` — TinyGPSPlus feed loop, populates `GPSData_t`
- [x] **T-033** `test/test_gps.cpp` — lat/lon/alt/sats at 1 Hz (guarded by `#define`)

### Phase 2 — FreeRTOS Multi-Task Architecture
- [x] **T-041** `setup()` — Wire fast-mode, sensor inits, queue/mutex creation
- [x] **T-042** `Task_IMU` — Priority 4, 100 Hz, `xQueueOverwrite`
- [x] **T-043** `Task_Compass` — Priority 3, 50 Hz, I²C mutex
- [x] **T-044** `Task_GPS` — Priority 2, 20 Hz polling
- [x] **T-045** `Task_Fusion` — Priority 5, 50 Hz, calls Madgwick, writes `navOut`
- [x] **T-046** `Task_Display` — Priority 1, 2 Hz, CSV to Serial
- [x] **T-047** Free-RAM reported via `freeRamBytes()` (AVR `__brkval` method; `heap_3` does not export `xPortGetFreeHeapSize`)

### Phase 3 — Madgwick AHRS
- [x] **T-051** Quaternion state `q0=1, q1=q2=q3=0`, tunable `beta = 0.1`
- [x] **T-052** `madgwick_update()` — full 9-DOF gradient-descent, fast inv-sqrt, zero-mag guard
- [x] **T-053** `madgwick_get_euler()` — Roll/Pitch/Yaw with Yaw normalised [0, 360)
- [x] **T-054** `madgwick_set_beta()` — runtime tuning via uplink command
- [ ] **T-055** ⬛ *Validation* — board stationary: Roll/Pitch ±1°, Yaw drift < 2°/min after 30 s warm-up *(needs hardware)*

### Phase A — HC-12 Wireless Telemetry
- [x] **T-062** `hc12_configure()` — AT commands at startup (CH10, P8, FU3, 9600)
- [x] **T-063** `NavPacket_t` — 21-byte packed struct with STX, len, type, CRC
- [x] **T-064** `crc8()` — polynomial 0x07
- [x] **T-065** `Task_TxTelemetry` — Priority 2, 5 Hz, packs & sends `NavPacket_t`
- [x] **T-066** `Task_RxCommand` — 5-state machine, dispatches `CMD_SET_BETA` / `CMD_ARM_DISARM`
- [x] **T-067** `ground_station.py` — pyserial, CRC parser, `--beta` uplink command
- [ ] **T-061** ⬛ `Serial2.begin(9600)` *wiring* — HC-12 on TX2/RX2 pins 16/17 *(needs hardware test)*

### Cross-Cutting
- [x] **T-101** `vApplicationStackOverflowHook()` — prints offending task name, halts
- [x] **T-103** `#define ENABLE_HC12` / `ENABLE_LORA` compile-time transport guards
- [x] **T-105** `README.md` — wiring table, library versions, smoke-test steps, AC table
- [x] `.gitignore` — excludes `.pio/` build artifacts

---

## 🔲 Backlog (Not Yet Implemented)

### Phase A — Remaining
| ID | Task | Notes |
|----|------|-------|
| T-061 | Wire HC-12 to Mega TX2/RX2 (pins 16/17), SET to pin 4 | Hardware only; code is ready |
| T-055 | Validate Madgwick on real hardware (AC-4) | Bench test with logic analyser |

### Phase B — LoRa SX1276 Upgrade
| ID | Task | Notes |
|----|------|-------|
| T-071 | Wire SX1276 to Mega SPI (MOSI 51, MISO 50, SCK 52, NSS 10, DIO0 2, RST 9) | Hardware wiring |
| T-072 | `lora_init()` — RadioLib `SX1276::begin(433.5, 125.0, 9, 7, 0x12, 17, 8)`, HW CRC, ISR | `src/comms/lora.h/.cpp` |
| T-073 | `LoRaNav_t` packed struct (22 bytes) — same as `NavPacket_t` + `bat_pct`, `status` | Extend `task_defs.h` |
| T-074 | `Task_LoRaTxRx` — Priority 2, 3 Hz TX; handle `rxDone`/`txDone` flags | Replace `Task_TxTelemetry` |
| T-075 | `lora_get_link_quality(float *rssi, float *snr)` | After each receive |
| T-076 | Adaptive Data Rate (ADR) — SF±1 based on SNR threshold ±5 dB | Inside `Task_LoRaTxRx` |

### Phase C — MAVLink v2
| ID | Task | Notes |
|----|------|-------|
| T-081 | Add `mavlink/c_library_v2` to `platformio.ini`; define `SYSTEM_ID=1`, `COMPONENT_ID=1` | `mavlink_tx.cpp` stubs ready |
| T-082 | `mavlink_send_heartbeat()` — `MAV_TYPE_QUADROTOR`, `MAV_AUTOPILOT_GENERIC` | Fill stub |
| T-083 | `mavlink_send_attitude()` — angles in rad, rates in rad/s | Fill stub |
| T-084 | `mavlink_send_global_position()` — lat/lon ×1e7, alt as mm | Fill stub |
| T-085 | `mavlink_send_sys_status()` — `UINT16_MAX` for unmeasured voltage/current | Fill stub |
| T-086 | `Task_MAVLink` — Priority 2, 2 Hz; HEARTBEAT at 1 Hz; routes to HC-12 or LoRa queue | New task in `main.cpp` |
| T-087 | Verify with QGroundControl — USB-TTL + HC-12; check HUD and map | Integration test |

### Phase D — ESP32 Port (Optional)
| ID | Task | Notes |
|----|------|-------|
| T-091 | Add `[env:esp32dev]` in `platformio.ini` | Already stubbed in `platformio.ini` |
| T-092 | `#ifdef ARDUINO_ARCH_AVR / ESP32` guards around FreeRTOS includes | Already in `task_defs.h` |
| T-093 | Remap GPS to `Serial2.begin(9600, SERIAL_8N1, 16, 17)` on ESP32 | In `gps.cpp` |
| T-094 | Increase all task stacks to ≥ 4096 bytes (ESP32 counts bytes, not words) | In `main.cpp` |
| T-095 | Pin `Task_Fusion` + `Task_MAVLink` to Core 1 with `xTaskCreatePinnedToCore` | Leave Core 0 for Wi-Fi |
| T-096 | `Task_WiFiBackup` (Core 0) — MQTT JSON publish at 1 Hz | New task |

### Cross-Cutting — Remaining
| ID | Task | Notes |
|----|------|-------|
| T-102 | Runtime stats task (debug only) — `vTaskGetRunTimeStats()` every 10 s | Wrap in `#ifdef DEBUG_SERIAL` |
| T-104 | EEPROM non-volatile calibration — `EEPROM.put/get` for compass offsets & LoRa settings | `src/storage/eeprom_cal.h/.cpp` |

---

## Acceptance Criteria Status

| ID | Test | Pass Condition | Status |
|----|------|---------------|--------|
| AC-1 | IMU static (flat surface) | Roll/Pitch ±1° | ⬛ Needs HW |
| AC-2 | Compass heading | Within ±5° of North (post-cal) | ⬛ Needs HW |
| AC-3 | GPS cold fix | Fix within 3 min outdoors | ⬛ Needs HW |
| AC-4 | Madgwick Yaw drift | < 2°/min after 30 s | ⬛ Needs HW |
| AC-5 | HC-12 link 50 m | 0% packet loss | ⬛ Needs HW |
| AC-6 | LoRa link 2 km | RSSI > −120 dBm | ⬛ Phase B |
| AC-7 | MAVLink / QGC | Attitude + GPS displayed | ⬛ Phase C |
| AC-8 | Heap margin | `freeRamBytes()` > 1024 B | ⬛ Needs HW |
| AC-9 | Task_IMU jitter | < ±2 ms (logic analyser) | ⬛ Needs HW |

---

## Quick-Start Checklist (Next Session)

```
[ ] 1. Wire sensors per README.md §3
[ ] 2. Run compass_calibrate() → paste OFFSET_X/Y into compass.cpp → rebuild
[ ] 3. Smoke-test each sensor individually (uncomment #define RUN_TEST_*)
[ ] 4. Flash main.cpp → verify CSV stream + freeRamBytes > 1024
[ ] 5. Wire HC-12 pair → run ground_station.py → verify telemetry decode
[ ] 6. Tune beta via ground_station.py --beta 0.05 if Yaw is noisy
[ ] 7. Start Phase B (LoRa) when HC-12 link is confirmed working
```
