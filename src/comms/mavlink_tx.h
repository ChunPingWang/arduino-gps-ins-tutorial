/**
 * mavlink_tx.h
 * ------------
 * MAVLink v2 transmit helpers (Phase C — stub for future implementation).
 *
 * BEGINNER NOTE:
 *   MAVLink is the standard telemetry protocol used by ArduPilot,
 *   PX4, and QGroundControl.  Including it requires the c_library_v2
 *   headers from https://github.com/mavlink/c_library_v2
 *
 *   For Phase A/B, the lightweight NavPacket_t protocol is used instead.
 *   This file is provided as a template for Phase C development.
 */

#pragma once

#include "../task_defs.h"

// Forward declarations — full implementation requires MAVLink headers
void mavlink_send_heartbeat();
void mavlink_send_attitude(float roll, float pitch, float yaw,
                           float rollRate, float pitchRate, float yawRate);
void mavlink_send_global_position(double lat, double lon, float alt, float spd);
void mavlink_send_sys_status(uint8_t bat_pct);
