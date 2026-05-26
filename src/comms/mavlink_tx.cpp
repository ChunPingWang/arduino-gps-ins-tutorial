/**
 * mavlink_tx.cpp — MAVLink v2 stub (Phase C).
 *
 * TODO (T-081 to T-086):
 *  1. Add   mavlink/mavlink (c_library_v2) to platformio.ini lib_deps
 *  2. Include <mavlink.h> and define SYSTEM_ID / COMPONENT_ID
 *  3. Replace each stub below with the real mavlink_msg_*_pack() call
 *  4. Write packed bytes to Serial2 (HC-12) or LoRa queue
 */

#include "mavlink_tx.h"

void mavlink_send_heartbeat() {
    // TODO: mavlink_msg_heartbeat_pack(...)
}

void mavlink_send_attitude(float roll, float pitch, float yaw,
                           float rollRate, float pitchRate, float yawRate) {
    // TODO: mavlink_msg_attitude_pack(...)
}

void mavlink_send_global_position(double lat, double lon, float alt, float spd) {
    // TODO: mavlink_msg_global_position_int_pack(...)
}

void mavlink_send_sys_status(uint8_t bat_pct) {
    // TODO: mavlink_msg_sys_status_pack(...)
}
