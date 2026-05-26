/**
 * packet.h
 * --------
 * HC-12 lightweight telemetry packet builder and receiver.
 *
 * BEGINNER NOTE — Why a custom packet format?
 *   Sending raw floats over a radio link is fragile:
 *    • Bytes can be lost (dropped packet)
 *    • Bytes can be corrupted (bit flip)
 *    • We need to know where one packet ends and the next begins
 *
 *   Our solution:
 *    1. Fixed start byte (STX = 0xAA) → marks the beginning of a frame
 *    2. Length field → tells the receiver how many bytes to expect
 *    3. CRC-8 at the end → detects single-byte corruption
 *    4. Integers instead of floats → saves 2–4 bytes per field and is
 *       endian-safe when encoded byte-by-byte
 */

#pragma once

#include "../task_defs.h"

/** Start-of-frame sentinel for downlink (Arduino → ground). */
static constexpr uint8_t PKT_STX_DOWN = 0xAA;

/** Start-of-frame sentinel for uplink (ground → Arduino). */
static constexpr uint8_t PKT_STX_UP   = 0xFC;

/** Packet type IDs. */
static constexpr uint8_t PKT_TYPE_NAV = 0x01;   ///< Navigation telemetry
static constexpr uint8_t PKT_TYPE_ACK = 0x02;   ///< Acknowledgement

/** Uplink command codes. */
static constexpr uint8_t CMD_SET_BETA   = 0x01;  ///< Payload: float beta (4 bytes)
static constexpr uint8_t CMD_ARM_DISARM = 0x02;  ///< Payload: 0x00=disarm, 0x01=arm

/**
 * Compute CRC-8 (polynomial 0x07, Dallas/Maxim) over a byte array.
 * @param data  Pointer to data.
 * @param len   Number of bytes.
 * @return      8-bit CRC.
 */
uint8_t crc8(const uint8_t *data, uint8_t len);

/**
 * Build a NavPacket_t from the current navigation state.
 * @param pkt   Destination packet (must not be nullptr).
 * @param nav   Source navigation state.
 * @param sats  Satellite count from GPSData_t.
 */
void packet_build(NavPacket_t *pkt, const NavOut_t *nav, uint8_t sats);

/**
 * Transmit a NavPacket_t over Serial2 (HC-12).
 */
void packet_send(const NavPacket_t *pkt);

/**
 * Configure the HC-12 module via AT commands.
 * Pull SET pin low, send configuration, release SET pin.
 * Call once from setup() before the RTOS scheduler starts.
 */
void hc12_configure();

/**
 * Read and parse one uplink byte from Serial2.
 * Implements a simple state machine over a 32-byte ring buffer.
 * Call from Task_RxCommand whenever Serial2.available() > 0.
 */
void packet_rx_process();
