#!/usr/bin/env python3
"""
ground_station.py — HC-12 Ground Station for Arduino GPS-INS Tutorial
======================================================================

BEGINNER NOTE:
    This script runs on your PC and communicates with the Arduino via
    two HC-12 modules (one on the Arduino, one on a USB-to-UART adapter).

USAGE:
    pip install pyserial
    python ground_station.py --port COM3 --baud 9600   # Windows
    python ground_station.py --port /dev/ttyUSB0       # Linux / macOS

PACKET FORMAT (downlink, Arduino → ground):
    Byte 0:    0xAA (STX)
    Byte 1:    len  (payload length)
    Byte 2:    type (0x01 = nav telemetry)
    Byte 3-4:  roll  × 100  (int16, little-endian)
    Byte 5-6:  pitch × 100  (int16, little-endian)
    Byte 7-8:  yaw   × 10   (uint16, little-endian)
    Byte 9-12: lat   × 1e6  (int32, little-endian)
    Byte13-16: lon   × 1e6  (int32, little-endian)
    Byte17-18: alt   × 10   (int16, decimetres, little-endian)
    Byte 19:   satellite count
    Byte 20:   CRC-8
"""

import argparse
import struct
import time
import serial

# ── Packet constants ──────────────────────────────────────────────────────────
STX_DOWN = 0xAA
STX_UP   = 0xFC
PKT_SIZE = 21   # sizeof(NavPacket_t)

CMD_SET_BETA   = 0x01
CMD_ARM_DISARM = 0x02


def crc8(data: bytes) -> int:
    """CRC-8, polynomial 0x07 (matches Arduino implementation)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def parse_packet(raw: bytes) -> dict | None:
    """
    Parse a raw 21-byte NavPacket_t.
    Returns a dict with decoded fields, or None if CRC fails.
    """
    if len(raw) < PKT_SIZE:
        return None
    if raw[0] != STX_DOWN:
        return None

    # CRC check — covers bytes [1..19] (len through sats)
    computed = crc8(raw[1:PKT_SIZE - 1])
    received = raw[PKT_SIZE - 1]
    if computed != received:
        print(f"[WARN] CRC mismatch: computed 0x{computed:02X}, got 0x{received:02X}")
        return None

    # Unpack fields (little-endian matches AVR)
    _, length, ptype, \
    roll100, pitch100, yaw10, \
    lat_e6, lon_e6, alt_dm, \
    sats, _ = struct.unpack_from('<BBBhHHiihhB', raw, 0)

    # wait — fix struct layout to match packed struct exactly
    # NavPacket_t layout:
    #   stx(1) len(1) type(1) roll100(2) pitch100(2) yaw10(2)
    #   lat_e6(4) lon_e6(4) alt_dm(2) sats(1) crc8(1) = 21 bytes
    (stx, length, ptype,
     roll100, pitch100, yaw10,
     lat_e6, lon_e6, alt_dm,
     sats, crc) = struct.unpack_from('<BBBhHHiiHBB', raw, 0)

    return {
        "roll":  roll100  / 100.0,
        "pitch": pitch100 / 100.0,
        "yaw":   yaw10    / 10.0,
        "lat":   lat_e6   / 1e6,
        "lon":   lon_e6   / 1e6,
        "alt":   alt_dm   / 10.0,
        "sats":  sats,
    }


def send_command(ser: serial.Serial, cmd: int, payload: bytes = b'') -> None:
    """
    Send an uplink command frame:
        STX_UP | len | cmd | payload | crc8
    """
    body = bytes([len(payload) + 1, cmd]) + payload
    crc  = crc8(body)
    frame = bytes([STX_UP]) + body + bytes([crc])
    ser.write(frame)
    print(f"[TX] cmd=0x{cmd:02X} payload={payload.hex()} crc=0x{crc:02X}")


def send_beta(ser: serial.Serial, beta: float) -> None:
    """Send a CMD_SET_BETA command with the given float value."""
    payload = struct.pack('<f', beta)
    send_command(ser, CMD_SET_BETA, payload)


def receive_loop(ser: serial.Serial) -> None:
    """Main receive loop — reads and prints decoded telemetry packets."""
    buf = bytearray()
    print(f"\n{'Roll':>8} {'Pitch':>8} {'Yaw':>8} "
          f"{'Lat':>12} {'Lon':>12} {'Alt':>7} {'Sats':>5}")
    print("-" * 70)

    while True:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)

        # Search for STX in buffer
        while len(buf) >= PKT_SIZE:
            idx = buf.find(STX_DOWN)
            if idx == -1:
                buf.clear()
                break
            if idx > 0:
                del buf[:idx]   # discard bytes before STX

            if len(buf) < PKT_SIZE:
                break

            raw = bytes(buf[:PKT_SIZE])
            pkt = parse_packet(raw)

            if pkt is not None:
                print(f"{pkt['roll']:8.2f} {pkt['pitch']:8.2f} {pkt['yaw']:8.2f} "
                      f"{pkt['lat']:12.6f} {pkt['lon']:12.6f} "
                      f"{pkt['alt']:7.1f} {pkt['sats']:5d}")
                del buf[:PKT_SIZE]
            else:
                del buf[:1]   # skip one byte and retry


def main() -> None:
    parser = argparse.ArgumentParser(
        description="HC-12 ground station for Arduino GPS-INS tutorial")
    parser.add_argument("--port",  default="/dev/ttyUSB0",
                        help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud",  type=int, default=9600,
                        help="Baud rate (default: 9600)")
    parser.add_argument("--beta",  type=float, default=None,
                        help="Send SET_BETA command and exit")
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        return

    time.sleep(0.5)   # let HC-12 settle

    if args.beta is not None:
        send_beta(ser, args.beta)
        print("Beta command sent.")
        ser.close()
        return

    try:
        receive_loop(ser)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
