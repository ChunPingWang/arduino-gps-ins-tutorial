/**
 * packet.cpp — HC-12 packet implementation.
 */

#include "packet.h"
#include "../fusion/madgwick.h"

// HC-12 SET pin (connect to HC-12 "SET" header pin)
static constexpr uint8_t HC12_SET_PIN = 4;

// ===========================================================================
uint8_t crc8(const uint8_t *data, uint8_t len)
// ===========================================================================
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ===========================================================================
void packet_build(NavPacket_t *pkt, const NavOut_t *nav, uint8_t sats)
// ===========================================================================
{
    pkt->stx       = PKT_STX_DOWN;
    pkt->type      = PKT_TYPE_NAV;

    pkt->roll100   = (int16_t)(nav->roll  * 100.0f);
    pkt->pitch100  = (int16_t)(nav->pitch * 100.0f);
    pkt->yaw10     = (uint16_t)(nav->yaw  * 10.0f);

    pkt->lat_e6    = (int32_t)(nav->lat * 1e6);
    pkt->lon_e6    = (int32_t)(nav->lon * 1e6);
    pkt->alt_dm    = (int16_t)(nav->alt * 10.0f);   // decimetres

    pkt->sats      = sats;

    // Length = everything between len and crc8 fields
    pkt->len = sizeof(NavPacket_t) - 3;   // exclude stx, len, crc8

    // CRC over [len .. sats]
    pkt->crc8 = crc8(&pkt->len,
                     (uint8_t)(sizeof(NavPacket_t) - 2));  // exclude stx, crc8
}

// ===========================================================================
void packet_send(const NavPacket_t *pkt)
// ===========================================================================
{
    Serial2.write(reinterpret_cast<const uint8_t*>(pkt), sizeof(NavPacket_t));
}

// ===========================================================================
void hc12_configure()
// ===========================================================================
{
    pinMode(HC12_SET_PIN, OUTPUT);
    digitalWrite(HC12_SET_PIN, LOW);   // Enter AT command mode
    delay(100);

    // AT commands — configure channel 10, max power, FU3, 9600 baud
    Serial2.print(F("AT+C010\r\n")); delay(100);
    Serial2.print(F("AT+P8\r\n"));   delay(100);
    Serial2.print(F("AT+FU3\r\n"));  delay(100);
    Serial2.print(F("AT+B9600\r\n")); delay(100);

    digitalWrite(HC12_SET_PIN, HIGH);  // Exit AT command mode
    delay(100);
}

// ===========================================================================
//  Uplink receiver state machine
// ===========================================================================
namespace {
    enum RxState { RX_IDLE, RX_LEN, RX_TYPE, RX_PAYLOAD, RX_CRC };
    RxState  rxState   = RX_IDLE;
    uint8_t  rxBuf[32] = {};
    uint8_t  rxIdx     = 0;
    uint8_t  rxLen     = 0;
    uint8_t  rxCmd     = 0;
}

void packet_rx_process()
{
    while (Serial2.available()) {
        uint8_t b = Serial2.read();

        switch (rxState) {
            case RX_IDLE:
                if (b == PKT_STX_UP) { rxIdx = 0; rxState = RX_LEN; }
                break;

            case RX_LEN:
                rxLen   = b;
                rxBuf[rxIdx++] = b;
                rxState = RX_TYPE;
                break;

            case RX_TYPE:
                rxCmd   = b;
                rxBuf[rxIdx++] = b;
                if (rxLen > 1) { rxState = RX_PAYLOAD; }
                else           { rxState = RX_CRC;     }
                break;

            case RX_PAYLOAD:
                rxBuf[rxIdx++] = b;
                if (rxIdx >= rxLen) { rxState = RX_CRC; }
                break;

            case RX_CRC: {
                // Verify CRC over rxBuf[0..rxLen-1] (len + type + payload)
                uint8_t expected = crc8(rxBuf, rxLen);
                if (expected == b) {
                    // Dispatch command
                    if (rxCmd == CMD_SET_BETA && rxLen >= 5) {
                        float newBeta;
                        memcpy(&newBeta, &rxBuf[2], sizeof(float));
                        madgwick_set_beta(newBeta);
#ifdef DEBUG_SERIAL
                        Serial.print(F("[CMD] beta = "));
                        Serial.println(newBeta, 3);
#endif
                    }
                    // CMD_ARM_DISARM: add your own handler here
                }
                rxState = RX_IDLE;
                break;
            }
        }
    }
}
