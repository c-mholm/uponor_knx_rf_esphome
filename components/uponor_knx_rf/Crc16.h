#pragma once
#include <stdint.h>

// CRC-16/EN-13757  (KNX RF 1.1 / Wireless M-Bus / prEN 50090-3-4)
//
// Parameters from CRC catalogue (reveng.sourceforge.io/crc-catalogue/16.htm):
//   Polynomial : 0x3D65
//   Init       : 0x0000
//   RefIn      : false  (bytes processed MSB-first, NOT reflected)
//   RefOut     : false
//   XorOut     : 0xFFFF (result is bitwise-inverted before return)
//   Check      : 0xC2B7 (CRC of "123456789")
//
// Reference implementation: libmbus (github.com/rscada/libmbus), mbus_crc()
// Previous version incorrectly used the reflected polynomial 0xA6BC
// (RefIn/RefOut=true), which caused blk1_crc=FAIL on every valid packet.

static inline uint16_t knx_crc16(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0x0000;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= ((uint16_t)data[i] << 8);  // XOR byte into MSB (non-reflected)
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x3D65);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return ~crc;  // XorOut = 0xFFFF
}
