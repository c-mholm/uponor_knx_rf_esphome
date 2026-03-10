#pragma once
#include <stdint.h>

// CRC-16/EN-13757  (KNX RF 1.1 / prEN 50090-3-4 / EN 13849)
//
// Parameters:
//   Polynomial : 0x3D65
//   Init       : 0x0000
//   RefIn      : true  (input bytes processed LSB-first)
//   RefOut     : true  (output naturally reflected by LSB-first processing)
//   XorOut     : 0xFFFF (result is bitwise-inverted before return)
//   Reflected poly for LSB-first algorithm: 0xA6BC
//
// Reference implementations (all produce identical results):
//   - aviborg/MonitorKNXRF (Python)
//   - libmbus wM-Bus open-source library (C)
//   - thelsing/knx Arduino/ESP KNX stack (C++)
//
// Previous file used CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) — wrong
// for KNX RF 1.1, which caused blk1_crc=FAIL on every packet.

static inline uint16_t knx_crc16(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0x0000;
  for (uint8_t i = 0; i < length; i++) {
    uint8_t byte = data[i];
    for (int bit = 0; bit < 8; bit++) {
      if ((crc ^ byte) & 0x0001) {
        crc = (crc >> 1) ^ 0xA6BC;  // reflected polynomial of 0x3D65
      } else {
        crc >>= 1;
      }
      byte >>= 1;
    }
  }
  return ~crc;  // XorOut = 0xFFFF
}
