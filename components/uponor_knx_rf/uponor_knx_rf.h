#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "Crc16.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <vector>
#include <string>
#include <cmath>

namespace esphome {
namespace uponor_knx_rf {

static const char *const TAG = "uponor_knx_rf";

// -----------------------------------------------------------------------
// KNX RF 1.1 packet structure (Uponor T-45 / T-55 / T-75 series)
//
// The CC1101 receives raw Manchester-encoded bytes at 32.73 kBaud (2-FSK).
// Each pair of raw bytes Manchester-decodes to one data byte.
// After decoding, the KNX frame starts at offset KNX_OFFSET (32) in the
// decoded buffer because preamble bytes are also received and decoded.
//
// Decoded KNX frame layout (relative to KNX_OFFSET):
//
//  Byte  Field       Notes
//  ----  ----------  ---------------------------------------------------
//  0     L-field     Telegram length (everything after this byte)
//  1     0x44        Fixed control byte for KNX RF 1.1 Fast Ack
//  2     0xFF        Fixed byte
//  3     RF-Info     Bit 1 (mask 0x02) = battery state (1=OK, 0=Low)
//                    Bit 0 (mask 0x01) = unidirectional device flag
//  4..9  Serial      6-byte thermostat individual address (the "MAC")
//                    T-45: starts with 00 74 6x xx xx xx
//                    T-55: starts with 00 74 4x xx xx xx
//                    T-75: starts with 00 74 0x xx xx xx
//  10..11  CRC-hi/lo   Block 1 CRC (CRC-16/EN-13757, poly 0x3D65, bytes 0..9)
//  12..    Data block  Application data (up to 16 data + 2 CRC = 18 bytes)
//
//  Block 2 layout (bytes 12..29 for L=0x13):
//    12..13  Source address
//    14..15  Destination address high
//    16      Destination address low → datapoint type:
//              0x01 = current temperature
//              0x02 = setpoint temperature
//              0x03 = status/mode
//    17      ?
//    18      APCI / data type indicator
//    19      Padding/reserved (0x80)
//    20..21  Data value (DPT 9.001 for dp 1 & 2)
//    22..23  Block 2 CRC
//
// Temperature and setpoint are sent as SEPARATE telegrams, each with
// 2-byte DPT 9.001 data at offset 20-21 in the KNX frame.
//
// DPT 9.001 encoding: value = 0.01 * M * 2^E
//   Bits 15:    sign of mantissa
//   Bits 14:11  E  (4-bit exponent, unsigned)
//   Bits 10:0   M  (11-bit mantissa, two's-complement)
// -----------------------------------------------------------------------

// Raw CC1101 buffer size (Manchester encoded, ~2x actual payload)
static const uint8_t RAW_BUF_LEN    = 128;
// Decoded buffer size
static const uint8_t DECODED_BUF_LEN = 64;
// Minimum raw bytes for any useful packet (even short ones get logged for debugging)
static const uint8_t RAW_MIN_LEN    = 10;
// Byte offsets within the KNX frame (relative to frame start)
static const uint8_t SERIAL_OFFSET  = 4;
// Invalid DPT 9.001 marker
static const uint16_t DPT9_INVALID  = 0x7FFF;

struct ThermostatEntry {
  std::string serial_hex;   // 12 hex chars, e.g. "007460AABBCC"
  std::string room_name;
  sensor::Sensor *temperature{nullptr};
  sensor::Sensor *setpoint{nullptr};
  sensor::Sensor *battery{nullptr};
};

class UponorKnxRf : public Component {
 public:
  // ---- Configuration API (called by generated C++ from __init__.py) ----
  void set_gdo0_pin(int p)  { gdo0_pin_  = p; }
  void set_cs_pin(int p)    { cs_pin_    = p; }
  void set_mosi_pin(int p)  { mosi_pin_  = p; }
  void set_miso_pin(int p)  { miso_pin_  = p; }
  void set_sck_pin(int p)   { sck_pin_   = p; }

  void add_thermostat(const std::string &serial, const std::string &name,
                      sensor::Sensor *temp, sensor::Sensor *setp,
                      sensor::Sensor *batt) {
    ThermostatEntry e;
    e.serial_hex  = to_upper_(serial);
    e.room_name   = name;
    e.temperature = temp;
    e.setpoint    = setp;
    e.battery     = batt;
    thermostats_.push_back(e);
  }

  // ---- ESPHome lifecycle ----
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  void        receive_and_process_();
  uint8_t     manchester_decode_(const uint8_t *raw, int raw_len, uint8_t *decoded);
  uint8_t     mandecode_byte_(uint8_t hi, uint8_t lo);
  std::string extract_serial_(const uint8_t *knx_frame);
  float       decode_dpt9_(uint8_t hi, uint8_t lo);
  uint16_t    transform_temperature_(uint16_t data);
  std::string to_upper_(const std::string &s);

  int  gdo0_pin_{2};
  int  cs_pin_{5};
  int  mosi_pin_{23};
  int  miso_pin_{19};
  int  sck_pin_{18};

  std::vector<ThermostatEntry> thermostats_;
  bool cc1101_ok_{false};
  bool first_loop_{true};
  uint32_t packet_count_{0};
  uint32_t unknown_serial_count_{0};
  uint32_t last_status_log_{0};
};

}  // namespace uponor_knx_rf
}  // namespace esphome
