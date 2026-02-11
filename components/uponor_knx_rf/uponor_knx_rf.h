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
// KNX RF packet structure (Uponor T-45 / T-55 / T-75 series)
//
// Raw bytes after CC1101 receive (variable length, ~22–26 bytes):
//
//  Byte  Field       Notes
//  ----  ----------  ---------------------------------------------------
//  0     L-field     Telegram length (everything after this byte)
//  1     0x44        Fixed control byte for KNX RF 1.1 Fast Ack
//  2     0xFF        Fixed byte
//  3     RF-Info     Bit 6 (0-indexed) = battery state (1=OK, 0=Low)
//  4..9  Serial      6-byte thermostat individual address (the "MAC")
//                    T-45: starts with 00 74 6x xx xx xx
//                    T-55: starts with 00 74 4x xx xx xx
//                    T-75: starts with 00 74 0x xx xx xx
//  10    Filler      0x60
//  11..  Data block  Application data
//
// Within the data block, the first two data group bytes hold temperature:
//  +0..+1  DPT 9.001  Actual room temperature
//  +2..+3  DPT 9.001  Setpoint temperature
//
// DPT 9.001 encoding: value = 0.01 * M * 2^E
//   Bits 15:    sign of mantissa (not used in E field)
//   Bits 14:11  E  (4-bit exponent, unsigned)
//   Bits 10:0   M  (11-bit mantissa, two's-complement)
//
// The last 2 bytes of every KNX RF block are CRC-16/CCITT (not included
// in the length that CC1101 reports when hardware CRC is disabled).
// -----------------------------------------------------------------------

static const uint8_t PKT_MAX_LEN   = 64;
static const uint8_t PKT_MIN_LEN   = 12;
static const uint8_t SERIAL_OFFSET = 4;   // byte index of first serial byte
static const uint8_t DATA_OFFSET   = 11;  // byte index of first data byte
static const uint16_t DPT9_INVALID = 0x7FFF;

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
  void      receive_and_process_();
  bool      validate_packet_(const uint8_t *buf, uint8_t len);
  std::string extract_serial_(const uint8_t *buf);
  bool      decode_dpt9_pair_(const uint8_t *buf, uint8_t len,
                               float &temperature, float &setpoint);
  float     decode_dpt9_(uint8_t hi, uint8_t lo);
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
