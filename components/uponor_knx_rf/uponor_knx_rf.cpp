#include "uponor_knx_rf.h"
#include "esphome/core/log.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <cstring>
#include <cctype>
#include <cstdio>

namespace esphome {
namespace uponor_knx_rf {

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void UponorKnxRf::setup() {
  ESP_LOGCONFIG(TAG, "Initialising Uponor KNX RF (T-45) component...");

  ELECHOUSE_cc1101.setSpiPin(sck_pin_, miso_pin_, mosi_pin_, cs_pin_);
  ELECHOUSE_cc1101.setGDO0(gdo0_pin_);

  if (!ELECHOUSE_cc1101.getCC1101()) {
    ESP_LOGE(TAG, "CC1101 NOT FOUND – check SPI wiring and power (must be 3.3 V)!");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "CC1101 detected OK");

  ELECHOUSE_cc1101.Init();

  // -----------------------------------------------------------------------
  // CC1101 configuration for KNX RF 1.1
  // setCCMode(0) + fixed 64-byte length is the configuration confirmed to
  // trigger GDO0 on incoming KNX RF bursts (blue LED blinks on receive).
  // Do NOT change these – setCCMode(1) or variable-length breaks reception.
  // CRC is handled in software (KNX uses CCITT-16, not CC1101 native CRC).
  // -----------------------------------------------------------------------
  ELECHOUSE_cc1101.setModulation(2);   // ASK/OOK
  ELECHOUSE_cc1101.setCCMode(0);       // Raw capture mode – required for KNX RF
  ELECHOUSE_cc1101.setMHZ(868.3);      // KNX RF EU frequency (433.92 for some regions)
  ELECHOUSE_cc1101.setSyncMode(2);     // 16-bit sync word
  ELECHOUSE_cc1101.setCrc(0);          // Disable hardware CRC; we validate with CCITT-16
  ELECHOUSE_cc1101.setLengthConfig(0); // Fixed-length mode
  ELECHOUSE_cc1101.setPacketLength(PKT_MAX_LEN); // 64 bytes captures full KNX RF frame
  ELECHOUSE_cc1101.setPktFormat(0);    // Normal FIFO mode
  ELECHOUSE_cc1101.setPA(12);

  ELECHOUSE_cc1101.SetRx();

  cc1101_ok_ = true;
  ESP_LOGCONFIG(TAG, "CC1101 ready – listening at 868.3 MHz (KNX RF 1.1)");
  ESP_LOGI(TAG, ">>> Trigger a thermostat now – every received packet will be logged below <<<");
}

// ---------------------------------------------------------------------------
// dump_config()
// ---------------------------------------------------------------------------
void UponorKnxRf::dump_config() {
  ESP_LOGCONFIG(TAG, "Uponor KNX RF:");
  ESP_LOGCONFIG(TAG, "  Model target  : T-45 (also compatible with T-33/T-37/T-54/T-55/T-75)");
  ESP_LOGCONFIG(TAG, "  Frequency     : 868.3 MHz");
  ESP_LOGCONFIG(TAG, "  CC1101 pins   : GDO0=%d  CS=%d  MOSI=%d  MISO=%d  SCK=%d",
                gdo0_pin_, cs_pin_, mosi_pin_, miso_pin_, sck_pin_);
  ESP_LOGCONFIG(TAG, "  Thermostats   : %d configured", (int)thermostats_.size());
  for (auto &t : thermostats_) {
    ESP_LOGCONFIG(TAG, "    - %-20s  serial: %s", t.room_name.c_str(), t.serial_hex.c_str());
  }
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void UponorKnxRf::loop() {
  if (!cc1101_ok_) return;

  // 100ms matches the original working configuration.
  // CheckRxFifo internally calls delay(t) and polls GDO0/RSSI.
  if (ELECHOUSE_cc1101.CheckRxFifo(100)) {
    receive_and_process_();
  }
}

// ---------------------------------------------------------------------------
// receive_and_process_()
// ---------------------------------------------------------------------------
void UponorKnxRf::receive_and_process_() {
  // This line prints at INFO every time CheckRxFifo says data is ready.
  // If you see "FIFO triggered" but NO "RAW RX" line below it, ReceiveData
  // returned len=0 (noise burst, not a real packet).
  ESP_LOGI(TAG, "--- FIFO triggered, reading packet ---");

  uint8_t buf[PKT_MAX_LEN + 2];
  memset(buf, 0, sizeof(buf));

  int len = ELECHOUSE_cc1101.ReceiveData(buf);

  ELECHOUSE_cc1101.SetRx();  // Re-arm receiver immediately after every read

  // --- Always print raw bytes at INFO level until serials are known -------
  // We use INFO (not DEBUG) so this is visible even without changing logger
  // level, and it's not gated by a compile-time constant.
  {
    char hex[PKT_MAX_LEN * 3 + 8];
    int  pos = 0;
    for (int i = 0; i < len && i < PKT_MAX_LEN; i++) {
      pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", buf[i]);
    }
    ESP_LOGI(TAG, "RAW RX [%d bytes]: %s", len, hex);
  }

  if (len < PKT_MIN_LEN) {
    ESP_LOGI(TAG, "  ^ Short packet (%d bytes) – noise or truncated, skipping parse", len);
    return;
  }

  // --- Log byte[1] to help identify KNX frame type -----------------------
  // KNX RF 1.1 data frames have byte[1]=0x44. But we intentionally do NOT
  // drop other values here – we log them so you can see what the T-45 sends.
  if (buf[1] != 0x44) {
    ESP_LOGI(TAG, "  ^ byte[1]=0x%02X (expected 0x44 for KNX RF 1.1 – still extracting serial)", buf[1]);
  }

  // --- Extract and log the serial regardless of byte[1] ------------------
  std::string serial = extract_serial_(buf);
  ESP_LOGI(TAG, "  ^ Serial bytes [4-9]: %s  <-- copy this into your YAML", serial.c_str());

  // --- Now try to find a matching configured thermostat ------------------
  ThermostatEntry *matched = nullptr;
  for (auto &t : thermostats_) {
    if (t.serial_hex == serial) {
      matched = &t;
      break;
    }
  }

  if (!matched) {
    ESP_LOGI(TAG, "  ^ No match in config. Add serial '%s' to uponor_knx_rf.yaml", serial.c_str());
    return;
  }

  // --- CRC check ---------------------------------------------------------
  if (!validate_packet_(buf, (uint8_t)len)) {
    ESP_LOGW(TAG, "[%s] CRC error – packet discarded", matched->room_name.c_str());
    return;
  }

  // --- Decode temperatures -----------------------------------------------
  float temperature = NAN;
  float setpoint    = NAN;
  if (!decode_dpt9_pair_(buf, (uint8_t)len, temperature, setpoint)) {
    ESP_LOGW(TAG, "[%s] Temperature decode failed (packet len=%d)", matched->room_name.c_str(), len);
    return;
  }

  // Battery: RF-Info byte 3, bit 6 (1=OK, 0=low)
  float battery_ok = (buf[3] & 0x40) ? 1.0f : 0.0f;

  ESP_LOGI(TAG, "[%s] temp=%.2f°C  setpoint=%.2f°C  battery=%s",
           matched->room_name.c_str(),
           temperature, setpoint,
           battery_ok > 0.0f ? "OK" : "LOW");

  if (matched->temperature && !std::isnan(temperature))
    matched->temperature->publish_state(temperature);

  if (matched->setpoint && !std::isnan(setpoint))
    matched->setpoint->publish_state(setpoint);

  if (matched->battery)
    matched->battery->publish_state(battery_ok);
}

// ---------------------------------------------------------------------------
// validate_packet_()
// KNX RF blocks end with 2-byte CRC-16/CCITT over all preceding bytes.
// ---------------------------------------------------------------------------
bool UponorKnxRf::validate_packet_(const uint8_t *buf, uint8_t len) {
  if (len < 3) return false;
  uint16_t expected = ((uint16_t)buf[len - 2] << 8) | buf[len - 1];
  uint16_t computed = knx_crc16(buf, len - 2);
  return expected == computed;
}

// ---------------------------------------------------------------------------
// extract_serial_()
// Bytes 4..9 form the 6-byte KNX individual address (thermostat serial).
// Returned as uppercase 12-char hex, e.g. "007460AABBCC".
// ---------------------------------------------------------------------------
std::string UponorKnxRf::extract_serial_(const uint8_t *buf) {
  char s[13];
  snprintf(s, sizeof(s), "%02X%02X%02X%02X%02X%02X",
           buf[SERIAL_OFFSET + 0], buf[SERIAL_OFFSET + 1],
           buf[SERIAL_OFFSET + 2], buf[SERIAL_OFFSET + 3],
           buf[SERIAL_OFFSET + 4], buf[SERIAL_OFFSET + 5]);
  return std::string(s);
}

// ---------------------------------------------------------------------------
// decode_dpt9_()
// Decodes a KNX DPT 9.001 (2-byte float) value.
// Returns NAN for the reserved invalid value 0x7FFF.
// ---------------------------------------------------------------------------
float UponorKnxRf::decode_dpt9_(uint8_t hi, uint8_t lo) {
  uint16_t raw = ((uint16_t)hi << 8) | lo;
  if (raw == DPT9_INVALID) return NAN;

  // Exponent: bits 14..11 (4 bits, unsigned)
  int exp = (raw >> 11) & 0x0F;
  // Mantissa: bits 10..0 (11 bits, two's-complement via sign bit at bit 10)
  int mant = raw & 0x07FF;
  if (mant & 0x0400) mant -= 0x0800;  // sign-extend

  return 0.01f * (float)mant * (float)(1 << exp);
}

// ---------------------------------------------------------------------------
// decode_dpt9_pair_()
// Actual temp at DATA_OFFSET+0..1, setpoint at DATA_OFFSET+2..3.
// Returns false if the packet is too short or values are out of range.
// ---------------------------------------------------------------------------
bool UponorKnxRf::decode_dpt9_pair_(const uint8_t *buf, uint8_t len,
                                    float &temperature, float &setpoint) {
  if (len < (uint8_t)(DATA_OFFSET + 4 + 2))  // +2 for CRC bytes
    return false;

  temperature = decode_dpt9_(buf[DATA_OFFSET + 0], buf[DATA_OFFSET + 1]);
  setpoint    = decode_dpt9_(buf[DATA_OFFSET + 2], buf[DATA_OFFSET + 3]);

  // Sanity bounds – reject physically impossible values
  if (!std::isnan(temperature) && (temperature < -20.0f || temperature > 60.0f))
    return false;
  if (!std::isnan(setpoint)    && (setpoint    <   0.0f || setpoint    > 45.0f))
    return false;

  return true;
}

// ---------------------------------------------------------------------------
// to_upper_()
// ---------------------------------------------------------------------------
std::string UponorKnxRf::to_upper_(const std::string &s) {
  std::string out = s;
  for (auto &c : out) c = (char)toupper((unsigned char)c);
  return out;
}

}  // namespace uponor_knx_rf
}  // namespace esphome
