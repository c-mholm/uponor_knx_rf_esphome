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
// Configure CC1101 for KNX RF 1.1: 2-FSK, 32.73 kBaud, 868.3 MHz
// Based on proven tahvane1/uponor_knx_rf_thermostat configuration.
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

  // --- KNX RF 1.1 radio parameters ---
  // Modulation: 2-FSK (NOT ASK/OOK!)
  // Data rate on air: 32.73 kBaud (Manchester symbol rate; effective 16.384 kbit/s)
  // Frequency deviation: 47.6 kHz
  // RX bandwidth: 270 kHz
  // Sync word: 0x7696 (KNX RF RX sync)
  // High-level API calls set the basic radio parameters.
  // setCCMode(1) must be first as it overwrites several registers.
  ELECHOUSE_cc1101.setCCMode(1);
  ELECHOUSE_cc1101.setModulation(0);          // 0 = 2-FSK
  ELECHOUSE_cc1101.setMHZ(868.3);
  ELECHOUSE_cc1101.setDeviation(47.607422);
  ELECHOUSE_cc1101.setChannel(0);
  ELECHOUSE_cc1101.setChsp(199.951172);
  ELECHOUSE_cc1101.setRxBW(270.833333);
  ELECHOUSE_cc1101.setDRate(32.7301);
  ELECHOUSE_cc1101.setPA(5);
  ELECHOUSE_cc1101.setSyncMode(2);            // 16/16 sync word match
  ELECHOUSE_cc1101.setSyncWord(0x76, 0x96);   // KNX RF RX sync word
  ELECHOUSE_cc1101.setPacketLength(61);       // Max packet length

  // Direct register writes — belt-and-braces to guarantee correct config.
  // These are written AFTER the high-level API calls, so they take precedence.
  // Register values match the proven tahvane1/uponor_knx_rf_thermostat and
  // thelsing/knx reference implementations.
  ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x06);  // IOCFG0:  GDO0 = sync word sent/received
  ELECHOUSE_cc1101.SpiWriteReg(0x03, 0x47);  // FIFOTHR: RX FIFO threshold = 33 bytes
  ELECHOUSE_cc1101.SpiWriteReg(0x04, 0x76);  // SYNC1:   sync word high
  ELECHOUSE_cc1101.SpiWriteReg(0x05, 0x96);  // SYNC0:   sync word low
  ELECHOUSE_cc1101.SpiWriteReg(0x06, 0x3D);  // PKTLEN:  fixed packet length = 61 bytes
  ELECHOUSE_cc1101.SpiWriteReg(0x07, 0x00);  // PKTCTRL1: no append status, no addr check
  ELECHOUSE_cc1101.SpiWriteReg(0x08, 0x00);  // PKTCTRL0: fixed length, CRC OFF, no whitening
  ELECHOUSE_cc1101.SpiWriteReg(0x0B, 0x08);  // FSCTRL1: IF frequency
  ELECHOUSE_cc1101.SpiWriteReg(0x0C, 0x00);  // FSCTRL0: frequency offset
  ELECHOUSE_cc1101.SpiWriteReg(0x10, 0x6A);  // MDMCFG4: RxBW=270kHz, DRate exp=0x0A
  ELECHOUSE_cc1101.SpiWriteReg(0x11, 0x4A);  // MDMCFG3: DRate mantissa → 32.73 kBaud
  ELECHOUSE_cc1101.SpiWriteReg(0x12, 0x02);  // MDMCFG2: 2-FSK, Manchester OFF, 16/16 sync
  ELECHOUSE_cc1101.SpiWriteReg(0x13, 0x22);  // MDMCFG1: 4 preamble bytes, chanspc exp
  ELECHOUSE_cc1101.SpiWriteReg(0x15, 0x47);  // DEVIATN: 47.6 kHz deviation
  ELECHOUSE_cc1101.SpiWriteReg(0x17, 0x30);  // MCSM1:   CCA mode, after RX/TX state
  ELECHOUSE_cc1101.SpiWriteReg(0x18, 0x18);  // MCSM0:   auto-cal IDLE→RX/TX
  ELECHOUSE_cc1101.SpiWriteReg(0x19, 0x2E);  // FOCCFG:  frequency offset compensation
  ELECHOUSE_cc1101.SpiWriteReg(0x1A, 0x6D);  // BSCFG:   bit synchronization
  ELECHOUSE_cc1101.SpiWriteReg(0x1B, 0x43);  // AGCCTRL2: AGC target 33 dB
  ELECHOUSE_cc1101.SpiWriteReg(0x1C, 0x40);  // AGCCTRL1: LNA priority
  ELECHOUSE_cc1101.SpiWriteReg(0x1D, 0x91);  // AGCCTRL0: filter samples = 16
  ELECHOUSE_cc1101.SpiWriteReg(0x20, 0xFB);  // WORCTRL: WOR control
  ELECHOUSE_cc1101.SpiWriteReg(0x21, 0xB6);  // FREND1:  front end RX
  ELECHOUSE_cc1101.SpiWriteReg(0x22, 0x10);  // FREND0:  front end TX (FSK)
  ELECHOUSE_cc1101.SpiWriteReg(0x23, 0xE9);  // FSCAL3:  freq synth cal
  ELECHOUSE_cc1101.SpiWriteReg(0x24, 0x2A);  // FSCAL2:  freq synth cal
  ELECHOUSE_cc1101.SpiWriteReg(0x25, 0x00);  // FSCAL1:  freq synth cal
  ELECHOUSE_cc1101.SpiWriteReg(0x26, 0x1F);  // FSCAL0:  freq synth cal
  ELECHOUSE_cc1101.SpiWriteReg(0x29, 0x59);  // FSTEST:  freq synth test
  ELECHOUSE_cc1101.SpiWriteReg(0x2C, 0x81);  // TEST2:   test setting
  ELECHOUSE_cc1101.SpiWriteReg(0x2D, 0x35);  // TEST1:   test setting
  ELECHOUSE_cc1101.SpiWriteReg(0x2E, 0x09);  // TEST0:   test setting

  ELECHOUSE_cc1101.SetRx();

  cc1101_ok_ = true;
  ESP_LOGCONFIG(TAG, "CC1101 listening on 868.3 MHz (KNX RF 1.1, 2-FSK, 32.73 kBaud)");
}

// ---------------------------------------------------------------------------
// dump_config()
// ---------------------------------------------------------------------------
void UponorKnxRf::dump_config() {
  ESP_LOGCONFIG(TAG, "Uponor KNX RF:");
  ESP_LOGCONFIG(TAG, "  Model target  : T-45 (also compatible with T-33/T-37/T-54/T-55/T-75)");
  ESP_LOGCONFIG(TAG, "  Frequency     : 868.3 MHz (2-FSK, 32.73 kBaud)");
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
  if (first_loop_) {
    first_loop_ = false;
    if (cc1101_ok_) {
      ESP_LOGI(TAG, "CC1101 is OK – listening on 868.3 MHz (2-FSK). Press thermostat buttons to discover serials.");
      // Dump registers here (not in setup) so they appear in the API log stream
      ESP_LOGI(TAG, "Regs: MDMCFG4=0x%02X MDMCFG3=0x%02X MDMCFG2=0x%02X MDMCFG1=0x%02X",
               ELECHOUSE_cc1101.SpiReadReg(0x10), ELECHOUSE_cc1101.SpiReadReg(0x11),
               ELECHOUSE_cc1101.SpiReadReg(0x12), ELECHOUSE_cc1101.SpiReadReg(0x13));
      ESP_LOGI(TAG, "Regs: SYNC1=0x%02X SYNC0=0x%02X PKTCTRL0=0x%02X PKTCTRL1=0x%02X PKTLEN=0x%02X",
               ELECHOUSE_cc1101.SpiReadReg(0x04), ELECHOUSE_cc1101.SpiReadReg(0x05),
               ELECHOUSE_cc1101.SpiReadReg(0x08), ELECHOUSE_cc1101.SpiReadReg(0x07),
               ELECHOUSE_cc1101.SpiReadReg(0x06));
      ESP_LOGI(TAG, "Regs: DEVIATN=0x%02X FREND1=0x%02X AGCCTRL2=0x%02X IOCFG0=0x%02X FSCAL3=0x%02X",
               ELECHOUSE_cc1101.SpiReadReg(0x15), ELECHOUSE_cc1101.SpiReadReg(0x21),
               ELECHOUSE_cc1101.SpiReadReg(0x1B), ELECHOUSE_cc1101.SpiReadReg(0x02),
               ELECHOUSE_cc1101.SpiReadReg(0x23));
      byte marcstate = ELECHOUSE_cc1101.SpiReadStatus(0x35);
      ESP_LOGI(TAG, "Regs: MARCSTATE=0x%02X (should be 0x0D=RX)", marcstate);
    } else {
      ESP_LOGE(TAG, "CC1101 FAILED at boot – check SPI wiring (GDO0=%d CS=%d MOSI=%d MISO=%d SCK=%d) and 3.3V power",
               gdo0_pin_, cs_pin_, mosi_pin_, miso_pin_, sck_pin_);
    }
  }

  if (!cc1101_ok_) return;

  // Periodic status heartbeat every 30 s — include CC1101 state for diagnostics
  uint32_t now = millis();
  if (now - last_status_log_ > 30000) {
    last_status_log_ = now;
    byte rxbytes = ELECHOUSE_cc1101.SpiReadStatus(0x3B);  // CC1101_RXBYTES
    byte marcstate = ELECHOUSE_cc1101.SpiReadStatus(0x35); // CC1101_MARCSTATE
    ESP_LOGI(TAG, "Status: %u packets received, %u unknown serials | RXBYTES=%d MARCSTATE=0x%02X",
             packet_count_, unknown_serial_count_, rxbytes, marcstate);
    // MARCSTATE: 0x0D = RX (good), 0x01 = IDLE, 0x11 = RX_OVERFLOW
    if ((marcstate & 0x1F) != 0x0D) {
      ESP_LOGW(TAG, "CC1101 not in RX state (0x%02X) – re-arming", marcstate);
      ELECHOUSE_cc1101.SetRx();
    }
  }

  // Check if there are bytes in the RX FIFO (direct register read, not library call)
  byte rxbytes_now = ELECHOUSE_cc1101.SpiReadStatus(0x3B) & 0x7F;
  if (rxbytes_now > 0) {
    // Wait for the full packet to arrive. At 32.73 kBaud, 61 bytes takes ~15ms.
    // Wait up to 25ms, checking every 5ms for no new bytes (end of packet).
    byte prev = rxbytes_now;
    for (int wait = 0; wait < 5; wait++) {
      delay(5);
      byte cur = ELECHOUSE_cc1101.SpiReadStatus(0x3B) & 0x7F;
      if (cur == prev) break;  // No new bytes → packet complete
      prev = cur;
    }
    receive_and_process_();
  }
}

// ---------------------------------------------------------------------------
// mandecode_byte_()
// Manchester-decode one byte from two raw bytes (hi, lo).
// Each pair of bits: 01 = 1, 10 = 0
// Returns the decoded byte.
// ---------------------------------------------------------------------------
uint8_t UponorKnxRf::mandecode_byte_(uint8_t hi, uint8_t lo) {
  unsigned int combined = ((unsigned int)hi << 8) | lo;
  uint8_t ret = 0;
  for (int i = 0; i < 8; i++) {
    int b2 = (combined >> ((7 - i) * 2)) & 0x03;
    switch (b2) {
      case 0x01: ret = (ret << 1) | 1; break;  // 01 = 1
      case 0x02: ret = (ret << 1) | 0; break;  // 10 = 0
      default:   ret = (ret << 1) | 0; break;  // Manchester error, treat as 0
    }
  }
  return ret;
}

// ---------------------------------------------------------------------------
// manchester_decode_()
// Decode raw CC1101 buffer (Manchester encoded at 2x rate) into actual bytes.
// Returns number of decoded bytes written to 'decoded'.
// ---------------------------------------------------------------------------
uint8_t UponorKnxRf::manchester_decode_(const uint8_t *raw, int raw_len, uint8_t *decoded) {
  uint8_t out_idx = 0;
  for (int i = 0; i + 1 < raw_len && out_idx < DECODED_BUF_LEN; i += 2) {
    decoded[out_idx++] = mandecode_byte_(raw[i], raw[i + 1]);
  }
  return out_idx;
}

// ---------------------------------------------------------------------------
// receive_and_process_()
// Full pipeline: raw CC1101 → log → Manchester decode → find KNX header →
// serial extraction → temperature decode → publish to HA
// ---------------------------------------------------------------------------
void UponorKnxRf::receive_and_process_() {
  uint8_t raw_buf[RAW_BUF_LEN];
  memset(raw_buf, 0, sizeof(raw_buf));

  // Read FIFO directly — we use fixed-length mode so ReceiveData() won't work
  // (it assumes variable-length where first byte = length)
  byte rxbytes = ELECHOUSE_cc1101.SpiReadStatus(0x3B) & 0x7F;  // RXBYTES, mask overflow bit
  int raw_len = (rxbytes > RAW_BUF_LEN) ? RAW_BUF_LEN : rxbytes;
  if (raw_len > 0) {
    ELECHOUSE_cc1101.SpiReadBurstReg(0x3F, raw_buf, raw_len);  // 0x3F = CC1101_RXFIFO
  }
  // Flush FIFO and re-arm receiver
  ELECHOUSE_cc1101.SpiStrobe(0x3A);  // SFRX — flush RX FIFO
  ELECHOUSE_cc1101.SetRx();          // Re-arm receiver

  if (raw_len < RAW_MIN_LEN || raw_len > RAW_BUF_LEN) {
    ESP_LOGD(TAG, "Ignored packet (%d raw bytes)", raw_len);
    return;
  }

  // Always log raw hex at DEBUG level for diagnostics
  {
    char hex[RAW_BUF_LEN * 3 + 4];
    int pos = 0;
    for (int i = 0; i < raw_len && pos < (int)sizeof(hex) - 4; i++) {
      pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", raw_buf[i]);
    }
    ESP_LOGD(TAG, "RAW [%d]: %s", raw_len, hex);
  }

  // Manchester decode: 2 raw bytes → 1 decoded byte
  uint8_t decoded[DECODED_BUF_LEN];
  memset(decoded, 0, sizeof(decoded));
  uint8_t decoded_len = manchester_decode_(raw_buf, raw_len, decoded);

  // Log Manchester-decoded hex
  {
    char hex[DECODED_BUF_LEN * 3 + 4];
    int pos = 0;
    for (int i = 0; i < decoded_len && pos < (int)sizeof(hex) - 4; i++) {
      pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", decoded[i]);
    }
    ESP_LOGD(TAG, "MAN [%d]: %s", decoded_len, hex);
  }

  // Also log the raw buffer AS-IS (without Manchester decode) to see if
  // the KNX frame is already decoded by CC1101
  // Check: does the raw buffer contain 0x44 0xFF at any reasonable position?
  int raw_knx_offset = -1;
  for (int i = 0; i + 10 < raw_len; i++) {
    if (raw_buf[i] == 0x44 && raw_buf[i + 1] == 0xFF) {
      raw_knx_offset = i - 1;  // -1 because byte before 0x44 is L-field
      break;
    }
  }

  // Search Manchester-decoded buffer for KNX header (0x44 0xFF)
  int man_knx_offset = -1;
  for (int i = 0; i + 10 < decoded_len; i++) {
    if (decoded[i] == 0x44 && decoded[i + 1] == 0xFF) {
      man_knx_offset = i - 1;  // -1 for L-field
      break;
    }
  }

  ESP_LOGD(TAG, "KNX header search: raw_offset=%d  manchester_offset=%d", raw_knx_offset, man_knx_offset);

  // Determine which buffer and offset to use
  const uint8_t *knx = nullptr;
  uint8_t knx_len = 0;
  bool used_manchester = false;

  if (man_knx_offset >= 0 && man_knx_offset + 10 < decoded_len) {
    knx = &decoded[man_knx_offset];
    knx_len = decoded_len - man_knx_offset;
    used_manchester = true;
  } else if (raw_knx_offset >= 0 && raw_knx_offset + 10 < raw_len) {
    // KNX frame found directly in raw buffer (no Manchester decode needed)
    knx = &raw_buf[raw_knx_offset];
    knx_len = raw_len - raw_knx_offset;
    used_manchester = false;
  } else {
    ESP_LOGD(TAG, "No KNX RF header (0x44 0xFF) found in packet");
    return;
  }

  // Validate header
  if (knx[1] != 0x44 || knx[2] != 0xFF) {
    ESP_LOGD(TAG, "KNX header validation failed");
    return;
  }

  packet_count_++;

  // Extract 6-byte serial (bytes 4..9 of KNX frame)
  std::string serial = extract_serial_(knx);

  // Battery: RF-Info byte 3, bit 6 (1=OK, 0=low)
  float battery_ok = (knx[3] & 0x40) ? 1.0f : 0.0f;

  ESP_LOGI(TAG, "KNX packet: serial=%s battery=%s via=%s offset=%d len=%d",
           serial.c_str(),
           battery_ok > 0 ? "OK" : "LOW",
           used_manchester ? "manchester" : "raw",
           used_manchester ? man_knx_offset : raw_knx_offset,
           knx_len);

  // Find matching thermostat
  ThermostatEntry *matched = nullptr;
  for (auto &t : thermostats_) {
    if (t.serial_hex == serial) {
      matched = &t;
      break;
    }
  }

  if (!matched) {
    unknown_serial_count_++;
    ESP_LOGI(TAG, "Unknown serial %s – add it to the YAML thermostats list", serial.c_str());
    return;
  }

  // Log full KNX frame for debugging temperature offsets
  {
    char hex[DECODED_BUF_LEN * 3 + 4];
    int pos = 0;
    for (int i = 0; i < knx_len && i < 40 && pos < (int)sizeof(hex) - 4; i++) {
      pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", knx[i]);
    }
    ESP_LOGD(TAG, "[%s] KNX frame [%d]: %s", matched->room_name.c_str(), knx_len, hex);
  }

  // Extract temperature data
  // Reference implementation uses offsets 20,21 for temp and 22,23 for setpoint
  // relative to the KNX frame start. These offsets span across block boundaries
  // (first block = 10 data + 2 CRC = 12 bytes, second block = 16 data + 2 CRC = 18 bytes).
  float temperature = NAN;
  float setpoint = NAN;

  if (knx_len >= 22) {
    uint16_t raw_temp = ((uint16_t)knx[20] << 8) | knx[21];
    if (raw_temp != DPT9_INVALID && raw_temp != 0x0000) {
      temperature = (float)transform_temperature_(raw_temp) / 100.0f;
    }
  }
  if (knx_len >= 24) {
    uint16_t raw_setp = ((uint16_t)knx[22] << 8) | knx[23];
    if (raw_setp != DPT9_INVALID && raw_setp != 0x0000) {
      setpoint = (float)transform_temperature_(raw_setp) / 100.0f;
    }
  }

  // Sanity bounds
  if (!std::isnan(temperature) && (temperature < -20.0f || temperature > 60.0f)) {
    ESP_LOGW(TAG, "[%s] Temperature %.2f out of range", matched->room_name.c_str(), temperature);
    temperature = NAN;
  }
  if (!std::isnan(setpoint) && (setpoint < 0.0f || setpoint > 45.0f)) {
    ESP_LOGW(TAG, "[%s] Setpoint %.2f out of range", matched->room_name.c_str(), setpoint);
    setpoint = NAN;
  }

  ESP_LOGI(TAG, "[%s] temp=%.2f°C  setpoint=%.2f°C  battery=%s",
           matched->room_name.c_str(),
           temperature, setpoint,
           battery_ok > 0 ? "OK" : "LOW");

  if (matched->temperature && !std::isnan(temperature))
    matched->temperature->publish_state(temperature);

  if (matched->setpoint && !std::isnan(setpoint))
    matched->setpoint->publish_state(setpoint);

  if (matched->battery)
    matched->battery->publish_state(battery_ok);
}

// ---------------------------------------------------------------------------
// extract_serial_()
// Bytes 4..9 of the KNX frame form the 6-byte individual address.
// Returned as uppercase 12-char hex, e.g. "007460AABBCC".
// ---------------------------------------------------------------------------
std::string UponorKnxRf::extract_serial_(const uint8_t *knx_frame) {
  char s[13];
  snprintf(s, sizeof(s), "%02X%02X%02X%02X%02X%02X",
           knx_frame[SERIAL_OFFSET + 0], knx_frame[SERIAL_OFFSET + 1],
           knx_frame[SERIAL_OFFSET + 2], knx_frame[SERIAL_OFFSET + 3],
           knx_frame[SERIAL_OFFSET + 4], knx_frame[SERIAL_OFFSET + 5]);
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

  int exp = (raw >> 11) & 0x0F;
  int mant = raw & 0x07FF;
  if (mant & 0x0400) mant -= 0x0800;

  return 0.01f * (float)mant * (float)(1 << exp);
}

// ---------------------------------------------------------------------------
// transform_temperature_()
// Temperature transform used by the reference implementation.
// Handles the exponent bit to produce a value in hundredths of degrees.
// ---------------------------------------------------------------------------
uint16_t UponorKnxRf::transform_temperature_(uint16_t data) {
  if (data & 0x800) {
    data = (data & 0x7FF) * 2;
  }
  return data;
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
