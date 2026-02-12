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
  ELECHOUSE_cc1101.setCCMode(1);
  ELECHOUSE_cc1101.setModulation(0);          // 0 = 2-FSK
  ELECHOUSE_cc1101.setMHZ(868.3);
  ELECHOUSE_cc1101.setDeviation(47.607422);
  ELECHOUSE_cc1101.setChannel(0);
  ELECHOUSE_cc1101.setChsp(199.951172);
  ELECHOUSE_cc1101.setRxBW(270.833333);
  ELECHOUSE_cc1101.setDRate(32.7301);
  ELECHOUSE_cc1101.setPA(5);
  ELECHOUSE_cc1101.setSyncMode(5);            // 15/16 sync + carrier sense
  ELECHOUSE_cc1101.setSyncWord(0x76, 0x96);   // KNX RF RX sync word
  ELECHOUSE_cc1101.setPacketLength(61);       // Max packet length

  // Direct register writes for optimal KNX RF reception
  // Note: setCCMode(1) sets PKTCTRL0=0x05 (variable length + CRC enabled).
  // The reference implementation uses the same setting and it works.
  // CC1101 CRC auto-flush is disabled by default (PKTCTRL1 bit 3 = 0),
  // so bad-CRC packets still stay in the FIFO — they just fail CheckCRC().
  ELECHOUSE_cc1101.SpiWriteReg(0x03, 0x40);  // FIFOTHR: RX FIFO threshold
  ELECHOUSE_cc1101.SpiWriteReg(0x0B, 0x08);  // FSCTRL1: IF frequency
  ELECHOUSE_cc1101.SpiWriteReg(0x0C, 0x00);  // FSCTRL0: frequency offset
  ELECHOUSE_cc1101.SpiWriteReg(0x13, 0x22);  // MDMCFG1: preamble + channel spacing exp
  ELECHOUSE_cc1101.SpiWriteReg(0x15, 0x47);  // DEVIATN: deviation
  ELECHOUSE_cc1101.SpiWriteReg(0x17, 0x30);  // MCSM1: CCA mode, after RX/TX state
  ELECHOUSE_cc1101.SpiWriteReg(0x19, 0x2E);  // FOCCFG: frequency offset compensation
  ELECHOUSE_cc1101.SpiWriteReg(0x1A, 0x6D);  // BSCFG: bit synchronization
  ELECHOUSE_cc1101.SpiWriteReg(0x1B, 0x43);  // AGCCTRL2: AGC max gain
  ELECHOUSE_cc1101.SpiWriteReg(0x1C, 0x40);  // AGCCTRL1: AGC LNA priority
  ELECHOUSE_cc1101.SpiWriteReg(0x1D, 0x91);  // AGCCTRL0: AGC filter samples
  ELECHOUSE_cc1101.SpiWriteReg(0x21, 0xB6);  // FREND1: front end RX
  ELECHOUSE_cc1101.SpiWriteReg(0x20, 0xFB);  // WORCTRL: WOR control
  ELECHOUSE_cc1101.SpiWriteReg(0x23, 0xEF);  // FSCAL3: freq synth cal
  ELECHOUSE_cc1101.SpiWriteReg(0x24, 0x2E);  // FSCAL2: freq synth cal
  ELECHOUSE_cc1101.SpiWriteReg(0x25, 0x19);  // FSCAL1: freq synth cal

  ELECHOUSE_cc1101.SetRx();

  cc1101_ok_ = true;
  ESP_LOGCONFIG(TAG, "CC1101 listening on 868.3 MHz (KNX RF 1.1, 2-FSK, 32.73 kBaud)");

  // Dump key registers for debugging
  ESP_LOGI(TAG, "CC1101 registers: MDMCFG4=0x%02X MDMCFG3=0x%02X MDMCFG2=0x%02X MDMCFG1=0x%02X",
           ELECHOUSE_cc1101.SpiReadReg(0x10), ELECHOUSE_cc1101.SpiReadReg(0x11),
           ELECHOUSE_cc1101.SpiReadReg(0x12), ELECHOUSE_cc1101.SpiReadReg(0x13));
  ESP_LOGI(TAG, "CC1101 registers: SYNC1=0x%02X SYNC0=0x%02X PKTCTRL0=0x%02X PKTCTRL1=0x%02X PKTLEN=0x%02X",
           ELECHOUSE_cc1101.SpiReadReg(0x04), ELECHOUSE_cc1101.SpiReadReg(0x05),
           ELECHOUSE_cc1101.SpiReadReg(0x08), ELECHOUSE_cc1101.SpiReadReg(0x07),
           ELECHOUSE_cc1101.SpiReadReg(0x06));
  ESP_LOGI(TAG, "CC1101 registers: DEVIATN=0x%02X FREND1=0x%02X AGCCTRL2=0x%02X IOCFG0=0x%02X",
           ELECHOUSE_cc1101.SpiReadReg(0x15), ELECHOUSE_cc1101.SpiReadReg(0x21),
           ELECHOUSE_cc1101.SpiReadReg(0x1B), ELECHOUSE_cc1101.SpiReadReg(0x02));
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

  // Note: Do NOT call CheckCRC() before ReceiveData — it flushes the FIFO on
  // CRC failure, and KNX RF Manchester-encoded packets may not pass CC1101 CRC.
  if (ELECHOUSE_cc1101.CheckRxFifo(100)) {
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
// Full pipeline: raw CC1101 → Manchester decode → KNX frame validation →
// serial extraction → temperature decode → publish to HA
// ---------------------------------------------------------------------------
void UponorKnxRf::receive_and_process_() {
  uint8_t raw_buf[RAW_BUF_LEN];
  memset(raw_buf, 0, sizeof(raw_buf));

  int raw_len = ELECHOUSE_cc1101.ReceiveData(raw_buf);

  ELECHOUSE_cc1101.SetRx();  // Re-arm receiver immediately

  if (raw_len < RAW_MIN_LEN) {
    ESP_LOGD(TAG, "Ignored short packet (%d raw bytes, need >= %d)", raw_len, RAW_MIN_LEN);
    return;
  }

  // Log raw hex at VERBOSE level
  if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
    char hex[RAW_BUF_LEN * 3 + 4];
    int pos = 0;
    for (int i = 0; i < raw_len && pos < (int)sizeof(hex) - 4; i++) {
      pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", raw_buf[i]);
    }
    ESP_LOGV(TAG, "RAW RX [%d bytes]: %s", raw_len, hex);
  }

  // Note: KNX RF uses its own block-based CRC-16, not standard CC1101 CRC.
  // We skip hardware CRC checking and validate at the application layer.

  // Manchester decode: 2 raw bytes → 1 decoded byte
  uint8_t decoded[DECODED_BUF_LEN];
  memset(decoded, 0, sizeof(decoded));
  uint8_t decoded_len = manchester_decode_(raw_buf, raw_len, decoded);

  // Log decoded hex at DEBUG level
  if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG) {
    char hex[DECODED_BUF_LEN * 3 + 4];
    int pos = 0;
    for (int i = 0; i < decoded_len && pos < (int)sizeof(hex) - 4; i++) {
      pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", decoded[i]);
    }
    ESP_LOGD(TAG, "Decoded [%d bytes]: %s", decoded_len, hex);
  }

  // KNX frame starts at KNX_OFFSET in the decoded buffer
  if (decoded_len < KNX_OFFSET + 10) {
    ESP_LOGD(TAG, "Decoded packet too short (%d bytes, need >= %d)", decoded_len, KNX_OFFSET + 10);
    return;
  }

  // Pointer to KNX frame
  const uint8_t *knx = &decoded[KNX_OFFSET];
  uint8_t knx_len = decoded_len - KNX_OFFSET;

  // Validate KNX RF 1.1 header bytes
  if (knx[1] != 0x44 || knx[2] != 0xFF) {
    ESP_LOGD(TAG, "Not a KNX RF 1.1 frame (byte[1]=0x%02X, byte[2]=0x%02X)", knx[1], knx[2]);
    return;
  }

  packet_count_++;

  // Extract 6-byte serial (bytes 4..9 of KNX frame)
  std::string serial = extract_serial_(knx);
  ESP_LOGD(TAG, "Packet serial: %s", serial.c_str());

  // Battery: RF-Info byte 3, bit 6 (1=OK, 0=low)
  float battery_ok = (knx[3] & 0x40) ? 1.0f : 0.0f;

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
    ESP_LOGI(TAG, "Unknown serial %s – if this is your thermostat, add it to the YAML", serial.c_str());
    return;
  }

  // Extract temperature data from the second KNX block
  // First block is 12 bytes (10 data + 2 CRC), starts at knx[0]
  // Second block starts at knx[12] (after first block CRC)
  // In the second block, temperature data location depends on the
  // KNX group addresses. For Uponor T-45 thermostats, the reference
  // implementation reads temperature at offsets 20 and 21 from KNX start.
  // That maps to: knx[20] and knx[21] for current temp.
  //
  // We try multiple known temperature positions to be robust:
  // - Position 20,21 = current temperature (from reference impl)
  // - Position 22,23 = setpoint temperature (from reference impl)
  float temperature = NAN;
  float setpoint = NAN;

  if (knx_len >= 24) {
    // Current temperature at offset 20-21
    uint16_t raw_temp = ((uint16_t)knx[20] << 8) | knx[21];
    if (raw_temp != DPT9_INVALID && raw_temp != 0x0000) {
      temperature = (float)transform_temperature_(raw_temp) / 100.0f;
    }

    // Setpoint at offset 22-23
    if (knx_len >= 26) {
      uint16_t raw_setp = ((uint16_t)knx[22] << 8) | knx[23];
      if (raw_setp != DPT9_INVALID && raw_setp != 0x0000) {
        setpoint = (float)transform_temperature_(raw_setp) / 100.0f;
      }
    }
  }

  // Sanity bounds
  if (!std::isnan(temperature) && (temperature < -20.0f || temperature > 60.0f)) {
    ESP_LOGW(TAG, "[%s] Temperature %.2f out of range, discarding", matched->room_name.c_str(), temperature);
    temperature = NAN;
  }
  if (!std::isnan(setpoint) && (setpoint < 0.0f || setpoint > 45.0f)) {
    ESP_LOGW(TAG, "[%s] Setpoint %.2f out of range, discarding", matched->room_name.c_str(), setpoint);
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
