# Uponor T-45 KNX RF – ESPHome External Component

Modern ESPHome integration for **Uponor T-45** wireless KNX RF thermostats (also compatible with T-33, T-37, T-54, T-55, T-75).

**Requires ESPHome 2025.2 or newer** — uses the `external_components` API. The old `custom_components` mechanism was removed in ESPHome 2025.2.0.

---

## Hardware

| Item | Details |
|------|---------|
| Microcontroller | ESP32 WROOM (or any ESP32 variant) |
| RF Transceiver | CC1101 module |
| Protocol | KNX RF 1.1 |
| Frequency | **868.3 MHz** (EU) — change to 433.92 MHz for other regions |
| Modulation | 2-FSK, 32.73 kBaud, 47.6 kHz deviation |
| Thermostat | Uponor T-45 (serial prefix varies by model, e.g. `007402...`, `007460...`) |

### CC1101 → ESP32 WROOM Wiring

| CC1101 Pin | ESP32 WROOM Pin | Notes |
|------------|-----------------|-------|
| VCC        | 3.3 V           | ⚠️ 3.3 V only — 5 V will destroy the CC1101 |
| GND        | GND             | |
| SI (MOSI)  | GPIO 23         | |
| SO (MISO)  | GPIO 19         | |
| SCK        | GPIO 18         | |
| CSN (CS)   | GPIO 5          | |
| GDO0       | GPIO 2          | Packet-ready signal |
| GDO2       | Not connected   | |

If you use different pins, update `gdo0_pin`, `cs_pin`, etc. in `uponor_knx_rf.yaml`.

---

## Repository Structure

```
uponor_knx_rf_esphome/
├── uponor_knx_rf.yaml          ← ESPHome device config (edit this)
├── secrets.yaml.template       ← Credentials template
├── README.md
└── components/
    └── uponor_knx_rf/
        ├── __init__.py         ← ESPHome component schema (Python)
        ├── uponor_knx_rf.h     ← C++ component header
        ├── uponor_knx_rf.cpp   ← C++ implementation
        └── Crc16.h             ← KNX CRC-16/EN-13757 helper (poly 0x3D65)
```

---

## Setup

### 1. Fork or clone this repository

Create your own GitHub repo containing these files (or fork this one). The `components/` folder **must be at the repo root**.

### 2. Edit `uponor_knx_rf.yaml`

Replace `YOUR_GITHUB_USERNAME` in the `external_components` block:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/YOUR_GITHUB_USERNAME/uponor_knx_rf_esphome
      ref: main
    components: [uponor_knx_rf]
```

### 3. Set up `secrets.yaml`

Copy `secrets.yaml.template` to `/config/esphome/secrets.yaml` (or add the keys to your existing file):

```yaml
wifi_ssid: "YourNetwork"
wifi_password: "YourPassword"
ap_password: "FallbackPassword"
ha_api_key: "base64-32-byte-key"   # openssl rand -base64 32
ota_password: "YourOtaPassword"
```

### 4. Flash with DEBUG logging

Keep `logger: level: DEBUG` for the first flash. This lets you see incoming packets.

### 5. Discover your thermostat serial numbers

1. After flashing, open the ESPHome log viewer in Home Assistant
2. **Press ▲ or ▼** on each T-45 thermostat (one at a time)
3. Watch the log for:
   ```
   [I][uponor_knx_rf]: Unknown serial 00740260227A – add it to the YAML thermostats list
   ```
4. Note which serial belongs to which room

Serial number prefixes vary by model (e.g. `007402...`, `007460...`, `007440...`, `007400...`). The component works with all Uponor KNX RF models.

### 6. Update the YAML with real serials

In `uponor_knx_rf.yaml`, replace the placeholder serials:

```yaml
thermostats:
  - serial: "007460AABBCC"   # ← your real serial
    name: "Living Room"
    temperature:
      name: "Living Room Temperature"
    setpoint:
      name: "Living Room Setpoint"
    battery:
      name: "Living Room Battery"
```

### 7. Reduce log level and reflash

```yaml
logger:
  level: INFO
```

---

## Home Assistant Entities

For each thermostat, three sensors are created and automatically discovered by Home Assistant:

| Sensor | Unit | Description |
|--------|------|-------------|
| `sensor.<room>_temperature` | °C | Current room temperature |
| `sensor.<room>_setpoint` | °C | Target setpoint temperature |
| `sensor.<room>_battery` | — | `1.0` = battery OK, `0.0` = battery low |

Plus device diagnostics:
- WiFi signal strength
- Device uptime
- IP address
- Connection status

---

## Understanding the KNX RF Packet

The CC1101 receives Manchester-encoded data at 32.73 kBaud (2-FSK). Each pair of raw bytes decodes to one data byte (bit pairs: `01`=1, `10`=0). After Manchester decoding, the KNX RF 1.1 frame is structured as follows:

### Frame structure

```
Byte    Field           Value / Notes
------  --------------  ------------------------------------------------
0       L-field         Telegram length (all bytes after this one)
1       Control         0x44  (fixed, KNX RF 1.1)
2       ESC             0xFF  (fixed)
3       RF-Info         Bit 1 (0x02) = battery (1=OK, 0=Low)
                        Bit 0 (0x01) = unidirectional device
4–9     Serial          6-byte individual address  ← this is your "serial"
10–11   Block 1 CRC     CRC-16/EN-13757 over bytes 0–9
12–15   Address info    Source / destination KNX addresses
16      Datapoint       Destination address low byte:
                          0x01 = current temperature
                          0x02 = setpoint temperature
                          0x03 = status / operating mode
17      —               Reserved
18      APCI            Data type indicator
19      —               Padding (0x80)
20–21   Data value      2-byte temperature (dp 1 & 2) or 1-byte status (dp 3)
22–23   Block 2 CRC     CRC-16/EN-13757 over bytes 12–21
```

### Important: separate telegrams

Temperature and setpoint are transmitted as **separate** KNX RF telegrams. The datapoint byte (offset 16) tells you which value is in the data field at bytes 20–21. You will never see both temperature and setpoint in the same frame.

### Temperature encoding

Uponor thermostats use a **simplified** encoding for the 2-byte temperature value (not standard DPT 9.001):

```
If bit 11 is set:  value = ((raw & 0x7FF) × 2) / 100.0  °C
If bit 11 is clear: value = raw / 100.0  °C
```

Examples from a real T-45:
- `0x0C0B` → bit 11 set → (0x40B × 2) / 100 = **20.70 °C** (temperature)
- `0x0C1A` → bit 11 set → (0x41A × 2) / 100 = **21.00 °C** (setpoint)

### CRC

Each frame uses block-based CRC-16/EN-13757 (polynomial `0x3D65`, init `0x0000`, XOR-out `0xFFFF`):
- **Block 1:** 10 data bytes (offsets 0–9) + 2 CRC bytes (offsets 10–11)
- **Block 2:** up to 16 data bytes + 2 CRC bytes

---

## Changing the RF Frequency

KNX RF 1.1 uses **868.3 MHz** in Europe. For regions using a different frequency, edit `uponor_knx_rf.cpp`:

```cpp
// Find this line in setup():
ELECHOUSE_cc1101.setMHZ(868.3);
// Change to your region's frequency:
ELECHOUSE_cc1101.setMHZ(433.92);
```

> **Note:** All other radio parameters (2-FSK modulation, 32.73 kBaud data rate, 47.6 kHz deviation, sync word 0x7696) are defined by the KNX RF 1.1 standard and should not be changed.

---

## Stack Size Warning (ESP32)

On some ESP32 boards the default Arduino loop task stack is too small and may cause random crashes. If you experience crashes, increase the stack size:

**File to edit:**
```
.platformio/packages/framework-arduinoespressif32/cores/esp32/main.cpp
```

**Find:**
```cpp
xTaskCreateUniversal(loopTask, "loopTask", ARDUINO_LOOP_STACK_SIZE, ...
```

**Change to:**
```cpp
xTaskCreateUniversal(loopTask, "loopTask", 32768, ...
```

See [ESPHome issue #855](https://github.com/esphome/issues/issues/855) for background.

---

## Troubleshooting

**CC1101 not found (setup fails):**
- Verify 3.3 V power — 5 V will break the module
- Check all SPI connections, especially MISO/MOSI aren't swapped
- Try a shorter wire to GDO0

**No packets received:**
- Confirm 868.3 MHz is correct for your region
- Trigger a transmission by pressing ▲ or ▼ on the thermostat
- Check the CC1101 antenna is present/connected
- Verify the CC1101 MARCSTATE is 0x0D (RX mode) in the status heartbeat log

**Many noise packets (no KNX header found):**
- This is normal — the CC1101 receives any 2-FSK signal on 868.3 MHz
- Only packets containing the KNX header (0x44 0xFF) are processed
- About 50% of received packets may be noise depending on your RF environment

**Temperature shows NaN:**
- The thermostat transmitted an "invalid" marker — normal on startup
- Will resolve on the next regular transmission (every ~10–30 min)
- Check the log for the raw data value to diagnose encoding issues

**Sensors not appearing in Home Assistant:**
- Verify the `ha_api_key` in `secrets.yaml` matches your HA configuration
- Check the ESP32 is reachable on the network

---

## Credits

Based on original work by:
- [hyotynen/uponor_knx_rf_thermostat](https://github.com/hyotynen/uponor_knx_rf_thermostat)
- [c-mholm/uponor_knx_rf_thermostat](https://github.com/c-mholm/uponor_knx_rf_thermostat)

Key reference implementations for CC1101 register config, Manchester decoding, battery bit, and temperature encoding:
- [tahvane1/uponor_knx_rf_thermostat](https://github.com/tahvane1/uponor_knx_rf_thermostat) — ESPHome implementation for Uponor T-75
- [thelsing/knx](https://github.com/thelsing/knx) — Reference KNX stack (CC1101 register values, RF data link layer)
- [aviborg/MonitorKNXRF](https://github.com/aviborg/MonitorKNXRF) — Raspberry Pi KNX RF monitor

KNX RF packet structure documented by:
- [jsunsch/t55_t75_knx_rf_decoder](https://github.com/jsunsch/t55_t75_knx_rf_decoder)

CC1101 driver:
- [LSatan/SmartRC-CC1101-Driver-Lib](https://github.com/LSatan/SmartRC-CC1101-Driver-Lib)
