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
| Thermostat | Uponor T-45 (serial prefix `00746...`) |

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
        └── Crc16.h             ← KNX CRC-16/CCITT helper
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
   [I][uponor_knx_rf]: Unknown serial 007460AABBCC – add it to YAML
   ```
4. Note which serial belongs to which room

The T-45 serial numbers always start with `00746`. If you see a different prefix, that's a different Uponor model — the component will still work, just update your notes.

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

The T-45 transmits a KNX RF 1.1 packet structured as follows:

```
Byte  Field       Value / Notes
----  ----------  ------------------------------------------------
0     L-field     Telegram length (all bytes after this one)
1     Control     0x44  (fixed, KNX RF 1.1 Fast Ack frame)
2     Fixed       0xFF
3     RF-Info     Bit 6 = battery (1=OK, 0=Low)
4–9   Serial      6-byte individual address  ← this is your "serial"
10    Filler      0x60
11    Data start  DPT 9.001 actual temp (2 bytes)
13                DPT 9.001 setpoint temp (2 bytes)
...
n-2   CRC high    CRC-16/CCITT
n-1   CRC low
```

**DPT 9.001 decoding:** `value = 0.01 × M × 2^E`  
Where E = bits [14:11], M = 11-bit two's-complement mantissa at bits [10:0].

---

## Changing the RF Frequency

For regions using 433 MHz instead of 868.3 MHz, edit `uponor_knx_rf.cpp`:

```cpp
// Find this line in setup():
ELECHOUSE_cc1101.setMHZ(868.3);
// Change to:
ELECHOUSE_cc1101.setMHZ(433.92);
```

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
- Trigger a transmission by adjusting the thermostat temperature
- Check the CC1101 antenna is present/connected

**All packets show CRC errors:**
- Usually means the CC1101 is receiving noise, not real signals
- Reduce interference (move away from WiFi antenna, check GDO0 length)

**Temperature shows NaN:**
- The thermostat transmitted an "invalid" marker — normal on startup
- Will resolve on the next regular transmission (every ~10–30 min)

**Sensors not appearing in Home Assistant:**
- Verify the `ha_api_key` in `secrets.yaml` matches your HA configuration
- Check the ESP32 is reachable on the network

---

## Credits

Based on original work by:
- [hyotynen/uponor_knx_rf_thermostat](https://github.com/hyotynen/uponor_knx_rf_thermostat)
- [c-mholm/uponor_knx_rf_thermostat](https://github.com/c-mholm/uponor_knx_rf_thermostat)

KNX RF packet structure documented by:
- [jsunsch/t55_t75_knx_rf_decoder](https://github.com/jsunsch/t55_t75_knx_rf_decoder)

CC1101 driver:
- [LSatan/SmartRC-CC1101-Driver-Lib](https://github.com/LSatan/SmartRC-CC1101-Driver-Lib)
