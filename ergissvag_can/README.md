# ergissvag_can — Arduino IDE sketch

Arduino IDE build (folder name = sketch name). Functionally equivalent to `esp32/` (PlatformIO).

## Arduino IDE setup

### 1. ESP32 board

`File → Preferences → Additional Boards Manager URLs`:
```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```
Then `Tools → Board → Boards Manager` → search **"esp32"** by Espressif → install the latest (3.x+).

Select board: **Tools → Board → ESP32 Arduino → ESP32 Dev Module**
Upload speed: 921600
Flash size: 4MB
Partition scheme: Default 4MB with spiffs

### 2. Libraries (Sketch → Include Library → Manage Libraries)

| Library | Author | Note |
|---|---|---|
| **Adafruit BMP280 Library** | Adafruit | pressure/temperature sensor |
| **Adafruit ADXL345** | Adafruit | GY-85 accelerometer |
| **Adafruit HMC5883 Unified** | Adafruit | GY-85 magnetometer |
| **Adafruit Unified Sensor** | Adafruit | common dependency (BusIO + Sensor base) |

> ITG3200 (gyroscope) is read raw over I2C in `sensors.cpp` — no library needed.

> `ESPmDNS`, `WiFi`, `driver/twai.h` ship with the ESP32 core — no separate install.

### 3. Open + Upload

`File → Open` → select `ergissvag_can.ino`. The Arduino IDE automatically includes
`config.h`, `sensors.h`, `sensors.cpp` (same folder).

Upload (Ctrl+U) with the ESP32 on USB. On the DevKit V1 you don't need to hold anything —
auto-reset works through the USB-Serial chip.

## Hardware

| Connection | ESP32 pin |
|---|---|
| BMP280 + GY-85 (I2C SDA) | **GPIO 21** |
| BMP280 + GY-85 (I2C SCL) | **GPIO 22** |
| SN65HVD230 CTX (CAN TX) | **GPIO 17** (TX2) |
| SN65HVD230 CRX (CAN RX) | **GPIO 16** (RX2) |
| Module power | **3V3** + **GND** |

## Network access

- mDNS hostname: **`ergisscan.local`**
- TCP JSON server: port **31416**
- WiFi STA: `YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD` (set in `config.h`)
- Fallback AP if STA fails: `ergCAN_ESP32` / `ergcan2026`

Test: `nc ergisscan.local 31416` (or `ncat`, `telnet`, `socat`)

## Quick config changes

Edit **`config.h`**:
- `CAN_BITRATE` — 500000 (Antrieb) / 100000 (Komfort)
- `WIFI_STA_SSID` / `WIFI_STA_PASS` — set to your own router
- `SENSOR_SAMPLE_MS` — 100 ms default (10 Hz); 50 ms = 20 Hz; 200 ms = 5 Hz
- `MDNS_HOSTNAME` — change the `.local` name

## See also

- `esp32/` — PlatformIO version (CLI build)
- [ErgissVAG F16 on Google Play](https://play.google.com/store/apps/details?id=ro.ergiss.vag.f16) — companion app
