# ergissvag_can — Arduino IDE sketch

Versiunea pentru Arduino IDE (folder = nume sketch). Echivalent functional cu `esp32/` (PlatformIO).

## Setup Arduino IDE

### 1. Board ESP32

`File → Preferences → Additional Boards Manager URLs`:
```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```
Apoi `Tools → Board → Boards Manager` → cauta **"esp32"** by Espressif → instaleaza ultima versiune (3.x+).

Selecteaza board: **Tools → Board → ESP32 Arduino → ESP32 Dev Module**
Upload speed: 921600
Flash size: 4MB
Partition scheme: Default 4MB with spiffs

### 2. Librarii (Sketch → Include Library → Manage Libraries)

| Librarie | Autor | Note |
|---|---|---|
| **Adafruit BMP280 Library** | Adafruit | senzor presiune/temperatura |
| **Adafruit ADXL345** | Adafruit | accelerometru GY-85 |
| **Adafruit HMC5883 Unified** | Adafruit | magnetometru GY-85 |
| **Adafruit Unified Sensor** | Adafruit | dependinta comuna (BusIO si Sensor base) |

> ITG3200 (giroscop) e citit raw I2C in `sensors.cpp` — nu ai nevoie de librarie.

> `ESPmDNS`, `WiFi`, `driver/twai.h` vin in core-ul ESP32, nu trebuie instalate separat.

### 3. Open + Upload

`File → Open` → selecteaza `ergissvag_can.ino`. Arduino IDE va include automat `config.h`, `sensors.h`, `sensors.cpp` (sunt in acelasi folder).

Upload (Ctrl+U) cu ESP32 conectat pe USB. Pe DevKit V1 nu trebuie sa tii nimic apasat — auto-reset functioneaza din USB-Serial chip.

## Hardware

| Conexiune | Pin ESP32 |
|---|---|
| BMP280 + GY-85 (I2C SDA) | **GPIO 21** |
| BMP280 + GY-85 (I2C SCL) | **GPIO 22** |
| SN65HVD230 CTX (CAN TX) | **GPIO 17** (TX2) |
| SN65HVD230 CRX (CAN RX) | **GPIO 16** (RX2) |
| Alimentare module | **3V3** + **GND** |

## Acces in retea

- Hostname mDNS: **`ergisscan.local`**
- TCP server JSON: port **31416**
- WiFi STA: `YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD` (hardcoded in `config.h`)
- Fallback AP daca STA esueaza: `ergCAN_ESP32` / `ergcan2026`

Test: `nc ergisscan.local 31416` (sau `ncat`, `telnet`, `socat`)

## Schimbari config rapide

Editezi **`config.h`**:
- `CAN_BITRATE` — 500000 (Antrieb) / 100000 (Komfort)
- `WIFI_STA_SSID` / `WIFI_STA_PASS` — daca schimbi router
- `SENSOR_SAMPLE_MS` — 100ms default (10Hz); 50ms = 20Hz; 200ms = 5Hz
- `MDNS_HOSTNAME` — daca vrei alt nume .local

## Vezi si

- `esp32/` — versiune PlatformIO (CLI build)
