# ergissvag_can — ESP32 J533 CAN tap (listen-only)

ESP32 DevKit v1 + SN65HVD230 firmware that sniffs the **CAN-Antrieb** or **CAN-Komfort**
bus (VW PQ35 — Seat Altea XL 2011 / Touran CBZB, J533 gateway) and streams decoded frames
as JSON over TCP to a companion Android app
([**ErgissVAG F16**](https://play.google.com/store/apps/details?id=ro.ergiss.vag.f16)).

- Author: Gabriel Diaconu (ERGISS Media)
- Version: 1.0 (2026-04-22)
- Framework: PlatformIO + Arduino-ESP32
- CAN driver: native ESP32 TWAI (`driver/twai.h`)

## Layout

```
esp32/
  platformio.ini      # board esp32dev + arduino framework
  src/
    main.cpp          # firmware (TWAI + WiFi AP + TCP server + deep sleep)
    config.h          # constants (bitrate, SSID, TCP port, pins, timing)
  README.md
```

## Flash

### PlatformIO (recommended)

```bash
cd esp32
pio run                              # compile
pio run -t upload                    # flash via USB (auto-detect port)
pio device monitor -b 115200         # serial log
```

### Arduino IDE

1. Board Manager -> install `esp32` by Espressif Systems
2. Board: `ESP32 Dev Module`, Upload speed 921600, Flash 4MB, Partition `Default`
3. Copy `src/main.cpp` as `main.ino` into a `main/` folder, put `config.h` next to it
4. Tools -> Port, Upload, then Serial Monitor 115200

## Config

Edit `src/config.h`:

| Define               | Default              | Note                                       |
|----------------------|----------------------|--------------------------------------------|
| `CAN_BITRATE`        | 500000               | 500k = Antrieb, 100k = Komfort             |
| `WIFI_AP_SSID`       | `ergCAN_ESP32`       |                                            |
| `WIFI_AP_PASS`       | `ergcan2026`         | WPA2, min 8 chars — **change this**        |
| `TCP_PORT`           | 31416                |                                            |
| `DEEP_SLEEP_IDLE_S`  | 30                   | seconds with no client -> sleep            |
| `HB_INTERVAL_MS`     | 5000                 | heartbeat JSON when no frames              |

After editing: `pio run -t upload`.

## Wiring

### ESP32 DevKit v1 <-> SN65HVD230

```
   ESP32 DevKit v1                 SN65HVD230 (listen-only)
   +---------------+                +------------------+
   | GPIO21 (CTX)  |--------------->| D  (TXD)         |   not electrically driving the bus
   |               |                |                  |   (listen-only), but required to init TWAI
   | GPIO22 (CRX)  |<---------------| R  (RXD)         |
   | 3V3           |--------------->| VCC (3V3)        |
   | GND           |--------------->| GND              |
   |               |                | RS   ----- GND   |   <-- REQUIRED: high-speed mode
   |               |                | CANH ----------- | ---> J533 pin 6 (CAN-Antrieb HIGH)
   |               |                | CANL ----------- | ---> J533 pin 16 (CAN-Antrieb LOW)
   +---------------+                +------------------+
```

Notes:
- **RS to GND**: forces the SN65HVD230 into high-speed mode. For passive listen-only the
  driver simply never transmits (`TWAI_MODE_LISTEN_ONLY`). RS to 3V3 would enable
  slope-control (not wanted here).
- **Do NOT** add a 120R termination resistor — the bus is already terminated in the car.
- Keep CAN-H/L a **twisted pair**, 10-30 cm, not run parallel to the 12V supply.
- Power: the ESP32 on-board 3V3 regulator from 5V USB, or a 5V buck from OBD. For a
  permanent install use a 12V->5V step-down with quiescent current <1 mA.

### J533 gateway pinout — PQ35 (Altea XL / Touran / Leon 1P)

Connector T20a (green, 20-pin):

| Pin | Signal              | Rate     |
|-----|---------------------|----------|
| 6   | CAN-Antrieb HIGH    | 500 kbps |
| 16  | CAN-Antrieb LOW     | 500 kbps |

Connector T20b carries CAN-Komfort HIGH/LOW (100 kbps) and CAN-Infotainment (100 kbps).
J533 pinouts for the PQ35 platform are widely documented (see VW SSP 269).

## TCP protocol

- Port **31416**, WPA2 SSID **ergCAN_ESP32**, ESP32 IP = **192.168.4.1**
- One client at a time (a second connection is rejected)
- Banner on connect:
  ```json
  {"hello":"ergCAN","bitrate":500000,"port":31416}
  ```
- CAN frame (one line, `\n`-delimited):
  ```json
  {"ts":1234,"id":"0x470","ext":0,"rtr":0,"len":8,"data":"01 00 00 00 00 00 00 00"}
  ```
  - `ts`   = millis() since ESP32 boot
  - `id`   = hex CAN ID (11-bit or 29-bit depending on `ext`)
  - `ext`  = 0 standard, 1 extended
  - `rtr`  = 0 data, 1 remote transmission request
  - `len`  = DLC (0..8)
  - `data` = hex bytes, space-separated
- Heartbeat every 5 s when no frames arrive:
  ```json
  {"hb":1,"uptime":42,"rx_total":0,"rx_err":0}
  ```

## Android client pattern

```kotlin
// 1. Phone joins SSID "ergCAN_ESP32" (WPA2 "ergcan2026")
// 2. Socket to 192.168.4.1:31416
val sock = Socket()
sock.connect(InetSocketAddress("192.168.4.1", 31416), 3000)
val br = BufferedReader(InputStreamReader(sock.getInputStream()))
while (isActive) {
    val line = br.readLine() ?: break
    val obj = JSONObject(line)
    if (obj.optInt("hb") == 1) continue          // heartbeat
    val id   = obj.getString("id")               // "0x470"
    val len  = obj.getInt("len")
    val data = obj.getString("data")             // "01 00 ..."
    // decode...
}
```

The [ErgissVAG F16](https://play.google.com/store/apps/details?id=ro.ergiss.vag.f16) app
consumes this stream as one of its vehicle-data sources.

## Troubleshooting

| Symptom                         | Likely cause / fix                                        |
|---------------------------------|-----------------------------------------------------------|
| `rx=0` on Serial, no frames     | Check RS=GND (not VCC!), check bitrate (500k/100k)        |
| `state=BUS_OFF` in log          | Wrong bitrate or CAN_H/L swapped                          |
| Continuous reboot               | TWAI driver crash — check GPIO21/22 are connected         |
| Android can't see the SSID      | ESP32 in deep sleep — turn ignition on for wake-on-CAN    |
| Connect accepted then dropped   | Another client already connected (1-client limit)         |
| `rx_err` rising fast            | Bad termination — do **NOT** add 120R, the car has it     |
| AP shows but won't connect      | Password min 8 chars; reset ESP32 after changing config   |

## Deep sleep & wake-on-CAN

After 30 s with no TCP client, the ESP32 enters deep sleep (~10 uA).
Wake source: GPIO22 (CAN RX) going HIGH — i.e. any bus activity after idle.

On PQ35:
- Ignition OFF, all buses asleep -> ESP32 sleeps
- Ignition ON or remote unlock -> CAN-Komfort/Antrieb wakes -> ESP32 wakes (a few ms)
- After wake the ESP32 restarts the AP + TCP server + TWAI

Average draw: ~80 mA @ 3V3 when active (WiFi AP), ~10 uA in deep sleep.

## References

- VW Self-Study Programme SSP 269 — CAN Data Bus (Antrieb + Komfort, PQ35)
- [ErgissVAG F16 on Google Play](https://play.google.com/store/apps/details?id=ro.ergiss.vag.f16)
