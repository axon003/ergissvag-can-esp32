# ErgissVAG CAN — ESP32 listen-only CAN tap for VW / Seat / Škoda / Audi

Open-source ESP32 firmware that taps the **CAN-Antrieb / CAN-Komfort** bus of VAG-group
cars (VW, Seat, Škoda, Audi — e.g. Touran, Seat Altea XL) in **listen-only** mode and
streams decoded vehicle data — wheel speeds, yaw rate, steering angle, brake, RPM — as
JSON over WiFi to a companion Android app.

> 📱 **Companion app: [ErgissVAG F16 on Google Play](https://play.google.com/store/apps/details?id=ro.ergiss.vag.f16)**
> — live VAG dashboard, gauges and trip telemetry. This firmware is the optional hardware
> module that feeds it the original CAN data the OBD port doesn't expose.

## Why
The OBD-II port on VAG cars exposes only a limited PID set through the J533 gateway.
Signals like individual **wheel speeds, yaw rate, lateral acceleration, steering angle and
original brake data** live on the internal CAN-Antrieb bus. This module is a **passive,
listen-only tap** — it never writes to the bus (safe, non-intrusive) — that decodes those
frames and serves them to the app over WiFi.

## Features
- ESP32 (native TWAI CAN driver) + SN65HVD230 transceiver — **listen-only, never transmits**
- Decodes CAN-Antrieb (500 kbit) / CAN-Komfort (100 kbit)
- WiFi AP + TCP JSON stream (or STA mode to join your own router)
- Optional GY-85 IMU (accel / gyro / magnetometer) + BMP280 barometer for motion & altitude
- Deep-sleep with wake-on-CAN — sleeps when the car is off
- 3D-printable enclosure (`case.scad` / `case.stl`)
- Two build layouts: Arduino IDE sketch (`ergissvag_can/`) and PlatformIO (`esp32/`)

## Hardware
- ESP32 DevKit v1 + SN65HVD230 CAN transceiver
- (optional) GY-85 IMU + BMP280

⚠️ **Safety:** listen-only tap, inline fuse, never connect TX. You are responsible for any
work on your vehicle's wiring.

## Quick start
1. Flash via PlatformIO (`pio run -t upload`) or Arduino IDE (open `ergissvag_can.ino`).
2. In `config.h` set your own `WIFI_AP_PASS` (STA fields are placeholders by default).
3. Power the module, connect the **ErgissVAG F16** app to the WiFi AP — data flows as JSON.

## License
**PolyForm Noncommercial License 1.0.0** — free for noncommercial use. See [`LICENSE.md`](LICENSE.md).
**Commercial use requires a separate license** — contact **office@ergiss.ro**.

Copyright © Gabriel Diaconu / ERGISS Media.

---
Built by **ERGISS Media** — [ergiss.ro](https://www.ergiss.ro) · [mensoft.ro](https://www.mensoft.ro)
Android: **[ErgissVAG F16 on Google Play](https://play.google.com/store/apps/details?id=ro.ergiss.vag.f16)**
