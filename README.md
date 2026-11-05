# ErgissVAG CAN — ESP32 OBD-II/UDS module for VW / Seat / Škoda / Audi

Open-source ESP32 firmware (current: **2.6.0**) that plugs into a car's **OBD-II diagnostic
port** and talks to it as an OBD-II/UDS diagnostic client, streaming the data as JSON over
WiFi to a companion Android app.

> 📱 **Companion app: [ErgissVAG F16 on Google Play](https://play.google.com/store/apps/details?id=ro.ergiss.vag.f16)**
> — live VAG dashboard, gauges and trip telemetry. This firmware is the optional hardware
> module that feeds it data beyond what a generic ELM327/Bluetooth adapter gives you.

## What it does
The OBD-II port already answers standard PID requests (SAE J1979 mode 01) — this module
reads those (RPM, speed, coolant/intake temperature, throttle, load, timing advance,
short/long fuel trim, O2/lambda, MAF, fuel level/rate/pressure, EGR, hybrid battery, torque,
mileage counters and more — **45 named parameters** out of the box). It also checks which
PIDs the ECU actually supports before asking for them, instead of guessing.

On top of that, for VW/Seat/Škoda/Audi (PQ35 platform built in, more via downloadable
vehicle profiles) it goes further over **UDS**: real odometer, fuel level, engine oil
temperature/level and generator load/performance — values that aren't in the standard PID
set — plus reading and clearing DTCs across multiple control units (not just the engine),
and reading a module's memory for diagnostics (security-access gated, read-only).

## Features
- ESP32 (native TWAI CAN driver) + SN65HVD230 transceiver, wired to the **OBD-II connector**
  (pin 6 CAN-H / pin 14 CAN-L / pin 5 signal ground) — no splicing into the car's wiring
- **Parameters requested by name** (`sub {"n":"rpm"}`) — the board resolves the name itself,
  in order: vehicle profile → vehicle-specific values (odometer/fuel/generator/oil) →
  standard OBD-II catalog. The client no longer needs to know PIDs or manufacturer DIDs;
  the response reports which source answered. The older PID-based subscription still works.
- **Generic vehicle profile with bitrate/addressing autodetection**: tries 500k/250k bitrate
  × 11/29-bit addressing, listening first (never transmits at a bitrate it hasn't validated),
  then a single probe frame — useful on any OBD-II-over-CAN car with no dedicated profile
  (⚠ untested on a 29-bit car). Console commands `PARAMS` (catalog + source of each name) and
  `AUTODET` (force a re-scan).
- Bitmap of ECU-supported PIDs, read once, so unsupported parameters aren't polled forever
- **Vehicle profiles fetched from a server** (`VEH GET <model>`) — a new car model doesn't
  need a firmware update, just a JSON profile
- DTC read/clear across modules (`DTC`, `DTCALL`), module memory read for diagnostics
  (`READEPROM`), long-coding read (`CODING`)
- WiFi AP + STA (join your own router) with automatic AP fallback, TCP JSON (31416) +
  telnet (23) + serial console, mDNS (`ergisscan.local`), UDP auto-discovery for the app
- OTA firmware updates over WiFi with automatic rollback
- Optional cloud logging and remote autoconfiguration, both off by default
- Optional GY-85 IMU (accelerometer/gyroscope/magnetometer, with calibration) + BMP280
  barometer — the module's own onboard sensors, for motion/orientation telemetry alongside
  the CAN data
- Deep-sleep when idle
- 3D-printable enclosure (`case.scad` / `case.stl`)
- Two build layouts: Arduino IDE sketch (`ergissvag_can/`, actively maintained) and
  PlatformIO (`esp32/`, kept for reference)

## Hardware
- ESP32 DevKit v1 + SN65HVD230 CAN transceiver
- (optional) GY-85 IMU + BMP280
- Connects to the car's OBD-II port (pins 6/14/5) via an extension/Y-cable — no wiring into
  the dashboard loom

⚠️ **Safety:** by default the firmware actively queries the vehicle over the diagnostic bus
(standard OBD-II PIDs plus targeted UDS requests) — it is **not** a purely passive tap. A
listen-only mode is available (`can.boot=listen` in the runtime settings, or the console
`MODE listen` command) for pure passive monitoring. It never writes vehicle
configuration/coding; the only "write" operations are clearing DTCs (a standard, reversible
diagnostic action) and an advanced PIN-protected console mode for sending raw frames during
development. Use an inline fuse. You are responsible for any work on your vehicle.

## Quick start
1. Flash via PlatformIO (`pio run -t upload`) or Arduino IDE (open `ergissvag_can.ino`).
2. In `config.h` set your own `WIFI_STA_SSID`/`WIFI_STA_PASS` (placeholders by default), and
   change `WIFI_AP_PASS` and the console PIN (`con.pin`, default is a placeholder too) before
   using it on a real car.
3. Power the module, connect the **ErgissVAG F16** app to it (or `nc ergisscan.local 31416`
   / telnet for the console) — data flows as JSON.

## License
**GNU General Public License v3.0 or later** (GPL-3.0-or-later). See [`LICENSE`](LICENSE).
Until 2026-11-05 this project was published under PolyForm Noncommercial 1.0.0; from this version on it is open source under the GPL.

Copyright © Gabriel Diaconu / ERGISS Media.

---
Built by **ERGISS Media** — [ergiss.ro](https://www.ergiss.ro) · [mensoft.ro](https://www.mensoft.ro)
Android: **[ErgissVAG F16 on Google Play](https://play.google.com/store/apps/details?id=ro.ergiss.vag.f16)**

## Support

If this helps you, you can buy me a coffee: **[paypal.me/axon003](https://paypal.me/axon003)**. Thank you!
