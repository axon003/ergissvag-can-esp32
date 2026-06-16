# ergissvag_can — ESP32 J533 CAN tap (listen-only)

Firmware ESP32 DevKit v1 + SN65HVD230 care face sniff pe CAN-Antrieb sau CAN-Komfort
(VW PQ35 — Seat Altea XL 2011 / Touran CBZB, gateway J533) si streameaza frame-uri
decodate ca JSON pe TCP, catre app Android `ergCAN` sau `ErgissVAG`.

- Autor: Gabriel Diaconu (ERGISS Media)
- Versiune: 1.0 (2026-04-22)
- Framework: PlatformIO + Arduino-ESP32
- CAN driver: TWAI nativ ESP32 (`driver/twai.h`)

## Structura

```
esp32/
  platformio.ini      # board esp32dev + framework arduino
  src/
    main.cpp          # firmware (TWAI + WiFi AP + TCP server + deep sleep)
    config.h          # constante (bitrate, SSID, TCP port, pini, timing)
  README.md
```

## Flash

### PlatformIO (recomandat)

```bash
cd esp32
pio run                              # compile
pio run -t upload                    # flash via USB (auto-detect port)
pio device monitor -b 115200         # serial log
```

### Arduino IDE

1. Board Manager -> instaleaza `esp32` by Espressif Systems
2. Board: `ESP32 Dev Module`, Upload speed 921600, Flash 4MB, Partition `Default`
3. Copiaza `src/main.cpp` ca `main.ino` intr-un folder `main/`, adauga `config.h` langa
4. Tools -> Port, Upload, apoi Serial Monitor 115200

## Config

Edit `src/config.h`:

| Define               | Default              | Note                                       |
|----------------------|----------------------|--------------------------------------------|
| `CAN_BITRATE`        | 500000               | 500k = Antrieb, 100k = Komfort             |
| `WIFI_AP_SSID`       | `ergCAN_ESP32`       |                                            |
| `WIFI_AP_PASS`       | `ergcan2026`         | WPA2, min 8 chars                          |
| `TCP_PORT`           | 31416                | 31415 rezervat ErgissVAG BCAST             |
| `DEEP_SLEEP_IDLE_S`  | 30                   | secunde fara client conectat -> sleep      |
| `HB_INTERVAL_MS`     | 5000                 | heartbeat JSON cand fara frames            |

Dupa modificare: `pio run -t upload`.

## Wiring

### ESP32 DevKit v1 <-> SN65HVD230

```
   ESP32 DevKit v1                 SN65HVD230 (listen-only)
   +---------------+                +------------------+
   | GPIO21 (CTX)  |--------------->| D  (TXD)         |   neconectat electric la bus (listen-only),
   |               |                |                  |   dar necesar pt init TWAI driver
   | GPIO22 (CRX)  |<---------------| R  (RXD)         |
   | 3V3           |--------------->| VCC (3V3)        |
   | GND           |--------------->| GND              |
   |               |                | RS   ----- GND   |   <-- OBLIGATORIU: high-speed silent mode
   |               |                | CANH ----------- | ---> J533 pin 6 (CAN-Antrieb HIGH)
   |               |                | CANL ----------- | ---> J533 pin 16 (CAN-Antrieb LOW)
   +---------------+                +------------------+
```

Note:
- **RS la GND**: forteaza SN65HVD230 in high-speed mode. Pt. listen-only pasiv trebuie
  doar sa NU trimita (TX inactiv la nivel de driver = TWAI_MODE_LISTEN_ONLY). RS la 3V3
  ar pune transceiver-ul in slope-control (nedorit).
- **NU** monta rezistor de terminare 120R — bus-ul e deja terminat in masina.
- CAN-H/L **twisted pair** 10-30cm, nu parallel la alimentare 12V.
- Alimentare: 3V3 regulator on-board ESP32 de pe 5V USB sau 5V boost din OBD.
  Pt. permanent montat, foloseste un step-down 12V->5V cu quiescent <1mA.

### J533 Gateway pinout PQ35 (Altea XL / Touran / Leon 1P)

Conector T20a (verde, 20 pini):

| Pin | Semnal              | Rate    |
|-----|---------------------|---------|
| 6   | CAN-Antrieb HIGH    | 500 kbps|
| 16  | CAN-Antrieb LOW     | 500 kbps|

Conector T20b (maro/altul, alte pini):
- CAN-Komfort HIGH/LOW — 100 kbps — vezi `gateway_j533.html` pt. map completa
- CAN-Infotainment — 100 kbps

Referinta vizuala: `gateway_j533.html`.

## Protocol TCP

- Port **31416**, WPA2 SSID **ergCAN_ESP32**, IP ESP32 = **192.168.4.1**
- Un singur client connect simultan (al doilea e respins)
- Banner initial la connect:
  ```json
  {"hello":"ergCAN","bitrate":500000,"port":31416}
  ```
- Frame CAN (1 linie, `\n` delimiter):
  ```json
  {"ts":1234,"id":"0x470","ext":0,"rtr":0,"len":8,"data":"01 00 00 00 00 00 00 00"}
  ```
  - `ts` = millis() de la boot ESP32
  - `id` = hex CAN ID (11-bit sau 29-bit in functie de `ext`)
  - `ext` = 0 standard, 1 extended
  - `rtr` = 0 data, 1 remote transmission request
  - `len` = DLC (0..8)
  - `data` = octeti hex separati cu spatiu
- Heartbeat la 5s daca nu vin frame-uri:
  ```json
  {"hb":1,"uptime":42,"rx_total":0,"rx_err":0}
  ```

## Android client pattern (ergCAN / ErgissVAG `WifiCanReader.kt`)

```kotlin
// 1. User conecteaza telefonul la SSID "ergCAN_ESP32" (WPA2 "ergcan2026")
// 2. Socket la 192.168.4.1:31416
val sock = Socket()
sock.connect(InetSocketAddress("192.168.4.1", 31416), 3000)
val br = BufferedReader(InputStreamReader(sock.getInputStream()))
while (isActive) {
    val line = br.readLine() ?: break
    val obj = JSONObject(line)
    if (obj.optInt("hb") == 1) { /* heartbeat */ continue }
    val id   = obj.getString("id")              // "0x470"
    val len  = obj.getInt("len")
    val data = obj.getString("data")            // "01 00 ..."
    // decode...
}
```

`WifiCanReader.kt` in ErgissVAG este sursa a 7-a de frame-uri (pe langa OBD-II, UDS,
GPS, senzori, BCAST, SyuCanReader). Prioritate: WifiCanReader > SyuCanReader > OBD.

## Troubleshooting

| Simptom                         | Cauza probabila / fix                                     |
|---------------------------------|-----------------------------------------------------------|
| `rx=0` pe Serial, fara frames   | Verifica RS=GND (nu la VCC!), verifica bitrate (500k/100k)|
| `state=BUS_OFF` in log          | Bitrate gresit sau CAN_H/L inversati                      |
| Reboot continu                  | TWAI driver crash — controleaza pini GPIO21/22 neconectati|
| Android nu vede SSID            | ESP32 deep-sleep — porneste contactul pt. wake-on-CAN     |
| Connect accept apoi disconnect  | Alt client deja conectat (limita 1 client) — deconecteaza |
| `rx_err` creste rapid           | Termination gresita — **NU** adauga 120R, e deja in masina|
| AP apare dar nu se conecteaza   | Parola min 8 chars; resetare ESP32 dupa schimbare config  |

## Deep sleep & wake-on-CAN

Dupa 30s fara client TCP conectat, ESP32 intra in deep sleep (~10uA).
Wake source: GPIO22 (CAN RX) la nivel HIGH — adica orice tranzitie pe bus dupa idle.

In practica PQ35:
- Contact OFF, toate busurile dorm -> ESP32 dormi
- Contact ON sau remote unlock -> CAN-Komfort/Antrieb se trezeste -> ESP32 wake (cateva ms)
- Dupa wake, ESP32 reporneste AP + TCP server + TWAI

Consum mediu: ~80 mA @ 3V3 cand activ (WiFi AP), ~10uA in deep sleep.

## Referinte

- `../gateway_j533.html` — ghid montaj fizic J533 + pin-out complet
- ErgissVAG: `D:/apps/ergissvag/app/src/main/java/ro/ergiss/vag/obd/` — client pattern
- ergCAN: `D:/apps/ergcan/` — app dedicat CAN sniffing (in dezvoltare)
- VW self-study SSP 269: CAN Data Bus (Antrieb + Komfort PQ35)
