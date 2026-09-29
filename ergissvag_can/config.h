/**
 * config.h — ergissvag_can ESP32 firmware configuration
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 3.5
 * @changes
 *   v1.0 2026-04-22 — creat initial; constants firmware (bitrate, WiFi AP, TCP port, deep sleep)
 *   v1.1 2026-05-23 — mutat CAN de pe 21/22 pe 5/4 (I2C ocupa 21/22 acum pt BMP280+GY-85);
 *                     adaugat WiFi STA mode (router propriu), I2C pini, sensor sample rate
 *   v1.2 2026-05-23 — UDP_DISC_PORT 31417 + UDP_BC_INTERVAL_MS 3000 pt auto-discovery LAN.
 *   v1.3 2026-09-13 — FW_VERSION unic; CAN_PROBE_* (probe OBD 01 00 pe bus idle → flag `can` corect pe
 *                     linia de diagnoza OBD, unde nu circula nimic nesolicitat); CAN_BOOT_MODE.
 *   v1.5 2026-09-15 — FW 2.0.0: clienti multipli, telnet, EXPLORE, SNIFF; WiFi implicit (setarile reale in NVS, cfg.cpp)
 *   v1.4 2026-09-14 — FW 1.26; TWAI_TX_QUEUE_LEN_NORMAL 8→4 (sa nu se adune cereri moarte fara ACK);
 *                     TCP_RX_PER_LOOP (limita bytes comenzi TCP procesate per trecere prin loop).
 *   v1.6 2026-09-16 — FW 2.2.2: WIFI_AP_FALLBACK_S — AP-ul de rezerva porneste abia dupa atatea secunde
 *                     de STA pierdut (inainte: 20 s, adica la orice clipire a hotspotului placa trecea pe
 *                     AP si apoi se REPORNEA pt bind curat: 5-8 reporniri/zi in datele din cloud).
 *                     WIFI_STA_TIMEOUT_S ramane doar cadenta de reincercare a WiFi.begin.
 *   v1.7 2026-09-16 — FW 2.2.3: litrii din rezervor (UDS 22 2206/2212 la bord) — comanda FUEL,
 *                     SELFTEST, cache difuzat la F16 ({"t":"fuel"}) si camp `fuel` in snapshot cloud.
 *   v1.8 2026-09-16 — FW 2.2.4: keepalive TCP pe clientii JSON + setul intern nu mai depinde de `bus_alive`
 *                     (blocaj circular: fara interogari nu vin cadre, deci nu porneau interogarile).
 *   v1.9 2026-09-16 — FW 2.2.5: diferente de timp CU SEMN (fage/eage/cache OBD — underflow intre nuclee)
 *                     + verificarea DID-ului ecou la citirea litrilor.
 *   v2.0 2026-09-16 — FW 2.2.6: setul intern porneste dupa ABONAMENTE, nu dupa clienti conectati;
 *                     multi-PID se reincearca periodic (nu mai ramane oprit definitiv = refresh lent).
 *   v2.1 2026-09-16 — FW 2.2.7: cheile setului intern = cele asteptate de F16 (speed/coolant/throttle/intake/
 *                     timing/runtime/battery) + PID-uri noi (pedal, catTemp, stft, ltft, equivRatio).
 *   v2.2 2026-09-16 — FW 2.2.8: setul intern = TOATE cele 24 de PID-uri din F16 (aceleasi chei si frecvente)
 *                     + maf/fuel pentru cloud. 26 itemi din 48 posibili.
 *   v2.3 2026-09-16 — FW 2.2.9: tabela 48 -> 64, oneshot-uri cu TTL, `unsupported` se reincearca la 5 min,
 *                     contoare noi in cloud (nrc, p78, txref, txfail, unsup).
 *   v2.4 2026-09-16 — FW 2.3.0: PROFIL DE VEHICUL (veh.h/veh.cpp). CAN_BITRATE nu mai e folosit direct la
 *                     twaiStart (vine din profil); chei noi veh.model / veh.url.
 *   v2.5 2026-09-17 — FW 2.3.1: CAN_IDLE_GATE_DEFAULT (fara cereri/upload cu CAN mort); FW 2.3.2: comentariul
 *                     despre revenirea „pasiva” corectat — pe mufa OBD revenirea e DOAR prin probe (blocare pana la reset).
 *   v2.6 2026-09-17 — FW 2.3.3: valori de pe ECU motor prin UDS (generator, ulei) — vezi proto.cpp v2.1 / veh.h v1.1.
 *   v2.7 2026-09-19 — FW 2.4.0: DTCALL — lista de instalare gateway + erori pe module (dtcall.cpp / proto.cpp v2.2).
 *   v2.8 2026-09-21 — FW 2.5.0: READEPROM (citire memorie modul TP2.0 ca VAG CAN PRO: 27 03/04 + 10 86 + 35/36/37, upload BYOS)
 *                     — READEPROM_MAX_LEN, TP20_SEC_KEY_ADD; SNIFF STOP revine la modul de DINAINTE (listen ramane listen).
 *   v3.6 2026-09-24 — FW 2.5.13: calibrare magnetometru (sensors.cpp v1.4, cfg mag.*, MAGCAL, mag in snapshot).
 *   v3.5 2026-09-24 — FW 2.5.12: magnetometru QMC5883P @0x2C (sensors.cpp v1.3).
 *   v3.4 2026-09-24 — FW 2.5.11: DID-uri generice din profil ("dids", comanda DIDS) + DTC/gateway automat doar la pornire.
 *   v3.3 2026-09-24 — FW 2.5.10: CAN se repara singur (driver agatat dupa bus-off) + jurnal automat de evenimente la BYOS.
 *   v3.2 2026-09-23 — FW 2.5.9: comanda DIDSCAN (proto.cpp v3.0).
 *   v3.1 2026-09-23 — FW 2.5.8: rute UDS din UDSSCAN (16=0x711, 52=0x74B; scoase 04/09/15/25) (dtcall.cpp v1.2, proto.cpp v2.9).
 *   v3.0 2026-09-23 — FW 2.5.7: pauze TP2.0 ca VCDS (130 ms dupa A1, 50 ms intre cereri, fara rafala A3 la busy);
 *                     clima 0x746 / usa sofer 0x74A pe UDS (tp20.cpp v1.6, dtcall.cpp v1.1, proto.cpp v2.8).
 *   v2.9 2026-09-21 — FW 2.5.1: subcomenzile consolei (STOP/NOW/ON/OFF/CLR/CHECK/UPDATE) case-insensitive (proto.cpp v2.4).
 *   v2.10 2026-09-21 — FW 2.5.2: READEPROM cu TRACE cadre TP2.0 (afisat la esec + `frames` in upload), NAK detectat,
 *                     pauza T3 intre cereri, 27 03 reincercat o data (tp20.cpp v1.3, proto.cpp v2.5); KM din ECU motor
 *                     (profil ekm_req/ekm_did/ekm_fmt = 7E0/121C/u32, bord rezerva; veh v1.2); UDS <id> <did> [nr] [ms].
 *   v2.11 2026-09-21 — FW 2.5.3: READEPROM pana la 32 KB pe bucati de 256 B (un singur seed/key/10 86, 35/36/37 repetat),
 *                     oprire la NRC (31 = margine memorie) cu adresa ultimei zone valide, upload BYOS pe bucati de 4 KB,
 *                     trace ring (ultimele 96 cadre); hex pe consola doar pana la 512 B (tp20.cpp v1.4, proto.cpp v2.6).
 */
#ifndef ERGCAN_CONFIG_H
#define ERGCAN_CONFIG_H

// ===== Versiune firmware (apare in discovery UDP, hello TCP, stat, mDNS) =====
#define FW_VERSION         "2.6.0"

// ===== Calibrare magnetometru (fw 2.5.13, sensors.cpp v1.4) =====
// Componenta orizontala a campului Pamantului in RO ~20-23 uT => un cerc complet da span ~40-46 uT pe X si Y.
#define MAGCAL_AUTO_SPAN_UT   30.0f   // salvare automata cand span X si Y >= prag (cerc aproape complet)
#define MAGCAL_MIN_SPAN_UT    15.0f   // prag minim la MAGCAL STOP (manual)
#define MAGCAL_MIN_SAMPLES    300     // >= 30 s de date la 10 Hz
#define MAGCAL_WARMUP         20      // esantioane care doar incarca filtrul
#define MAGCAL_EMA            0.15f   // filtru exponential (10 Hz): taie varfurile scurte
#define MAGCAL_RAW_MAX_UT     600.0f  // citiri peste = saturatie/eroare, ignorate

// ===== READEPROM (fw 2.5.0) — citire memorie modul prin KWP requestUpload, manual, doar pe loc =====
#define READEPROM_MAX_LEN  32768          // v2.11: octeti maxim per comanda; citirea se face pe bucati de READEPROM_CHUNK intr-o singura sesiune
#define READEPROM_CHUNK    256            // octeti per 35 requestUpload (BCM trimite blocuri de blockmax ~67 B in 36)
// Cheia de securitate KWP nivel 03/04 (algoritm seed+add pe module VAG mai vechi): key = seed(u32) + TP20_SEC_KEY_ADD.
// Valoarea de mai jos e un PLACEHOLDER — se deduce dintr-o pereche seed/key capturata cu un instrument de
// diagnoza (ex. sniff pe magistrala in timpul unei sesiuni 27 03/27 04) pentru MODULUL TAU si se inlocuieste aici.
// Daca modulul raspunde NRC 35 (cheie gresita), firmware-ul NU mai incearca pana la reboot (sa nu blocheze accesul).
#define TP20_SEC_KEY_ADD   0x00000000UL   // CHANGE_ME — vezi comentariul de mai sus

// ===== ergCAN 2.0: clienti + consola =====
#define MAX_JSON_CLIENTS   3              // TCP 31416 (F16, scripturi, aplicatii)
#define MAX_TELNET_CLIENTS 2              // telnet 23 (consola PuTTY)
#define TELNET_PORT        23
#define EXPLORE_TIMEOUT_MS 1800000UL      // mod EXPLORE expira dupa 30 min
#define SNIFF_CHUNK_FRAMES 1500           // cadre per POST la upload SNIFF
#define SNIFF_REC_SIZE     17             // octeti per cadru in fisierul de captura

// ===== Mod TWAI la boot: "normal" (OBD master) | "listen" (zero emisie; sigur la prima conectare) =====
// Se poate schimba la cald cu {"cmd":"mode","m":"..."}.
#define CAN_BOOT_MODE      "normal"

// ===== Probe OBD pe bus idle =====
// Pe mufa OBD (CAN diagnoza prin gateway) nu circula frame-uri nesolicitate: fara probe, `can` ar ramane 0
// si F16 ar sta pe ELM. Cand nu s-a primit nimic de CAN_PROBE_IDLE_MS trimitem 7DF 02 01 00; raspunsul 7E8
// tine `can`=1. Dupa TX esuat (masina oprita / nimic pe bus) intervalul creste la CAN_PROBE_BACKOFF_MS.
#define CAN_PROBE_IDLE_MS      5000
#define CAN_PROBE_BACKOFF_MS   30000

// v2.3.1: cand magistrala e moarta (contact luat, masina parcata) nu mai are rost nici sa interogam,
// nici sa trimitem in cloud. Masurat pe 2.3.0, in 934 s cu `can`=0: 699 cereri -> 0 raspunsuri, 701
// timeout-uri. Zero informatie, doar contoare umflate si trafic.
// Revenirea NU e pasiva pe mufa OBD (v2.3.2 — ce scria aici era gresit): pe linia de diagnoza nu circula
// cadre nesolicitate, deci singura cale inapoi la `can`=1 e probe-ul CAN_PROBE_* (7DF 01 00, single-shot),
// care ruleaza cat timp magistrala e moarta, INDIFERENT de abonamente (altfel: blocare pana la reset).
#define CAN_IDLE_GATE_DEFAULT  "1"      // cfg `can.idle_gate` — 0 dezactiveaza comportamentul de mai sus
#define CAN_OK_WINDOW_MS       15000     // `can`=1 daca raspuns ECU sau frame in ultimele X ms

// ===== CAN bitrate =====
// 500000 = CAN-Antrieb (powertrain, J533 pin 6+16 PQ35)
// 100000 = CAN-Komfort (comfort bus, J533 pins comfort CAN-L/H)
#define CAN_BITRATE        500000

// ===== Pin map ESP32 DevKit v1 <-> SN65HVD230 =====
// MUTAT 2026-05-23: era 21/22, dar acum I2C ocupa 21/22 (BMP280 + GY-85).
// Folosim RX2/TX2 marcati pe DevKit v1 — pini liberi (UART2 nu e folosit), label clar pe placa.
#define CAN_TX_PIN         GPIO_NUM_17   // TX2 marcat pe placa -> SN65HVD230 CTX (D/TXD)
#define CAN_RX_PIN         GPIO_NUM_16   // RX2 marcat pe placa <- SN65HVD230 CRX (R/RXD)
#define LED_PIN            2             // LED builtin ESP32 DevKit v1

// ===== I2C senzori (BMP280 + GY-85: ADXL345 + ITG3200 + HMC5883L) =====
#define I2C_SDA_PIN        21
#define I2C_SCL_PIN        22
#define I2C_FREQ_HZ        50000        // 400kHz safe pt fire scurte <20cm

// I2C adrese
#define BMP280_I2C_ADDR    0x76          // 0x77 daca SDO=VCC

// Debug: dezactiveaza init BMP280 temporar — pune 1 cand vrei sa testezi daca BMP-ul
// blocheaza magnetometrul (pull-up dublu / interferenta init). Default 0 = BMP normal.
#define DISABLE_BMP280     0

// Debug: cand 1 → Serial Monitor curat (doar init + scan + STAT, FARA stream JSON sensors).
// TCP continua sa trimita JSON normal pt APK. Default 0 = JSON pe ambele (Serial + TCP).
#define DEBUG_SERIAL_QUIET 0
#define ADXL345_I2C_ADDR   0x53          // GY-85 accelerometer
#define ITG3200_I2C_ADDR   0x68          // GY-85 gyroscope
#define HMC5883L_I2C_ADDR  0x1E          // GY-85 magnetometer

// Sample rate senzori
#define SENSOR_SAMPLE_MS   100           // citire la fiecare 100ms (10Hz)

// ===== WiFi STA (conectare la router) — preferinta principala =====
#define WIFI_STA_SSID      "YOUR_WIFI_SSID"
#define WIFI_STA_PASS      "YOUR_WIFI_PASSWORD"
#define WIFI_STA_TIMEOUT_S 20            // cadenta de reincercare a WiFi.begin cat timp STA nu e conectat
#define WIFI_AP_FALLBACK_S 120           // v2.2.2: AP-ul de rezerva porneste abia dupa atata timp de STA pierdut.
                                         // Se aplica DOAR dupa ce am fost conectati macar o data in acest ciclu
                                         // de alimentare; la prima pornire ramane pragul scurt (WIFI_STA_TIMEOUT_S),
                                         // ca sa se poata intra pe AP cand reteaua nu exista deloc.
#define WIFI_STA_RETRY_S   30            // din AP fallback, reincearca STA la fiecare X sec (revine la HU/router cand redevine accesibil)

// ===== mDNS hostname (.local) — accesibil ca "ergisscan.local" in LAN =====
#define MDNS_HOSTNAME      "ergisscan"

// ===== WiFi AP fallback (daca STA esueaza) =====
#define WIFI_AP_SSID       "ergCAN_ESP32"
#define WIFI_AP_PASS       "ergcan2026"
#define WIFI_AP_CHANNEL    6
#define WIFI_AP_MAX_CONN   1

// ===== TCP server =====
#define TCP_PORT           31416          // 31415 e rezervat ErgissVAG BCAST; 31416 = ergCAN

// ===== UDP discovery broadcast =====
// ESP32 anunta presenta in LAN dupa WiFi STA up. APK F16 asculta UDP pe 31417 →
// la receptie auto-pornit TCP client catre TCP_PORT 31416. Daca lipsesc 3 broadcast-uri
// la rand (=> 9s), APK marcheaza ergCAN offline si revine la fallback (BT ELM / device).
#define UDP_DISC_PORT          31417
#define UDP_BC_INTERVAL_MS     3000

// ===== Timing =====
#define DEEP_SLEEP_IDLE_S  300            // 5 min fara client -> deep sleep (era 30s; marit pt debug/test)
#define HB_INTERVAL_MS     5000           // heartbeat JSON daca fara frames
#define STATS_INTERVAL_MS  10000          // log statistici Serial
#define LED_BLINK_MS       20             // durata puls LED la RX frame

// ===== Buffers =====
#define TWAI_RX_QUEUE_LEN  64
#define TWAI_TX_QUEUE_LEN         0       // legacy (listen-only) — pastrat pt compat referinte
#define TWAI_TX_QUEUE_LEN_NORMAL  4       // OBD/UDS in mode NORMAL (v1.26: 8→4, fara ACK nu adunam cereri moarte)
#define TCP_RX_PER_LOOP           128     // bytes comenzi TCP procesate per trecere prin loop (v1.26)
#define JSON_BUF_SZ        320            // marit pt sensor frame (mai multe fields)
// v2.6.0: lungimea maxima a unei linii JSON PRIMITE de la un client (TCP/serial). Pana la 2.5.13 era 320, iar o linie
// mai lunga era taiata in tacere: abonamentul F16 (24 de itemi, ~1100 caractere) nu s-a inregistrat NICIODATA — placa
// ii servea datele doar prin setul intern. Acum: 1024 + raspuns de eroare la depasire. Clientii noi trimit bucati scurte.
#define JSON_LINE_MAX      1024

#endif // ERGCAN_CONFIG_H
