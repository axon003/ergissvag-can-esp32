/**
 * main.cpp — ergissvag_can ESP32 J533 CAN tap + sensors + OBD master firmware
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.24
 * @changes
 *   v1.24 2026-05-24
 *     - Fix `can` flag fals positive in discovery JSON: era doar `g_can_ok && g_can_bus_ok`,
 *       dar in listen-only fara TX, BUS_OFF nu se declanseaza chiar fara bus efectiv →
 *       flag ramanea true ne-corect. Adaugat verificare `lastFrameMs` (frame primit in <10s).
 *       APK corect trece pe ELM ca sursa OBD cand ergCAN nu primeste frame-uri.
 *   v1.23 2026-05-23
 *     - print_sensor_status() — sumar BMP/ADXL/ITG/MAG, apelat in init si la fiecare STAT (10s).
 *       Asa user vede status init si daca Serial Monitor s-a deschis dupa boot.
 *   v1.22 2026-05-23
 *     - Flag nou DEBUG_SERIAL_QUIET in config.h: cand 1 → emitJson() omitem Serial
 *       (Serial Monitor curat pt diagnoza init/scan/STAT). TCP continua sa trimita
 *       JSON normal pt APK. Default 0 = JSON pe ambele (Serial + TCP).
 *     - I2C revert 50kHz → 400kHz (cleanup test). Tot magnetometru absent — confirmat
 *       chip lipsa fizica; alimentare 5V pe pinul "3.3V" GY-85 (LDO intern) e singurul
 *       test ramas conform tutorial Instructables.
 *   v1.21 2026-05-23
 *     - I2C frecventa 400kHz → 50kHz (test detectie magnetometru cu timing extra tolerant).
 *   v1.20 2026-05-23
 *     - emitJson() reactivat pe Serial — JSON sensors stream apare din nou pe USB-Serial,
 *       pt conectare APK via USB CDC ACM ca alternativa la TCP/WiFi (port 31416).
 *       Serial Monitor in Arduino IDE va arata spam JSON la 10Hz — pt diagnoza pura,
 *       comentat Serial.write linia 230.
 *   v1.19 2026-05-23
 *     - Cleanup debug: I2C 100k → 400k, DISABLE_BMP280 → 0, dump 0x2C scos din STAT
 *       (chip identificat ca EEPROM pasiv board, ignorat). Scan I2C ramane in STAT 10s
 *       pt monitoring. Concluzie: HMC5883L lipseste fizic — cumpara GY-271 pt compass.
 *   v1.18 2026-05-23
 *     - DISABLE_BMP280=1 in config.h → test daca BMP280 init blocheaza HMC5883L pe bus.
 *       Daca dupa flash scan-ul arata 0x1E sau 0x0D → BMP cauzeaza conflict.
 *   v1.17 2026-05-23
 *     - sensors_debug_print() dump extins 0x2C reg 0x00..0x3F + activity test (2 citiri
 *       la 500ms distanta) — vad daca chip pasiv (EEPROM/OTP/ID) sau activ (sensor).
 *   v1.16 2026-05-23
 *     - sensors_debug_print() extins: forced-read probe pe 0x0C/0x0D/0x0E/0x1C/0x1D/0x1E/0x1F
 *       (toate adresele magnetometre cunoscute — pt cazul cand chip-ul prezent dar bug-uit
 *       nu raspunde la write-probe scan dar raspunde la read direct). HMC5883L ID check
 *       la 0x1E ('H','4','3' confirm).
 *   v1.15 2026-05-23
 *     - I2C frecventa scazuta 400kHz → 100kHz (standard mode) — tolerant la pull-up
 *       dublu (GY-85 + BMP280 ambele cu onboard pull-up) si cabluri zgomotoase.
 *   v1.14 2026-05-23
 *     - sensors_debug_print() apelat la fiecare ciclu STAT (10s) → scan I2C + dump 0x2C +
 *       write-read test vizibile si daca Serial Monitor a fost deschis dupa boot ESP32.
 *   v1.13 2026-05-23
 *     - Diagnoza adresa I2C 0x2C neasteptata: dump primii 16 regs + write-read test.
 *       Pattern-ul output identifica chip-ul (sau confirma ghost ACK).
 *   v1.12 2026-05-23
 *     - emitJson() NU mai trimite pe Serial (doar TCP). Serial Monitor ramane curat —
 *       vezi clar init logs ([I2C] scan + sensor detect + [WiFi] + [TCP] + [STAT] 10s).
 *       Pt re-activare debug Serial: decomenteaza Serial.write linia in emitJson().
 *   v1.11 2026-05-23
 *     - i2c_scan_print() la boot — printeaza toate adresele I2C detectate (diagnoza
 *       magnetometru). Vezi Serial Monitor: "[I2C] scan: 0x1E 0x53 0x68 0x76 (4 devices)"
 *       sau similar. Daca apare adresa neasteptata pt magnetometru → identificam clona.
 *   v1.10 2026-05-23
 *     - Magnetometru detect auto HMC5883L (Honeywell @0x1E) sau QMC5883L (QST clone @0x0D).
 *       Init manual via I2C (NU Adafruit_HMC5883_U care esueaza tacit pe clone) → date reale.
 *     - JSON "mag":{"chip":"HMC|QMC|none",...} pt diagnoza. APK ignora field-ul (parser flat).
 *     - Verificare ID HMC (regs 0x0A-C='H43') inainte init; QMC SET/RESET 0x01 + control 0x09.
 *     - Conversie LSB→µT corecta per chip (HMC 1.3G=10.9 LSB/µT, QMC 2G=12 LSB/µT).
 *   v1.9 2026-05-23
 *     - Fix bug "STA pierdut definitiv": dupa AP fallback, ESP32 retry STA la fiecare
 *       WIFI_STA_RETRY_S (60s). Cand HU hotspot/router revine dupa cadere, ESP32 se
 *       reconecteaza automat fara reboot. Inainte ramanea in AP `ergCAN_ESP32` la infinit.
 *   v1.8 2026-05-23
 *     - Discovery JSON include "can":<0|1> = stare CAN (g_can_ok && g_can_bus_ok).
 *     - APK F16 decide sursa OBD: can=1 → ergCAN inlocuieste BT ELM; can=0 → ELM ramane
 *       activ pt OBD, dar ergCAN ramane sursa pentru BMP280 + GY-85 (atitudine/BARO/IMU).
 *   v1.7 2026-05-23
 *     - UDP discovery trimite pe TREI destinatii: (1) UNICAST la gateway (WiFi.gatewayIP())
 *       — esential cand HU ruleaza hotspot; Android hotspot izoleaza broadcast intre clienti AP,
 *       dar nu filtreaza unicast catre gateway-ul AP-ului (HU);
 *       (2) Subnet broadcast (WiFi.broadcastIP() = ex 192.168.43.255) — pt alte device-uri din retea;
 *       (3) Limited 255.255.255.255 — fallback (mai mult Android-uri blocheaza, dar nu strica).
 *     - Confirmat functional cu HU + ESP32 client pe hotspot HU (varianta 1 = unicast).
 *   v1.6 2026-05-23
 *     - UDP broadcast discovery la fiecare UDP_BC_INTERVAL_MS (3s) pe 255.255.255.255:31417
 *     - Mesaj JSON {"app":"ergCAN","ip":"<IP>","tcp_port":31416,"ts":..,"fw":"1.7"}
 *     - APK F16 asculta UDP si auto-pornit TCP catre IP-ul descoperit. La 3 lipsuri (9s)
 *       APK marcheaza ergCAN offline si revine la fallback (BT ELM / device sensors).
 *     - Activ dupa WiFi STA up (sau AP fallback) — pornit din netServicesUp(); merge si in AP mode.
 *   v1.0 2026-04-22 — creat initial
 *     - TWAI listen-only (500k Antrieb / 100k Komfort config-selectabil)
 *     - WiFi AP "ergCAN_ESP32" (1 client)
 *     - TCP server port 31416, 1 frame = 1 linie JSON
 *     - Heartbeat 5s cand fara frames
 *     - Deep sleep 30s fara client + wake-on-CAN
 *     - Reboot auto la TWAI bus-off / hw error
 *   v1.1 2026-05-23
 *     - CAN mutat pe RX2/TX2 (GPIO 16/17) — 21/22 ocupati de I2C BMP280+GY-85
 *     - WiFi STA mode catre router "YOUR_WIFI_SSID" (cu fallback AP daca esueaza)
 *     - Senzori BMP280 + GY-85 (ADXL345 + ITG3200 + HMC5883L) pe I2C 21/22 @ 400kHz
 *     - JSON sensors emis pe Serial si TCP la fiecare 100ms (10Hz)
 *     - Scos deep sleep (alimentare din USB HU se taie cu contactul)
 *   v1.2 2026-05-23
 *     - mDNS hostname "ergisscan.local" — accesibil din LAN fara IP (nu are afisaj)
 *     - DHCP hostname set in STA mode (apare in lista router ca "ergisscan")
 *     - Anunt serviciu _ergcan._tcp.31416 pentru auto-discovery
 *   v1.3 2026-05-23
 *     - WiFi connect NON-BLOCKING — sensors+CAN+Serial emit imediat la boot
 *     - State machine in loop: WIFI_STA_TRYING -> WIFI_STA_OK | WIFI_AP_OK
 *     - mDNS + TCP server pornesc DUPA ce WiFi e up (la transition)
 *     - HU boot poate dura zeci de sec, dar Serial USB merge instant
 *   v1.4 2026-05-23
 *     - TWAI mode NORMAL (era LISTEN_ONLY) — pot transmite query OBD2/UDS pe bus
 *     - ISO-TP layer (iso_tp.cpp) — SF + FF/CF/FC, multi-ECU broadcast collection
 *     - OBD/UDS commands (obd_cmd.cpp) — mode 01/03/04/07/09 + UDS 10/19/22
 *     - USB JSON command parser (usb_proto.cpp) — F16 trimite query, ESP32 dispatches
 *     - DTC se citeste in broadcast 0x7DF → toate ECU-urile raspund pe 7E8-7EF
 *   v1.5 2026-05-23
 *     - Auto-recovery la BUS_OFF (fara reboot) — sensors+WiFi continua functional
 *     - Flag global g_can_bus_ok — iso_tp blocheaza TX cand bus absent
 *     - Eveniment JSON "buststate" emit la trans tranzitiile ok/bus_off
 *     - Alerts ALERT_BUS_RECOVERY_IN_PROGRESS / ALERT_BUS_RECOVERY_COMPLETE wired
 *
 * Hardware:
 *   ESP32 DevKit v1
 *     + SN65HVD230 (RS=GND silent listen-only)
 *       GPIO17 (TX2) -> CTX  (neconectat electric la CAN, doar init driver)
 *       GPIO16 (RX2) <- CRX
 *       3V3 + GND
 *       CAN_H/CAN_L la J533 pin 6/16 (CAN-Antrieb PQ35)
 *     + BMP280 (3.3V version) — adresa 0x76
 *     + GY-85 9DOF (ADXL345 0x53 + ITG3200 0x68 + HMC5883L 0x1E)
 *       3V3 + GND + GPIO21 (SDA) + GPIO22 (SCL)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include "driver/twai.h"
#include "esp_system.h"
#include "config.h"
#include "sensors.h"
#include "iso_tp.h"
#include "obd_cmd.h"
#include "usb_proto.h"

// ===== State =====
static WiFiServer tcpServer(TCP_PORT);
static WiFiClient tcpClient;
static WiFiUDP    udpDisc;             // UDP broadcast discovery 255.255.255.255:31417
static uint32_t   lastUdpBcMs = 0;

static uint32_t rxTotal      = 0;
static uint32_t rxErr        = 0;
static uint32_t lastFrameMs  = 0;
static uint32_t lastHbMs     = 0;
static uint32_t lastStatsMs  = 0;
static uint32_t lastSensorMs = 0;
static uint32_t ledOffAt     = 0;
static uint32_t bootMs       = 0;
static bool     g_can_ok     = false;

// WiFi state machine (non-blocking connect)
enum WifiState : uint8_t {
    WIFI_STA_TRYING,    // STA.begin() lansat, asteptam status WL_CONNECTED
    WIFI_STA_OK,        // STA conectat, mDNS+TCP up
    WIFI_AP_OK          // fallback AP activ dupa timeout STA
};
static WifiState g_wifi_state    = WIFI_STA_TRYING;
static uint32_t  g_wifi_start_ms = 0;
static bool      g_net_services_up = false;   // mDNS + TCP server pornite

// CAN bus state — blocheaza TX iso_tp cand bus absent (evita BUS_OFF loop)
volatile bool g_can_bus_ok = true;             // extern in iso_tp.cpp

// ===== Helpers =====

static void ledPulse() {
    digitalWrite(LED_PIN, HIGH);
    ledOffAt = millis() + LED_BLINK_MS;
}

static void ledService() {
    if (ledOffAt != 0 && (int32_t)(millis() - ledOffAt) >= 0) {
        digitalWrite(LED_PIN, LOW);
        ledOffAt = 0;
    }
}

static bool twaiTimingFor(uint32_t bps, twai_timing_config_t &out) {
    switch (bps) {
        case 1000000: out = TWAI_TIMING_CONFIG_1MBITS();   return true;
        case 800000:  out = TWAI_TIMING_CONFIG_800KBITS(); return true;
        case 500000:  out = TWAI_TIMING_CONFIG_500KBITS(); return true;
        case 250000:  out = TWAI_TIMING_CONFIG_250KBITS(); return true;
        case 125000:  out = TWAI_TIMING_CONFIG_125KBITS(); return true;
        case 100000:  out = TWAI_TIMING_CONFIG_100KBITS(); return true;
        case 50000:   out = TWAI_TIMING_CONFIG_50KBITS();  return true;
        default:      return false;
    }
}

static bool twaiStart() {
    // Mode NORMAL (era LISTEN_ONLY) — pot transmite OBD/UDS query pe bus
    // Sniffing pasiv RX continua sa functioneze identic
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL
    );
    g.rx_queue_len = TWAI_RX_QUEUE_LEN;
    g.tx_queue_len = TWAI_TX_QUEUE_LEN_NORMAL;
    g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_ERROR
                      | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_ERR_PASS
                      | TWAI_ALERT_TX_FAILED
                      | TWAI_ALERT_BUS_RECOVERED;

    twai_timing_config_t t;
    if (!twaiTimingFor(CAN_BITRATE, t)) {
        Serial.printf("[TWAI] bitrate %u nesuportat\n", CAN_BITRATE);
        return false;
    }
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t e = twai_driver_install(&g, &t, &f);
    if (e != ESP_OK) {
        Serial.printf("[TWAI] driver_install err 0x%x\n", e);
        return false;
    }
    e = twai_start();
    if (e != ESP_OK) {
        Serial.printf("[TWAI] start err 0x%x\n", e);
        twai_driver_uninstall();
        return false;
    }
    Serial.printf("[TWAI] OK NORMAL mode @%u bps  (TX=GPIO%d RX=GPIO%d)\n",
                  CAN_BITRATE, (int)CAN_TX_PIN, (int)CAN_RX_PIN);
    return true;
}

static void twaiStopAndReboot(const char *why) {
    Serial.printf("[TWAI] FATAL %s -> reboot\n", why);
    twai_stop();
    twai_driver_uninstall();
    delay(200);
    esp_restart();
}

// Format 1 TWAI message -> 1 line JSON
static size_t frameToJson(const twai_message_t &m, char *buf, size_t bufSz) {
    char dataHex[3 * 8 + 1];
    dataHex[0] = 0;
    size_t dlc = m.data_length_code;
    if (dlc > 8) dlc = 8;
    size_t p = 0;
    for (size_t i = 0; i < dlc; ++i) {
        if (i) dataHex[p++] = ' ';
        static const char H[] = "0123456789ABCDEF";
        dataHex[p++] = H[(m.data[i] >> 4) & 0xF];
        dataHex[p++] = H[m.data[i] & 0xF];
    }
    dataHex[p] = 0;

    uint32_t canId = m.identifier;
    int ext = (m.flags & TWAI_MSG_FLAG_EXTD) ? 1 : 0;
    int rtr = (m.flags & TWAI_MSG_FLAG_RTR)  ? 1 : 0;

    return snprintf(buf, bufSz,
        "{\"ts\":%lu,\"type\":\"can\",\"id\":\"0x%lX\",\"ext\":%d,\"rtr\":%d,\"len\":%u,\"data\":\"%s\"}\n",
        (unsigned long)millis(),
        (unsigned long)canId,
        ext, rtr,
        (unsigned)m.data_length_code,
        dataHex);
}

// Trimite JSON la Serial (pt USB-Serial APK consume) si TCP (pt WiFi APK).
// Cand DEBUG_SERIAL_QUIET=1 → omitem Serial → Serial Monitor curat pt diagnoza init/scan/STAT.
void emitJson(const char *line, size_t len) {
#if !DEBUG_SERIAL_QUIET
    Serial.write((const uint8_t *)line, len);
#endif
    if (g_net_services_up && tcpClient && tcpClient.connected()) {
        tcpClient.write((const uint8_t *)line, len);
    }
}

static const char *wifiStateName() {
    switch (g_wifi_state) {
        case WIFI_STA_TRYING: return "STA-trying";
        case WIFI_STA_OK:     return "STA";
        case WIFI_AP_OK:      return "AP";
    }
    return "?";
}

static String wifiCurrentIp() {
    if (g_wifi_state == WIFI_STA_OK) return WiFi.localIP().toString();
    if (g_wifi_state == WIFI_AP_OK)  return WiFi.softAPIP().toString();
    return String("0.0.0.0");
}

static void sendHeartbeat() {
    char buf[JSON_BUF_SZ];
    int n = snprintf(buf, sizeof(buf),
        "{\"ts\":%lu,\"type\":\"hb\",\"uptime\":%lu,\"rx_total\":%lu,\"rx_err\":%lu,"
        "\"wifi\":\"%s\",\"ip\":\"%s\",\"rssi\":%d}\n",
        (unsigned long)millis(),
        (unsigned long)((millis() - bootMs) / 1000UL),
        (unsigned long)rxTotal,
        (unsigned long)rxErr,
        wifiStateName(),
        wifiCurrentIp().c_str(),
        g_wifi_state == WIFI_STA_OK ? WiFi.RSSI() : 0);
    if (n > 0) emitJson(buf, (size_t)n);
}

// ===== WiFi non-blocking state machine =====

// Lanseaza tentativa STA (non-blocking — doar setup, nu asteapta connect)
static void wifiStaBeginAsync() {
    Serial.printf("[WiFi] STA begin async to '%s' (timeout %ds)\n",
                  WIFI_STA_SSID, WIFI_STA_TIMEOUT_S);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setHostname(MDNS_HOSTNAME);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    g_wifi_state    = WIFI_STA_TRYING;
    g_wifi_start_ms = millis();
}

static void wifiApFallback() {
    Serial.println("[WiFi] STA timeout -> fallback AP");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAPsetHostname(MDNS_HOSTNAME);
    bool ok = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS,
                          WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN);
    Serial.printf("[WiFi] AP %s ok=%d ip=%s\n",
                  WIFI_AP_SSID, ok ? 1 : 0, WiFi.softAPIP().toString().c_str());
    g_wifi_state = WIFI_AP_OK;
}

// Porneste mDNS + TCP server (apelat la prima conectare reusita STA sau AP)
static void netServicesUp() {
    if (g_net_services_up) return;

    if (!MDNS.begin(MDNS_HOSTNAME)) {
        Serial.println("[mDNS] start FAILED");
    } else {
        MDNS.addService("ergcan", "tcp", TCP_PORT);
        MDNS.addServiceTxt("ergcan", "tcp", "app", "ergCAN");
        MDNS.addServiceTxt("ergcan", "tcp", "ver", "1.3");
        Serial.printf("[mDNS] hostname '%s.local' service _ergcan._tcp.%d\n",
                      MDNS_HOSTNAME, TCP_PORT);
    }

    tcpServer.begin();
    tcpServer.setNoDelay(true);
    Serial.printf("[TCP] listen on :%d\n", TCP_PORT);

    // UDP discovery socket — broadcast pe 255.255.255.255:UDP_DISC_PORT la fiecare UDP_BC_INTERVAL_MS
    if (udpDisc.begin(UDP_DISC_PORT)) {
        Serial.printf("[UDP] discovery socket bound :%d, broadcast la %u ms\n",
                      UDP_DISC_PORT, (unsigned)UDP_BC_INTERVAL_MS);
    } else {
        Serial.printf("[UDP] discovery bind FAILED on :%d\n", UDP_DISC_PORT);
    }
    lastUdpBcMs = 0;   // trimite primul broadcast la prima trecere prin loop

    g_net_services_up = true;
}

// Emite UDP cu IP-ul ESP32 catre TOATE destinatiile relevante:
//  1) Gateway (unicast) — esential cand HU ruleaza hotspot iar APK e pe HU. Android hotspot
//     izoleaza broadcast-uri intre clienti AP, dar nu filtreaza UNICAST catre gateway.
//  2) Subnet broadcast (WiFi.broadcastIP() = ex 192.168.43.255) — pt alte device-uri din LAN.
//  3) Limited broadcast 255.255.255.255 — fallback ultim (mult Android e blocheaza, dar nu strica).
static void sendUdpDiscovery() {
    IPAddress ip = (g_wifi_state == WIFI_AP_OK) ? WiFi.softAPIP() : WiFi.localIP();
    // can:1 = TWAI driver instalat + bus prezent + frames recente (in ultimele 10s).
    // Listen-only fara TX nu declanseaza BUS_OFF chiar daca bus absent → g_can_bus_ok ramane
    // true fals. Verificare suplimentara `lastFrameMs` — daca nicio frame in 10s → can=0,
    // APK trece pe ELM ca sursa OBD. La motor pornit + CAN cablat, frame-uri vin <1s.
    bool canFrames = (lastFrameMs > 0) && (millis() - lastFrameMs < 10000);
    int  canOk     = (g_can_ok && g_can_bus_ok && canFrames) ? 1 : 0;
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "{\"app\":\"ergCAN\",\"ip\":\"%s\",\"tcp_port\":%d,\"ts\":%lu,\"fw\":\"1.10\",\"can\":%d}",
        ip.toString().c_str(), TCP_PORT, (unsigned long)millis(), canOk);
    if (n <= 0) return;

    // (1) UNICAST la gateway — fix Android hotspot AP-isolation
    if (g_wifi_state == WIFI_STA_OK) {
        IPAddress gw = WiFi.gatewayIP();
        if (gw[0] != 0) {
            udpDisc.beginPacket(gw, UDP_DISC_PORT);
            udpDisc.write((const uint8_t *)buf, (size_t)n);
            udpDisc.endPacket();
        }
        // (2) Subnet broadcast
        IPAddress bc = WiFi.broadcastIP();
        if (bc[0] != 0) {
            udpDisc.beginPacket(bc, UDP_DISC_PORT);
            udpDisc.write((const uint8_t *)buf, (size_t)n);
            udpDisc.endPacket();
        }
    }
    // (3) Limited broadcast — fallback (in AP mode e singura optiune)
    udpDisc.beginPacket(IPAddress(255, 255, 255, 255), UDP_DISC_PORT);
    udpDisc.write((const uint8_t *)buf, (size_t)n);
    udpDisc.endPacket();
}

// Apelat in fiecare loop pentru a avansa state machine WiFi (non-blocking)
static void wifiServiceTick() {
    static uint32_t lastDotMs = 0;

    switch (g_wifi_state) {
    case WIFI_STA_TRYING: {
        wl_status_t st = WiFi.status();
        if (st == WL_CONNECTED) {
            Serial.printf("\n[WiFi] STA OK ip=%s rssi=%d\n",
                          WiFi.localIP().toString().c_str(), WiFi.RSSI());
            g_wifi_state = WIFI_STA_OK;
            netServicesUp();
        } else if (millis() - g_wifi_start_ms > (uint32_t)WIFI_STA_TIMEOUT_S * 1000UL) {
            wifiApFallback();
            netServicesUp();
        } else {
            // Print progress dot la fiecare 500ms (non-blocking)
            if (millis() - lastDotMs >= 500) {
                lastDotMs = millis();
                Serial.print(".");
            }
        }
        break;
    }
    case WIFI_STA_OK: {
        // Detecteaza pierderea conexiunii STA → reincearca async
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("\n[WiFi] STA pierdut -> reconect async");
            // Pastram services up (TCP/mDNS); WiFi.reconnect e auto la STA.begin
            g_wifi_state    = WIFI_STA_TRYING;
            g_wifi_start_ms = millis();
        }
        break;
    }
    case WIFI_AP_OK: {
        // Retry STA la fiecare WIFI_STA_RETRY_S din AP fallback. Cand HU hotspot revine
        // (sau routerul) dupa o cadere, ESP32 se reconecteaza automat fara reboot.
        static uint32_t lastReintry = 0;
        if (millis() - lastReintry >= (uint32_t)WIFI_STA_RETRY_S * 1000UL) {
            lastReintry = millis();
            Serial.println("[WiFi] STA reintry from AP fallback");
            WiFi.disconnect(true, true);
            WiFi.mode(WIFI_STA);
            WiFi.setHostname(MDNS_HOSTNAME);
            WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
            g_wifi_state    = WIFI_STA_TRYING;
            g_wifi_start_ms = millis();
            // Daca esueaza in WIFI_STA_TIMEOUT_S → wifiApFallback() re-activeaza AP.
            // Daca succes → trecere WIFI_STA_OK + netServicesUp() (no-op daca deja up).
        }
        break;
    }
    }
}

// ===== Setup / Loop =====

void setup() {
    Serial.begin(115200);
    delay(200);
    bootMs = millis();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println();
    Serial.println("======================================");
    Serial.println(" ergissvag_can ESP32 v1.5");
    Serial.println(" CAN NORMAL + BMP280 + GY-85");
    Serial.println(" OBD/UDS master + auto-recovery bus");
    Serial.println(" WiFi async (Serial USB emite imediat)");
    Serial.println("======================================");

    usb_proto_init();
    Serial.printf(" CAN bitrate: %u bps (TX=GPIO%d RX=GPIO%d)\n",
                  CAN_BITRATE, (int)CAN_TX_PIN, (int)CAN_RX_PIN);
    Serial.printf(" I2C: SDA=GPIO%d SCL=GPIO%d @ %u Hz\n",
                  I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);

    // Senzori I2C (BMP280 + GY-85) — pornesc imediat, independente de WiFi
    if (!sensors_init()) {
        Serial.println("[I2C] WARN: niciun senzor detectat — verifica firele");
    }

    // TWAI — independent de WiFi
    g_can_ok = twaiStart();
    if (!g_can_ok) {
        Serial.println("[TWAI] init FAILED — continui doar cu senzori");
    }

    // WiFi NON-BLOCKING (poate dura zeci de sec pana HU porneste si propaga retea)
    // Sensors + CAN + Serial deja emit din primul loop
    wifiStaBeginAsync();
    // tcpServer + mDNS pornesc in netServicesUp() cand WiFi devine ready
}

void loop() {
    ledService();

    // === USB JSON command parser (F16 → ESP32) ===
    usb_proto_service();

    // === ISO-TP timeout cleanup (sesiuni abandonate) ===
    iso_tp_tick(millis());

    // === WiFi state machine (non-blocking) ===
    wifiServiceTick();

    // === UDP discovery broadcast (doar daca services up) ===
    if (g_net_services_up && (millis() - lastUdpBcMs) >= UDP_BC_INTERVAL_MS) {
        lastUdpBcMs = millis();
        sendUdpDiscovery();
    }

    // === TCP accept / drop (doar daca services up) ===
    if (g_net_services_up) {
        if (tcpServer.hasClient()) {
            WiFiClient neu = tcpServer.available();
            if (tcpClient && tcpClient.connected()) {
                neu.stop();   // refuza al doilea
            } else {
                tcpClient = neu;
                tcpClient.setNoDelay(true);
                Serial.printf("[TCP] client connect %s\n",
                              tcpClient.remoteIP().toString().c_str());
                char hello[JSON_BUF_SZ];
                int n = snprintf(hello, sizeof(hello),
                    "{\"ts\":%lu,\"type\":\"hello\",\"app\":\"ergCAN\",\"bitrate\":%u,\"port\":%d,\"can\":%d}\n",
                    (unsigned long)millis(), CAN_BITRATE, TCP_PORT, g_can_ok ? 1 : 0);
                if (n > 0) tcpClient.write((const uint8_t *)hello, n);
            }
        }
        if (tcpClient && !tcpClient.connected()) {
            Serial.println("[TCP] client disconnect");
            tcpClient.stop();
        }
        if (tcpClient && tcpClient.connected()) {
            while (tcpClient.available()) tcpClient.read();   // drain input
        }
    }

    // === TWAI alerts ===
    if (g_can_ok) {
        uint32_t alerts = 0;
        if (twai_read_alerts(&alerts, 0) == ESP_OK && alerts) {
            if (alerts & TWAI_ALERT_BUS_OFF) {
                // Bus absent / nimic conectat → NU reboot. Sensors + WiFi raman live.
                Serial.println("[TWAI] BUS_OFF -> auto-recovery (sensors continua)");
                g_can_bus_ok = false;
                // Flush TX queue (mesaje neacknowledged) si initiere recovery
                twai_clear_transmit_queue();
                twai_initiate_recovery();
                // Notify F16 via JSON event
                char buf[128];
                int n = snprintf(buf, sizeof(buf),
                    "{\"ts\":%lu,\"type\":\"buststate\",\"ok\":0,\"reason\":\"bus_off\"}\n",
                    (unsigned long)millis());
                if (n > 0) emitJson(buf, (size_t)n);
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                Serial.println("[TWAI] bus recovered -> restart driver");
                twai_start();
                g_can_bus_ok = true;
                char buf[128];
                int n = snprintf(buf, sizeof(buf),
                    "{\"ts\":%lu,\"type\":\"buststate\",\"ok\":1,\"reason\":\"recovered\"}\n",
                    (unsigned long)millis());
                if (n > 0) emitJson(buf, (size_t)n);
            }
            if (alerts & TWAI_ALERT_BUS_ERROR)     rxErr++;
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL) rxErr++;
            if (alerts & TWAI_ALERT_ERR_PASS) Serial.println("[TWAI] error-passive");
        }

        // === Receive frames (burst <=32 per loop) ===
        int burst = 0;
        while (burst < 32) {
            twai_message_t m;
            esp_err_t r = twai_receive(&m, 0);
            if (r == ESP_ERR_TIMEOUT) break;
            if (r != ESP_OK) { rxErr++; break; }

            rxTotal++;
            lastFrameMs = millis();
            ledPulse();

            char buf[JSON_BUF_SZ];
            size_t n = frameToJson(m, buf, sizeof(buf));
            if (n > 0) emitJson(buf, n);

            // ISO-TP layer: feed every frame to session manager (multi-ECU broadcast collection)
            // Frame-urile in afara range-ului 0x700-0x7FF nu vor crea sesiuni utile, dar
            // codul filtreaza intern prin PCI byte (orice frame care nu e SF/FF/CF e ignorat).
            iso_tp_on_frame(m.identifier, m.data, m.data_length_code);

            burst++;
        }

        // Drain ISO-TP sesiuni complete (OBD responses) → emit JSON per ECU
        iso_tp_drain(obd_process_response);
    }

    uint32_t now = millis();

    // === Sensors @ 10Hz ===
    if ((now - lastSensorMs) >= SENSOR_SAMPLE_MS) {
        lastSensorMs = now;
        SensorData sd;
        sensors_read(sd);
        char buf[JSON_BUF_SZ];
        size_t n = sensors_to_json(sd, buf, sizeof(buf));
        if (n > 0) emitJson(buf, n);
    }

    // === Heartbeat ===
    if ((now - lastHbMs) >= HB_INTERVAL_MS &&
        (now - lastFrameMs) >= HB_INTERVAL_MS) {
        sendHeartbeat();
        lastHbMs = now;
    }

    // === Stats Serial ===
    if ((now - lastStatsMs) >= STATS_INTERVAL_MS) {
        lastStatsMs = now;
        // Diagnoza I2C (scan + dump 0x2C) repetata la fiecare 10s — vizibila si daca Serial
        // Monitor a fost deschis dupa boot ESP32 (cand pierzi mesajele de init).
        sensors_debug_print();
        int rssi = (g_wifi_state == WIFI_STA_OK) ? WiFi.RSSI() : 0;
        int tcpOk = (tcpClient && tcpClient.connected()) ? 1 : 0;
        if (g_can_ok) {
            twai_status_info_t st;
            if (twai_get_status_info(&st) == ESP_OK) {
                Serial.printf("[STAT] rx=%lu err=%lu wifi=%s tcp=%d state=%d tec=%u rec=%u rxQ=%u rssi=%d\n",
                    (unsigned long)rxTotal, (unsigned long)rxErr,
                    wifiStateName(), tcpOk,
                    (int)st.state, (unsigned)st.tx_error_counter,
                    (unsigned)st.rx_error_counter, (unsigned)st.msgs_to_rx, rssi);
            }
        } else {
            Serial.printf("[STAT] CAN=off wifi=%s tcp=%d rssi=%d\n",
                wifiStateName(), tcpOk, rssi);
        }
    }

    delay(1);   // yield
}
