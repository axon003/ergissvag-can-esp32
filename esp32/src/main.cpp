/**
 * main.cpp — ergissvag_can ESP32 J533 CAN tap + sensors + OBD master firmware
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.4
 * @changes
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

// Trimite JSON la Serial (mereu) si la clientul TCP (doar daca services up + client conectat)
// NON-static — apelata si din obd_cmd.cpp + usb_proto.cpp
void emitJson(const char *line, size_t len) {
    Serial.write((const uint8_t *)line, len);
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

    g_net_services_up = true;
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
    case WIFI_AP_OK:
        // Nimic — AP ramane up; client se conecteaza direct
        break;
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
