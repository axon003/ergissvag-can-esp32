/**
 * config.h — ergissvag_can ESP32 firmware configuration
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.1
 * @changes
 *   v1.0 2026-04-22 — creat initial; constants firmware (bitrate, WiFi AP, TCP port, deep sleep)
 *   v1.1 2026-05-23 — mutat CAN de pe 21/22 pe 5/4 (I2C ocupa 21/22 acum pt BMP280+GY-85);
 *                     adaugat WiFi STA mode (YOUR_WIFI_SSID), I2C pini, sensor sample rate
 */
#ifndef ERGCAN_CONFIG_H
#define ERGCAN_CONFIG_H

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
#define I2C_FREQ_HZ        400000        // 400kHz safe pt fire scurte <20cm

// I2C adrese
#define BMP280_I2C_ADDR    0x76          // 0x77 daca SDO=VCC
#define ADXL345_I2C_ADDR   0x53          // GY-85 accelerometer
#define ITG3200_I2C_ADDR   0x68          // GY-85 gyroscope
#define HMC5883L_I2C_ADDR  0x1E          // GY-85 magnetometer

// Sample rate senzori
#define SENSOR_SAMPLE_MS   100           // citire la fiecare 100ms (10Hz)

// ===== WiFi STA (conectare la router) — preferinta principala =====
#define WIFI_STA_SSID      "YOUR_WIFI_SSID"
#define WIFI_STA_PASS      "YOUR_WIFI_PASSWORD"
#define WIFI_STA_TIMEOUT_S 20            // dupa atatea sec de tentativa esuata -> fallback AP

// ===== mDNS hostname (.local) — accesibil ca "ergisscan.local" in LAN =====
#define MDNS_HOSTNAME      "ergisscan"

// ===== WiFi AP fallback (daca STA esueaza) =====
#define WIFI_AP_SSID       "ergCAN_ESP32"
#define WIFI_AP_PASS       "ergcan2026"
#define WIFI_AP_CHANNEL    6
#define WIFI_AP_MAX_CONN   1

// ===== TCP server =====
#define TCP_PORT           31416          // 31415 e rezervat ErgissVAG BCAST; 31416 = ergCAN

// ===== Timing =====
#define DEEP_SLEEP_IDLE_S  300            // 5 min fara client -> deep sleep (era 30s; marit pt debug/test)
#define HB_INTERVAL_MS     5000           // heartbeat JSON daca fara frames
#define STATS_INTERVAL_MS  10000          // log statistici Serial
#define LED_BLINK_MS       20             // durata puls LED la RX frame

// ===== Buffers =====
#define TWAI_RX_QUEUE_LEN  64
#define TWAI_TX_QUEUE_LEN         0       // legacy (listen-only) — pastrat pt compat referinte
#define TWAI_TX_QUEUE_LEN_NORMAL  8       // pentru OBD/UDS query active in mode NORMAL
#define JSON_BUF_SZ        320            // marit pt sensor frame (mai multe fields)

#endif // ERGCAN_CONFIG_H
