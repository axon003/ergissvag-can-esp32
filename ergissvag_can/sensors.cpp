/**
 * sensors.cpp — BMP280 + GY-85 reader implementation
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.4
 * @changes
 *   v1.4 2026-09-24 — fw 2.5.13: calibrare magnetometru. In masina campul propriu (hard-iron) domina: mag_hdg varia doar
 *                     6..34 grd cand GPS acoperea 0..360. Corectie offset + scara X/Y + orientare montaj (mag.hofs, mag.inv),
 *                     salvate in cfg; colectare AUTOMATA cand nu exista calibrare (min/max pe semnal filtrat EMA, salvare cand
 *                     span X si Y >= MAGCAL_AUTO_SPAN_UT); comanda MAGCAL; eveniment in jurnal la salvare.
 *   v1.3 2026-09-24 — fw 2.5.12: magnetometru QMC5883P @0x2C (varianta noua pe clonele GY-85; ID reg 0x00 = 0x80).
 *                     Adresa 0x2C aparea la scan din mai, dar nu era recunoscuta => MAG=MISSING.
 *   v1.0 2026-05-23 — creat initial
 *   v1.2 2026-05-23 — i2c_scan_print() printeaza la boot toate adresele I2C prezente
 *                     (diagnoza pt magnetometru "none" — daca apar adrese neasteptate,
 *                     senzorul poate fi clona cu adresa diferita ex AK8963=0x0C / IST8310=0x0E).
 *   v1.1 2026-05-23 — magnetometru detect HMC5883L (0x1E) + QMC5883L (0x0D) cu init manual I2C
 *                     (nu Adafruit_HMC5883 — esueaza tacit pe clone returnand 0). Tip chip
 *                     raportat in JSON ca "chip":"HMC|QMC|none". Conversie LSB→µT corecta
 *                     per chip (HMC 1.3G=10.9 LSB/µT, QMC 2G=12 LSB/µT).
 */
#include "sensors.h"
#include "config.h"
#include "cfg.h"     // v1.4: calibrare magnetometru in NVS
#include "proto.h"   // v1.4: proto_event (jurnal automat)
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>

static Adafruit_BMP280            bmp;
static Adafruit_ADXL345_Unified   accel(12345);

static bool g_bmp_present   = false;
static bool g_accel_present = false;
static bool g_gyro_present  = false;
static bool g_mag_present   = false;

// Magnetometru — tip chip detectat
enum MagChip : uint8_t { MAG_NONE = 0, MAG_HMC = 1, MAG_QMC = 2, MAG_QMCP = 3 };
static MagChip g_mag_chip = MAG_NONE;
#define HMC5883L_ADDR  0x1E
#define QMC5883L_ADDR  0x0D
#define QMC5883P_ADDR  0x2C   // v1.3

// ----- I2C helpers generice -----

static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    size_t got = Wire.requestFrom((int)addr, (int)len, (int)true);
    if (got != len) return false;
    for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
    return true;
}

static bool i2c_probe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// ----- HMC5883L (Honeywell original) — init manual -----

static bool hmc_init() {
    if (!i2c_probe(HMC5883L_ADDR)) return false;
    // Verifica ID: reg 0x0A,0x0B,0x0C = 'H','4','3' = 0x48,0x34,0x33
    uint8_t id[3];
    if (!i2c_read_regs(HMC5883L_ADDR, 0x0A, id, 3)) return false;
    if (id[0] != 'H' || id[1] != '4' || id[2] != '3') {
        Serial.printf("[I2C] 0x1E raspunde dar ID = %02X %02X %02X (NU HMC5883L)\n", id[0], id[1], id[2]);
        return false;
    }
    // ConfigA (0x00): 8 samples avg, 15 Hz, normal
    if (!i2c_write_reg(HMC5883L_ADDR, 0x00, 0x70)) return false;
    // ConfigB (0x01): gain 1.3 Ga (default; 1090 LSB/Gauss = 10.9 LSB/µT)
    if (!i2c_write_reg(HMC5883L_ADDR, 0x01, 0x20)) return false;
    // Mode (0x02): continuous measurement
    if (!i2c_write_reg(HMC5883L_ADDR, 0x02, 0x00)) return false;
    return true;
}

// HMC data: 0x03..0x08 = X H, X L, Z H, Z L, Y H, Y L (X-Z-Y order!)
static bool hmc_read(float &mx, float &my, float &mz) {
    uint8_t b[6];
    if (!i2c_read_regs(HMC5883L_ADDR, 0x03, b, 6)) return false;
    int16_t rx = (int16_t)((b[0] << 8) | b[1]);
    int16_t rz = (int16_t)((b[2] << 8) | b[3]);
    int16_t ry = (int16_t)((b[4] << 8) | b[5]);
    // Saturated reading sentinel = -4096
    if (rx == -4096 || ry == -4096 || rz == -4096) return false;
    const float LSB_PER_UT = 10.9f;  // 1.3 Ga range
    mx = (float)rx / LSB_PER_UT;
    my = (float)ry / LSB_PER_UT;
    mz = (float)rz / LSB_PER_UT;
    return true;
}

// ----- QMC5883L (QST clone — frecvent in GY-85/GY-271 noi) — init manual -----

static bool qmc_init() {
    if (!i2c_probe(QMC5883L_ADDR)) return false;
    // Soft reset (control 2 reg 0x0A bit 7)
    i2c_write_reg(QMC5883L_ADDR, 0x0A, 0x80);
    delay(10);
    // SET/RESET period (recomandat 0x01)
    if (!i2c_write_reg(QMC5883L_ADDR, 0x0B, 0x01)) return false;
    // Control 1 (0x09): OSR=512 (00), RNG=2G (00), ODR=50Hz (10), MODE=continuous (01) → 0b00001001 = 0x09
    if (!i2c_write_reg(QMC5883L_ADDR, 0x09, 0x09)) return false;
    return true;
}

// QMC data: 0x00..0x05 = X L, X H, Y L, Y H, Z L, Z H (LSB first!)
static bool qmc_read(float &mx, float &my, float &mz) {
    uint8_t b[6];
    if (!i2c_read_regs(QMC5883L_ADDR, 0x00, b, 6)) return false;
    int16_t rx = (int16_t)((b[1] << 8) | b[0]);
    int16_t ry = (int16_t)((b[3] << 8) | b[2]);
    int16_t rz = (int16_t)((b[5] << 8) | b[4]);
    const float LSB_PER_UT = 12.0f;  // 2 Ga range: 12000 LSB/Gauss
    mx = (float)rx / LSB_PER_UT;
    my = (float)ry / LSB_PER_UT;
    mz = (float)rz / LSB_PER_UT;
    return true;
}

// ----- v1.3: QMC5883P (QST, adresa fixa 0x2C) — alt set de registre decat QMC5883L -----
// ID 0x00 = 0x80; date 0x01..0x06 = X L,X H,Y L,Y H,Z L,Z H; 0x0A control 1, 0x0B control 2, 0x29 semn axe.
static bool qmcp_init() {
    if (!i2c_probe(QMC5883P_ADDR)) return false;
    uint8_t id = 0;
    if (!i2c_read_regs(QMC5883P_ADDR, 0x00, &id, 1)) return false;
    if (id != 0x80) { Serial.printf("[I2C] 0x2C raspunde dar ID = %02X (NU QMC5883P, astept 80)\n", id); return false; }
    if (!i2c_write_reg(QMC5883P_ADDR, 0x29, 0x06)) return false;   // semnul axelor (secventa din datasheet)
    if (!i2c_write_reg(QMC5883P_ADDR, 0x0B, 0x08)) return false;   // RNG 8 G, set/reset pornit
    if (!i2c_write_reg(QMC5883P_ADDR, 0x0A, 0xCD)) return false;   // mod normal (continuu), ODR 200 Hz, OSR
    delay(10);
    return true;
}
static bool qmcp_read(float &mx, float &my, float &mz) {
    uint8_t b[6];
    if (!i2c_read_regs(QMC5883P_ADDR, 0x01, b, 6)) return false;
    int16_t rx = (int16_t)((b[1] << 8) | b[0]);
    int16_t ry = (int16_t)((b[3] << 8) | b[2]);
    int16_t rz = (int16_t)((b[5] << 8) | b[4]);
    const float LSB_PER_UT = 37.5f;  // 8 G: 3750 LSB/Gauss
    mx = (float)rx / LSB_PER_UT;
    my = (float)ry / LSB_PER_UT;
    mz = (float)rz / LSB_PER_UT;
    return true;
}

// ----- v1.4: calibrare magnetometru -----
static bool     g_cal_loaded = false;
static bool     g_cal_ok = false;                 // exista calibrare salvata (mag.cal=1)
static float    g_ox = 0, g_oy = 0, g_oz = 0;     // offset hard-iron (uT)
static float    g_sx = 1, g_sy = 1;               // scara soft-iron simpla (egalizeaza X/Y)
static float    g_hofs = 0;                       // orientare montaj (grade adunate la heading)
static bool     g_inv = false;                    // sens invers (senzor montat „cu fata in jos” fata de conventie)
static bool     g_col = false, g_col_auto = false;
static uint32_t g_col_n = 0;
static float    g_fx, g_fy, g_fz;                 // filtru EMA — taie varfurile scurte (tramvai, masina vecina)
static float    g_mnx, g_mxx, g_mny, g_mxy, g_mnz, g_mxz;
static bool     g_last_ok = false;
static float    g_lx, g_ly;                       // ultima citire bruta (pt MAGCAL HDG)

static void col_start(bool automat) { g_col = true; g_col_auto = automat; g_col_n = 0; }

static void cal_load() {
    g_cal_loaded = true;
    g_cal_ok = cfg_int("mag.cal") == 1;
    g_ox = cfg_int("mag.ox") / 10.0f; g_oy = cfg_int("mag.oy") / 10.0f; g_oz = cfg_int("mag.oz") / 10.0f;
    g_sx = cfg_int("mag.sx") / 1000.0f; g_sy = cfg_int("mag.sy") / 1000.0f;
    if (g_sx <= 0) g_sx = 1; if (g_sy <= 0) g_sy = 1;
    g_hofs = cfg_int("mag.hofs") / 10.0f;
    g_inv = cfg_int("mag.inv") == 1;
    if (!g_cal_ok) col_start(true);
}

static void col_add(float x, float y, float z) {
    if (fabsf(x) > MAGCAL_RAW_MAX_UT || fabsf(y) > MAGCAL_RAW_MAX_UT || fabsf(z) > MAGCAL_RAW_MAX_UT) return;
    if (g_col_n == 0) {
        g_fx = x; g_fy = y; g_fz = z;
        g_mnx = g_mxx = x; g_mny = g_mxy = y; g_mnz = g_mxz = z;
    } else {
        g_fx += MAGCAL_EMA * (x - g_fx); g_fy += MAGCAL_EMA * (y - g_fy); g_fz += MAGCAL_EMA * (z - g_fz);
        if (g_col_n >= MAGCAL_WARMUP) {   // primele esantioane doar incarca filtrul
            g_mnx = fminf(g_mnx, g_fx); g_mxx = fmaxf(g_mxx, g_fx);
            g_mny = fminf(g_mny, g_fy); g_mxy = fmaxf(g_mxy, g_fy);
            g_mnz = fminf(g_mnz, g_fz); g_mxz = fmaxf(g_mxz, g_fz);
        } else {
            g_mnx = g_mxx = g_fx; g_mny = g_mxy = g_fy; g_mnz = g_mxz = g_fz;
        }
    }
    g_col_n++;
}

static bool save_int(const char *k, long v, String &err) {
    char b[16]; snprintf(b, sizeof(b), "%ld", v);
    return cfg_set(k, b, err);
}

// calculeaza + salveaza; minSpan = pragul de acoperire pe X si Y
static bool col_finish(float minSpan, String &msg) {
    float spx = g_mxx - g_mnx, spy = g_mxy - g_mny;
    if (g_col_n < MAGCAL_MIN_SAMPLES || spx < minSpan || spy < minSpan) {
        char b[160];
        snprintf(b, sizeof(b), "MAGCAL: acoperire insuficienta (esantioane %lu/%d, span X %.1f Y %.1f uT, prag %.0f) — mai fa viraje",
                 (unsigned long)g_col_n, MAGCAL_MIN_SAMPLES, spx, spy, minSpan);
        msg = b; return false;
    }
    float ox = (g_mxx + g_mnx) / 2, oy = (g_mxy + g_mny) / 2, oz = (g_mxz + g_mnz) / 2;
    float avg = (spx + spy) / 2, sx = avg / spx, sy = avg / spy;
    String err;
    if (!save_int("mag.ox", lroundf(ox * 10), err) || !save_int("mag.oy", lroundf(oy * 10), err) ||
        !save_int("mag.oz", lroundf(oz * 10), err) || !save_int("mag.sx", lroundf(sx * 1000), err) ||
        !save_int("mag.sy", lroundf(sy * 1000), err) || !save_int("mag.cal", 1, err)) {
        msg = "MAGCAL: salvare ESUATA: " + err; return false;
    }
    g_ox = ox; g_oy = oy; g_oz = oz; g_sx = sx; g_sy = sy; g_cal_ok = true; g_col = false;
    char b[200];
    snprintf(b, sizeof(b), "MAGCAL salvat: offset X %.1f Y %.1f Z %.1f uT, scara X %.3f Y %.3f, span X %.1f Y %.1f uT, %lu esantioane",
             ox, oy, oz, sx, sy, spx, spy, (unsigned long)g_col_n);
    msg = b; return true;
}

static float norm360(float h) { while (h < 0) h += 360.0f; while (h >= 360.0f) h -= 360.0f; return h; }

// heading fara mag.hofs (pt MAGCAL HDG) si cu el
static float heading_base(float x, float y) {
    float cx = x, cy = y;
    if (g_cal_ok) { cx = (x - g_ox) * g_sx; cy = (y - g_oy) * g_sy; }
    float h = atan2f(cy, cx) * 180.0f / (float)M_PI;
    if (g_inv) h = -h;
    return norm360(h);
}

String sensors_magcal_cmd(const char *sub, const char *arg) {
    if (!g_cal_loaded) cal_load();
    if (!g_mag_present) return "MAGCAL: magnetometru absent";
    String msg;
    if (sub && !strcasecmp(sub, "START")) {
        col_start(false);
        return "MAGCAL: colectare pornita — viraje in toate directiile (un cerc complet), apoi MAGCAL STOP";
    }
    if (sub && !strcasecmp(sub, "STOP")) {
        if (!g_col) return "MAGCAL: nu colectez (MAGCAL START intai)";
        if (col_finish(MAGCAL_MIN_SPAN_UT, msg)) proto_event("%s (manual)", msg.c_str());
        return msg;
    }
    if (sub && !strcasecmp(sub, "RESET")) {
        const char *keys[] = {"mag.cal", "mag.ox", "mag.oy", "mag.oz", "mag.sx", "mag.sy", "mag.hofs", "mag.inv"};
        for (const char *k : keys) cfg_reset_key(k);
        cal_load();
        return "MAGCAL: calibrare stearsa; colectare automata pornita (se salveaza singura dupa un cerc complet)";
    }
    if (sub && !strcasecmp(sub, "HDG")) {
        if (!arg) return "MAGCAL HDG <grade_reale 0..359>  (directia reala a masinii acum, ex. din GPS in mers)";
        if (!g_last_ok) return "MAGCAL: nu am citire magnetometru";
        float real = atof(arg);
        float ofs = real - heading_base(g_lx, g_ly);
        while (ofs > 180) ofs -= 360; while (ofs < -180) ofs += 360;
        String err;
        if (!save_int("mag.hofs", lroundf(ofs * 10), err)) return "MAGCAL: salvare ESUATA: " + err;
        g_hofs = ofs;
        char b[96]; snprintf(b, sizeof(b), "MAGCAL: orientare montaj %.1f grd salvata", ofs); return b;
    }
    if (sub && !strcasecmp(sub, "INV")) {
        if (!arg || (strcmp(arg, "0") && strcmp(arg, "1"))) return "MAGCAL INV 0|1";
        String err;
        if (!save_int("mag.inv", atoi(arg), err)) return "MAGCAL: salvare ESUATA: " + err;
        g_inv = atoi(arg) == 1;
        return g_inv ? "MAGCAL: sens inversat" : "MAGCAL: sens normal";
    }
    if (sub && *sub) return "MAGCAL [START|STOP|RESET|HDG <grade>|INV 0|1]";
    char b[420];
    int n = snprintf(b, sizeof(b), "MAGCAL: calibrare %s | offset X %.1f Y %.1f Z %.1f uT | scara X %.3f Y %.3f | montaj %.1f grd%s\r\n",
                     g_cal_ok ? "SALVATA" : "LIPSA", g_ox, g_oy, g_oz, g_sx, g_sy, g_hofs, g_inv ? " INV" : "");
    if (g_last_ok)
        n += snprintf(b + n, sizeof(b) - n, "  brut X %.1f Y %.1f uT -> heading %.1f grd\r\n", g_lx, g_ly, norm360(heading_base(g_lx, g_ly) + g_hofs));
    if (g_col)
        snprintf(b + n, sizeof(b) - n, "  colectare %s: %lu esantioane, span X %.1f Y %.1f Z %.1f uT (salvare automata la %.0f)",
                 g_col_auto ? "AUTOMATA" : "manuala", (unsigned long)g_col_n,
                 g_col_n ? g_mxx - g_mnx : 0.0f, g_col_n ? g_mxy - g_mny : 0.0f, g_col_n ? g_mxz - g_mnz : 0.0f, MAGCAL_AUTO_SPAN_UT);
    return String(b);
}

String sensors_mag_json(const SensorData &s) {
    if (!g_mag_present) return "";
    char b[200];
    int n = snprintf(b, sizeof(b), ",\"mag\":{\"ok\":%d,\"x\":%.1f,\"y\":%.1f,\"z\":%.1f,\"h\":%.1f,\"cal\":%d,\"col\":%d",
                     s.mag_ok ? 1 : 0, s.mag_x_ut, s.mag_y_ut, s.mag_z_ut, s.mag_heading_deg, g_cal_ok ? 1 : 0, g_col ? 1 : 0);
    if (g_col && g_col_n)
        n += snprintf(b + n, sizeof(b) - n, ",\"n\":%lu,\"spx\":%.1f,\"spy\":%.1f", (unsigned long)g_col_n, g_mxx - g_mnx, g_mxy - g_mny);
    snprintf(b + n, sizeof(b) - n, "}");
    return String(b);
}

// ----- ITG3200 raw I2C helpers -----

static bool itg_write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ITG3200_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool itg_read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(ITG3200_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    size_t got = Wire.requestFrom((int)ITG3200_I2C_ADDR, (int)len, (int)true);
    if (got != len) return false;
    for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
    return true;
}

static bool itg_init() {
    // WHO_AM_I (0x00) trebuie sa fie 0x68 (sau adresa I2C - bit 0)
    uint8_t who;
    if (!itg_read_regs(0x00, &who, 1)) return false;
    if ((who & 0x7E) != 0x68) return false;

    // Sample Rate Divider (0x15) = 7 -> 1kHz / 8 = 125Hz
    if (!itg_write_reg(0x15, 0x07)) return false;
    // DLPF (0x16): FS_SEL=3 (±2000°/s full scale), DLPF_CFG=3 (BW 42Hz, internal 1kHz)
    if (!itg_write_reg(0x16, 0x1B)) return false;
    // Power: clock source = X-gyro reference
    if (!itg_write_reg(0x3E, 0x01)) return false;
    return true;
}

static bool itg_read(float &gx, float &gy, float &gz, float &gt) {
    uint8_t b[8];
    // 0x1B = TEMP_OUT_H, then GYRO_X H/L, Y H/L, Z H/L (8 bytes total)
    if (!itg_read_regs(0x1B, b, 8)) return false;
    int16_t rawT  = (int16_t)((b[0] << 8) | b[1]);
    int16_t rawGX = (int16_t)((b[2] << 8) | b[3]);
    int16_t rawGY = (int16_t)((b[4] << 8) | b[5]);
    int16_t rawGZ = (int16_t)((b[6] << 8) | b[7]);

    // Datasheet ITG3200: temp = 35 + (rawT + 13200) / 280   [°C]
    gt = 35.0f + ((float)rawT + 13200.0f) / 280.0f;
    // Sensibilitate ±2000°/s -> 14.375 LSB/(°/s)
    const float SENS = 14.375f;
    gx = (float)rawGX / SENS;
    gy = (float)rawGY / SENS;
    gz = (float)rawGZ / SENS;
    return true;
}

// ----- Public API -----

// Scan I2C bus + print toate adresele care raspund. Util pentru diagnoza cand chip-uri nu sunt
// detectate la adresele asteptate (clone cu adrese diferite, AK8963=0x0C, IST8310=0x0E, etc.).
static void i2c_scan_print() {
    Serial.print("[I2C] scan: ");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("0x%02X ", addr);
            found++;
        }
    }
    Serial.printf("(%d devices)\n", found);
}

// Dump primii N registers de la o adresa I2C — util sa identificam ce chip raspunde
// la o adresa neasteptata (compari pattern cu datasheet-uri cunoscute).
static void i2c_dump(uint8_t addr, uint8_t startReg, uint8_t count) {
    uint8_t buf[32];
    if (count > 32) count = 32;
    if (i2c_read_regs(addr, startReg, buf, count)) {
        Serial.printf("[I2C] 0x%02X regs 0x%02X..0x%02X:", addr, startReg, startReg + count - 1);
        for (uint8_t i = 0; i < count; i++) Serial.printf(" %02X", buf[i]);
        Serial.println();
    } else {
        Serial.printf("[I2C] 0x%02X dump FAILED (no response to read)\n", addr);
    }
}

// Test scriere-citire pe registru = verifica daca e device real sau ghost ACK.
// Daca scrie 0xAA pe reg si la citire returneaza 0xAA → device real; altfel ghost / read-only / chip diferit.
static bool i2c_write_read_test(uint8_t addr, uint8_t reg) {
    if (!i2c_write_reg(addr, reg, 0xAA)) return false;
    uint8_t v = 0;
    if (!i2c_read_regs(addr, reg, &v, 1)) return false;
    Serial.printf("[I2C] 0x%02X write 0xAA reg 0x%02X → read back 0x%02X %s\n",
                  addr, reg, v, (v == 0xAA) ? "(REAL writable)" : "(ghost or read-only/different value)");
    return v == 0xAA;
}

// Publica — apelata din loop() la fiecare ciclu STAT (10s).
// Magnetometru absent confirmat (forced-read pe 0x0C-0x1F toate FAIL).
// Focus acum: identificare chip 0x2C — dump extins reg 0x00..0x3F + comparatie 2 citiri
// la 500ms diferenta. Daca valorile sunt identice → chip pasiv (EEPROM/OTP). Daca difera
// → chip activ (sensor sau contor).
// Print status sumar pe Serial — apelat in init (boot) si in STAT (10s).
static void print_sensor_status() {
    Serial.printf("[SENSORS] BMP280=%s ADXL345=%s ITG3200=%s MAG=%s\n",
        g_bmp_present   ? "OK" : "MISSING",
        g_accel_present ? "OK" : "MISSING",
        g_gyro_present  ? "OK" : "MISSING",
        g_mag_chip == MAG_HMC ? "HMC@0x1E" :
        g_mag_chip == MAG_QMC ? "QMC@0x0D" :
        g_mag_chip == MAG_QMCP ? "QMC5883P@0x2C" : "MISSING");
}

// Publica — apelata din loop() la fiecare ciclu STAT (10s). Scan I2C + sumar status senzori.
// Asa user vede status si daca Serial Monitor s-a deschis dupa boot (pierde primele linii).
void sensors_debug_print() {
    i2c_scan_print();
    print_sensor_status();
}

bool sensors_init() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQ_HZ);

    // Scan complet bus la boot — vad ce e prezent
    i2c_scan_print();

    // BMP280
#if DISABLE_BMP280
    Serial.println("[I2C] BMP280  DISABLED (debug — test magnetometer conflict)");
    g_bmp_present = false;
#else
    g_bmp_present = bmp.begin(BMP280_I2C_ADDR);
    if (g_bmp_present) {
        bmp.setSampling(
            Adafruit_BMP280::MODE_NORMAL,
            Adafruit_BMP280::SAMPLING_X2,    // temp
            Adafruit_BMP280::SAMPLING_X16,   // press
            Adafruit_BMP280::FILTER_X16,
            Adafruit_BMP280::STANDBY_MS_125);
        Serial.printf("[I2C] BMP280  @0x%02X OK\n", BMP280_I2C_ADDR);
    } else {
        Serial.printf("[I2C] BMP280  @0x%02X MISSING\n", BMP280_I2C_ADDR);
    }
#endif

    // ADXL345
    g_accel_present = accel.begin(ADXL345_I2C_ADDR);
    if (g_accel_present) {
        accel.setRange(ADXL345_RANGE_4_G);
        Serial.printf("[I2C] ADXL345 @0x%02X OK (±4g)\n", ADXL345_I2C_ADDR);
    } else {
        Serial.printf("[I2C] ADXL345 @0x%02X MISSING\n", ADXL345_I2C_ADDR);
    }

    // ITG3200
    g_gyro_present = itg_init();
    Serial.printf("[I2C] ITG3200 @0x%02X %s (±2000°/s)\n", ITG3200_I2C_ADDR,
                  g_gyro_present ? "OK" : "MISSING");

    // Magnetometru — incearca HMC5883L original la 0x1E intai, apoi QMC5883L la 0x0D
    if (hmc_init()) {
        g_mag_chip = MAG_HMC;
        g_mag_present = true;
        Serial.printf("[I2C] HMC5883L @0x%02X OK (Honeywell original, 1.3 Ga, 10.9 LSB/uT)\n", HMC5883L_ADDR);
    } else if (qmc_init()) {
        g_mag_chip = MAG_QMC;
        g_mag_present = true;
        Serial.printf("[I2C] QMC5883L @0x%02X OK (QST clone, 2 Ga, 12 LSB/uT)\n", QMC5883L_ADDR);
    } else if (qmcp_init()) {                                   // v1.3
        g_mag_chip = MAG_QMCP;
        g_mag_present = true;
        Serial.printf("[I2C] QMC5883P @0x%02X OK (QST, 8 G, 37.5 LSB/uT)\n", QMC5883P_ADDR);
    } else {
        g_mag_chip = MAG_NONE;
        g_mag_present = false;
        Serial.println("[I2C] Magnetometru MISSING (nici HMC @0x1E, nici QMC @0x0D, nici QMC5883P @0x2C)");
    }

    // Sumar status init — repetabil identic la STAT 10s (vezi sensors_debug_print)
    print_sensor_status();

    return g_bmp_present || g_accel_present || g_gyro_present || g_mag_present;
}

void sensors_read(SensorData &s) {
    memset(&s, 0, sizeof(s));

    // BMP280
    if (g_bmp_present) {
        s.bmp_temp_c     = bmp.readTemperature();
        s.bmp_press_hpa  = bmp.readPressure() / 100.0f;
        s.bmp_alt_m      = bmp.readAltitude(1013.25f);
        s.bmp_ok         = !isnan(s.bmp_temp_c) && !isnan(s.bmp_press_hpa);
    }

    // ADXL345
    if (g_accel_present) {
        sensors_event_t e;
        if (accel.getEvent(&e)) {
            s.accel_x_g = e.acceleration.x / 9.80665f;
            s.accel_y_g = e.acceleration.y / 9.80665f;
            s.accel_z_g = e.acceleration.z / 9.80665f;
            s.accel_ok = true;
        }
    }

    // ITG3200
    if (g_gyro_present) {
        s.gyro_ok = itg_read(s.gyro_x_dps, s.gyro_y_dps, s.gyro_z_dps, s.gyro_temp_c);
    }

    // Magnetometru — read prin functie potrivita pe chip detectat
    if (g_mag_present) {
        bool ok = false;
        switch (g_mag_chip) {
            case MAG_HMC: ok = hmc_read(s.mag_x_ut, s.mag_y_ut, s.mag_z_ut); break;
            case MAG_QMC: ok = qmc_read(s.mag_x_ut, s.mag_y_ut, s.mag_z_ut); break;
            case MAG_QMCP: ok = qmcp_read(s.mag_x_ut, s.mag_y_ut, s.mag_z_ut); break;   // v1.3
            default: break;
        }
        if (ok) {
            // v1.4: calibrare — colectare (automata daca lipseste) + heading din X/Y corectate
            if (!g_cal_loaded) cal_load();
            g_lx = s.mag_x_ut; g_ly = s.mag_y_ut; g_last_ok = true;
            if (g_col) {
                col_add(s.mag_x_ut, s.mag_y_ut, s.mag_z_ut);
                if (g_col_auto && g_col_n >= MAGCAL_MIN_SAMPLES &&
                    g_mxx - g_mnx >= MAGCAL_AUTO_SPAN_UT && g_mxy - g_mny >= MAGCAL_AUTO_SPAN_UT) {
                    String msg;
                    if (col_finish(MAGCAL_AUTO_SPAN_UT, msg)) proto_event("%s (automat)", msg.c_str());
                    else { proto_event("%s", msg.c_str()); g_col = false; }   // salvare esuata: raportat, nu reincerc la 10 Hz
                    Serial.println(msg);
                }
            }
            s.mag_heading_deg = norm360(heading_base(s.mag_x_ut, s.mag_y_ut) + g_hofs);
            s.mag_cal = g_cal_ok;
            s.mag_ok = true;
        }
    }
}

// Helper folosit de sensors_to_json sa raporteze tipul chip-ului magnetic detectat.
static const char* mag_chip_str() {
    switch (g_mag_chip) {
        case MAG_HMC: return "HMC";
        case MAG_QMC: return "QMC";
        case MAG_QMCP: return "QMC5883P";   // v1.3
        default:      return "none";
    }
}

size_t sensors_to_json(const SensorData &s, char *buf, size_t buf_sz) {
    int n = snprintf(buf, buf_sz,
        "{\"ts\":%lu,\"type\":\"sensors\","
        "\"bmp\":{\"ok\":%d,\"t\":%.2f,\"p\":%.2f,\"alt\":%.2f},"
        "\"acc\":{\"ok\":%d,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
        "\"gyr\":{\"ok\":%d,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"t\":%.1f},"
        "\"mag\":{\"ok\":%d,\"chip\":\"%s\",\"x\":%.1f,\"y\":%.1f,\"z\":%.1f,\"hdg\":%.1f}}\n",
        (unsigned long)millis(),
        s.bmp_ok ? 1 : 0, s.bmp_temp_c, s.bmp_press_hpa, s.bmp_alt_m,
        s.accel_ok ? 1 : 0, s.accel_x_g, s.accel_y_g, s.accel_z_g,
        s.gyro_ok ? 1 : 0, s.gyro_x_dps, s.gyro_y_dps, s.gyro_z_dps, s.gyro_temp_c,
        s.mag_ok ? 1 : 0, mag_chip_str(), s.mag_x_ut, s.mag_y_ut, s.mag_z_ut, s.mag_heading_deg);
    if (n < 0) return 0;
    return (size_t)n < buf_sz ? (size_t)n : buf_sz - 1;
}
