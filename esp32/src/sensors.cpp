/**
 * sensors.cpp — BMP280 + GY-85 reader implementation
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-05-23 — creat initial
 */
#include "sensors.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>
#include <math.h>

static Adafruit_BMP280            bmp;
static Adafruit_ADXL345_Unified   accel(12345);
static Adafruit_HMC5883_Unified   mag(54321);

static bool g_bmp_present   = false;
static bool g_accel_present = false;
static bool g_gyro_present  = false;
static bool g_mag_present   = false;

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

bool sensors_init() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQ_HZ);

    // BMP280
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

    // HMC5883L
    g_mag_present = mag.begin();
    Serial.printf("[I2C] HMC5883 @0x%02X %s\n", HMC5883L_I2C_ADDR,
                  g_mag_present ? "OK" : "MISSING");

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

    // HMC5883L
    if (g_mag_present) {
        sensors_event_t e;
        if (mag.getEvent(&e)) {
            // Adafruit_HMC5883 returneaza µT direct (din Gauss * 100)
            s.mag_x_ut = e.magnetic.x;
            s.mag_y_ut = e.magnetic.y;
            s.mag_z_ut = e.magnetic.z;
            // Heading din atan2(Y, X), ajustat la 0..360
            float h = atan2f(e.magnetic.y, e.magnetic.x) * 180.0f / (float)M_PI;
            if (h < 0) h += 360.0f;
            s.mag_heading_deg = h;
            s.mag_ok = true;
        }
    }
}

size_t sensors_to_json(const SensorData &s, char *buf, size_t buf_sz) {
    int n = snprintf(buf, buf_sz,
        "{\"ts\":%lu,\"type\":\"sensors\","
        "\"bmp\":{\"ok\":%d,\"t\":%.2f,\"p\":%.2f,\"alt\":%.2f},"
        "\"acc\":{\"ok\":%d,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
        "\"gyr\":{\"ok\":%d,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"t\":%.1f},"
        "\"mag\":{\"ok\":%d,\"x\":%.1f,\"y\":%.1f,\"z\":%.1f,\"hdg\":%.1f}}\n",
        (unsigned long)millis(),
        s.bmp_ok ? 1 : 0, s.bmp_temp_c, s.bmp_press_hpa, s.bmp_alt_m,
        s.accel_ok ? 1 : 0, s.accel_x_g, s.accel_y_g, s.accel_z_g,
        s.gyro_ok ? 1 : 0, s.gyro_x_dps, s.gyro_y_dps, s.gyro_z_dps, s.gyro_temp_c,
        s.mag_ok ? 1 : 0, s.mag_x_ut, s.mag_y_ut, s.mag_z_ut, s.mag_heading_deg);
    if (n < 0) return 0;
    return (size_t)n < buf_sz ? (size_t)n : buf_sz - 1;
}
