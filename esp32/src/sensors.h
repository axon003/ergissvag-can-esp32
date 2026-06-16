/**
 * sensors.h — BMP280 + GY-85 (ADXL345 + ITG3200 + HMC5883L) reader
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-05-23 — creat initial; modul I2C senzori pentru ergissvag_can ESP32
 */
#ifndef ERGCAN_SENSORS_H
#define ERGCAN_SENSORS_H

#include <Arduino.h>

struct SensorData {
    bool   bmp_ok;
    float  bmp_temp_c;       // °C
    float  bmp_press_hpa;    // hPa
    float  bmp_alt_m;        // m (altitudine relativa fata de sea-level standard)

    bool   accel_ok;
    float  accel_x_g;        // g (1g = 9.81 m/s²)
    float  accel_y_g;
    float  accel_z_g;

    bool   gyro_ok;
    float  gyro_x_dps;       // °/s
    float  gyro_y_dps;
    float  gyro_z_dps;
    float  gyro_temp_c;      // °C (din chip ITG3200)

    bool   mag_ok;
    float  mag_x_ut;         // µT
    float  mag_y_ut;
    float  mag_z_ut;
    float  mag_heading_deg;  // 0-360°, 0=N (calculat din X/Y, fara compensare tilt)
};

// Initializare I2C + senzori. Returneaza true daca cel putin un senzor a raspuns.
bool sensors_init();

// Citeste toti senzorii in struct. Campurile *_ok indica daca citirea a reusit.
void sensors_read(SensorData &out);

// Serializeaza datele in JSON intr-un buffer dat. Returneaza nr de octeti scrisi (fara terminator).
// Formatul include un \n la final pentru streaming line-delimited.
size_t sensors_to_json(const SensorData &s, char *buf, size_t buf_sz);

#endif // ERGCAN_SENSORS_H
