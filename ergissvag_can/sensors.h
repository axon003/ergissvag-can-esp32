/**
 * sensors.h — BMP280 + GY-85 (ADXL345 + ITG3200 + HMC5883L) reader
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.1
 * @changes
 *   v1.0 2026-05-23 — creat initial; modul I2C senzori pentru ergissvag_can ESP32
 *   v1.1 2026-09-24 — fw 2.5.13: calibrare magnetometru (hard-iron + scara X/Y + orientare montaj), MAGCAL, mag in snapshot
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
    float  mag_x_ut;         // µT, BRUT (necorectat) — din el se face calibrarea
    float  mag_y_ut;
    float  mag_z_ut;
    float  mag_heading_deg;  // 0-360°, 0=N (din X/Y CORECTATE daca exista calibrare, fara compensare tilt)
    bool   mag_cal;          // v1.1: heading-ul foloseste calibrarea salvata
};

// Initializare I2C + senzori. Returneaza true daca cel putin un senzor a raspuns.
bool sensors_init();

// Citeste toti senzorii in struct. Campurile *_ok indica daca citirea a reusit.
void sensors_read(SensorData &out);

// Serializeaza datele in JSON intr-un buffer dat. Returneaza nr de octeti scrisi (fara terminator).
// Formatul include un \n la final pentru streaming line-delimited.
size_t sensors_to_json(const SensorData &s, char *buf, size_t buf_sz);

// Diagnoza I2C — scan complet bus + dump primii regs de la 0x2C (chip neidentificat).
// Apelata periodic din loop() ca sa fie vizibil in Serial Monitor chiar si dupa boot.
void sensors_debug_print();

// v1.1 — calibrare magnetometru. Masina are un camp propriu (hard-iron) mult mai mare decat componenta orizontala a
// campului Pamantului, deci fara corectie heading-ul oscileaza doar cateva zeci de grade. Corectie: x' = (x-ox)*sx,
// y' = (y-oy)*sy; heading = atan2(y',x') (sau -atan2 cu mag.inv) + mag.hofs. Valorile stau in cfg (NVS).
// Fara calibrare salvata, colectarea porneste SINGURA si se salveaza cand s-a acoperit un cerc (span X si Y >= prag).
// Comanda consola: MAGCAL [START|STOP|RESET|HDG <grade_reale>|INV 0|1]. Intoarce textul raspunsului.
String sensors_magcal_cmd(const char *sub, const char *arg);
// fragment JSON pt snapshot cloud: ,"mag":{"ok":1,"x":..,"y":..,"z":..,"h":..,"cal":1,"col":0,"sx":..,"sy":..}
String sensors_mag_json(const SensorData &s);

#endif // ERGCAN_SENSORS_H
