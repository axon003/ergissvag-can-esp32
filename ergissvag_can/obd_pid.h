/**
 * obd_pid.h — PID-uri OBD-II mode 01 (SAE J1979): lungime raspuns + decodare in unitati fizice
 *   C++ portabil (fara Arduino) — compilat si in testele de pe PC (tests/).
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-09-15 — creat initial (ergCAN 2.0): tabel lungimi 0x00-0x60 pt parsare raspunsuri multi-PID
 */
#ifndef ERGCAN_OBD_PID_H
#define ERGCAN_OBD_PID_H

#include <cstdint>
#include <cstddef>

// Nr. octeti de date dupa PID in raspunsul mode 01 (41 PID A B ...). -1 = necunoscut.
int obd_pid_len(uint8_t pid);

// Decodare in unitate fizica. Intoarce false daca PID-ul nu are formula (se trimite doar raw).
bool obd_pid_decode(uint8_t pid, const uint8_t *d, size_t len, double *out, const char **unit);

// Bitmap PID-uri suportate (raspuns la 01 00 / 20 / 40 / 60): marcheaza in supported[256].
void obd_pid_bitmap(uint8_t base, const uint8_t *d, size_t len, bool *supported);

#endif
