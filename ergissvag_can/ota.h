/**
 * ota.h — update firmware prin WiFi (ergCAN 2.0) cu revenire automata
 *   - ota_check: GET <ota.url>?fw=<ver>&dev=<mac> → {"fw":"2.0.1","url":"...ergcan_dl.php","md5":"...","size":N,"notes":"..."}
 *   - ota_update: HTTPS (ISRG Root X1), refuza raspuns fara Content-Length, verifica MD5, scrie in zona OTA libera
 *   - revenire: firmware-ul nou trebuie sa se confirme (ota_mark_ok, automat dupa OTA_OK_AFTER_MS de functionare);
 *     daca porneste de OTA_MAX_BOOTS ori fara confirmare, placa revine pe zona veche.
 *   Declansare doar manuala (decizie user: buton „ergCAN update” in F16 + consola OTA).
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-09-15 — creat initial
 */
#ifndef ERGCAN_OTA_H
#define ERGCAN_OTA_H

#include <Arduino.h>

#define OTA_MAX_BOOTS    3
#define OTA_OK_AFTER_MS  60000

struct OtaInfo {
    String latest, url, md5, notes;
    long   size;
    bool   newer;
};

typedef void (*ota_progress_cb)(size_t done, size_t total);

void   ota_boot_check();                     // devreme in setup(), inainte de orice altceva
void   ota_service(uint32_t now_ms);         // in loop(): confirma automat firmware-ul nou
void   ota_mark_ok();
bool   ota_check(OtaInfo &info, String &err);
bool   ota_cached_newer(OtaInfo &out);       // rezultat memorat de la ultimul ota_check (<5 min, newer)
bool   ota_update(const OtaInfo &info, String &err, ota_progress_cb progress);   // true = scris, cere restart
String ota_status_json();                    // zona curenta, urmatoarea, pending, boots, ultima eroare
int    ota_version_cmp(const char *a, const char *b);   // <0, 0, >0

#endif
