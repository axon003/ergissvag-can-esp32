/**
 * sniff.h — captura pasiva a traficului CAN in LittleFS + upload la final (ergCAN 2.0)
 *   SNIFF <label> [url] → mod listen, planificator oprit, fiecare cadru scris in /sniff.bin (record binar compact).
 *   SNIFF STOP → citeste fisierul, il trimite in loturi JSON la url (sau cloud.url BYOS ?type=can_log), revine la
 *   modul CAN de DINAINTE de SNIFF (daca placa era pe LISTEN ramane pe LISTEN; daca era pe normal revine pe normal).
 *   Ex. citire EEPROM BCM cu VAG CAN PRO: lasi SNIFF pornit cat ruleaza uneltul, apoi salvezi tot.
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.1
 * @changes
 *   v1.0 2026-09-15 — creat
 *   v1.1 2026-09-21 — SNIFF STOP nu mai forteaza "normal": revine la modul de dinainte (cerinta user: listen ramane listen).
 *                     sniff_post_json() public (folosit si de READEPROM pentru upload la BYOS).
 */
#ifndef ERGCAN_SNIFF_H
#define ERGCAN_SNIFF_H

#include <Arduino.h>

bool     sniff_begin(const char *label, const char *url, uint8_t owner);
void     sniff_frame(uint32_t id, bool ext, const uint8_t *d, uint8_t dlc, uint32_t ts);  // din bucla RX
bool     sniff_is_active();
uint32_t sniff_frames();
String   sniff_end_and_upload();   // salveaza + upload; intoarce rezumat text

// POST JSON (Bearer cloud.token, ISRG root pt ergiss.ro) — comun SNIFF / READEPROM
bool     sniff_post_json(const String &url, const String &body, String &err);
// cloud.url + "?type=<type>" (sau "&type=" daca url-ul are deja parametri)
String   sniff_cloud_url(const char *type);

#endif
