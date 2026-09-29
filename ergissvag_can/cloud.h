/**
 * cloud.h — cloud logging BYOS autonom (ergCAN posteaza singur, independent de F16)
 *   La pornire, daca cfg cloud.on=1, la fiecare cloud.int_s posteaza un snapshot JSON la cloud.url (?type=ergcan)
 *   cu Bearer cloud.token. O singura sesiune TLS per postare (ISRG Root X1). Independent de F16 (fix incident 11.07).
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-09-15 — creat (ergCAN 2.1)
 */
#ifndef ERGCAN_CLOUD_H
#define ERGCAN_CLOUD_H

#include <Arduino.h>

void   cloud_service(uint32_t now_ms);     // in loop()
String cloud_status();                     // pt consola: on/off, ultima postare, ultimul cod/eroare
void   cloud_post_now();                   // fortare (consola CLOUD NOW)

#endif
