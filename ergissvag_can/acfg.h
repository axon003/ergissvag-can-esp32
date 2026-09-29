/**
 * acfg.h — autoconfigurare de pe server (ergCAN 2.1)
 *   Daca cfg acfg.on=1: la pornire + la fiecare acfg.int_s, GET acfg.url?dev=<mac>&cfg=<ver> (Bearer acfg.token).
 *   Raspuns: {"cfg_version":N, "<cheie cfg>":val, ..., "fw_target":"x"}. Aplica DOAR daca cfg_version > cel stocat,
 *   si DOAR cheile trimise. SIGURANTA: NU schimba wifi.ssid/wifi.pass de la distanta (altfel s-ar auto-bloca);
 *   fw_target e doar raportat (OTA ramane manual, decizia userului). Decizie 7: serverul suprascrie cheile trimise.
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-09-15 — creat
 */
#ifndef ERGCAN_ACFG_H
#define ERGCAN_ACFG_H

#include <Arduino.h>

void   acfg_service(uint32_t now_ms);
String acfg_status();
String acfg_fetch_now();   // fortare (consola ACFG NOW), intoarce rezumat

#endif
