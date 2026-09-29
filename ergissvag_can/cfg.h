/**
 * cfg.h — „Setari ergCAN” salvate nevolatil in NVS (Preferences, namespace "ergcan")
 *   Editabile din F16 (cfg_get / cfg_set JSON), consola (CFG / SET) si autoconfigurare de pe server.
 *   Chei max 15 caractere (limita NVS). Secretele (parole, tokenuri, PIN) sunt mascate la afisare.
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-09-15 — creat initial (ergCAN 2.0)
 */
#ifndef ERGCAN_CFG_H
#define ERGCAN_CFG_H

#include <Arduino.h>

struct CfgDef {
    const char *key;
    char        type;      // 's' text, 'i' numar, 'p' secret (text mascat)
    const char *def;
    long        min, max;  // pt 'i'
    const char *help;
};

void          cfg_init();
const char   *cfg_str(const char *key);
long          cfg_int(const char *key);
// seteaza + salveaza in NVS; err primeste motivul refuzului
bool          cfg_set(const char *key, const char *val, String &err);
bool          cfg_reset_key(const char *key);
void          cfg_factory_reset();
size_t        cfg_count();
const CfgDef *cfg_def(size_t i);
String        cfg_json(bool mask);           // {"wifi.ssid":"...","cloud.on":0,...}
uint32_t      cfg_version();                 // versiunea configurarii primite de pe server
void          cfg_set_version(uint32_t v);

#endif
