/**
 * cfg.cpp — „Setari ergCAN” in NVS (vezi cfg.h)
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.2
 * @changes
 *   v1.2 2026-09-29 — fw 2.6.0: veh.abr / veh.aext = ultima autodetectie reusita (profil generic), incercata prima la pornire
 *   v1.0 2026-09-15 — creat initial (ergCAN 2.0)
 *   v1.1 2026-09-24 — fw 2.5.13: chei calibrare magnetometru mag.* (scrise de MAGCAL)
 */
#include "cfg.h"
#include "config.h"
#include <Preferences.h>

static const CfgDef DEFS[] = {
    {"wifi.ssid",   's', WIFI_STA_SSID, 0, 0,        "retea WiFi (hotspot radio)"},
    {"wifi.pass",   'p', WIFI_STA_PASS, 0, 0,        "parola WiFi"},
    {"wifi.ap",     'i', "1",           0, 1,        "AP propriu de rezerva daca WiFi lipseste"},
    {"net.dns",     's', "",            0, 0,        "DNS fix (gol=de la retea/DHCP; ex 8.8.8.8 daca reteaua nu da DNS). Implicit automat."},
    {"dev.name",    's', "ergCAN",      0, 0,        "nume dispozitiv"},
    {"can.boot",    's', CAN_BOOT_MODE, 0, 0,        "mod CAN la pornire: normal | listen"},
    {"ser.json",    'i', "0",           0, 1,        "JSON pe serial fara sa fie cerut (1 = ca in 1.x)"},
    {"con.telnet",  'i', "1",           0, 1,        "consola telnet port 23"},
    {"con.pin",     'p', "0000",        0, 0,        "PIN mod EXPLORE — schimba-l din consola (SET con.pin ...)"},
    {"dtc.clear",   'i', "2",           0, 2,        "stergere erori: 0 nicaieri, 1 consola, 2 consola+F16"},
    {"dtc.scan_s",  'i', "120",         60, 86400,   "scan erori periodic (s)"},
    {"elm.on",      'i', "0",           0, 1,        "emulare ELM327 pe WiFi"},
    {"elm.port",    'i', "35000",       1, 65535,    "port ELM327"},
    {"ota.url",     's', "https://app.ergiss.ro:89/cell/ergcan/ergcan_version.php", 0, 0, "URL versiune firmware"},
    {"cloud.on",    'i', "0",           0, 1,        "cloud logging BYOS"},
    {"cloud.url",   's', "https://app.ergiss.ro:89/byos/byos.php", 0, 0, "URL BYOS"},
    {"cloud.token", 'p', "",            0, 0,        "token BYOS"},
    {"cloud.int_s", 'i', "10",          5, 3600,     "interval trimitere cloud (s)"},
    // v2.3.1: 1 = cu magistrala moarta nu se interogheaza si nu se trimite in cloud (vezi config.h)
    {"can.idle_gate",'i', CAN_IDLE_GATE_DEFAULT, 0, 1, "fara citiri/upload cand CAN e mort"},
    {"acfg.on",     'i', "0",           0, 1,        "autoconfigurare de pe server"},
    {"acfg.url",    's', "",            0, 0,        "URL autoconfigurare"},
    {"acfg.token",  'p', "",            0, 0,        "token autoconfigurare"},
    {"acfg.int_s",  'i', "3600",        300, 86400,  "cat de des verifica configurarea (s)"},
    // v2.3.0: profil de vehicul (ce e specific masinii: bord, DID-uri km/litri, bitrate). Vezi veh.h.
    {"veh.model",   's', "",            0, 0,        "model vehicul (ex: pq35). Gol = profilul compilat. SET mode <x> il aduce de pe server"},
    {"mag.cal",     'i', "0",           0, 1,        "magnetometru calibrat (1) — scris de MAGCAL; 0 = colectare automata"},
    {"mag.ox",      'i', "0",           -20000, 20000, "offset X magnetometru (0.1 uT)"},
    {"mag.oy",      'i', "0",           -20000, 20000, "offset Y magnetometru (0.1 uT)"},
    {"mag.oz",      'i', "0",           -20000, 20000, "offset Z magnetometru (0.1 uT)"},
    {"mag.sx",      'i', "1000",        200, 5000,   "scara X magnetometru (x1000)"},
    {"mag.sy",      'i', "1000",        200, 5000,   "scara Y magnetometru (x1000)"},
    {"mag.hofs",    'i', "0",           -1800, 1800, "orientare montaj: grade x10 adunate la heading (MAGCAL HDG)"},
    {"mag.inv",     'i', "0",           0, 1,        "sens invers heading"},
    {"veh.url",     's', "https://app.ergiss.ro:89/cell/ergcan/veh.php", 0, 0, "URL catalog profile vehicul"},
    {"veh.abr",     'i', "0",           0, 1000000,  "autodetectie: viteza gasita ultima data (0 = niciuna) — scrisa de placa"},
    {"veh.aext",    'i', "0",           0, 1,        "autodetectie: 1 = adresare pe 29 de biti — scrisa de placa"},
};
#define N_DEFS (sizeof(DEFS) / sizeof(DEFS[0]))

static Preferences g_prefs;
static String      g_val[N_DEFS];
static uint32_t    g_cfgver = 0;

static int idx_of(const char *key) {
    for (size_t i = 0; i < N_DEFS; i++) if (strcmp(DEFS[i].key, key) == 0) return (int)i;
    return -1;
}

void cfg_init() {
    g_prefs.begin("ergcan", false);
    for (size_t i = 0; i < N_DEFS; i++) g_val[i] = g_prefs.getString(DEFS[i].key, DEFS[i].def);
    g_cfgver = g_prefs.getUInt("cfg.ver", 0);
}

const char *cfg_str(const char *key) {
    int i = idx_of(key);
    return i < 0 ? "" : g_val[i].c_str();
}

long cfg_int(const char *key) {
    int i = idx_of(key);
    return i < 0 ? 0 : strtol(g_val[i].c_str(), nullptr, 10);
}

static bool is_int(const char *s) {
    if (!*s) return false;
    if (*s == '-') s++;
    if (!*s) return false;
    for (; *s; s++) if (*s < '0' || *s > '9') return false;
    return true;
}

bool cfg_set(const char *key, const char *val, String &err) {
    int i = idx_of(key);
    if (i < 0) { err = String("cheie necunoscuta: ") + key; return false; }
    const CfgDef &d = DEFS[i];
    if (d.type == 'i') {
        if (!is_int(val)) { err = String(key) + " cere numar"; return false; }
        long v = strtol(val, nullptr, 10);
        if (v < d.min || v > d.max) { err = String(key) + " in afara intervalului " + d.min + ".." + d.max; return false; }
    }
    if (strcmp(key, "can.boot") == 0 && strcmp(val, "normal") != 0 && strcmp(val, "listen") != 0) {
        err = "can.boot: normal | listen"; return false;
    }
    if (strcmp(key, "wifi.ssid") == 0 && !*val) { err = "wifi.ssid nu poate fi gol"; return false; }
    if (strlen(val) > 200) { err = String(key) + " prea lung (max 200)"; return false; }
    if (g_prefs.putString(d.key, val) == 0 && *val) { err = String("NVS: scriere esuata pt ") + key; return false; }
    g_val[i] = val;
    return true;
}

bool cfg_reset_key(const char *key) {
    int i = idx_of(key);
    if (i < 0) return false;
    g_prefs.remove(DEFS[i].key);
    g_val[i] = DEFS[i].def;
    return true;
}

void cfg_factory_reset() {
    g_prefs.clear();
    for (size_t i = 0; i < N_DEFS; i++) g_val[i] = DEFS[i].def;
    g_cfgver = 0;
}

size_t cfg_count() { return N_DEFS; }
const CfgDef *cfg_def(size_t i) { return i < N_DEFS ? &DEFS[i] : nullptr; }

static void json_escape(String &out, const String &s) {
    for (size_t k = 0; k < s.length(); k++) {
        char c = s[k];
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if ((uint8_t)c < 0x20) out += ' ';
        else out += c;
    }
}

String cfg_json(bool mask) {
    String o = "{";
    for (size_t i = 0; i < N_DEFS; i++) {
        if (i) o += ",";
        o += "\""; o += DEFS[i].key; o += "\":";
        if (DEFS[i].type == 'i') { o += String(strtol(g_val[i].c_str(), nullptr, 10)); continue; }
        o += "\"";
        if (DEFS[i].type == 'p' && mask) o += g_val[i].length() ? "***" : "";
        else json_escape(o, g_val[i]);
        o += "\"";
    }
    o += ",\"cfg.ver\":"; o += String(g_cfgver);
    o += "}";
    return o;
}

uint32_t cfg_version() { return g_cfgver; }
void cfg_set_version(uint32_t v) { g_cfgver = v; g_prefs.putUInt("cfg.ver", v); }
