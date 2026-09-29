/**
 * acfg.cpp — autoconfigurare de pe server (vezi acfg.h)
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-09-15 — creat
 */
#include "acfg.h"
#include "config.h"
#include "cfg.h"
#include "jsonlite.h"
#include "ca_isrg.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>

extern bool net_online();

static uint32_t g_last = 0;
static bool     g_booted = false;
static int      g_last_code = 0;
static uint32_t g_applied_ver = 0;
static String   g_last_msg;

static bool remote_allowed(const char *key) {
    // SIGURANTA: nu lasam serverul sa schimbe de la distanta cheile care ar putea auto-bloca placa
    return strcmp(key, "wifi.ssid") && strcmp(key, "wifi.pass") &&
           strcmp(key, "acfg.url") && strcmp(key, "acfg.token") && strcmp(key, "acfg.on");
}

// aplica cheile cfg prezente in raspuns (doar cele permise). Intoarce nr aplicate; scrie in msg.
static int apply_body(const String &body, String &msg) {
    long ver = 0;
    if (!jl_int(body.c_str(), "cfg_version", &ver)) { msg = "fara cfg_version"; return -1; }
    if ((uint32_t)ver <= cfg_version()) { msg = String("la zi (ver ") + cfg_version() + ")"; return 0; }
    int applied = 0; String changed;
    for (size_t i = 0; i < cfg_count(); i++) {
        const CfgDef *d = cfg_def(i);
        if (!remote_allowed(d->key)) continue;
        char val[210]; bool have = false;
        if (jl_str(body.c_str(), d->key, val, sizeof(val))) have = true;
        else { long n; if (jl_int(body.c_str(), d->key, &n)) { snprintf(val, sizeof(val), "%ld", n); have = true; } }
        if (!have) continue;
        String err;
        if (cfg_set(d->key, val, err)) { applied++; changed += String(d->key) + " "; }
    }
    cfg_set_version((uint32_t)ver);
    g_applied_ver = (uint32_t)ver;
    char ft[24];
    String extra;
    if (jl_str(body.c_str(), "fw_target", ft, sizeof(ft)) && strcmp(ft, FW_VERSION))
        extra = String(" fw_target=") + ft + " (OTA manual)";
    msg = String("ver ") + ver + ": aplicat " + applied + " (" + changed + ")" + extra;
    return applied;
}

static bool fetch(String &msg) {
    if (!net_online()) { msg = "offline"; return false; }
    String base = cfg_str("acfg.url");
    if (!base.length()) { msg = "acfg.url gol"; return false; }
    String mac = WiFi.macAddress(); mac.replace(":", "");
    String url = base + (base.indexOf('?') >= 0 ? "&" : "?") + "dev=" + mac + "&cfg=" + cfg_version();

    NetworkClientSecure client;
    if (url.indexOf("ergiss.ro") >= 0) client.setCACert(CA_ISRG_ROOT_X1); else client.setInsecure();
    client.setHandshakeTimeout(8); client.setTimeout(8000);   // v2.1.3: nu atarna daca reteaua nu raspunde
    HTTPClient http; http.setConnectTimeout(6000); http.setTimeout(12000); http.setReuse(false);
    if (!http.begin(client, url)) { msg = "URL invalid"; return false; }
    String tok = cfg_str("acfg.token");
    if (tok.length()) http.addHeader("Authorization", String("Bearer ") + tok);
    int code = http.GET();
    g_last_code = code;
    if (code != 200) { http.end(); msg = code < 0 ? http.errorToString(code) : String("HTTP ") + code; return false; }
    String body = http.getString();
    http.end();
    int a = apply_body(body, msg);
    return a >= 0;
}

void acfg_service(uint32_t now_ms) {
    if (!cfg_int("acfg.on")) return;
    uint32_t interval = (uint32_t)cfg_int("acfg.int_s") * 1000UL;
    if (interval < 60000) interval = 60000;
    bool due = (!g_booted) || (g_last != 0 && now_ms - g_last >= interval) || (g_last == 0);
    if (!due) return;
    if (!net_online()) return;                 // asteapta reteaua (mai ales la boot)
    g_booted = true; g_last = now_ms;
    String msg; fetch(msg); g_last_msg = msg;
}

String acfg_fetch_now() {
    String msg;
    g_last = millis(); g_booted = true;
    fetch(msg); g_last_msg = msg;
    return msg;
}

String acfg_status() {
    String s = "acfg: ";
    s += cfg_int("acfg.on") ? "ON" : "off";
    s += String(" url=") + cfg_str("acfg.url") + " ver_local=" + cfg_version() + " int=" + cfg_int("acfg.int_s") + "s";
    s += String(" ultim_cod=") + g_last_code;
    if (g_last_msg.length()) s += String(" [") + g_last_msg + "]";
    return s;
}
