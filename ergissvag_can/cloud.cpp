/**
 * cloud.cpp — cloud logging BYOS autonom (vezi cloud.h)
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-09-15 — creat (ergCAN 2.1)
 */
#include "cloud.h"
#include "config.h"
#include "cfg.h"
#include "ca_isrg.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>

extern bool   net_online();
extern String cloud_snapshot_json();   // construit in .ino (mac, fw, uptime, can, senzori, obd cache)

static uint32_t g_last = 0;
static int      g_last_code = 0;
static String   g_last_err;
static uint32_t g_sent = 0;

static bool do_post(String &err) {
    if (!net_online()) { err = "offline"; return false; }
    String url = cfg_str("cloud.url");
    if (!url.length()) { err = "cloud.url gol"; return false; }
    url += (url.indexOf('?') >= 0 ? "&" : "?");
    url += "type=ergcan";

    NetworkClientSecure client;
    if (url.indexOf("ergiss.ro") >= 0) client.setCACert(CA_ISRG_ROOT_X1);
    else client.setInsecure();
    client.setHandshakeTimeout(8);   // v2.1.3: nu atarna la handshake TLS daca reteaua (ex HU fara internet) nu raspunde
    client.setTimeout(8000);
    HTTPClient http;
    http.setConnectTimeout(6000);    // v2.1.3: limiteaza si TCP connect (setTimeout acopera doar read)
    http.setTimeout(12000);
    http.setReuse(false);
    if (!http.begin(client, url)) { err = "URL invalid"; return false; }
    http.addHeader("Content-Type", "application/json");
    String tok = cfg_str("cloud.token");
    http.addHeader("Authorization", String("Bearer ") + tok);
    String body = cloud_snapshot_json();
    int code = http.POST((uint8_t *)body.c_str(), body.length());
    http.end();
    g_last_code = code;
    if (code == 200 || code == 201) { g_sent++; return true; }
    err = code < 0 ? http.errorToString(code) : String("HTTP ") + code;
    return false;
}

// v2.3.1: definita in ergissvag_can.ino — acelasi criteriu cu flag-ul `can` din snapshot.
extern bool can_is_live();
static bool g_was_live = true;   // ca prima tranzitie sa fie live->mort, nu invers

void cloud_service(uint32_t now_ms) {
    if (!cfg_int("cloud.on")) return;

    // v2.3.1: cu magistrala moarta (masina parcata) nu mai trimitem periodic — snapshot-urile ar fi
    // identice si fara informatie de masina. Trimitem insa UNUL SINGUR la trecerea viu->mort, ca pe
    // server sa ramana marcat momentul si ultima stare cunoscuta (altfel nu s-ar putea deosebi
    // „masina parcata" de „placa moarta"). La revenire, cadenta normala porneste singura.
    bool live = can_is_live();
    if (cfg_int("can.idle_gate") && !live) {
        if (!g_was_live) return;                 // deja raportat ca oprit — tacere pana revine CAN-ul
        g_was_live = false;
        g_last = now_ms;
        String err;
        if (!do_post(err)) g_last_err = err; else g_last_err = "";
        return;
    }
    g_was_live = live;

    uint32_t interval = (uint32_t)cfg_int("cloud.int_s") * 1000UL;
    if (interval < 5000) interval = 5000;
    if (g_last != 0 && now_ms - g_last < interval) return;
    g_last = now_ms;
    String err;
    if (!do_post(err)) g_last_err = err; else g_last_err = "";
}

void cloud_post_now() {
    g_last = millis();
    String err;
    if (!do_post(err)) g_last_err = err; else g_last_err = "";
}

String cloud_status() {
    String s = "cloud: ";
    s += cfg_int("cloud.on") ? "ON" : "off";
    s += String(" url=") + cfg_str("cloud.url") + " int=" + cfg_int("cloud.int_s") + "s";
    s += String(" trimise=") + g_sent + " ultim_cod=" + g_last_code;
    if (g_last_err.length()) s += String(" eroare=") + g_last_err;
    return s;
}
