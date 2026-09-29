/**
 * sniff.cpp — captura CAN in LittleFS + upload (vezi sniff.h)
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.1
 * @changes
 *   v1.0 2026-09-15 — creat
 *   v1.1 2026-09-21 — la SNIFF STOP revine la modul CAN de dinainte de SNIFF (nu mai forteaza "normal");
 *                     post_chunk -> sniff_post_json() public + sniff_cloud_url() (refolosite de READEPROM).
 */
#include "sniff.h"
#include "config.h"
#include "cfg.h"
#include "ca_isrg.h"
#include <LittleFS.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>

extern bool   can_set_mode(const char *mode);
extern const char *can_mode_name();
extern void   proto_pump(uint32_t ms);

static bool    g_active = false;
static File    g_file;
static uint32_t g_n = 0, g_t0 = 0;
static char    g_label[32];
static char    g_url[220];
static char    g_prev_mode[8] = "normal";   // v1.1: modul de dinainte de SNIFF, restaurat la STOP
static bool    g_fs_ok = false;

static bool fs_ready() {
    if (g_fs_ok) return true;
    g_fs_ok = LittleFS.begin(true);   // formateaza la prima folosire daca e nevoie
    return g_fs_ok;
}

bool sniff_begin(const char *label, const char *url, uint8_t) {
    if (g_active || !fs_ready()) return false;
    strncpy(g_label, label ? label : "sniff", sizeof(g_label) - 1); g_label[sizeof(g_label) - 1] = 0;
    strncpy(g_url, url ? url : "", sizeof(g_url) - 1); g_url[sizeof(g_url) - 1] = 0;
    g_file = LittleFS.open("/sniff.bin", "w");
    if (!g_file) return false;
    strncpy(g_prev_mode, can_mode_name(), sizeof(g_prev_mode) - 1); g_prev_mode[sizeof(g_prev_mode) - 1] = 0;
    can_set_mode("listen");            // zero emisie cat ascultam
    g_n = 0; g_t0 = millis(); g_active = true;
    return true;
}

// record: ts(4 LE) id(4 LE) ext(1) dlc(1) data(8) = 18 octeti fixi (SNIFF_REC_SIZE+1 rotunjit)
void sniff_frame(uint32_t id, bool ext, const uint8_t *d, uint8_t dlc, uint32_t ts) {
    if (!g_active || !g_file) return;
    uint8_t rec[18];
    rec[0] = ts; rec[1] = ts >> 8; rec[2] = ts >> 16; rec[3] = ts >> 24;
    rec[4] = id; rec[5] = id >> 8; rec[6] = id >> 16; rec[7] = id >> 24;
    rec[8] = ext ? 1 : 0; rec[9] = dlc > 8 ? 8 : dlc;
    for (int i = 0; i < 8; i++) rec[10 + i] = i < dlc ? d[i] : 0;
    g_file.write(rec, sizeof(rec));
    g_n++;
    if ((g_n & 0x7F) == 0) g_file.flush();
}

bool sniff_is_active() { return g_active; }
uint32_t sniff_frames() { return g_n; }

bool sniff_post_json(const String &url, const String &body, String &err) {
    NetworkClientSecure client;
    if (url.startsWith("https://app.ergiss.ro") || url.indexOf("ergiss.ro") >= 0) client.setCACert(CA_ISRG_ROOT_X1);
    else client.setInsecure();   // url arbitrar dat de user in EXPLORE
    HTTPClient http;
    http.setTimeout(15000);
    if (!http.begin(client, url)) { err = "URL invalid"; return false; }
    http.addHeader("Content-Type", "application/json");
    String tok = cfg_str("cloud.token");
    if (tok.length()) http.addHeader("Authorization", String("Bearer ") + tok);
    int code = http.POST((uint8_t *)body.c_str(), body.length());
    http.end();
    if (code != 200 && code != 201) { err = String("HTTP ") + code; return false; }
    return true;
}

String sniff_cloud_url(const char *type) {
    const char *base = cfg_str("cloud.url");
    return String(base) + (strchr(base, '?') ? "&" : "?") + "type=" + type;
}

String sniff_end_and_upload() {
    if (!g_active) return "SNIFF nu era pornit";
    g_active = false;
    if (g_file) { g_file.flush(); g_file.close(); }
    can_set_mode(g_prev_mode);         // v1.1: inapoi la ce era (listen ramane listen)
    uint32_t dur = (millis() - g_t0) / 1000;

    String url = g_url[0] ? String(g_url) : sniff_cloud_url("can_log");
    File f = LittleFS.open("/sniff.bin", "r");
    if (!f) return String("captura: ") + g_n + " cadre in " + dur + "s, dar fisierul lipseste";
    if (WiFi.status() != WL_CONNECTED) { f.close(); return String("captura: ") + g_n + " cadre in " + dur + "s salvate in /sniff.bin (WiFi offline, upload amanat)"; }

    uint32_t sent = 0, chunks = 0; String err;
    uint8_t rec[18];
    String body; int inbatch = 0;
    body.reserve(SNIFF_CHUNK_FRAMES * 60);
    auto flush_batch = [&](bool last) -> bool {
        if (inbatch == 0) return true;
        String head = String("{\"type\":\"can_log\",\"label\":\"") + g_label + "\",\"chunk\":" + chunks +
                      ",\"ts\":" + (unsigned long)millis() + ",\"frames\":[";
        String full = head + body + "]}";
        chunks++;
        bool ok = sniff_post_json(url, full, err);
        if (ok) sent += inbatch;
        body = ""; inbatch = 0;
        proto_pump(5);
        return ok || last;
    };
    while (f.read(rec, sizeof(rec)) == sizeof(rec)) {
        uint32_t ts = rec[0] | (rec[1] << 8) | (rec[2] << 16) | ((uint32_t)rec[3] << 24);
        uint32_t id = rec[4] | (rec[5] << 8) | (rec[6] << 16) | ((uint32_t)rec[7] << 24);
        uint8_t dlc = rec[9] > 8 ? 8 : rec[9];
        char hx[26]; int p = 0;
        for (int i = 0; i < dlc; i++) p += snprintf(hx + p, sizeof(hx) - p, "%02X", rec[10 + i]);
        if (inbatch) body += ",";
        body += String("{\"t\":") + ts + ",\"id\":\"" + String(id, HEX) + "\",\"e\":" + rec[8] + ",\"d\":\"" + hx + "\"}";
        if (++inbatch >= SNIFF_CHUNK_FRAMES) if (!flush_batch(false)) break;
    }
    flush_batch(true);
    f.close();

    String mode_note = String(" (mod CAN: ") + g_prev_mode + ")";
    if (sent >= g_n) { LittleFS.remove("/sniff.bin"); return String("SNIFF '") + g_label + "': " + g_n + " cadre in " + dur + "s, urcate " + sent + " in " + chunks + " loturi la " + url + mode_note; }
    return String("SNIFF '") + g_label + "': " + g_n + " cadre, urcate " + sent + "/" + g_n + " (eroare: " + err + "); /sniff.bin pastrat" + mode_note;
}
