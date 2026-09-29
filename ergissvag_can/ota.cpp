/**
 * ota.cpp — update firmware prin WiFi cu revenire automata (vezi ota.h)
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-09-15 — creat initial
 */
#include "ota.h"
#include "config.h"
#include "cfg.h"
#include "jsonlite.h"
#include "ca_isrg.h"
#include <Preferences.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <Update.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"

static Preferences g_p;
static bool        g_pending = false;
static uint8_t     g_boots = 0;
static OtaInfo     g_ci;            // rezultatul ultimului ota_check (evita a 2-a sesiune TLS in acelasi handler)
static uint32_t    g_ci_ms = 0;    // cand a fost facut

bool ota_cached_newer(OtaInfo &out) {
    if (g_ci_ms && millis() - g_ci_ms < 300000UL && g_ci.newer) { out = g_ci; return true; }
    return false;
}

void ota_boot_check() {
    g_p.begin("ergota", false);
    g_pending = g_p.getUChar("pending", 0) != 0;
    if (!g_pending) return;
    g_boots = (uint8_t)(g_p.getUChar("boots", 0) + 1);
    g_p.putUChar("boots", g_boots);
    Serial.printf("[OTA] firmware nou neconfirmat, pornirea %u din %u\n", g_boots, OTA_MAX_BOOTS);
    if (g_boots <= OTA_MAX_BOOTS) return;

    String prev = g_p.getString("prev", "");
    const esp_partition_t *pp = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, prev.c_str());
    g_p.putUChar("pending", 0);
    g_p.putUChar("boots", 0);
    if (pp && esp_ota_set_boot_partition(pp) == ESP_OK) {
        g_p.putString("last_err", String("revenire pe ") + prev + ": firmware-ul nou nu s-a confirmat in " + OTA_MAX_BOOTS + " porniri");
        Serial.printf("[OTA] REVENIRE pe %s si repornire\n", prev.c_str());
        delay(200);
        esp_restart();
    }
    g_p.putString("last_err", String("revenire imposibila: zona '") + prev + "' negasita");
    Serial.printf("[OTA] EROARE: nu gasesc zona %s pentru revenire\n", prev.c_str());
}

void ota_mark_ok() {
    if (!g_pending) return;
    g_pending = false;
    g_boots = 0;
    g_p.putUChar("pending", 0);
    g_p.putUChar("boots", 0);
    g_p.putString("ok_ver", FW_VERSION);
    esp_ota_mark_app_valid_cancel_rollback();   // fara efect daca bootloader-ul nu are rollback activ
    Serial.println("[OTA] firmware confirmat");
}

void ota_service(uint32_t now_ms) {
    if (g_pending && now_ms >= OTA_OK_AFTER_MS) ota_mark_ok();
}

int ota_version_cmp(const char *a, const char *b) {
    while (*a || *b) {
        long x = strtol(a, (char **)&a, 10), y = strtol(b, (char **)&b, 10);
        if (x != y) return x < y ? -1 : 1;
        if (*a == '.') a++;
        if (*b == '.') b++;
        if (!*a && !*b) break;
        if (!isdigit((unsigned char)*a) && *a) a++;
        if (!isdigit((unsigned char)*b) && *b) b++;
    }
    return 0;
}

static bool http_get_small(const String &url, String &body, String &err) {
    NetworkClientSecure client;
    client.setCACert(CA_ISRG_ROOT_X1);
    HTTPClient http;
    http.setTimeout(15000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    if (!http.begin(client, url)) { err = "URL invalid: " + url; return false; }
    int code = http.GET();
    if (code != 200) {
        err = code < 0 ? String("conexiune esuata: ") + http.errorToString(code) : String("HTTP ") + code;
        http.end();
        return false;
    }
    body = http.getString();
    http.end();
    if (body.length() == 0) { err = "raspuns gol"; return false; }
    return true;
}

bool ota_check(OtaInfo &info, String &err) {
    if (WiFi.status() != WL_CONNECTED) { err = "fara WiFi"; return false; }
    String base = cfg_str("ota.url");
    if (!base.length()) { err = "ota.url necompletat"; return false; }
    String mac = WiFi.macAddress();
    mac.replace(":", "");   // MAC fara ':' (altfel strica parsarea URL-ului)
    String url = base + (base.indexOf('?') >= 0 ? "&" : "?") + "fw=" + FW_VERSION + "&dev=" + mac;
    String body;
    if (!http_get_small(url, body, err)) return false;

    char fw[24], dl[240], md5[40], notes[200];
    long size = 0;
    if (!jl_str(body.c_str(), "fw", fw, sizeof(fw)) || !jl_str(body.c_str(), "url", dl, sizeof(dl)) ||
        !jl_str(body.c_str(), "md5", md5, sizeof(md5))) {
        err = "raspuns versiune invalid: " + body.substring(0, 120);
        return false;
    }
    if (strlen(md5) != 32) { err = "md5 invalid in raspuns"; return false; }
    if (!jl_str(body.c_str(), "notes", notes, sizeof(notes))) notes[0] = 0;
    jl_int(body.c_str(), "size", &size);
    info.latest = fw;
    info.url = dl;
    info.md5 = md5;
    info.notes = notes;
    info.size = size;
    info.newer = ota_version_cmp(fw, FW_VERSION) > 0;
    g_ci = info; g_ci_ms = millis();   // memoreaza pt un OTA UPDATE ulterior (o singura sesiune TLS)
    return true;
}

bool ota_update(const OtaInfo &info, String &err, ota_progress_cb progress) {
    if (WiFi.status() != WL_CONNECTED) { err = "fara WiFi"; return false; }
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
    if (!running || !next) { err = "schema de memorie fara zona OTA (flash-uieste o data pe USB cu partitions.csv)"; return false; }

    NetworkClientSecure client;
    client.setCACert(CA_ISRG_ROOT_X1);
    HTTPClient http;
    http.setTimeout(20000);
    http.setReuse(false);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    // reincercare la conectare: LTE / hairpin-NAT pot refuza tranzitoriu prima incercare (mai ales
    // imediat dupa OTA_CHECK, care lasa o intrare NAT recenta pe acelasi IP:port)
    int code = 0;
    for (int attempt = 0; attempt < 4; attempt++) {
        if (attempt) { delay(1500); client.stop(); }
        if (!http.begin(client, info.url)) { err = "URL descarcare invalid"; return false; }
        code = http.GET();
        if (code == 200) break;
        http.end();
    }
    if (code != 200) {
        err = code < 0 ? String("descarcare esuata: ") + http.errorToString(code) : String("descarcare HTTP ") + code;
        return false;
    }
    int total = http.getSize();
    if (total <= 0) {
        err = "serverul nu a trimis Content-Length (descarcarea trebuie servita prin wrapper PHP cu Content-Length)";
        http.end();
        return false;
    }
    if ((size_t)total > next->size) {
        err = String("firmware prea mare: ") + total + " > zona " + next->size;
        http.end();
        return false;
    }
    if (info.size > 0 && info.size != total) {
        err = String("dimensiune diferita: anuntat ") + info.size + ", primit " + total;
        http.end();
        return false;
    }
    if (!Update.begin((size_t)total, U_FLASH)) {
        err = String("Update.begin: ") + Update.errorString();
        http.end();
        return false;
    }
    Update.setMD5(info.md5.c_str());

    NetworkClient *stream = http.getStreamPtr();
    uint8_t buf[2048];
    size_t done = 0;
    uint32_t last_data = millis();
    while (done < (size_t)total) {
        size_t avail = stream->available();
        if (avail) {
            size_t n = stream->readBytes(buf, avail > sizeof(buf) ? sizeof(buf) : avail);
            if (n == 0) continue;
            if (Update.write(buf, n) != n) {
                err = String("scriere flash: ") + Update.errorString();
                Update.abort();
                http.end();
                return false;
            }
            done += n;
            last_data = millis();
            if (progress) progress(done, (size_t)total);
        } else {
            if (!http.connected() && done < (size_t)total) break;
            if (millis() - last_data > 20000) break;
            delay(2);
        }
    }
    http.end();
    if (done != (size_t)total) {
        err = String("descarcare incompleta: ") + done + " din " + total;
        Update.abort();
        return false;
    }
    if (!Update.end(true)) {
        err = String("verificare esuata (MD5 / imagine): ") + Update.errorString();
        return false;
    }
    g_p.putString("prev", running->label);
    g_p.putUChar("boots", 0);
    g_p.putUChar("pending", 1);
    g_p.putString("last_err", "");
    g_p.putString("new_ver", info.latest);
    return true;
}

String ota_status_json() {
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *nxt = esp_ota_get_next_update_partition(nullptr);
    char esc[160];
    jl_escape(esc, sizeof(esc), g_p.getString("last_err", "").c_str());
    String o = "{\"fw\":\"" FW_VERSION "\",\"running\":\"";
    o += run ? run->label : "?";
    o += "\",\"next\":\"";
    o += nxt ? nxt->label : "-";
    o += "\",\"next_size\":";
    o += nxt ? String(nxt->size) : String(0);
    o += ",\"pending\":";
    o += g_pending ? "1" : "0";
    o += ",\"boots\":";
    o += String(g_boots);
    o += ",\"last_err\":\"";
    o += esc;
    o += "\"}";
    return o;
}
