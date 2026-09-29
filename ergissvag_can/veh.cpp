/**
 * veh.cpp — profilul vehiculului (vezi veh.h)
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.5
 * @changes
 *   v1.5 2026-09-29 — fw 2.6.0: cheia "auto" (profil generic cu autodetectie), veh_set_detected/veh_detected,
 *                     veh_resp_id pe 29 de biti (18DAxxF1 -> 18DAF1xx), veh_status arata ce s-a detectat.
 *   v1.4 2026-09-24 — fw 2.5.11: cheia "dids" (DID-uri citite periodic, definite doar in profil) -> proto_load_dids().
 *   v1.3 2026-09-23 — fw 2.5.8: cheia "mods" (rutele modulelor) -> dtcall_load_routes(); VEH_MAX_BYTES 4096 -> 8192; veh_status arata sursa rutelor.
 *   v1.0 2026-09-16 — creat initial
 *   v1.2 2026-09-21 — fw 2.5.2: ekm_req/ekm_did/ekm_fmt (km din ECU, u32) + "u32" in km_fmt_from; profil strain fara ekm_req => 0
 *   v1.1 2026-09-17 — campuri ECU motor (eng_req, genload/genperf/oilt/oill _did); profil strain fara `eng_req` => fara ele
 */
#include "veh.h"
#include "cfg.h"
#include "config.h"
#include "jsonlite.h"
#include "ca_isrg.h"
#include "dtcall.h"
extern int proto_load_dids(const char *json);   // v1.4 (proto.cpp)
#include <LittleFS.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>

#define VEH_FILE      "/veh.json"
#define VEH_MAX_BYTES 8192          // v1.3: cu "mods" un profil are ~2-3 KB; peste 8 KB refuzam (fisier gresit / pagina HTML)

// Profilul COMPILAT = exact valorile hardcodate pana in 2.2.9 (SEAT Altea XL, PQ35).
// Asa, o placa fara /veh.json se comporta identic cu inainte.
static VehProfile g_v = {
    "pq35", "VAG PQ35 (compilat)", 500000,
    0x714, 0x6A,
    0x2203, VEH_KM_U16X10,
    0x2206, 0x2212, VEH_FUEL_U8L,
    0x7E0, 0x2050, 0x20A8, 0x10E6, 0x2006,   // v1.1: SIMOS10 (Altea CBZB), confirmate cu VCDS 2026-09-17
    0x7E0, 0x121C, VEH_KM_U32,               // v1.2: km din ECU (VCDS: 7E0 22 121C -> 00 04 CB 80 = 314240)
    0, false,
    0                                        // v1.5: profilul compilat (PQ35) nu se autodetecteaza
};
static bool g_inited = false;
// v1.5: ce a gasit autodetectia pe masina (0 = nimic; se foloseste `bitrate` din profil)
static uint32_t g_det_bitrate = 0;
static bool     g_det_ext29   = false;

static uint8_t km_fmt_from(const char *s) {
    if (!strcmp(s, "u16x10")) return VEH_KM_U16X10;
    if (!strcmp(s, "u24"))    return VEH_KM_U24;
    if (!strcmp(s, "u16"))    return VEH_KM_U16;
    if (!strcmp(s, "u32"))    return VEH_KM_U32;
    return VEH_KM_NONE;
}
static uint8_t fuel_fmt_from(const char *s) {
    if (!strcmp(s, "u8L"))   return VEH_FUEL_U8L;
    if (!strcmp(s, "u8pct")) return VEH_FUEL_U8PCT;
    return VEH_FUEL_NONE;
}

// Parseaza un profil JSON peste `out`. Campurile lipsa raman cele din `out` (adica din profilul compilat),
// ca un profil minimal sa nu strice restul. Intoarce false doar daca lipseste identitatea (model).
static bool parse_profile(const char *body, VehProfile &out, String &err) {
    char s[64]; long n = 0;
    if (!jl_str(body, "model", s, sizeof(s)) || !s[0]) { err = "profil fara \"model\""; return false; }
    strncpy(out.model, s, sizeof(out.model) - 1); out.model[sizeof(out.model) - 1] = 0;
    if (jl_str(body, "name", s, sizeof(s))) { strncpy(out.name, s, sizeof(out.name) - 1); out.name[sizeof(out.name) - 1] = 0; }
    if (jl_int(body, "ver", &n))          out.ver = (uint16_t)n;
    if (jl_int(body, "bitrate", &n)) {
        if (n != 500000 && n != 250000 && n != 125000 && n != 100000) { err = "bitrate nesuportat"; return false; }
        out.bitrate = (uint32_t)n;
    }
    if (jl_int(body, "cluster_req", &n)) out.cluster_req = (uint16_t)n;
    if (jl_int(body, "resp_off", &n))    out.resp_off = (uint16_t)n;
    if (jl_int(body, "km_did", &n))      out.km_did = (uint16_t)n;
    if (jl_int(body, "fuel_did", &n))    out.fuel_did = (uint16_t)n;
    if (jl_int(body, "fuel_alt", &n))    out.fuel_alt = (uint16_t)n;
    if (jl_str(body, "km_fmt", s, sizeof(s)))   out.km_fmt = km_fmt_from(s);
    if (jl_str(body, "fuel_fmt", s, sizeof(s))) out.fuel_fmt = fuel_fmt_from(s);
    // v1.1: DID-urile de motor sunt specifice SIMOS/PQ35. Un profil de ALT model care nu le declara NU are voie sa le
    // mosteneasca din profilul compilat (am interoga DID-uri VAG pe o masina straina).
    if (jl_int(body, "eng_req", &n)) out.eng_req = (uint16_t)n;
    else if (strcmp(out.model, "pq35") != 0) out.eng_req = 0;
    if (jl_int(body, "genload_did", &n)) out.genload_did = (uint16_t)n;
    if (jl_int(body, "genperf_did", &n)) out.genperf_did = (uint16_t)n;
    if (jl_int(body, "oilt_did", &n))    out.oilt_did = (uint16_t)n;
    if (jl_int(body, "oill_did", &n))    out.oill_did = (uint16_t)n;
    // v1.2: km din ECU — aceeasi regula ca eng_req (profil strain fara camp => nu interogam ECU-ul)
    if (jl_int(body, "ekm_req", &n)) out.ekm_req = (uint16_t)n;
    else if (strcmp(out.model, "pq35") != 0) out.ekm_req = 0;
    if (jl_int(body, "ekm_did", &n)) out.ekm_did = (uint16_t)n;
    if (jl_str(body, "ekm_fmt", s, sizeof(s))) out.ekm_fmt = km_fmt_from(s);
    // v1.5: "auto" nu se mosteneste niciodata — un profil care nu-l cere explicit ramane pe valorile lui fixe
    out.autodet = (jl_int(body, "auto", &n) && n != 0) ? 1 : 0;
    return true;
}

static bool fs_ready() {
    static bool ok = false;
    if (!ok) ok = LittleFS.begin(true);
    return ok;
}

static void load_from_file() {
    if (!fs_ready() || !LittleFS.exists(VEH_FILE)) return;
    File f = LittleFS.open(VEH_FILE, "r");
    if (!f) return;
    size_t sz = f.size();
    if (sz == 0 || sz > VEH_MAX_BYTES) { f.close(); return; }
    String body = f.readString();
    f.close();
    VehProfile p = g_v; String err;
    if (parse_profile(body.c_str(), p, err)) { p.from_file = true; g_v = p; dtcall_load_routes(body.c_str()); proto_load_dids(body.c_str()); }
    else Serial.printf("[veh] %s invalid: %s (raman pe profilul compilat)\n", VEH_FILE, err.c_str());
}

const VehProfile &veh() {
    if (!g_inited) { g_inited = true; load_from_file(); }
    return g_v;
}
uint32_t veh_bitrate() { return (veh().autodet && g_det_bitrate) ? g_det_bitrate : veh().bitrate; }
void veh_set_detected(uint32_t bitrate, bool ext29) { g_det_bitrate = bitrate; g_det_ext29 = bitrate ? ext29 : false; }
bool veh_detected(uint32_t *bitrate, bool *ext29) {
    if (bitrate) *bitrate = g_det_bitrate;
    if (ext29)   *ext29 = g_det_ext29;
    return g_det_bitrate != 0;
}
uint32_t veh_resp_id(uint32_t req) {
    // 0x7E0-0x7E7 -> +8 (standard OBD-II, la fel pe orice masina); restul dupa regula producatorului
    if (req >= 0x7E0 && req <= 0x7E7) return req + 8;
    // v1.5: adresare fizica pe 29 de biti (ISO 15765-4): 18DA<tinta>F1 -> 18DAF1<tinta>
    if ((req & 0x1FFF00FFUL) == 0x18DA00F1UL) return 0x18DAF100UL | ((req >> 8) & 0xFFUL);
    return req + veh().resp_off;
}

static bool http_get(const String &url, String &body, String &err) {
    NetworkClientSecure client;
    client.setCACert(CA_ISRG_ROOT_X1);
    HTTPClient http;
    http.setTimeout(15000);
    http.setConnectTimeout(6000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    if (!http.begin(client, url)) { err = "URL invalid: " + url; return false; }
    int code = http.GET();
    if (code != 200) {
        err = code < 0 ? String("conexiune esuata: ") + http.errorToString(code) : String("HTTP ") + code;
        http.end(); return false;
    }
    body = http.getString();
    http.end();
    if (!body.length())             { err = "raspuns gol"; return false; }
    if (body.length() > VEH_MAX_BYTES) { err = "profil prea mare"; return false; }
    return true;
}

static String base_url() {
    const char *u = cfg_str("veh.url");
    return (u && *u) ? String(u) : String("https://app.ergiss.ro:89/cell/ergcan/veh.php");
}

bool veh_fetch(const char *model, String &err) {
    if (!model || !*model) { err = "lipseste modelul"; return false; }
    for (const char *p = model; *p; p++)
        if (!((*p >= 'a' && *p <= 'z') || (*p >= '0' && *p <= '9') || *p == '_')) { err = "nume invalid (doar a-z 0-9 _)"; return false; }
    String body;
    String url = base_url() + (base_url().indexOf('?') >= 0 ? "&" : "?") + "model=" + model;
    if (!http_get(url, body, err)) return false;

    // validam INAINTE sa scriem fisierul — altfel o pagina de eroare ar ramane pe placa
    VehProfile p = g_v;
    if (!parse_profile(body.c_str(), p, err)) return false;
    if (strcmp(p.model, model) != 0) { err = String("serverul a dat alt model: ") + p.model; return false; }

    if (!fs_ready()) { err = "LittleFS indisponibil"; return false; }
    File f = LittleFS.open(VEH_FILE, "w");
    if (!f) { err = "nu pot scrie " VEH_FILE; return false; }
    size_t w = f.print(body);
    f.close();
    if (w != body.length()) { LittleFS.remove(VEH_FILE); err = "scriere incompleta"; return false; }

    p.from_file = true;
    uint32_t old_bitrate = veh_bitrate();
    veh_set_detected(0, false);         // v1.5: profil nou => ce s-a detectat pe profilul vechi nu mai e valabil
    g_v = p; g_inited = true;
    proto_load_dids(body.c_str());      // v1.4: DID-urile din profil
    dtcall_load_routes(body.c_str());   // v1.3: rutele modulelor din profil (sau inapoi la cele compilate daca lipsesc)
    String e2;
    if (veh_bitrate() != old_bitrate) {
        extern bool can_set_mode(const char *mode);
        extern const char *can_mode_name();
        can_set_mode(can_mode_name());   // reinstaleaza driverul TWAI cu bitrate-ul nou
    }
    (void)e2;
    return true;
}

bool veh_clear(String &err) {
    if (!fs_ready()) { err = "LittleFS indisponibil"; return false; }
    if (LittleFS.exists(VEH_FILE) && !LittleFS.remove(VEH_FILE)) { err = "nu pot sterge " VEH_FILE; return false; }
    err = "sters; profilul compilat se aplica la repornire";
    return true;
}

bool veh_list(String &out, String &err) {
    String url = base_url() + (base_url().indexOf('?') >= 0 ? "&" : "?") + "list=1";
    return http_get(url, out, err);
}

String veh_status() {
    const VehProfile &v = veh();
    String s = String("profil: ") + v.model + " v" + v.ver + " (" + v.name + ") " + (v.from_file ? "[/veh.json]" : "[compilat]");
    s += String("\r\n  bitrate ") + veh_bitrate() + "  bord 0x" + String(v.cluster_req, HEX) + " resp+0x" + String(v.resp_off, HEX);
    if (v.autodet) s += g_det_bitrate ? (String("\r\n  AUTO: detectat ") + g_det_bitrate + " bps, " + (g_det_ext29 ? "29" : "11") + " biti")
                                      : String("\r\n  AUTO: inca nedetectat (contact pus? AUTODET pe consola)");
    s += String("\r\n  km ECU 0x") + String(v.ekm_req, HEX) + " DID 0x" + String(v.ekm_did, HEX) + " fmt " + v.ekm_fmt + " (primar)";
    s += String("\r\n  km bord DID 0x") + String(v.km_did, HEX) + " fmt " + v.km_fmt +
         "   litri DID 0x" + String(v.fuel_did, HEX) + (v.fuel_alt ? (String("/0x") + String(v.fuel_alt, HEX)) : String("")) + " fmt " + v.fuel_fmt;
    s += String("\r\n  motor 0x") +String(v.eng_req, HEX) + "  gen 0x" + String(v.genload_did, HEX) + "/0x" + String(v.genperf_did, HEX) +
         "  ulei T 0x" + String(v.oilt_did, HEX) + " niv 0x" + String(v.oill_did, HEX);
    s += String("\r\n  module: ") + dtcall_route_count() + (dtcall_routes_from_profile() ? " (din profil)" : " (tabela compilata - profilul nu are \"mods\")");
    return s;
}
