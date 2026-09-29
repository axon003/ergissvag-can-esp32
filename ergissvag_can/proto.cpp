/**
 * proto.cpp — dispecer comenzi ergCAN 2.0 (vezi proto.h)
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 3.4
 * @changes
 *   v3.4 2026-09-29 — fw 2.6.0: PARAMETRI CU NUME (cerinta user: F16 sa nu mai stie PID-uri/DID-uri).
 *                     (a) `sub` accepta {"n":"rpm","hz":5}: numele e rezolvat pe placa — profil ("dids") -> valori de vehicul
 *                     (km, litri, generator, ulei) -> catalogul standard OBD-II (params.cpp); raspunsul spune de unde vine
 *                     fiecare ("res":{"rpm":"obd","lant_p2":"uds","espYawRate":"none"}). Forma veche (k/src/pid) ramane valabila.
 *                     (b) toate valorile pleaca la fel: {"t":"d","k":<nume>,"v":..,"src":..} — inclusiv DID-urile din profil si
 *                     cele lente (odoKm, fuelL, genLoad, genPerf, oilTemp, oilLevel); liniile vechi t:km/fuel/... raman pt F16 <= 1.83.
 *                     (c) DID-urile din profil: "sg":1 = valoare cu semn, "bg":0 = citit doar cand e abonat cineva.
 *                     (d) bitmap-ul PID-urilor suportate (01 00/20/40..) citit o data; ce lipseste nu se mai cere pe ghicite.
 *                     (e) comenzi: PARAMS, AUTODET (consola); {"cmd":"params"}, {"cmd":"veh"[,"model":"x"]} (JSON).
 *   v3.3 2026-09-24 — fw 2.5.13: comanda MAGCAL [START|STOP|RESET|HDG <grade>|INV 0|1] (calibrare magnetometru, sensors.cpp v1.4)
 *   v3.2 2026-09-24 — fw 2.5.11: (a) DID-uri GENERICE din profil — cheia "dids": [{n,req,did,nb,mul,off,s,dec}] citite periodic
 *                     cand motorul raspunde, difuzate la F16 {"t":"did","n":..,"v":..,"raw":..}, in snapshot-ul cloud "dids":{n:v}
 *                     si pe consola cu DIDS (fara firmware nou pt fiecare valoare). Prima utilizare: pozitiile lantului de distributie
 *                     (0x7E0 DID 1189..118F, sniff VCDS „lant" 24.09). (b) DTC + lista gateway AUTOMAT doar la pornirea placii
 *                     (o scanare reusita, max 5 incercari); fara verificarea la 2 min si rescan la 10 min. Manual neschimbat.
 *   v3.1 2026-09-24 — fw 2.5.10: proto_event() = jurnal AUTOMAT de evenimente (boot, driver CAN, bus-off, motor tace, F16
 *                     conectat/deconectat, scanare DTC automata), urcat de task-ul de retea la BYOS kind=event label ev_<MAC>;
 *                     BYOSDEBUG prinde si raspunsurile JSON ale comenzilor de consola (STAT).
 *   v3.0 2026-09-23 — fw 2.5.9: (b) BYOSDEBUG <nume>/STOP: jurnalul consolei urcat la BYOS de task-ul de retea (kind=debug);
 *                     (c) CODING/CODINGALL pe module KWP fara DID 0600: codarea scurta din 1A 9B octetii 17..19 (airbag 0021303 = VCDS);
 *                     (a) DIDSCAN <reqid> <de_la> <pana_la> [nume] — `22 <did>` pe fiecare DID din interval (doar citire),
 *                     afiseaza DID-urile care raspund pozitiv (hex + u8-40/u8-60 la 1 octet), cu nume => upload BYOS kind=didscan.
 *                     Cerut de user: cautare DID de temperatura ulei MASURATA (10E6 = „calculated oil temperature").
 *   v2.9 2026-09-23 — fw 2.5.8: MODPROBE/SELFTEST sondeaza modulele din PROFILUL vehiculului (cheia "mods"); listele compilate
 *                     (= rezultatul UDSSCAN pe Altea 09-23) raman doar rezerva.
 *   v2.8 2026-09-23 — fw 2.5.7: MODPROBE/SELFTEST sondeaza clima pe UDS 0x746 si usa sofer pe 0x74A (din sniff VCDS 09-23).
 *                     UDSSCAN [de_la] [pana_la]: `22 F187` pe fiecare ID 0x700..0x7DF, raspuns pe ID+0x6A => harta REALA
 *                     module UDS (in loc de adrese presupuse). Doar citire. Cu nume => upload BYOS kind=udsscan.
 *   v2.7 2026-09-23 — fw 2.5.6: TPQ <adr> <cerere_hex> [cerere2..4] — cereri KWP de CITIRE pe un canal TP2.0 deschis FARA 10 89,
 *                     in ordine, cu raspunsul fiecareia + trace-ul complet al cadrelor (diagnostic ABS 0x03 = NRC 0x21).
 *   v2.0 2026-09-15 — creat
 *   v2.3 2026-09-21 — fw 2.5.0: READEPROM <adr_modul> <adresa> <lungime> <nume> — citire memorie modul TP2.0 exact ca VAG CAN PRO
 *                     (tp20_read_mem), hex pe consola + upload BYOS (?type=can_log, kind=eeprom, frames=[]); modul CAN restaurat
 *                     dupa comanda. UDS afiseaza si u32 (ECU 7E0 DID 121C = km pe 4 octeti, de verificat rezolutia).
 *   v2.6 2026-09-21 — fw 2.5.3: READEPROM pana la 32 KB (bucati READEPROM_CHUNK intr-o sesiune): la oprire arata adresa si NRC
 *                     (31 = margine memorie), hex pe consola doar pana la 512 B (restul in BYOS), upload BYOS pe bucati de 4 KB
 *                     (`chunk` 0..n, `addr` al bucatii, `total`; cadrele doar in chunk 0).
 *   v2.5 2026-09-21 — fw 2.5.2: (a) KM: read_km() citeste INTAI ECU-ul (profil ekm_req/ekm_did, PQ35 7E0 22 121C u32) si doar
 *                     la esec bordul (2203 u16x10); `KM` arata sursa. (b) `UDS <id> <did> [nr] [ms]`: nr citiri (implicit 1;
 *                     0 = pana la tasta q + Enter), pauza intre ele in ms (implicit 1000); linia „q" e prinsa in proto_console_line.
 *                     (c) READEPROM afiseaza TRACE-ul cadrelor TP2.0 la esec (> trimis / < primit, ms) si il urca in
 *                     `frames` (acelasi format ca SNIFF + "x":1 pe cadrele noastre); mentioneaza daca 27 03 a mers la a 2-a incercare.
 *   v2.5 2026-09-23 — fw 2.5.5: CODING <adr> [nume] [did] si CODINGALL [nume] [did] — long coding (SID 22, DID 0600
 *                     confirmat pe BCM in sniff_20260921_bcm), simetric cu DTC/DTCALL; cu <nume> urca si la BYOS
 *                     (kind=coding). CODINGALL citeste doar modulele marcate CODATE in lista de instalare.
 *   v2.4 2026-09-21 — fw 2.5.1: subcomenzile consolei case-insensitive (strcasecmp): `sniff stop`, `cloud now`, `ota check`, `dtc clr`...
 *                     Comanda principala era deja pe majuscule; doar PIN-ul din EXPLORE ramane exact.
 *   v2.1 2026-09-17 — fw 2.3.3: valori de pe ECU motor prin UDS (generator load/performance, temperatura + nivel ulei):
 *                     citire in fundal (`eng_tick`), difuzare {"t":"genload|genperf|oilt|oill"}, `proto_eng_json()`
 *                     pentru cloud, comanda `ENG` pe consola.
 *   v2.2 2026-09-19 — fw 2.4.0: DTCALL (consola + {"cmd":"dtcall"}): lista de instalare a gateway-ului (`22 04A1`) + DTC pe modulele
 *                     cu erori, scan automat in fundal, difuzare {"t":"gwil"/"dtc"/"dtcdone"} si retrimitere la `sub`; `proto_dtc_json()`.
 */
#include "proto.h"
#include "config.h"
#include "cfg.h"
#include "veh.h"
#include "ota.h"
#include "obd_sched.h"
#include "obd_pid.h"
#include "jsonlite.h"
#include "obd_cmd.h"
#include "iso_tp.h"
#include "tp20.h"
#include "dtcall.h"
#include "sniff.h"     // v2.5.0: sniff_post_json / sniff_cloud_url pentru upload READEPROM
#include "sensors.h"   // v3.3: MAGCAL
#include "params.h"    // v3.4: parametri cu nume + bitmap PID-uri suportate
#include <WiFi.h>      // v2.5.0: WiFi.status() in READEPROM
#include "freertos/FreeRTOS.h"   // v2.5.9: mutex pt jurnalul BYOSDEBUG (loop core 1 <-> task retea core 0)
#include "freertos/semphr.h"

// ==== hook-uri implementate in .ino (acces la starea firmware-ului) ====
extern bool     can_send_iso(uint32_t id, const uint8_t *data, size_t len);   // ISO-TP SF
extern bool     can_set_mode(const char *mode);
extern const char *can_mode_name();
extern size_t   can_stat_json(char *buf, size_t sz);
extern void     can_selftest_wiring(char *verdict, size_t vsz, char *hint, size_t hsz);
extern uint32_t can_last_frame_age();
extern uint32_t can_last_ecu_age();
extern bool     net_online();
extern String   net_ip();
extern void     proto_pump(uint32_t ms);          // ruleaza loop-ul CAN/net cateva ms
extern void     proto_keepalive();                // v2.4.0: senzori catre clienti cat timp o operatie lunga blocheaza loop()
extern bool     sniff_start(const char *label, const char *url, uint8_t owner);
extern String   sniff_stop_upload();
extern bool     sniff_active();
extern uint32_t sniff_count();
extern bool     can_autodetect(String &msg);      // v3.4 (.ino): autodetectie viteza + adresare, pt profilul generic

static proto_reply_fn g_reply = nullptr;

// ==== captura raspuns punctual (VIN/KM/SELFTEST) ====
static bool     g_await = false;
static uint32_t g_await_id = 0;
static uint8_t  g_await_sid = 0;      // SID asteptat in raspuns (ex 0x41, 0x49, 0x62)
static uint8_t  g_abuf[ISO_TP_MSG_MAX];
static size_t   g_alen = 0;
static bool     g_agot = false;

static bool await_resp(uint32_t req_id, const uint8_t *req, size_t rlen, uint8_t want_sid, uint32_t timeout_ms) {
    g_await = true; g_await_id = req_id; g_await_sid = want_sid; g_agot = false; g_alen = 0;
    if (!can_send_iso(req_id, req, rlen)) { g_await = false; return false; }
    uint32_t t0 = millis();
    while (!g_agot && millis() - t0 < timeout_ms) proto_pump(5);
    g_await = false;
    return g_agot;
}

// apelat din .ino pt fiecare mesaj ISO-TP reasamblat (feed scheduler + await)
bool proto_consume_obd(uint32_t src_id, const uint8_t *data, size_t len) {
    osch_on_message(src_id, data, len, millis());
    // await: raspunsul vine pe ID-ul de raspuns (ex. 0x77E la cererea 0x714, 0x7E8 la 0x7DF), nu pe req —
    // asa ca potrivim dupa SID, nu dupa src_id (in timpul unui probe manual e o singura cerere in asteptare)
    if (g_await && !g_agot && len >= 1) {
        bool match = (data[0] == g_await_sid) ||
                     (data[0] == 0x7F && len >= 3 && data[2] != 0x78);   // NRC final = si el raspuns
        if (match) {
            g_await_id = src_id;   // retine de pe ce ID a venit (pt raportare)
            g_alen = len > sizeof(g_abuf) ? sizeof(g_abuf) : len;
            memcpy(g_abuf, data, g_alen);
            g_agot = true;
        }
    }
    return false;
}

// v2.5.9: BYOSDEBUG — jurnalul consolei (comanda + raspunsul text) se aduna aici si il urca task-ul de retea (core 0),
// deci comenzile si CAN-ul nu asteapta dupa HTTP. Liniile JSON (incep cu '{') nu intra in jurnal.
static bool     g_dbg_on = false;
static char     g_dbg_label[32] = "";
static String   g_dbg_cur;              // textul comenzii in curs (loop, core 1)
static String   g_dbg_ready;            // gata de urcat (protejat de g_dbg_mx)
static uint32_t g_dbg_chunk = 0, g_dbg_sent = 0, g_dbg_lost = 0;
static bool     g_dbg_stop_pending = false;
static SemaphoreHandle_t g_dbg_mx = nullptr;
#define DBG_CUR_FLUSH   2048            // comenzile lungi (UDS repetat, DTCALL) se trimit pe bucati
#define DBG_READY_MAX   24576           // peste atat (retea cazuta) se pierde cel mai vechi text, cu contor
static void dbg_move_ready() {
    if (!g_dbg_cur.length() || !g_dbg_mx) return;
    xSemaphoreTake(g_dbg_mx, portMAX_DELAY);
    if (g_dbg_ready.length() + g_dbg_cur.length() > DBG_READY_MAX) {
        g_dbg_lost += g_dbg_ready.length(); g_dbg_ready = "[... text pierdut: server indisponibil ...]\n";
    }
    g_dbg_ready += g_dbg_cur;
    xSemaphoreGive(g_dbg_mx);
    g_dbg_cur = "";
}
static bool     g_dbg_console = false;   // v2.5.10: comanda de consola in curs => prindem si liniile JSON (ex. STAT)
static void dbg_capture(const char *s, size_t n) {
    if (!g_dbg_on || !n || (s[0] == '{' && !g_dbg_console)) return;
    g_dbg_cur.concat(s, n);
    if (g_dbg_cur.length() >= DBG_CUR_FLUSH) dbg_move_ready();
}
static void reply(uint8_t owner, const char *s) { size_t n = strlen(s); dbg_capture(s, n); if (g_reply) g_reply(owner, s, n); }
static void replyf(uint8_t owner, const char *fmt, ...) {
    char b[512]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
    size_t n = strlen(b); dbg_capture(b, n);
    if (g_reply) g_reply(owner, b, n);
}

void proto_init(proto_reply_fn r) { g_reply = r; }
static void pd_unsub_owner(uint8_t owner);                        // v3.4 (definita langa DID-urile din profil)
static const char *sub_by_name(uint8_t owner, const char *n, double hz, uint32_t now);   // v3.4
static void params_list(uint8_t owner, bool json);                // v3.4
void proto_client_gone(uint8_t owner) { osch_remove_owner(owner); pd_unsub_owner(owner); }

static void dtcall_run(uint8_t owner, bool all, bool text);      // v2.4.0 (definite mai jos, dupa helperii UDS)
static void cmd_readeprom(uint8_t owner, char *tok[], int nt);   // v2.5.0 (definita la final)
static void cmd_coding(uint8_t owner, char *tok[], int nt);       // v2.5.5 (definite la final)
static void cmd_tpq(uint8_t owner, char *tok[], int nt);          // v2.5.6 (definita la final)
static void cmd_udsscan(uint8_t owner, char *tok[], int nt);      // v2.5.7
static void cmd_didscan(uint8_t owner, char *tok[], int nt);      // v2.5.9
static String pd_json();                                          // v2.5.11 (definita dupa proto_eng_json)
static void codingall_run(uint8_t owner, uint16_t did, const char *name);
static void coding_sanitize(char *s);                              // v2.5.5
#define CODING_DID_DEF   0x0600                                      // DID long coding (confirmat pe BCM)
static void dtcall_replay(uint8_t owner);
static void dtcall_auto_tick(uint32_t now);

// ==================== JSON protocol (F16 / scripturi) ====================

// v3.4 — PE NUME (placa alege sursa):   {"cmd":"sub","items":[{"n":"rpm","hz":5},{"n":"odoKm"},{"n":"*"}]}
//        "*" = toate valorile din profilul vehiculului marcate pentru aplicatie (clientul nu le stie numele dinainte)
// vechi — PID dat de client (F16 <= 1.83): {"cmd":"sub","items":[{"k":"rpm","src":"obd","pid":12,"hz":20}]}
// Un item care are si "n", si k/pid (F16 nou vorbind cu firmware necunoscut) e tratat PE NUME.
// "add":1 = ADAUGA la abonamentul existent al clientului (fara el, `sub` il inlocuieste). Asa un client isi poate trimite
// lista in mai multe linii scurte, fara sa depinda de lungimea maxima a liniei (JSON_LINE_MAX).
static bool name_ok(const char *s) {
    if (!*s) return false;
    if (!strcmp(s, "*")) return true;
    for (; *s; s++) if (!isalnum((unsigned char)*s) && *s != '_') return false;
    return true;
}
static void json_sub(uint8_t owner, const char *line) {
    const char *arr = strstr(line, "\"items\"");
    if (!arr) { reply(owner, "{\"type\":\"sub\",\"ok\":0,\"err\":\"items lipsa\"}\n"); return; }
    long add = 0; jl_int(line, "add", &add);
    if (arr > line) {                                   // "add" de la nivelul comenzii, nu din vreun item
        char head[48]; size_t hl = (size_t)(arr - line); if (hl >= sizeof(head)) hl = sizeof(head) - 1;
        memcpy(head, line, hl); head[hl] = 0;
        add = 0; jl_int(head, "add", &add);
    }
    if (!add) { osch_remove_owner(owner); pd_unsub_owner(owner); }
    size_t pos = (size_t)(arr - line);
    char obj[160]; int added = 0, skipped = 0; uint32_t now = millis();
    String res;
    while (jl_next_object(line, &pos, obj, sizeof(obj))) {
        char n[OSCH_KEY_MAX] = "", k[OSCH_KEY_MAX] = "", src[8] = "obd";
        long pid = -1; double hz = 0;
        jl_str(obj, "n", n, sizeof(n));
        jl_str(obj, "k", k, sizeof(k));
        jl_str(obj, "src", src, sizeof(src));
        jl_int(obj, "pid", &pid);
        jl_num(obj, "hz", &hz);
        if (n[0]) {
            const char *how = name_ok(n) ? sub_by_name(owner, n, hz, now) : "none";
            if (strcmp(how, "none") != 0 && strcmp(how, "full") != 0) added++; else skipped++;
            if (name_ok(n) && res.length() < 1400) { if (res.length()) res += ","; res += "\""; res += n; res += "\":\""; res += how; res += "\""; }
            continue;
        }
        if (strcmp(src, "obd") == 0 && pid >= 0 && pid <= 0xFF && k[0]) {
            uint32_t per = hz > 0 ? (uint32_t)(1000.0 / hz) : 1000;
            if (osch_add(k, (uint8_t)pid, per, owner, false, now) >= 0) added++; else skipped++;
        } else skipped++;
    }
    String o = String("{\"type\":\"sub\",\"ok\":1,\"proto\":") + PARAMS_PROTO + ",\"veh\":\"" + veh().model +
               "\",\"add\":" + (add ? 1 : 0) + ",\"added\":" + added + ",\"skipped\":" + skipped;
    if (res.length()) { o += ",\"res\":{"; o += res; o += "}"; }
    o += "}\n";
    reply(owner, o.c_str());
    if (!add) dtcall_replay(owner);   // v2.4.0: lista de instalare + erorile deja citite se ataseaza automat la abonament
}

static void json_dispatch(uint8_t owner, const char *line) {
    char cmd[24];
    if (!jl_str(line, "cmd", cmd, sizeof(cmd))) return;
    long pid = -1, id = -1, ecu = -1, did = -1, sub = -1;
    bool hpid = jl_int(line, "pid", &pid), hecu = jl_int(line, "ecu", &ecu),
         hdid = jl_int(line, "did", &did), hsub = jl_int(line, "sub", &sub);

    if (!strcmp(cmd, "sub")) { json_sub(owner, line); return; }
    if (!strcmp(cmd, "params")) { params_list(owner, true); return; }   // v3.4: ce nume se pot cere pe masina asta
    if (!strcmp(cmd, "veh")) {      // v3.4: {"cmd":"veh"} = ce profil ruleaza; {"cmd":"veh","model":"pq35"} = adu-l de pe server
        char model[24] = ""; String err; bool ok = true;
        if (jl_str(line, "model", model, sizeof(model)) && model[0] && strcmp(model, veh().model) != 0) {
            ok = veh_fetch(model, err);
            if (ok) { String e2; cfg_set("veh.model", model, e2); }
        }
        uint32_t br = 0; bool ext = false; veh_detected(&br, &ext);
        char e[128]; jl_escape(e, sizeof(e), err.c_str());
        replyf(owner, "{\"type\":\"veh\",\"ok\":%d,\"proto\":%d,\"fw\":\"%s\",\"model\":\"%s\",\"ver\":%u,\"auto\":%d,\"bitrate\":%lu,\"ext\":%d,\"err\":\"%s\"}\n",
               ok ? 1 : 0, PARAMS_PROTO, FW_VERSION, veh().model, (unsigned)veh().ver, (int)veh().autodet, (unsigned long)veh_bitrate(), ext ? 1 : 0, e);
        return;
    }
    if (!strcmp(cmd, "dtcall")) {   // v2.4.0: {"cmd":"dtcall","all":1} - all=1 citeste TOATE modulele codate, altfel doar cele cu erori
        long all = 0; jl_int(line, "all", &all);
        if (strcmp(can_mode_name(), "normal") != 0) can_set_mode("normal");
        dtcall_run(owner, all != 0, false);
        return;
    }
    if (!strcmp(cmd, "obd1") && hpid) {
        osch_add("", (uint8_t)pid, 0, owner, true, millis());   // legacy F16 1.75: oneshot prin planificator
        return;
    }
    if (!strcmp(cmd, "ping")) { replyf(owner, "{\"ts\":%lu,\"type\":\"pong\"}\n", (unsigned long)millis()); return; }
    if (!strcmp(cmd, "stat")) { char b[640]; size_t n = can_stat_json(b, sizeof(b)); if (n) g_reply(owner, b, n); return; }
    if (!strcmp(cmd, "cfg_get")) { String j = cfg_json(true); replyf(owner, "{\"type\":\"cfg\",\"cfg\":%s}\n", j.c_str()); return; }
    if (!strcmp(cmd, "cfg_set")) {
        char key[24], val[210]; String err;
        if (!jl_str(line, "key", key, sizeof(key)) || !jl_str(line, "val", val, sizeof(val))) { reply(owner, "{\"type\":\"cfg_set\",\"ok\":0,\"err\":\"key/val lipsa\"}\n"); return; }
        bool ok = cfg_set(key, val, err);
        char e[128]; jl_escape(e, sizeof(e), err.c_str());
        replyf(owner, "{\"type\":\"cfg_set\",\"ok\":%d,\"key\":\"%s\",\"err\":\"%s\"}\n", ok, key, e);
        return;
    }
    if (!strcmp(cmd, "ota_check")) {
        OtaInfo inf; String err;
        if (ota_check(inf, err)) replyf(owner, "{\"type\":\"ota\",\"ok\":1,\"fw\":\"%s\",\"cur\":\"%s\",\"newer\":%d,\"size\":%ld,\"notes\":\"%s\"}\n",
                                        inf.latest.c_str(), FW_VERSION, inf.newer, inf.size, inf.notes.c_str());
        else { char e[160]; jl_escape(e, sizeof(e), err.c_str()); replyf(owner, "{\"type\":\"ota\",\"ok\":0,\"err\":\"%s\"}\n", e); }
        return;
    }
    if (!strcmp(cmd, "ota_update")) {
        // download = O SINGURA sesiune TLS (fragmentarea heap ESP32 pica a 2-a). Folosim rezultatul memorat
        // de la un ota_check anterior; daca lipseste, verificam acum si cerem retrimiterea (F16: check -> update).
        OtaInfo inf; String err;
        if (!ota_cached_newer(inf)) {
            if (!ota_check(inf, err)) { char e[160]; jl_escape(e, sizeof(e), err.c_str()); replyf(owner, "{\"type\":\"ota\",\"ok\":0,\"err\":\"%s\"}\n", e); return; }
            if (!inf.newer) { replyf(owner, "{\"type\":\"ota\",\"ok\":0,\"err\":\"deja la %s\"}\n", FW_VERSION); return; }
            replyf(owner, "{\"type\":\"ota\",\"ok\":0,\"phase\":\"checked\",\"fw\":\"%s\",\"retry\":1}\n", inf.latest.c_str());
            return;
        }
        replyf(owner, "{\"type\":\"ota\",\"phase\":\"download\",\"fw\":\"%s\"}\n", inf.latest.c_str());
        if (ota_update(inf, err, nullptr)) { reply(owner, "{\"type\":\"ota\",\"ok\":1,\"phase\":\"done\",\"reboot\":1}\n"); proto_pump(300); ESP.restart(); }
        else { char e[160]; jl_escape(e, sizeof(e), err.c_str()); replyf(owner, "{\"type\":\"ota\",\"ok\":0,\"err\":\"%s\"}\n", e); }
        return;
    }
    if (!strcmp(cmd, "mode")) {
        char m[12]; if (!jl_str(line, "m", m, sizeof(m))) { reply(owner, "{\"type\":\"mode\",\"ok\":0}\n"); return; }
        bool ok = can_set_mode(m); osch_set_enabled(strcmp(can_mode_name(), "listen") != 0);
        replyf(owner, "{\"type\":\"mode\",\"ok\":%d,\"mode\":\"%s\"}\n", ok, can_mode_name());
        return;
    }
    if (!strcmp(cmd, "raw")) {
        char ds[64]; uint8_t data[8]; long pad = 1;
        if (!jl_int(line, "id", &id) || id < 0 || id > 0x7FF) { reply(owner, "{\"type\":\"tx\",\"ok\":0,\"err\":\"id\"}\n"); return; }
        if (!jl_str(line, "data", ds, sizeof(ds))) ds[0] = 0;
        size_t n = jl_hex(ds, data, 8);
        jl_int(line, "pad", &pad);
        // pastram comportamentul legacy pt scripturile de test (obd_speed.py/vcds_sniff): frame brut
        extern bool can_send_raw_frame(uint32_t id, const uint8_t *d, size_t n, bool pad);
        can_send_raw_frame((uint32_t)id, data, n, pad != 0);
        return;
    }
    if (!strcmp(cmd, "probe")) { osch_add("", 0x00, 0, owner, true, millis()); return; }
    if (!strcmp(cmd, "obd9") && hpid) { obd_query_info09((uint8_t)pid); return; }
    if (!strcmp(cmd, "obd3")) { obd_query_dtcs(); return; }
    if (!strcmp(cmd, "uds22") && hdid) { uds_read_did(hecu ? (uint32_t)ecu : 0x7E0, (uint16_t)did); return; }
    if (!strcmp(cmd, "uds10")) { uds_diag_session(hecu ? (uint32_t)ecu : 0x7E0, hsub ? (uint8_t)sub : 0x03); return; }
    // obd4 (sterge DTC) — refuzat implicit (decizie user: doar din consola/F16 cu confirmare)
    if (!strcmp(cmd, "obd4")) { reply(owner, "{\"type\":\"err\",\"err\":\"stergere DTC blocata (foloseste consola: DTC CLR <ecu>)\"}\n"); return; }
    replyf(owner, "{\"type\":\"err\",\"cmd\":\"%s\",\"err\":\"necunoscut\"}\n", cmd);
}

void proto_json_line(uint8_t owner, const char *line) { json_dispatch(owner, line); }

// ==================== consola text (telnet / serial, stil AT) ====================

static bool g_explore[MAX_JSON_CLIENTS + MAX_TELNET_CLIENTS + 2] = {false};

bool proto_console_prompt(uint8_t owner, char *out, size_t sz) {
    snprintf(out, sz, (owner < sizeof(g_explore) && g_explore[owner]) ? "EXPLORE# " : "ergCAN> ");
    return true;
}

// citeste VIN prin OBD 09 02 (ISO-TP multi-frame). Intoarce "" daca nu raspunde.
static String read_vin() {
    uint8_t req[2] = {0x09, 0x02};
    if (!await_resp(obd_func_id(), req, 2, 0x49, 1500)) return "";
    // 49 02 [NumItems] V I N ...
    String v; size_t off = (g_alen > 3 && g_abuf[2] <= 1) ? 3 : 2;
    for (size_t i = off; i < g_alen; i++) { char c = (char)g_abuf[i]; if (c >= 0x20 && c < 0x7F) v += c; }
    v.trim(); return v;
}

// KM prin UDS 22 2203 la 0x714 -> 0x77E. Intoarce -1 daca nu raspunde.
// Cluster PQ35 SEAT/VW (Altea): raspunsul e 62 22 03 <hi> <lo> (2 octeti date) = km/10 -> km = u16*10
//   (verificat pe masina: 2203=0x7AAE=31406 -> 314060, bord 314061). Alte clustere/WiCAN: 3 octeti date = km direct.
// v2.3.0: adresa bordului si DID-ul vin din PROFILUL vehiculului (veh.h), nu mai sunt hardcodate.
static char g_km_src = '-';   // v2.5: 'E' = ECU motor, 'B' = bord, '-' = indisponibil
static long read_km_ecu() {
    const VehProfile &v = veh();
    if (!v.ekm_req || !v.ekm_did || v.ekm_fmt == VEH_KM_NONE) return -1;
    iso_tp_allow(veh_resp_id(v.ekm_req), v.ekm_req);
    uint8_t req[3] = {0x22, (uint8_t)(v.ekm_did >> 8), (uint8_t)(v.ekm_did & 0xFF)};
    if (!await_resp(v.ekm_req, req, 3, 0x62, 1200) || g_alen < 4 || g_abuf[0] != 0x62) return -1;
    if (g_abuf[1] != (uint8_t)(v.ekm_did >> 8) || g_abuf[2] != (uint8_t)(v.ekm_did & 0xFF)) return -1;   // DID ecou
    size_t n = g_alen - 3; const uint8_t *d = g_abuf + 3;
    if (v.ekm_fmt == VEH_KM_U32) { if (n < 4) return -1; return ((long)d[0] << 24) | ((long)d[1] << 16) | ((long)d[2] << 8) | d[3]; }
    if (v.ekm_fmt == VEH_KM_U24) { if (n < 3) return -1; return ((long)d[0] << 16) | ((long)d[1] << 8) | d[2]; }
    if (n < 2) return -1;
    long raw = ((long)d[0] << 8) | d[1];
    return (v.ekm_fmt == VEH_KM_U16X10) ? raw * 10L : raw;
}
static long read_km_bord();
static long read_km() {
    long k = read_km_ecu();                       // v2.5: ECU intai (1 km rezolutie asteptata), bordul e rezerva (x10)
    if (k >= 0) { g_km_src = 'E'; return k; }
    k = read_km_bord();
    g_km_src = (k >= 0) ? 'B' : '-';
    return k;
}
static long read_km_bord() {
    const VehProfile &v = veh();
    if (!v.cluster_req || !v.km_did || v.km_fmt == VEH_KM_NONE) return -1;   // masina fara bord interogabil
    iso_tp_allow(veh_resp_id(v.cluster_req), v.cluster_req);
    uint8_t req[3] = {0x22, (uint8_t)(v.km_did >> 8), (uint8_t)(v.km_did & 0xFF)};
    if (!await_resp(v.cluster_req, req, 3, 0x62, 1200) || g_alen < 4 || g_abuf[0] != 0x62) return -1;
    if (g_abuf[1] != (uint8_t)(v.km_did >> 8) || g_abuf[2] != (uint8_t)(v.km_did & 0xFF)) return -1;   // DID ecou
    if (v.km_fmt == VEH_KM_U24) {
        if (g_alen < 6) return -1;
        return ((long)g_abuf[3] << 16) | ((long)g_abuf[4] << 8) | g_abuf[5];
    }
    if (g_alen < 5) return -1;
    long raw = ((long)g_abuf[3] << 8) | g_abuf[4];
    return (v.km_fmt == VEH_KM_U16X10) ? raw * 10L : raw;
}

// Litri in rezervor prin UDS 22 2206 la 0x714 -> 0x77E (bord). Intoarce -1 daca nu raspunde.
// CONFIRMAT pe masina (Altea PQ35, 2026-09-16): 2206 SI 2212 dau acelasi u8 = litri
//   (0x14=20 cand erau ~21 L, 0x10=16 cand erau 16 L) -> ipoteza "unul e temperatura" a picat.
//   Citim 2206; daca nu raspunde, incercam 2212 ca rezerva (DID-ul care a mers ramane memorat).
static uint16_t g_fuel_did_ok = 0;            // v2.2.5: DID-ul de pe care a venit ultima valoare buna
static long read_fuel() {
    const VehProfile &v = veh();              // v2.3.0: DID-urile vin din profilul vehiculului
    if (!v.cluster_req || !v.fuel_did || v.fuel_fmt == VEH_FUEL_NONE) return -1;
    const uint16_t FUEL_DIDS[2] = { v.fuel_did, v.fuel_alt ? v.fuel_alt : v.fuel_did };
    static uint8_t        idx = 0;            // DID-ul care a raspuns ultima data
    iso_tp_allow(veh_resp_id(v.cluster_req), v.cluster_req);
    for (int t = 0; t < 2; t++) {
        uint16_t did = FUEL_DIDS[(idx + t) & 1];
        uint8_t req[3] = {0x22, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF)};
        // v2.2.5: VERIFICAM DID-ul ecou din raspuns (62 <hi> <lo> <val>). `await_resp` potriveste doar dupa SID,
        // iar cand F16 e abonat si busul e plin de trafic putea fi acceptat raspunsul altei cereri 0x62
        // (sau al celuilalt DID) => valoarea sarea 14 -> 6 -> 21 L, desi rezervorul avea ~16 L.
        if (await_resp(v.cluster_req, req, 3, 0x62, 1200) && g_alen >= 4 && g_abuf[0] == 0x62 &&
            g_abuf[1] == (uint8_t)(did >> 8) && g_abuf[2] == (uint8_t)(did & 0xFF)) {
            idx = (idx + t) & 1;
            g_fuel_did_ok = did;
            return (long)g_abuf[3];
        }
    }
    return -1;
}

// ==== v2.2: cache VIN/KM citit in fundal (pt cloud + F16, fara sa blocheze cand bus-ul e mut) ====
static String   g_vin_cache;
static long     g_km_cache = -1;
static long     g_fuel_cache = -1;                               // v2.2.3: litri in rezervor (UDS 2206/2212)
static uint32_t g_vin_try_ms = 0, g_km_try_ms = 0, g_fuel_try_ms = 0;
extern void emitJson(const char *line, size_t len);   // broadcast la clientii JSON (F16)
// v3.4: forma UNICA a unei valori, indiferent de sursa (obd / uds din profil / veh = citita de firmware)
static void emit_d(const char *k, double v, int dec, const char *src) {
    char b[112]; int n = snprintf(b, sizeof(b), "{\"t\":\"d\",\"k\":\"%s\",\"v\":%.*f,\"src\":\"%s\"}\n", k, dec, v, src);
    if (n > 0 && (size_t)n < sizeof(b)) emitJson(b, (size_t)n);
}
// ==== v2.3.3: valori de pe ECU MOTOR prin UDS 22 — generator (load/performance), temperatura si nivel ulei ====
// DID-urile au fost gasite cu SNIFF pe VCDS (docs/research/sniff_vcds_20260917.md) si vin din profilul vehiculului.
// Formate fixe: gen = u8 x 100/256 %, temp ulei = u8 - 60 °C, nivel ulei = u16 / 16 mm.
struct EngSlot { const char *t; const char *k; uint8_t nbytes; uint32_t per_ms; uint32_t try_ms; uint32_t ok_ms; uint8_t fails; uint8_t nrc; float val; bool have; };
static EngSlot g_eng[4] = {      // t = linia veche {"t":"genload"}; k = numele din {"t":"d","k":"genLoad"} (v3.4)
    {"genload", "genLoad",  1,  3000, 0, 0, 0, 0, 0, false},
    {"genperf", "genPerf",  1,  3000, 0, 0, 0, 0, 0, false},
    {"oilt",    "oilTemp",  1, 10000, 0, 0, 0, 0, 0, false},
    {"oill",    "oilLevel", 2, 30000, 0, 0, 0, 0, 0, false},
};
#define ENG_N            4
#define ENG_TIMEOUT_MS   300      // ECU raspunde in 2-10 ms (masurat in SNIFF); 300 ms nu infometeaza planificatorul
#define ENG_BACKOFF_MS   60000    // dupa 3 esecuri la rand, DID-ul se reincearca doar o data pe minut
#define ENG_OILL_CEIL    0x0780   // 120,0 mm = plafonul senzorului cu motorul pornit, nu o masuratoare

static uint16_t eng_did(int i) {
    const VehProfile &v = veh();
    return i == 0 ? v.genload_did : i == 1 ? v.genperf_did : i == 2 ? v.oilt_did : v.oill_did;
}
// Intoarce valoarea BRUTA sau -1 (fara raspuns / NRC / ecou DID gresit). Motivul ramane in slot (`nrc`), pentru consola.
static long read_eng_raw(int i) {
    const VehProfile &v = veh(); uint16_t did = eng_did(i);
    if (!v.eng_req || !did) return -1;
    iso_tp_allow(veh_resp_id(v.eng_req), v.eng_req);
    uint8_t req[3] = {0x22, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF)};
    g_eng[i].nrc = 0;
    if (!await_resp(v.eng_req, req, 3, 0x62, ENG_TIMEOUT_MS)) { g_eng[i].nrc = 0xFF; return -1; }   // 0xFF = tacere
    if (g_alen >= 1 && g_abuf[0] == 0x7F) { g_eng[i].nrc = g_alen >= 3 ? g_abuf[2] : 0xFE; return -1; }
    // DID-ul ecou se verifica (lectia din 2.2.5): `await_resp` potriveste doar dupa SID
    if (g_alen < (size_t)(3 + g_eng[i].nbytes) || g_abuf[0] != 0x62 ||
        g_abuf[1] != (uint8_t)(did >> 8) || g_abuf[2] != (uint8_t)(did & 0xFF)) { g_eng[i].nrc = 0xFD; return -1; }
    return g_eng[i].nbytes == 2 ? (((long)g_abuf[3] << 8) | g_abuf[4]) : (long)g_abuf[3];
}
static float eng_scale(int i, long raw) {
    if (i <= 1) return raw * 100.0f / 256.0f;
    if (i == 2) return (float)raw - 60.0f;
    return raw / 16.0f;
}
// O singura citire pe apel (prima restanta).
static void eng_tick(uint32_t now) {
    if (!veh().eng_req) return;
    for (int i = 0; i < ENG_N; i++) {
        EngSlot &s = g_eng[i];
        if (!eng_did(i)) continue;
        uint32_t per = s.fails >= 3 ? ENG_BACKOFF_MS : s.per_ms;
        if (s.try_ms && now - s.try_ms < per) continue;
        s.try_ms = now ? now : 1;
        long raw = read_eng_raw(i);
        if (raw < 0) { if (s.fails < 255) s.fails++; return; }
        s.fails = 0;
        if (i == 3 && raw == ENG_OILL_CEIL) return;           // plafon: pastram ultima masuratoare reala
        s.val = eng_scale(i, raw); s.have = true; s.ok_ms = now;
        char b[56]; int n = snprintf(b, sizeof(b), "{\"t\":\"%s\",\"v\":%.1f}\n", s.t, s.val);
        if (n > 0) emitJson(b, n);
        emit_d(s.k, s.val, 1, "veh");
        return;
    }
}
// fragment pentru snapshot-ul cloud: doar valorile proaspete (4x perioada), altfel ar ramane „inghetate" pe server
String proto_eng_json() {
    String o; uint32_t now = millis();
    for (int i = 0; i < ENG_N; i++) {
        const EngSlot &s = g_eng[i];
        if (!s.have || now - s.ok_ms > s.per_ms * 4UL) continue;
        o += ",\""; o += s.t; o += "\":"; o += String(s.val, 1);
    }
    o += pd_json();                                  // v2.5.11
    return o;
}
static void cmd_eng(uint8_t owner) {
    const VehProfile &v = veh();
    if (!v.eng_req) { reply(owner, "ENG: profilul vehiculului nu are valori de motor pe UDS\r\n"); return; }
    for (int i = 0; i < ENG_N; i++) {
        uint16_t did = eng_did(i); if (!did) continue;
        long raw = read_eng_raw(i);
        if (raw >= 0) replyf(owner, "  %-8s 0x%03X DID %04X raw %ld = %.1f%s\r\n", g_eng[i].t, v.eng_req, did, raw, eng_scale(i, raw),
                             (i == 3 && raw == ENG_OILL_CEIL) ? "  (plafon senzor, ignorat)" : "");
        else if (g_eng[i].nrc == 0xFF) replyf(owner, "  %-8s 0x%03X DID %04X: fara raspuns\r\n", g_eng[i].t, v.eng_req, did);
        else replyf(owner, "  %-8s 0x%03X DID %04X: NRC 0x%02X\r\n", g_eng[i].t, v.eng_req, did, g_eng[i].nrc);
    }
}

// ==== v2.5.11: DID-uri GENERICE din profilul vehiculului (cheia "dids") ====
// {"n":"lant_p2","req":"0x7E0","did":"0x1189","nb":2,"mul":0.01,"off":0,"s":10,"dec":2}
//   n = nume scurt (cheie in F16/cloud), req = ID cerere UDS, did = DID, nb = 1|2|4 octeti (u, big-endian),
//   valoare = raw * mul + off, s = perioada de CITIRE in secunde, dec = zecimale afisate,
//   up = 1 urca pe server in snapshot-ul cloud (la cloud.int_s), f16 = 1 se trimite la F16 la fiecare citire (implicit ambele 1).
// v3.4 (fw 2.6.0): sg = 1 valoare CU SEMN (complement fata de 2 pe nb octeti); bg = 0 => se citeste DOAR cat timp e abonat
//   cineva la numele ei (implicit 1 = citita mereu „in fundal", ca pana acum). Un client abonat poate cere si o frecventa mai mare (hz).
struct PDid { char n[16]; uint16_t req, did; uint8_t nb, dec, fails, nrc, up, f16, sg, autord; float mul, off;
              uint32_t per_ms, try_ms, ok_ms, sub_ms; uint16_t sub_mask; long raw; float val; bool have; };
#define PD_MAX 24
#define PD_MIN_PERIOD_MS 200      // un DID se citeste cu cerere blocanta (<= ENG_TIMEOUT_MS): nu mai des de 5/s
static PDid g_pd[PD_MAX];
static int  g_npd = 0;
int proto_load_dids(const char *json) {
    g_npd = 0;
    if (!json) return 0;
    const char *k = strstr(json, "\"dids\"");
    if (!k) return 0;
    const char *arr = strchr(k, '[');
    if (!arr) return 0;
    size_t pos = 1; char obj[224]; long v; double d;
    while (g_npd < PD_MAX && jl_next_object(arr, &pos, obj, sizeof(obj))) {
        PDid p; memset(&p, 0, sizeof(p));
        if (!jl_str(obj, "n", p.n, sizeof(p.n)) || !p.n[0]) continue;
        if (!jl_int(obj, "req", &v) || v < 0x100 || v > 0x7FF) continue; p.req = (uint16_t)v;
        if (!jl_int(obj, "did", &v) || v < 0 || v > 0xFFFF) continue;    p.did = (uint16_t)v;
        p.nb  = (jl_int(obj, "nb", &v) && (v == 1 || v == 2 || v == 4)) ? (uint8_t)v : 2;
        p.mul = jl_num(obj, "mul", &d) ? (float)d : 1.0f;
        p.off = jl_num(obj, "off", &d) ? (float)d : 0.0f;
        p.per_ms = (jl_int(obj, "s", &v) && v >= 1 && v <= 3600) ? (uint32_t)v * 1000UL : 10000UL;
        p.dec = (jl_int(obj, "dec", &v) && v >= 0 && v <= 4) ? (uint8_t)v : 2;
        p.up  = jl_int(obj, "up", &v)  ? (uint8_t)(v != 0) : 1;
        p.f16 = jl_int(obj, "f16", &v) ? (uint8_t)(v != 0) : 1;
        p.sg     = jl_int(obj, "sg", &v)   ? (uint8_t)(v != 0) : 0;
        p.autord = jl_int(obj, "bg", &v) ? (uint8_t)(v != 0) : 1;     // NU "auto": cheia aceea e a profilului (autodetectie)
        g_pd[g_npd++] = p;
    }
    return g_npd;
}
// valoarea bruta sau -1; motivul in p.nrc (0xFF tacere, 0xFD ecou gresit, altfel NRC)
static long pd_read(PDid &p) {
    iso_tp_allow(veh_resp_id(p.req), p.req);
    uint8_t q[3] = {0x22, (uint8_t)(p.did >> 8), (uint8_t)p.did};
    p.nrc = 0;
    if (!await_resp(p.req, q, 3, 0x62, ENG_TIMEOUT_MS)) { p.nrc = 0xFF; return -1; }
    if (g_alen >= 1 && g_abuf[0] == 0x7F) { p.nrc = g_alen >= 3 ? g_abuf[2] : 0xFE; return -1; }
    if (g_alen < (size_t)(3 + p.nb) || g_abuf[0] != 0x62 || g_abuf[1] != q[1] || g_abuf[2] != q[2]) { p.nrc = 0xFD; return -1; }
    long r = 0; for (uint8_t i = 0; i < p.nb; i++) r = (r << 8) | g_abuf[3 + i];
    return r;
}
// v3.4: valoarea fizica din cea bruta (pd_read intoarce mereu brutul FARA semn, ca -1 sa ramana „citire esuata")
static float pd_scale(const PDid &p, long raw) {
    long s = raw;
    if (p.sg) {
        if (p.nb == 1 && raw > 0x7F) s = raw - 0x100L;
        else if (p.nb == 2 && raw > 0x7FFFL) s = raw - 0x10000L;
        else if (p.nb == 4) s = (long)(int32_t)(uint32_t)raw;
    }
    return (float)s * p.mul + p.off;
}
static uint32_t pd_period(const PDid &p) {
    if (p.fails >= 3) return ENG_BACKOFF_MS;
    return (p.sub_mask && p.sub_ms && p.sub_ms < p.per_ms) ? p.sub_ms : p.per_ms;
}
static void pd_tick(uint32_t now) {
    for (int i = 0; i < g_npd; i++) {
        PDid &p = g_pd[i];
        if (!p.autord && !p.sub_mask) continue;       // v3.4: citit doar la cerere si nu-l cere nimeni
        if (p.try_ms && now - p.try_ms < pd_period(p)) continue;
        p.try_ms = now ? now : 1;
        long raw = pd_read(p);
        if (raw < 0) { if (p.fails < 255) p.fails++; return; }
        p.fails = 0; p.raw = raw; p.val = pd_scale(p, raw); p.have = true; p.ok_ms = now;
        // v3.4: aceeasi forma ca orice alta valoare ({"t":"d"}); linia {"t":"did"} din 2.5.11 nu avea niciun consumator
        char b[112]; int n = snprintf(b, sizeof(b), "{\"t\":\"d\",\"k\":\"%s\",\"v\":%.*f,\"raw\":%ld,\"src\":\"uds\"}\n", p.n, (int)p.dec, p.val, raw);
        if (n > 0 && (size_t)n < sizeof(b) && (p.f16 || p.sub_mask)) emitJson(b, (size_t)n);
        return;                                       // o singura citire pe apel
    }
}
static String pd_json() {                             // fragment cloud: doar valorile proaspete (4x perioada)
    String o; uint32_t now = millis(); bool any = false;
    for (int i = 0; i < g_npd; i++) {
        const PDid &p = g_pd[i];
        if (!p.up || !p.have || now - p.ok_ms > p.per_ms * 4UL) continue;
        o += any ? "," : ",\"dids\":{"; any = true;
        o += "\""; o += p.n; o += "\":"; o += String(p.val, (unsigned int)p.dec);
    }
    if (any) o += "}";
    return o;
}
static void cmd_dids(uint8_t owner) {
    if (!g_npd) { reply(owner, "DIDS: profilul vehiculului nu are cheia \"dids\" (VEH GET <model> dupa actualizarea profilului)\r\n"); return; }
    replyf(owner, "DIDS: %d valori din profil (citite acum)\r\n", g_npd);
    for (int i = 0; i < g_npd; i++) {
        PDid &p = g_pd[i];
        long raw = pd_read(p);
        if (raw >= 0) { p.raw = raw; p.val = pd_scale(p, raw); p.have = true; p.ok_ms = millis();
            replyf(owner, "  %-14s 0x%03X DID %04X raw %6ld = %.*f%s%s\r\n", p.n, p.req, p.did, raw, (int)p.dec, p.val,
                   p.autord ? "" : "  [la cerere]", p.sub_mask ? "  [abonat]" : ""); }
        else if (p.nrc == 0xFF) replyf(owner, "  %-14s 0x%03X DID %04X: fara raspuns\r\n", p.n, p.req, p.did);
        else replyf(owner, "  %-14s 0x%03X DID %04X: NRC 0x%02X\r\n", p.n, p.req, p.did, p.nrc);
        proto_pump(2);
    }
}

// ==== v3.4 (fw 2.6.0): ABONARE PE NUME — rezolvarea numelui in sursa ====
static int pd_find(const char *n) { for (int i = 0; i < g_npd; i++) if (!strcmp(g_pd[i].n, n)) return i; return -1; }
static uint16_t owner_bit(uint8_t owner) { return owner < 16 ? (uint16_t)(1u << owner) : 0; }
static void pd_sub(PDid &p, uint8_t owner, double hz) {
    p.sub_mask |= owner_bit(owner);
    if (hz > 0) {
        uint32_t per = (uint32_t)(1000.0 / hz);
        if (per < PD_MIN_PERIOD_MS) per = PD_MIN_PERIOD_MS;
        if (!p.sub_ms || per < p.sub_ms) p.sub_ms = per;
    }
    if (p.fails >= 3) { p.fails = 0; p.try_ms = 0; }     // abonare noua => o reincercam acum, nu peste un minut
}
static void pd_unsub_owner(uint8_t owner) {
    uint16_t b = owner_bit(owner);
    for (int i = 0; i < g_npd; i++) { g_pd[i].sub_mask &= (uint16_t)~b; if (!g_pd[i].sub_mask) g_pd[i].sub_ms = 0; }
}
// valorile pe care le citeste firmware-ul din campurile FIXE ale profilului (km, litri, generator, ulei) + VIN
static const char *const VEH_NAMES[] = {"vin", "odoKm", "fuelL", "genLoad", "genPerf", "oilTemp", "oilLevel"};
#define VEH_NAMES_N (sizeof(VEH_NAMES) / sizeof(VEH_NAMES[0]))
static bool veh_param_has(const char *n) {
    const VehProfile &v = veh();
    if (!strcmp(n, "vin"))      return true;                                     // mode 09 PID 02, standard
    if (!strcmp(n, "odoKm"))    return (v.ekm_req && v.ekm_did && v.ekm_fmt != VEH_KM_NONE) ||
                                       (v.cluster_req && v.km_did && v.km_fmt != VEH_KM_NONE);
    if (!strcmp(n, "fuelL"))    return v.cluster_req && v.fuel_did && v.fuel_fmt != VEH_FUEL_NONE;
    if (!strcmp(n, "genLoad"))  return v.eng_req && v.genload_did;
    if (!strcmp(n, "genPerf"))  return v.eng_req && v.genperf_did;
    if (!strcmp(n, "oilTemp"))  return v.eng_req && v.oilt_did;
    if (!strcmp(n, "oilLevel")) return v.eng_req && v.oill_did;
    return false;
}
// Valorile lente se citesc rar (km la 60 s, litri la 30 s, DID-urile din profil la perioada lor). Un client care se
// aboneaza primeste pe loc ce are placa deja in memorie, in loc sa astepte urmatoarea citire. Fara trafic pe magistrala.
static void replay_pd(uint8_t owner, const PDid &p, uint32_t now) {
    if (!p.have || (int32_t)(now - p.ok_ms) > (int32_t)(p.per_ms * 4UL)) return;
    replyf(owner, "{\"t\":\"d\",\"k\":\"%s\",\"v\":%.*f,\"raw\":%ld,\"src\":\"uds\",\"age\":%lu}\n",
           p.n, (int)p.dec, p.val, p.raw, (unsigned long)((now - p.ok_ms) / 1000UL));
}
static void replay_veh(uint8_t owner, const char *n, uint32_t now) {
    if (!strcmp(n, "vin"))   { if (g_vin_cache.length()) replyf(owner, "{\"t\":\"vin\",\"v\":\"%s\"}\n", g_vin_cache.c_str()); return; }
    if (!strcmp(n, "odoKm")) { if (g_km_cache >= 0) replyf(owner, "{\"t\":\"d\",\"k\":\"odoKm\",\"v\":%ld,\"src\":\"veh\"}\n", g_km_cache); return; }
    if (!strcmp(n, "fuelL")) { if (g_fuel_cache >= 0 && veh().fuel_fmt == VEH_FUEL_U8L)
                                   replyf(owner, "{\"t\":\"d\",\"k\":\"fuelL\",\"v\":%ld,\"src\":\"veh\"}\n", g_fuel_cache); return; }
    for (int i = 0; i < ENG_N; i++) {
        const EngSlot &s = g_eng[i];
        if (strcmp(s.k, n) != 0) continue;
        if (s.have && (int32_t)(now - s.ok_ms) <= (int32_t)(s.per_ms * 4UL))
            replyf(owner, "{\"t\":\"d\",\"k\":\"%s\",\"v\":%.1f,\"src\":\"veh\"}\n", s.k, s.val);
        return;
    }
}
// Intoarce sursa aleasa: "uds" (profil), "veh" (citita de firmware), "obd" (PID standard), "unsup" (PID standard pe care
// ECU-ul nu-l declara), "full" (tabela planificatorului plina), "none" (nume necunoscut pe masina asta).
static const char *sub_by_name(uint8_t owner, const char *n, double hz, uint32_t now) {
    if (!strcmp(n, "*")) {
        int c = 0;
        for (int i = 0; i < g_npd; i++) if (g_pd[i].f16) { pd_sub(g_pd[i], owner, 0); replay_pd(owner, g_pd[i], now); c++; }
        return c ? "uds" : "none";
    }
    int i = pd_find(n);
    if (i >= 0) { pd_sub(g_pd[i], owner, hz); replay_pd(owner, g_pd[i], now); return "uds"; }   // 1. profilul vehiculului bate tot
    if (veh_param_has(n)) { replay_veh(owner, n, now); return "veh"; }                          // 2. valorile de vehicul (se citesc oricum)
    const StdParam *sp = std_param_find(n);                            // 3. catalogul standard
    if (!sp) return "none";
    uint32_t per = hz > 0 ? (uint32_t)(1000.0 / hz) : sp->per_ms;
    if (per < 50) per = 50;
    int idx = osch_add(sp->n, sp->pid, per, owner, false, now);
    if (idx < 0) return "full";
    if (param_sup_known() && !param_sup_has(sp->pid)) { osch_set_unsupported((size_t)idx, true); return "unsup"; }
    return "obd";
}

// ==== v3.4: PID-urile suportate de ECU (01 00 / 20 / 40 ...), citite o data dupa ce motorul raspunde ====
static uint8_t  g_sup_base = 0, g_sup_fails = 0;
static bool     g_sup_done = false;
static uint32_t g_sup_try_ms = 0;
static void sup_apply() {
    for (size_t i = 0; i < osch_capacity(); i++) {
        const OschItem *it = osch_item(i);
        if (!it->active || it->oneshot || it->pid == 0x00 || it->unsupported || param_sup_has(it->pid)) continue;
        osch_set_unsupported(i, true);
        // clientii abonati inainte ca bitmap-ul sa fie citit afla acum ca valoarea nu exista pe masina asta
        char b[80]; int n = snprintf(b, sizeof(b), "{\"t\":\"unsup\",\"k\":\"%s\",\"pid\":%u}\n", it->key, it->pid);
        if (n > 0 && (size_t)n < sizeof(b) && it->key[0]) emitJson(b, (size_t)n);
    }
}
static void sup_finish() {
    param_sup_done(); g_sup_done = true;
    if (!param_sup_known()) { proto_event("PID-uri suportate: ECU nu a raspuns la 01 00 (raman toate permise)"); return; }
    sup_apply();
    proto_event("PID-uri suportate: %u declarate de ECU 0x%lX", (unsigned)param_sup_count(), (unsigned long)osch_resp_id());
}
static bool sup_tick(uint32_t now) {                 // true = a folosit apelul curent
    if (g_sup_done || (g_sup_try_ms && now - g_sup_try_ms < 2000)) return false;
    g_sup_try_ms = now ? now : 1;
    uint8_t req[2] = {0x01, g_sup_base};
    bool ok = await_resp(osch_req_id(), req, 2, 0x41, ENG_TIMEOUT_MS) && g_alen >= 6 && g_abuf[0] == 0x41 && g_abuf[1] == g_sup_base;
    if (!ok) { if (++g_sup_fails >= 5) sup_finish(); return true; }
    g_sup_fails = 0;
    uint8_t next = param_sup_feed(g_sup_base, g_abuf + 2, g_alen - 2);
    if (next) { g_sup_base = next; g_sup_try_ms = 0; } else sup_finish();
    return true;
}
void proto_sup_restart() { param_sup_reset(); g_sup_base = 0; g_sup_fails = 0; g_sup_done = false; g_sup_try_ms = 0; }
// fragment cloud: cheile marcate nesuportate (o data fiecare) + cate PID-uri declara ECU-ul
String proto_unsup_json() {
    String l;
    for (size_t i = 0; i < osch_capacity(); i++) {
        const OschItem *it = osch_item(i);
        if (!it->active || !it->unsupported || !it->key[0]) continue;
        String k = it->key;
        if ((String(",") + l + ",").indexOf(String(",") + k + ",") >= 0) continue;
        if (l.length() > 160) break;
        if (l.length()) l += ",";
        l += k;
    }
    String o;
    if (l.length()) { o += ",\"unsl\":\""; o += l; o += "\""; }
    if (param_sup_known()) { o += ",\"nsup\":"; o += String((unsigned)param_sup_count()); }
    return o;
}

// PARAMS (consola) / {"cmd":"params"} (JSON, o linie pe parametru + o linie de incheiere)
static void params_list(uint8_t owner, bool json) {
    int n = 0;
    if (!json) replyf(owner, "PARAMS: ce se poate cere PE NUME (profil %s v%u)\r\n", veh().model, (unsigned)veh().ver);
    for (int i = 0; i < g_npd; i++) {
        const PDid &p = g_pd[i];
        if (json) replyf(owner, "{\"t\":\"param\",\"n\":\"%s\",\"src\":\"uds\",\"req\":\"0x%X\",\"did\":\"0x%04X\",\"bg\":%d,\"sup\":%d}\n",
                         p.n, p.req, p.did, (int)p.autord, p.fails >= 3 ? 0 : 1);
        else replyf(owner, "  %-14s profil   0x%03X DID %04X%s%s\r\n", p.n, p.req, p.did, p.autord ? "" : "  la cerere", p.fails >= 3 ? "  NU RASPUNDE" : "");
        n++; proto_pump(1);
    }
    for (size_t i = 0; i < VEH_NAMES_N; i++) {
        if (pd_find(VEH_NAMES[i]) >= 0 || !veh_param_has(VEH_NAMES[i])) continue;
        if (json) replyf(owner, "{\"t\":\"param\",\"n\":\"%s\",\"src\":\"veh\",\"sup\":1}\n", VEH_NAMES[i]);
        else replyf(owner, "  %-14s vehicul\r\n", VEH_NAMES[i]);
        n++;
    }
    for (size_t i = 0; i < std_param_count(); i++) {
        const StdParam *sp = std_param_at(i);
        if (pd_find(sp->n) >= 0 || veh_param_has(sp->n)) continue;      // numele e acoperit de profil
        bool sup = param_sup_has(sp->pid);
        if (json) replyf(owner, "{\"t\":\"param\",\"n\":\"%s\",\"src\":\"obd\",\"pid\":%u,\"u\":\"%s\",\"sup\":%d}\n", sp->n, sp->pid, sp->u, sup ? 1 : 0);
        else replyf(owner, "  %-14s OBD-II   PID %02X  %-6s %s\r\n", sp->n, sp->pid, sp->u,
                    !param_sup_known() ? "(inca nu stiu ce suporta ECU-ul)" : (sup ? "" : "NESUPORTAT de ECU"));
        n++; if ((i & 7) == 7) proto_pump(1);
    }
    if (json) replyf(owner, "{\"type\":\"params\",\"proto\":%d,\"veh\":\"%s\",\"n\":%d,\"supknown\":%d}\n", PARAMS_PROTO, veh().model, n, param_sup_known() ? 1 : 0);
    else replyf(owner, "%d nume. PID-uri declarate de ECU: %s\r\n", n,
                param_sup_known() ? String((unsigned)param_sup_count()).c_str() : "necitite inca (motorul trebuie sa raspunda)");
}

void proto_slow_read_tick(uint32_t now) {
    if (osch_busy()) return;                         // nu ne suprapunem peste o cerere OBD in lucru
    if (sup_tick(now)) return;                       // v3.4: intai aflam ce PID-uri are ECU-ul (5-7 cereri, o data)
    if (!g_vin_cache.length() && now - g_vin_try_ms > 5000) {
        g_vin_try_ms = now; String v = read_vin();
        if (v.length()) { g_vin_cache = v; char b[64]; int n = snprintf(b, sizeof(b), "{\"t\":\"vin\",\"v\":\"%s\"}\n", v.c_str()); if (n > 0) emitJson(b, n); }
    } else if (now - g_km_try_ms > 60000) {          // KM se schimba lent: o data pe minut
        g_km_try_ms = now; long k = read_km();
        if (k >= 0) { g_km_cache = k; char b[48]; int n = snprintf(b, sizeof(b), "{\"t\":\"km\",\"v\":%ld}\n", k); if (n > 0) emitJson(b, n);
                      emit_d("odoKm", (double)k, 0, "veh"); }
    } else if (now - g_fuel_try_ms > 30000) {        // v2.2.3: litri — rezolutie 1 L, ajunge o data la 30 s
        g_fuel_try_ms = now; long f = read_fuel();
        if (f >= 0) { g_fuel_cache = f; char b[48]; int n = snprintf(b, sizeof(b), "{\"t\":\"fuel\",\"v\":%ld}\n", f); if (n > 0) emitJson(b, n);
                      if (veh().fuel_fmt == VEH_FUEL_U8L) emit_d("fuelL", (double)f, 0, "veh"); else emit_d("fuelLevel", (double)f, 0, "veh"); }
    } else {
        eng_tick(now);                               // v2.3.3: generator + ulei de pe ECU motor (o citire pe apel)
        pd_tick(now);                                // v2.5.11: DID-urile din profil (o citire pe apel)
        dtcall_auto_tick(now);                       // v2.4.0: lista de instalare + erori pe module (rar, in fundal)
    }
}
String proto_vin_cached() { return g_vin_cache; }
long   proto_km_cached()  { return g_km_cache; }
long   proto_fuel_cached(){ return g_fuel_cache; }
int    proto_fuel_did()   { return (int)g_fuel_did_ok; }   // v2.2.5: de pe ce DID vin litrii (2206/2212)

// raspuns UDS = req+8 pt 0x7E0-0x7E7 (motor/cutie), altfel req+0x6A (VW: 0x714->0x77E)
static uint32_t uds_resp_id(uint32_t req) { return veh_resp_id(req); }   // v2.3.0: regula din profilul vehiculului

// citeste un DID UDS de pe reqid. Umple out (datele dupa 62 DIDhi DIDlo). false + err la esec.
static bool uds_did(uint32_t reqid, uint16_t did, uint8_t *out, size_t osz, size_t *olen, String &err) {
    iso_tp_allow(uds_resp_id(reqid), reqid);
    uint8_t req[3] = {0x22, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF)};
    if (!await_resp(reqid, req, 3, 0x62, 1000)) { err = "fara raspuns"; return false; }
    if (g_alen >= 1 && g_abuf[0] == 0x7F) { err = "NRC 0x" + String(g_alen >= 3 ? g_abuf[2] : 0, HEX); return false; }
    if (g_alen < 3 || g_abuf[0] != 0x62) { err = "raspuns neasteptat"; return false; }
    size_t n = g_alen - 3; if (n > osz) n = osz;
    memcpy(out, g_abuf + 3, n); *olen = n; return true;
}
static bool uds_session_ok(uint32_t reqid) {
    iso_tp_allow(uds_resp_id(reqid), reqid);
    uint8_t req[2] = {0x10, 0x03};
    if (!await_resp(reqid, req, 2, 0x50, 500)) return false;
    return g_alen >= 1 && (g_abuf[0] == 0x50 || (g_abuf[0] == 0x7F && g_alen >= 3 && g_abuf[2] != 0x11));
}

// ==================== v2.4.0: DTCALL - lista de instalare a gateway-ului + erorile de pe module ====================
// Citeste `22 04A1` de pe gateway (TP2.0 0x1F, servicii UDS): 64 x [adr][pos][flags][bus], unde `pos` = adresa logica TP2.0 a
// modulului, flags bit0 = codat, bit2 = ARE ERORI. Apoi citeste DTC-urile modulelor cu erori (sau TOATE cu `ALL`):
// UDS `19 02 FF` (module UDS + gateway) sau KWP `18 02 FF 00` (module TP2.0). Rezultatul se difuzeaza ca linii JSON
// {"t":"gwil"} / {"t":"dtc"} / {"t":"dtcdone"} si se pastreaza, ca sa fie retrimis automat oricarui client care da `sub`.
// ATENTIE: transporturile de la module in afara de 01/17/15/44/19/03 sunt PRESUPUSE (vezi dtcall.cpp) si NEVALIDATE pe masina.
#define DTC_MAX_ITEMS     24
#define DTC_RESCAN_MS     600000UL      // scan complet automat: la 10 min
#define DTC_PEEK_MS       120000UL      // lista de instalare (ieftina) se recitește la 2 min: apare o eroare noua => scan
#define DTC_FIRST_MS      25000UL       // primul scan automat: dupa ce VIN/KM/litri/motor au apucat sa se citeasca
struct DtcItem { char k[12]; char p[8]; uint8_t st; };
struct DtcRes { char tr; char err[28]; uint8_t n; DtcItem it[DTC_MAX_ITEMS]; };
static DtcRes   g_dr;
static String   g_gwil_json, g_dtcdone_json;
static String   g_dtc_json[24];
static uint8_t  g_dtc_json_n = 0;
static char     g_dtc_faults[48] = "";
static uint16_t g_dtc_total = 0;
static bool     g_dtc_running = false;
static bool     g_dtc_pub     = false;   // v2.5.4: exista un rezultat nou de urcat in cloud (lista de coduri)
static uint32_t g_dtc_scan_ms = 0, g_dtc_peek_ms = 0, g_dtc_next_ms = 0;
static uint8_t  g_dtc_fails = 0;
static uint32_t g_gw_sig = 0;
static uint8_t  g_mod_fail[256];         // esecuri la rand pe modul, doar in modul automat (dupa 3 nu mai insistam)

static void dr_reset(DtcRes &r, char tr) { memset(&r, 0, sizeof(r)); r.tr = tr; }
static void dr_err(DtcRes &r, const char *s) { snprintf(r.err, sizeof(r.err), "%s", s); }

static bool dr_parse59(const uint8_t *p, size_t len, DtcRes &r) {          // 59 02 mask [dtc(3) status]*
    if (len < 3 || p[0] != 0x59) { dr_err(r, "raspuns 19 neasteptat"); return false; }
    size_t nd = (len - 3) / 4;
    for (size_t k = 0; k < nd && r.n < DTC_MAX_ITEMS; k++) {
        DtcItem &d = r.it[r.n++];
        dtcall_fmt_uds(p + 3 + 4 * k, d.k); d.p[0] = 0; d.st = p[6 + 4 * k];
    }
    return true;
}
static bool dr_parse58(const uint8_t *p, size_t len, DtcRes &r) {          // 58 n [hi lo status]*n  (KWP2000 VAG)
    if (len < 2 || p[0] != 0x58) { dr_err(r, "raspuns 18 neasteptat"); return false; }
    int n = p[1];
    for (int k = 0; k < n && (size_t)(4 + 3 * k) < len && r.n < DTC_MAX_ITEMS; k++) {
        DtcItem &d = r.it[r.n++];
        uint16_t code = ((uint16_t)p[2 + 3 * k] << 8) | p[3 + 3 * k];
        dtcall_fmt_vag(code, d.k);
        if (!dtcall_vag_to_pcode(code, d.p)) d.p[0] = 0;
        d.st = p[4 + 3 * k];
    }
    return true;
}

static bool dtc_via_uds(uint32_t req, DtcRes &r) {
    iso_tp_allow(uds_resp_id(req), req);
    uint8_t q[3] = {0x19, 0x02, 0xFF};
    for (int att = 0; att < 2; att++) {
        if (!await_resp(req, q, 3, 0x59, 1000)) { dr_err(r, "fara raspuns"); return false; }
        if (g_alen >= 3 && g_abuf[0] == 0x7F) {
            char e[28]; snprintf(e, sizeof(e), "NRC %02X", g_abuf[2]); dr_err(r, e);
            if (att == 0 && g_abuf[2] != 0x11 && g_abuf[2] != 0x12) { uds_session_ok(req); continue; }   // poate cere sesiune extinsa
            return false;
        }
        return dr_parse59(g_abuf, g_alen, r);
    }
    return false;
}
static bool dtc_via_tp(uint8_t pos, bool uds_svc, DtcRes &r) {
    String err;
    if (!tp20_open(pos, err)) { dr_err(r, err.c_str()); return false; }
    uint8_t q[4]; size_t ql;
    if (uds_svc) { q[0] = 0x19; q[1] = 0x02; q[2] = 0xFF; ql = 3; } else { q[0] = 0x18; q[1] = 0x02; q[2] = 0xFF; q[3] = 0x00; ql = 4; }
    uint8_t resp[160]; size_t rl = 0;
    if (!tp20_kwp(q, ql, resp, sizeof(resp), &rl, err)) { dr_err(r, err.c_str()); return false; }
    return uds_svc ? dr_parse59(resp, rl, r) : dr_parse58(resp, rl, r);
}
// Incearca transportul preferat (tabela dtcall.cpp), apoi alternativa. r.tr = transportul care a mers ('u' UDS/CAN, 't' TP2.0).
static bool dtc_module(const GwMod &m, DtcRes &r) {
    const ModRoute *rt = dtcall_route(m.adr);
    uint32_t ureq = rt ? rt->uds_req : 0;
    bool pref_uds = rt && rt->pref_uds && ureq, uds_svc = rt && rt->tp_uds;
    for (int pass = 0; pass < 2; pass++) {
        bool do_uds = (pass == 0) ? pref_uds : !pref_uds;
        if (do_uds) { if (!ureq) continue; dr_reset(r, 'u'); if (dtc_via_uds(ureq, r)) return true; }
        else        { dr_reset(r, 't'); if (dtc_via_tp(m.pos, uds_svc, r)) return true; }
    }
    return false;    // r.err = ultima eroare
}

// citeste lista de instalare. Canalul TP2.0 spre gateway RAMANE deschis (gateway-ul are si el DTC-uri).
static bool gw_read_install(GwMod *mods, int *n, String &err) {
    if (!tp20_open(0x1F, err)) return false;
    uint8_t q[3] = {0x22, 0x04, 0xA1}; uint8_t resp[300]; size_t rl = 0;
    if (!tp20_kwp(q, 3, resp, sizeof(resp), &rl, err)) { tp20_close(); return false; }
    if (rl < 7 || resp[0] != 0x62 || resp[1] != 0x04 || resp[2] != 0xA1) { err = "raspuns 22 04A1 neasteptat"; tp20_close(); return false; }
    *n = dtcall_parse_install(resp + 3, rl - 3, mods, GW_MAX_MODS);
    return true;
}
static uint32_t gw_signature(const GwMod *m, int n) {   // FNV-1a peste (adr, flags): se schimba cand apare/dispare o eroare
    uint32_t h = 2166136261u;
    for (int i = 0; i < n; i++) { h = (h ^ m[i].adr) * 16777619u; h = (h ^ m[i].flags) * 16777619u; }
    return h;
}
static String gwil_json(const GwMod *m, int n) {
    String s = "{\"t\":\"gwil\",\"ok\":1,\"n\":"; s += n; s += ",\"m\":[";
    for (int i = 0; i < n; i++) {
        char b[72]; snprintf(b, sizeof(b), "%s{\"a\":\"%02X\",\"p\":\"%02X\",\"f\":%u,\"b\":%u}", i ? "," : "", m[i].adr, m[i].pos, m[i].flags, m[i].bus);
        s += b;
    }
    s += "]}\n"; return s;
}
static String dtc_json(const GwMod &m, const DtcRes &r, bool ok) {
    char h[96]; snprintf(h, sizeof(h), "{\"t\":\"dtc\",\"a\":\"%02X\",\"nm\":\"%s\",", m.adr, dtcall_mod_name(m.adr));
    String s = h;
    if (!ok) { char e[64]; jl_escape(e, sizeof(e), r.err); s += "\"err\":\""; s += e; s += "\"}\n"; return s; }
    snprintf(h, sizeof(h), "\"tr\":\"%s\",\"n\":%u,\"c\":[", r.tr == 'u' ? "uds" : "tp", r.n); s += h;
    for (int k = 0; k < r.n; k++) {
        char b[80];
        if (r.it[k].p[0]) snprintf(b, sizeof(b), "%s{\"k\":\"%s\",\"p\":\"%s\",\"s\":%u}", k ? "," : "", r.it[k].k, r.it[k].p, r.it[k].st);
        else              snprintf(b, sizeof(b), "%s{\"k\":\"%s\",\"s\":%u}", k ? "," : "", r.it[k].k, r.it[k].st);
        s += b;
    }
    s += "]}\n"; return s;
}
static bool dtc_is_target(const GwMod &m, bool all) { return (m.flags & GW_FLAG_CODED) && (all || (m.flags & GW_FLAG_FAULT)); }

static void dtcall_run(uint8_t owner, bool all, bool text) {
    if (g_dtc_running) { if (text) reply(owner, "DTCALL: deja ruleaza\r\n"); else reply(owner, "{\"t\":\"dtcdone\",\"ok\":0,\"err\":\"busy\"}\n"); return; }
    g_dtc_running = true;
    bool manual = owner != 0xFF;
    if (manual) memset(g_mod_fail, 0, sizeof(g_mod_fail));
    uint32_t t0 = millis();
    GwMod mods[GW_MAX_MODS]; int n = 0; String err;
    bool ok = gw_read_install(mods, &n, err);
    if (!ok) { proto_pump(200); err = ""; ok = gw_read_install(mods, &n, err); }   // o a doua incercare (canal ocupat / ACK pierdut)
    if (!ok) {
        char e[80]; jl_escape(e, sizeof(e), err.c_str());
        char b[160]; int bn = snprintf(b, sizeof(b), "{\"t\":\"gwil\",\"ok\":0,\"err\":\"%s\"}\n", e);
        emitJson(b, bn);
        if (text) replyf(owner, "DTCALL: lista de instalare esuata: %s\r\n", err.c_str());
        g_dtc_fails = g_dtc_fails < 250 ? g_dtc_fails + 1 : g_dtc_fails;
        g_dtc_running = false; return;
    }
    proto_keepalive();
    g_gw_sig = gw_signature(mods, n); g_dtc_peek_ms = millis();
    g_gwil_json = gwil_json(mods, n); emitJson(g_gwil_json.c_str(), g_gwil_json.length());
    int coded = 0, faulty = 0;
    for (int i = 0; i < n; i++) { if (mods[i].flags & GW_FLAG_CODED) coded++; if ((mods[i].flags & GW_FLAG_CODED) && (mods[i].flags & GW_FLAG_FAULT)) faulty++; }
    if (text) {
        replyf(owner, "=== DTCALL === lista de instalare: %d module, %d codate, %d cu erori\r\n", n, coded, faulty);
        for (int i = 0; i < n; i++)
            replyf(owner, "  %02X %-14s pos %02X bus %u flags %02X%s%s\r\n", mods[i].adr, dtcall_mod_name(mods[i].adr), mods[i].pos, mods[i].bus, mods[i].flags,
                   (mods[i].flags & GW_FLAG_CODED) ? "" : "  (necodat)", (mods[i].flags & GW_FLAG_FAULT) ? "  <-- ERORI" : "");
    }
    g_dtc_json_n = 0; g_dtc_total = 0; g_dtc_faults[0] = 0;
    int rd = 0, bad = 0, skipped = 0;
    for (int pass = 0; pass < 2; pass++)            // gateway-ul intai: canalul TP2.0 spre el e deja deschis
        for (int i = 0; i < n; i++) {
            const GwMod &m = mods[i];
            if ((pass == 0) != (m.adr == 0x19) || !dtc_is_target(m, all)) continue;
            if (!manual && g_mod_fail[m.adr] >= 3) { skipped++; continue; }
            bool okm = dtc_module(m, g_dr);
            String js = dtc_json(m, g_dr, okm);
            if (g_dtc_json_n < 24) g_dtc_json[g_dtc_json_n++] = js;
            emitJson(js.c_str(), js.length());
            if (okm) {
                rd++; g_mod_fail[m.adr] = 0; g_dtc_total += g_dr.n;
                if (g_dr.n) { size_t l = strlen(g_dtc_faults); snprintf(g_dtc_faults + l, sizeof(g_dtc_faults) - l, "%s%02X", l ? "," : "", m.adr); }
                if (text) {
                    replyf(owner, "  %02X %-14s [%s] %u DTC%s", m.adr, dtcall_mod_name(m.adr), g_dr.tr == 'u' ? "UDS" : "TP2.0", g_dr.n, g_dr.n ? ":" : "\r\n");
                    for (int k = 0; k < g_dr.n; k++) replyf(owner, " %s%s%s/st%02X", g_dr.it[k].k, g_dr.it[k].p[0] ? "=" : "", g_dr.it[k].p, g_dr.it[k].st);
                    if (g_dr.n) reply(owner, "\r\n");
                }
            } else {
                bad++; if (g_mod_fail[m.adr] < 250) g_mod_fail[m.adr]++;
                if (text) replyf(owner, "  %02X %-14s NU RASPUNDE: %s\r\n", m.adr, dtcall_mod_name(m.adr), g_dr.err);
            }
            proto_pump(3); proto_keepalive();
        }
    tp20_close();
    char d[160]; int dn = snprintf(d, sizeof(d), "{\"t\":\"dtcdone\",\"ok\":%d,\"err\":%d,\"skip\":%d,\"tot\":%u,\"faults\":\"%s\",\"all\":%d,\"ms\":%lu}\n",
                                    rd, bad, skipped, g_dtc_total, g_dtc_faults, all ? 1 : 0, (unsigned long)(millis() - t0));
    g_dtcdone_json = String(d); emitJson(d, dn);
    if (text) replyf(owner, "=== gata: %d module citite, %d esuate, %u coduri, %lu ms ===\r\n", rd, bad, g_dtc_total, (unsigned long)(millis() - t0));
    g_dtc_scan_ms = millis() ? millis() : 1; g_dtc_fails = 0;
    g_dtc_pub = true;                 // v2.5.4: urmatorul snapshot cloud duce si codurile, o singura data
    g_dtc_running = false;
}

// retrimite clientului nou (la `sub`) ce s-a citit deja - "atasat automat la abonament"
static void dtcall_replay(uint8_t owner) {
    if (!g_gwil_json.length() || !g_reply) return;
    g_reply(owner, g_gwil_json.c_str(), g_gwil_json.length());
    for (int i = 0; i < g_dtc_json_n; i++) g_reply(owner, g_dtc_json[i].c_str(), g_dtc_json[i].length());
    if (g_dtcdone_json.length()) g_reply(owner, g_dtcdone_json.c_str(), g_dtcdone_json.length());
}

// fundal: primul scan la ~25 s dupa boot; apoi lista de instalare la 2 min (ieftin) si scan complet doar daca apare o eroare
// noua sau la 10 min. Esec => pauza crescatoare (1, 2, 4 ... max 30 min), ca sa nu blocheze magistrala cu incercari.
static uint8_t g_dtc_auto_tries = 0;   // v2.5.11
static void dtcall_auto_tick(uint32_t now) {
    if (g_dtc_running || now < DTC_FIRST_MS || now < g_dtc_next_ms) return;
    // v2.5.11 (cerinta user): automat DOAR la pornirea placii — o scanare reusita (sau una manuala) si gata; max 5 incercari.
    // Verificarea gateway la 2 min + rescan la 10 min deschideau canale TP2.0 in mers (suspect la pierderea datelor F16 din 24.09).
    if (g_dtc_scan_ms || g_dtc_auto_tries >= 5) return;
    g_dtc_auto_tries++;
    if (strcmp(can_mode_name(), "normal") != 0) return;
    bool first = g_dtc_scan_ms == 0;
    bool full  = first || now - g_dtc_scan_ms >= DTC_RESCAN_MS;
    if (!full && now - g_dtc_peek_ms >= DTC_PEEK_MS) {
        GwMod m[GW_MAX_MODS]; int n = 0; String e; g_dtc_peek_ms = now;
        if (gw_read_install(m, &n, e)) { tp20_close(); full = gw_signature(m, n) != g_gw_sig; }
        else { g_dtc_fails = g_dtc_fails < 250 ? g_dtc_fails + 1 : g_dtc_fails; }
    }
    if (!full) return;
    proto_event("DTC automat pornit (%s)", first ? "primul" : "rescan");
    uint32_t td = millis();
    dtcall_run(0xFF, false, false);
    proto_event("DTC automat gata in %lu ms (esecuri la rand %u)", (unsigned long)(millis() - td), (unsigned)g_dtc_fails);
    if (g_dtc_fails) { uint32_t back = 60000UL << (g_dtc_fails > 5 ? 5 : g_dtc_fails - 1); if (back > 1800000UL) back = 1800000UL; g_dtc_next_ms = millis() + back; }
}
String proto_dtc_json() {           // fragment pt snapshot-ul cloud (dupa ultimul scan): coduri + module cu erori + varsta
    if (!g_dtc_scan_ms) return "";
    char b[110]; snprintf(b, sizeof(b), ",\"dtcn\":%u,\"dtcm\":\"%s\",\"dtca\":%lu", g_dtc_total, g_dtc_faults, (unsigned long)((millis() - g_dtc_scan_ms) / 1000));
    return String(b);
}

// v2.5.4: LISTA de coduri pe module, pentru BYOS. Pana acum in cloud pleca doar rezumatul (cate coduri si pe ce
// module), deci codurile in clar se vedeau pe server doar daca rula si aplicatia F16. Trimitem acum si liniile
// retinute de la ultimul DTCALL ({"a":"09","nm":..,"tr":..,"n":..,"c":[{"k","p","s"}]} sau {"a":..,"err":..}).
// DOAR in primul snapshot de dupa fiecare scanare (flag g_dtc_pub): altfel, la 10 s pe snapshot, am repeta
// cateva KB degeaba. Serverul (byos.php) salveaza tot obiectul in jsonb, deci nu e nevoie de nimic acolo.
String proto_dtc_list_json() {
    if (!g_dtc_pub || !g_dtc_json_n) { g_dtc_pub = false; return ""; }
    g_dtc_pub = false;
    String s = ",\"dtcl\":[";
    for (int i = 0; i < g_dtc_json_n; i++) {
        String one = g_dtc_json[i];
        one.trim();                                  // liniile difuzate se termina cu \n
        if (!one.length()) continue;
        if (s.length() > 9) s += ",";
        s += one;
    }
    s += "]";
    return s;
}

static void cmd_modules(uint8_t owner, char cmd[], char *tok[], int nt);
static void selftest_modules(uint8_t owner);   // v2.2.3: sondarea modulelor, folosita si de SELFTEST

static void cmd_selftest(uint8_t owner) {
    reply(owner, "=== ergCAN SELFTEST ===\r\n");
    // 1) placa
    replyf(owner, "Placa   : fw %s  heap %lu KB  %s\r\n", FW_VERSION, (unsigned long)(ESP.getFreeHeap() / 1024), ota_status_json().c_str());
    // 2) WiFi/internet
    replyf(owner, "WiFi    : %s  ip %s\r\n", net_online() ? "conectat" : "OFFLINE", net_ip().c_str());
    if (net_online()) { OtaInfo inf; String e; bool ok = ota_check(inf, e);
        replyf(owner, "Internet: %s\r\n", ok ? (String("OTA server OK, ultima fw ") + inf.latest).c_str() : (String("verificare OTA esuata: ") + e).c_str()); }
    // 3) cablaj CAN
    { char v[16], h[160]; can_selftest_wiring(v, sizeof(v), h, sizeof(h)); replyf(owner, "Cablaj  : %s — %s\r\n", v, h); }
    // 4) bus + motor
    bool was_listen = strcmp(can_mode_name(), "normal") != 0;
    if (was_listen) can_set_mode("normal");
    uint8_t p0[2] = {0x01, 0x00};
    bool eng = await_resp(obd_func_id(), p0, 2, 0x41, 800);
    if (eng) {
        replyf(owner, "Motor   : raspunde pe 0x%03lX, lat %lums\r\n", (unsigned long)g_await_id, (unsigned long)0);
        // multi-PID test
        uint8_t pm[6] = {0x01, 0x0C, 0x0D, 0x0B, 0x11, 0x04};
        bool multi = await_resp(0x7E0, pm, 6, 0x41, 400);
        replyf(owner, "Multi   : %s\r\n", multi ? "motorul accepta 6 PID intr-o cerere" : "cate un PID pe cerere");
    } else {
        reply(owner, "Motor   : NU raspunde (contact pus? pin 6/14/5? bus 500k?)\r\n");
    }
    // 5) VIN / KM
    String vin = read_vin();
    replyf(owner, "VIN     : %s\r\n", vin.length() ? vin.c_str() : "(indisponibil)");
    long km = read_km();
    if (km >= 0) replyf(owner, "KM      : %ld km (%s)\r\n", km, g_km_src == 'E' ? "ECU motor UDS 22 121C" : "bord UDS 22 2203, x10");
    else reply(owner, "KM      : (indisponibil pe UDS; posibil doar prin TP2.0 bord — disponibil in 2.1)\r\n");
    long fuel = read_fuel();   // v2.2.3: bordul da litrii pe UDS 22 2206 (sau 2212), u8
    if (fuel >= 0) replyf(owner, "Litri   : %ld L (bord UDS 22 2206/2212, rezolutie 1 L)\r\n", fuel);
    else reply(owner, "Litri   : (nu raspunde nici 2206, nici 2212 — contact pus?)\r\n");
    selftest_modules(owner);   // v2.2.3: chiar sondeaza modulele (ca MODPROBE), nu mai e placeholder
    // 6) senzori din stat
    reply(owner, "=== gata ===\r\n");
}

static void upper(char *s) { for (; *s; s++) if (*s >= 'a' && *s <= 'z') *s -= 32; }

static bool    g_rep_active = false;   // v2.5: o comanda repetitiva (UDS ... nr ms) ruleaza
static uint8_t g_rep_owner  = 0;
static bool    g_rep_stop   = false;
static void console_line_impl(uint8_t owner, const char *raw) {
    // linie care incepe cu { → comanda JSON pe consola
    while (*raw == ' ') raw++;
    if (g_rep_active && owner == g_rep_owner) {   // v2.5: in timpul repetarii, orice linie (q) opreste; nu dispecerizam reentrant
        g_rep_stop = true; return;
    }
    if (*raw == '{') { json_dispatch(owner, raw); return; }
    char line[128]; strncpy(line, raw, sizeof(line) - 1); line[sizeof(line) - 1] = 0;
    // tokenizare simpla
    char *tok[6] = {0}; int nt = 0;
    for (char *p = strtok(line, " \t"); p && nt < 6; p = strtok(nullptr, " \t")) tok[nt++] = p;
    if (nt == 0) return;
    char cmd[16]; strncpy(cmd, tok[0], sizeof(cmd) - 1); cmd[sizeof(cmd) - 1] = 0; upper(cmd);

    if (!strcmp(cmd, "HELP")) {
        reply(owner, "Comenzi: HELP STAT VER VIN KM FUEL ENG DIDS PARAMS SELFTEST NETDIAG PID <hh..> SUB MON <id> LISTEN NORMAL\r\n"
                     "  PARAMS = ce se poate cere pe nume pe masina asta   AUTODET = cauta viteza + adresarea (profil generic obd2)\r\n"
                     "  VEH | VEH LIST | VEH GET <model> | VEH CLEAR   (profil vehicul; scurt: SET mode <model>)\r\n"
                     "  CFG  SET <cheie> <val>  SAVE  OTA CHECK|UPDATE  CLOUD [NOW|ON|OFF]  ACFG [NOW|ON|OFF]  SNIFF <label> [url] | SNIFF STOP\r\n"
                     "  DTC <ecu> | DTC CLR <ecu>  DTCALL [ALL]  MOD ID GRP DID (module)  EXPLORE <pin> | EXPLORE OFF  REBOOT\r\n"
                     "  READEPROM <adr_modul_hex> <adresa_hex> <lungime_dec> <nume>   (ex: READEPROM 20 000C01 47 bcm_cfg; pe loc, manual)\r\n"
                     "  CODING <adr_tp20|canid_uds> [nume_byos] [did=0600]   CODINGALL [nume_byos] [did]   (long coding; cu nume => upload BYOS)\r\n"
                     "  TPQ <adr_tp20> <cerere_hex> [cerere2..4]   (canal fara 10 89 + trace; doar citire. ex: TPQ 03 1089 2101)\r\n"
                     "  UDSSCAN [nume_byos] [de_la=700] [pana_la=7DF]   (cine raspunde UDS: 22 F187 pe ID, ~20 s; cu nume => upload)\r\n"
                     "  DIDSCAN <reqid> <did_de_la> <did_pana_la> [nume_byos]   (22 pe fiecare DID, doar citire; ex: DIDSCAN 7E0 1000 10FF ulei)\r\n"
                     "  BYOSDEBUG <nume> | BYOSDEBUG STOP | BYOSDEBUG   (jurnal consola: fiecare comanda + rezultat urca la BYOS, fara a bloca CAN)\r\n"
                     "  MAGCAL [START|STOP|RESET|HDG <grade_reale>|INV 0|1]   (calibrare busola; fara calibrare se face singura dupa un cerc)\r\n");
        return;
    }
    if (!strcmp(cmd, "VER")) { replyf(owner, "ergCAN %s\r\n", FW_VERSION); return; }
    if (!strcmp(cmd, "STAT")) { char b[640]; can_stat_json(b, sizeof(b)); reply(owner, b); return; }
    if (!strcmp(cmd, "REBOOT")) { reply(owner, "reboot...\r\n"); proto_pump(200); ESP.restart(); return; }
    if (!strcmp(cmd, "LISTEN")) { can_set_mode("listen"); osch_set_enabled(false); reply(owner, "mod LISTEN\r\n"); return; }
    if (!strcmp(cmd, "NORMAL")) { can_set_mode("normal"); osch_set_enabled(true); reply(owner, "mod NORMAL\r\n"); return; }
    if (!strcmp(cmd, "SELFTEST")) { cmd_selftest(owner); return; }
    if (!strcmp(cmd, "VIN")) { String v = read_vin(); replyf(owner, "VIN: %s\r\n", v.length() ? v.c_str() : "(indisponibil)"); return; }
    if (!strcmp(cmd, "KM")) { long km = read_km(); if (km >= 0) replyf(owner, "KM: %ld (%s)\r\n", km, g_km_src == 'E' ? "ECU" : "bord x10"); else reply(owner, "KM: indisponibil (UDS)\r\n"); return; }
    if (!strcmp(cmd, "DIDS")) { cmd_dids(owner); return; }   // v2.5.11: DID-urile din profil
    if (!strcmp(cmd, "PARAMS")) { params_list(owner, false); return; }   // v3.4: numele disponibile + sursa fiecaruia
    if (!strcmp(cmd, "AUTODET")) {                                        // v3.4: autodetectie (viteza + 11/29 de biti)
        reply(owner, "AUTODET: caut magistrala (500k/250k, 11/29 de biti), ~2 s...\r\n");
        String msg; bool ok = can_autodetect(msg);
        replyf(owner, "%s%s\r\n", ok ? "OK: " : "ESUAT: ", msg.c_str());
        return;
    }
    if (!strcmp(cmd, "MAGCAL")) {   // v3.3 (fw 2.5.13): calibrare magnetometru
        String r = sensors_magcal_cmd(nt >= 2 ? tok[1] : nullptr, nt >= 3 ? tok[2] : nullptr);
        replyf(owner, "%s\r\n", r.c_str()); return;
    }
    if (!strcmp(cmd, "ENG")) { cmd_eng(owner); return; }   // v2.3.3: generator + ulei (ECU motor)
    if (!strcmp(cmd, "FUEL")) { long f = read_fuel(); if (f >= 0) replyf(owner, "FUEL: %ld L\r\n", f); else reply(owner, "FUEL: indisponibil (UDS 2206/2212)\r\n"); return; }
    if (!strcmp(cmd, "PID")) {
        if (nt < 2) { reply(owner, "PID <hh> [hh..]\r\n"); return; }
        uint8_t pids[6]; int n = 0;
        for (int i = 1; i < nt && n < 6; i++) pids[n++] = (uint8_t)strtol(tok[i], nullptr, 16);
        uint8_t req[7]; req[0] = 0x01; memcpy(req + 1, pids, n);
        if (strcmp(can_mode_name(), "normal") != 0) can_set_mode("normal");
        if (!await_resp(osch_req_id(), req, 1 + n, 0x41, 500)) { reply(owner, "fara raspuns\r\n"); return; }
        size_t i = 1;
        while (i < g_alen) {
            uint8_t pid = g_abuf[i]; int L = obd_pid_len(pid);
            if (L < 0 || i + 1 + L > g_alen) break;
            double v; const char *u;
            if (obd_pid_decode(pid, g_abuf + i + 1, L, &v, &u)) replyf(owner, "  %02X = %.2f %s\r\n", pid, v, u);
            else { replyf(owner, "  %02X =", pid); for (int k = 0; k < L; k++) replyf(owner, " %02X", g_abuf[i + 1 + k]); reply(owner, "\r\n"); }
            i += 1 + L;
        }
        return;
    }
    if (!strcmp(cmd, "SUB")) {
        int shown = 0;
        for (size_t i = 0; i < osch_capacity(); i++) {
            const OschItem *it = osch_item(i);
            if (!it->active) continue;
            replyf(owner, "  %-14s pid %02X  %s  cerute %lu ok %lu miss %lu%s\r\n",
                   it->key[0] ? it->key : "(oneshot)", it->pid,
                   it->oneshot ? "1x" : (String(1000.0 / (it->period_ms ? it->period_ms : 1000)) + "Hz").c_str(),
                   (unsigned long)it->n_req, (unsigned long)it->n_ok, (unsigned long)it->n_miss,
                   it->unsupported ? " NESUPORTAT" : "");
            shown++;
        }
        const OschStats &s = osch_stats();
        replyf(owner, "total: cereri %lu resp %lu timeout %lu nrc %lu 0x78 %lu multi=%d, %d abonamente\r\n",
               (unsigned long)s.req, (unsigned long)s.resp, (unsigned long)s.timeout, (unsigned long)s.nrc,
               (unsigned long)s.pending78, s.multi_ok, shown);
        return;
    }
    if (!strcmp(cmd, "CFG")) {
        for (size_t i = 0; i < cfg_count(); i++) {
            const CfgDef *d = cfg_def(i);
            const char *val = (d->type == 'p') ? (strlen(cfg_str(d->key)) ? "***" : "") : cfg_str(d->key);
            replyf(owner, "  %-12s = %-28s (%s)\r\n", d->key, val, d->help);
        }
        replyf(owner, "  cfg.ver = %lu\r\n", (unsigned long)cfg_version());
        return;
    }
    if (!strcmp(cmd, "SET")) {
        if (nt < 3) { reply(owner, "SET <cheie> <valoare>   |   SET mode <model>  (profil vehicul)\r\n"); return; }
        // v2.3.0: `SET mode <model>` = scurtatura pentru veh.model + aducerea profilului de pe server
        if (!strcasecmp(tok[1], "mode") || !strcasecmp(tok[1], "veh.model")) {
            String err;
            if (!veh_fetch(tok[2], err)) { replyf(owner, "EROARE profil '%s': %s\r\n", tok[2], err.c_str()); return; }
            String e2; cfg_set("veh.model", tok[2], e2);
            replyf(owner, "%s\r\n", veh_status().c_str());
            return;
        }
        String err; bool ok = cfg_set(tok[1], tok[2], err);
        replyf(owner, ok ? "OK (SAVE pt persistenta e automat)\r\n" : "EROARE: %s\r\n", err.c_str());
        return;
    }
    if (!strcmp(cmd, "VEH")) {
        if (nt >= 2 && !strcasecmp(tok[1], "LIST")) {
            String out, err;
            if (veh_list(out, err)) replyf(owner, "%s\r\n", out.c_str());
            else replyf(owner, "lista esuata: %s\r\n", err.c_str());
            return;
        }
        if (nt >= 3 && !strcasecmp(tok[1], "GET")) {
            String err;
            if (!veh_fetch(tok[2], err)) { replyf(owner, "EROARE profil '%s': %s\r\n", tok[2], err.c_str()); return; }
            String e2; cfg_set("veh.model", tok[2], e2);
            replyf(owner, "%s\r\n", veh_status().c_str());
            return;
        }
        if (nt >= 2 && !strcasecmp(tok[1], "CLEAR")) {
            String err; bool ok = veh_clear(err);
            replyf(owner, "%s%s\r\n", ok ? "OK: " : "EROARE: ", err.c_str());
            return;
        }
        replyf(owner, "%s\r\n(VEH LIST | VEH GET <model> | VEH CLEAR)\r\n", veh_status().c_str());
        return;
    }
    if (!strcmp(cmd, "SAVE")) { reply(owner, "setarile se salveaza la fiecare SET (NVS)\r\n"); return; }
    if (!strcmp(cmd, "OTA")) {
        if (nt >= 2 && !strcasecmp(tok[1], "CHECK")) {
            OtaInfo inf; String e;
            if (ota_check(inf, e)) replyf(owner, "server: fw %s (acum %s) %s\r\n", inf.latest.c_str(), FW_VERSION, inf.newer ? "-> update disponibil" : "(la zi)");
            else replyf(owner, "OTA check esuat: %s\r\n", e.c_str());
        } else if (nt >= 2 && !strcasecmp(tok[1], "UPDATE")) {
            // download = o singura sesiune TLS: foloseste rezultatul unui CHECK anterior (evita fragmentarea heap)
            OtaInfo inf; String e;
            if (!ota_cached_newer(inf)) {
                if (!ota_check(inf, e)) { replyf(owner, "check esuat: %s\r\n", e.c_str()); return; }
                if (!inf.newer) { replyf(owner, "deja la %s\r\n", FW_VERSION); return; }
                replyf(owner, "verificat: fw %s disponibil. Ruleaza 'OTA UPDATE' din nou pt instalare.\r\n", inf.latest.c_str());
                return;
            }
            replyf(owner, "descarc %s...\r\n", inf.latest.c_str());
            if (ota_update(inf, e, nullptr)) { reply(owner, "OK, repornire\r\n"); proto_pump(300); ESP.restart(); }
            else replyf(owner, "update esuat: %s\r\n", e.c_str());
        } else reply(owner, "OTA CHECK | OTA UPDATE\r\n");
        return;
    }
    if (!strcmp(cmd, "SNIFF")) {
        if (nt >= 2 && !strcasecmp(tok[1], "STOP")) { String r = sniff_stop_upload(); replyf(owner, "%s\r\n", r.c_str()); return; }
        if (nt < 2) { reply(owner, "SNIFF <label> [url]  |  SNIFF STOP\r\n"); return; }
        bool ok = sniff_start(tok[1], nt >= 3 ? tok[2] : "", owner);
        replyf(owner, ok ? "SNIFF pornit (listen). SNIFF STOP pt salvare+upload (revine la modul de dinainte).\r\n" : "SNIFF: nu am putut porni (deja activ / FS)\r\n");
        return;
    }
    if (!strcmp(cmd, "READEPROM")) { cmd_readeprom(owner, tok, nt); return; }   // v2.5.0
    if (!strcmp(cmd, "TPQ")) { cmd_tpq(owner, tok, nt); return; }               // v2.5.6
    if (!strcmp(cmd, "UDSSCAN")) { cmd_udsscan(owner, tok, nt); return; }       // v2.5.7
    if (!strcmp(cmd, "DIDSCAN")) { cmd_didscan(owner, tok, nt); return; }       // v2.5.9
    if (!strcmp(cmd, "CODING")) { cmd_coding(owner, tok, nt); return; }           // v2.5.5
    if (!strcmp(cmd, "CODINGALL")) {                                              // v2.5.5
        uint16_t did = (nt >= 3) ? (uint16_t)strtol(tok[2], nullptr, 16) : CODING_DID_DEF;
        char nm[32]; nm[0] = 0;
        if (nt >= 2) { strncpy(nm, tok[1], sizeof(nm) - 1); nm[sizeof(nm) - 1] = 0; coding_sanitize(nm); }
        codingall_run(owner, did, nm); return;
    }
    if (!strcmp(cmd, "EXPLORE")) {
        if (nt >= 2 && !strcasecmp(tok[1], "OFF")) { if (owner < sizeof(g_explore)) g_explore[owner] = false; reply(owner, "EXPLORE off\r\n"); return; }
        if (nt < 2) { reply(owner, "EXPLORE <pin> | EXPLORE OFF\r\n"); return; }
        if (!strcmp(tok[1], cfg_str("con.pin"))) { if (owner < sizeof(g_explore)) g_explore[owner] = true;
            reply(owner, "EXPLORE activ: RAW <id> <hex>, UDS/TP2.0 module noi. Doar citiri; scrieri tot blocate.\r\n"); }
        else reply(owner, "PIN gresit\r\n");
        return;
    }
    if (!strcmp(cmd, "ACFG")) {
        extern String acfg_status(); extern String acfg_fetch_now();
        if (nt >= 2 && !strcasecmp(tok[1], "NOW")) { replyf(owner, "%s\r\n", acfg_fetch_now().c_str()); return; }
        if (nt >= 2 && !strcasecmp(tok[1], "ON"))  { String e; cfg_set("acfg.on", "1", e); reply(owner, "acfg ON (SET acfg.url / acfg.token)\r\n"); return; }
        if (nt >= 2 && !strcasecmp(tok[1], "OFF")) { String e; cfg_set("acfg.on", "0", e); reply(owner, "acfg off\r\n"); return; }
        replyf(owner, "%s\r\n", acfg_status().c_str());
        return;
    }
    if (!strcmp(cmd, "NETDIAG")) {
        extern String net_diag();
        reply(owner, net_diag().c_str());
        return;
    }
    if (!strcmp(cmd, "CLOUD")) {
        extern String cloud_status(); extern void cloud_post_now();
        if (nt >= 2 && !strcasecmp(tok[1], "NOW")) { cloud_post_now(); replyf(owner, "%s\r\n", cloud_status().c_str()); return; }
        if (nt >= 2 && !strcasecmp(tok[1], "ON"))  { String e; cfg_set("cloud.on", "1", e); reply(owner, "cloud ON (SET cloud.token / cloud.url daca e nevoie)\r\n"); return; }
        if (nt >= 2 && !strcasecmp(tok[1], "OFF")) { String e; cfg_set("cloud.on", "0", e); reply(owner, "cloud off\r\n"); return; }
        replyf(owner, "%s\r\n", cloud_status().c_str());
        return;
    }
    if (!strcmp(cmd, "MON")) {
        extern void con_set_mon(int owner, long id, bool on);
        if (nt >= 2 && !strcasecmp(tok[1], "OFF")) { con_set_mon(owner, 0, false); reply(owner, "MON off\r\n"); return; }
        long id = nt >= 2 ? strtol(tok[1], nullptr, 16) : 0;
        con_set_mon(owner, id, true);
        replyf(owner, "MON pornit%s (MON OFF pt stop)\r\n", id ? (String(" pe 0x") + String(id, HEX)).c_str() : " (tot busul)");
        return;
    }
    if (!strcmp(cmd, "DTCALL")) {   // v2.4.0: DTCALL [ALL] - lista de instalare gateway + erorile modulelor (doar cu erori; ALL = toate codate)
        if (strcmp(can_mode_name(), "normal") != 0) can_set_mode("normal");
        dtcall_run(owner, nt >= 2 && !strcasecmp(tok[1], "ALL"), true);
        return;
    }
    if (!strcmp(cmd, "MOD") || !strcmp(cmd, "MODPROBE") || !strcmp(cmd, "ID") || !strcmp(cmd, "GRP") ||
        !strcmp(cmd, "DID") || !strcmp(cmd, "UDS") || !strcmp(cmd, "UDSSCAN") || !strcmp(cmd, "DTC")) {
        if (strcmp(can_mode_name(), "normal") != 0) can_set_mode("normal");
        cmd_modules(owner, cmd, tok, nt);
        return;
    }
    if (!strcmp(cmd, "RAW")) {
        if (owner >= sizeof(g_explore) || !g_explore[owner]) { reply(owner, "RAW doar in EXPLORE (EXPLORE <pin>)\r\n"); return; }
        if (nt < 2) { reply(owner, "RAW <id> <hex bytes>\r\n"); return; }
        long id = strtol(tok[1], nullptr, 16);
        uint8_t d[8]; size_t n = 0;
        for (int i = 2; i < nt && n < 8; i++) d[n++] = (uint8_t)strtol(tok[i], nullptr, 16);
        extern bool can_send_raw_frame(uint32_t id, const uint8_t *d, size_t n, bool pad);
        bool ok = can_send_raw_frame((uint32_t)id, d, n, true);
        replyf(owner, "TX 0x%lX %s\r\n", id, ok ? "ok" : "refuzat");
        return;
    }
    replyf(owner, "necunoscut: %s (HELP)\r\n", cmd);
}

// ==================== module: UDS (motor/bord/... 0x7xx) + TP2.0 (ABS/gateway/... adr logica) ====================
// Harta reala Altea (Auto-Scan VCDS): UDS = 01 motor(7E0), 08 clima, 16 volan, 17 bord(714), 25 immo, 42/52 usi;
// TP2.0/KWP = 03 ABS, 04 unghi volan, 09 electrica, 15 airbag, 19 gateway, 44 EPS.
struct UdsCand { uint32_t req; const char *name; };
static const UdsCand UDS_CANDS[] = {
    {0x7E0, "01 Motor"}, {0x711, "25 Immo"}, {0x714, "17 Bord"}, {0x746, "08 Clima"},
    {0x74A, "42 Usa sof"}, {0x74B, "52 Usa pas"},   // v2.9: lista = rezultatul UDSSCAN pe Altea 09-23 (fara presupuneri)
};
struct TpCand { uint8_t addr; const char *name; };
// v2.9 (fw 2.5.8): candidatii MODPROBE/SELFTEST vin din rutele PROFILULUI (uds = ID cerere UDS, tp = adresa logica TP2.0).
// Fara "mods" in profil => listele compilate UDS_CANDS / TP_CANDS.
struct ModCand { uint16_t uds; uint8_t tp; char label[24]; };
static const TpCand TP_CANDS[] = {
    {0x01, "01 Motor"}, {0x03, "03 ABS"}, {0x13, "04 Unghi volan"}, {0x05, "15 Airbag"},
    {0x09, "44 EPS"}, {0x20, "09 Electrica"}, {0x1F, "19 Gateway"},
};

static int mod_cands(ModCand *out, int max) {
    int n = 0;
    if (dtcall_routes_from_profile()) {
        for (int i = 0; i < dtcall_route_count() && n < max; i++) {
            const ModRoute *r = dtcall_route_at(i);
            if (r->uds_req && n < max) { out[n].uds = r->uds_req; out[n].tp = 0; snprintf(out[n].label, sizeof(out[n].label), "%02X %s", r->adr, r->name); n++; }
            if (r->tp && n < max)      { out[n].uds = 0; out[n].tp = r->tp; snprintf(out[n].label, sizeof(out[n].label), "%02X %s", r->adr, r->name); n++; }
        }
        return n;
    }
    for (size_t i = 0; i < sizeof(UDS_CANDS) / sizeof(UDS_CANDS[0]) && n < max; i++) { out[n].uds = (uint16_t)UDS_CANDS[i].req; out[n].tp = 0; snprintf(out[n].label, sizeof(out[n].label), "%s", UDS_CANDS[i].name); n++; }
    for (size_t i = 0; i < sizeof(TP_CANDS) / sizeof(TP_CANDS[0]) && n < max; i++)   { out[n].uds = 0; out[n].tp = TP_CANDS[i].addr; snprintf(out[n].label, sizeof(out[n].label), "%s", TP_CANDS[i].name); n++; }
    return n;
}

// v2.2.3: ce module raspund efectiv — aceeasi sondare ca MODPROBE, dar rezumata pe doua linii,
// ca sa poata fi chemata si din SELFTEST (inainte acolo scria doar "disponibil in 2.1").
static void selftest_modules(uint8_t owner) {
    reply(owner, "Module  : sondez UDS + TP2.0 (cateva secunde)...\r\n");
    String uds_ok, tp_ok;
    ModCand mc[40]; int n = mod_cands(mc, 40);
    for (int i = 0; i < n; i++) {
        String err; bool ok = mc[i].uds ? uds_session_ok(mc[i].uds) : tp20_open(mc[i].tp, err);
        if (!mc[i].uds && ok) tp20_close();
        if (ok) { String &d = mc[i].uds ? uds_ok : tp_ok; if (d.length()) d += ", "; d += mc[i].label; }
    }
    replyf(owner, "  UDS   : %s\r\n", uds_ok.length() ? uds_ok.c_str() : "(niciunul)");
    replyf(owner, "  TP2.0 : %s\r\n", tp_ok.length() ? tp_ok.c_str() : "(niciunul)");
}

static void cmd_modules(uint8_t owner, char cmd[], char *tok[], int nt) {
    if (!strcmp(cmd, "MOD") || !strcmp(cmd, "MODPROBE")) {
        replyf(owner, "=== MODPROBE (ce raspunde pe masina) === module %s\r\n", dtcall_routes_from_profile() ? "din profil" : "din tabela compilata");
        ModCand mc[40]; int n = mod_cands(mc, 40);
        for (int i = 0; i < n; i++) {
            String err; bool ok;
            if (mc[i].uds) { ok = uds_session_ok(mc[i].uds); replyf(owner, "  UDS  0x%03X %-16s : %s\r\n", mc[i].uds, mc[i].label, ok ? "RASPUNDE (UDS)" : "-"); }
            else { ok = tp20_open(mc[i].tp, err); replyf(owner, "  TP20 0x%02X  %-16s : %s\r\n", mc[i].tp, mc[i].label, ok ? "CANAL OK (TP2.0)" : ("- " + err).c_str()); if (ok) tp20_close(); }
        }
        reply(owner, "=== gata ===\r\n");
        return;
    }
    if (!strcmp(cmd, "UDS")) {
        if (nt < 3) { reply(owner, "UDS <reqid_hex> <did_hex> [nr_citiri=1, 0=pana la q+Enter] [pauza_ms=1000]  (ex: UDS 7E0 121C 0 500 = km ECU la 0,5 s)\r\n"); return; }
        uint32_t reqid = strtol(tok[1], nullptr, 16); uint16_t did = strtol(tok[2], nullptr, 16);
        long reps = nt >= 4 ? atol(tok[3]) : 1; long gap = nt >= 5 ? atol(tok[4]) : 1000;   // v2.5
        if (reps < 0) reps = 1; if (gap < 50) gap = 50; if (gap > 600000) gap = 600000;
        bool repeat = (reps != 1);
        if (repeat) { g_rep_active = true; g_rep_owner = owner; g_rep_stop = false;
                      replyf(owner, "UDS 0x%03lX %04X: %s citiri la %ld ms — q + Enter opreste\r\n", (unsigned long)reqid, did, reps ? String(reps).c_str() : "nelimitat", gap); }
        uint32_t t_start = millis();
        for (long k = 0; (reps == 0 || k < reps) && !g_rep_stop; k++) {
            uint8_t out[64]; size_t n; String err;
            uint32_t tk = millis();
            if (repeat) replyf(owner, "#%ld %6lu.%03lus ", k + 1, (unsigned long)((tk - t_start) / 1000), (unsigned long)((tk - t_start) % 1000));
            if (!uds_did(reqid, did, out, sizeof(out), &n, err)) replyf(owner, "0x%03lX DID %04X: %s\r\n", (unsigned long)reqid, did, err.c_str());
            else {
                String hx; for (size_t i = 0; i < n; i++) { char b[4]; snprintf(b, 4, "%02X ", out[i]); hx += b; }
                replyf(owner, "0x%03lX DID %04X = %s", (unsigned long)reqid, did, hx.c_str());
                if (n >= 4) replyf(owner, " [u32=%lu u24=%lu]", (unsigned long)(((uint32_t)out[n-4]<<24)|((uint32_t)out[n-3]<<16)|(out[n-2]<<8)|out[n-1]),
                                   (unsigned long)(((uint32_t)out[n-3]<<16)|(out[n-2]<<8)|out[n-1]));   // v2.5.0: ex. ECU 7E0 DID 121C = km pe 4 octeti
                else if (n == 3) replyf(owner, " [u24=%lu]", (unsigned long)(((uint32_t)out[n-3]<<16)|(out[n-2]<<8)|out[n-1]));
                else if (n == 2) replyf(owner, " [u16=%u]", (out[0]<<8)|out[1]);
                else if (n == 1) replyf(owner, " [u8=%u]", out[0]);
                reply(owner, "\r\n");
            }
            if (!repeat || (reps && k + 1 >= reps)) break;
            // pauza intre citiri: pompam reteaua ca sa prindem „q" (si sa nu cada clientii)
            while (millis() - tk < (uint32_t)gap && !g_rep_stop) proto_pump(10);
        }
        if (repeat) { g_rep_active = false; reply(owner, g_rep_stop ? "UDS: oprit (q)\r\n" : "UDS: gata\r\n"); g_rep_stop = false; }
        return;
    }
    if (!strcmp(cmd, "UDSSCAN")) {
        if (nt < 4) { reply(owner, "UDSSCAN <reqid> <did_start_hex> <count>  (cauta DID-uri care raspund)\r\n"); return; }
        uint32_t reqid = strtol(tok[1], nullptr, 16); uint16_t d0 = strtol(tok[2], nullptr, 16); int cnt = atoi(tok[3]);
        if (cnt > 256) cnt = 256;
        int found = 0;
        for (int k = 0; k < cnt; k++) {
            uint8_t out[64]; size_t n; String err;
            if (uds_did(reqid, d0 + k, out, sizeof(out), &n, err)) {
                String hx; for (size_t i = 0; i < n && i < 12; i++) { char b[4]; snprintf(b, 4, "%02X ", out[i]); hx += b; }
                replyf(owner, "  %04X = %s\r\n", d0 + k, hx.c_str()); found++;
            }
            proto_pump(3);
        }
        replyf(owner, "gata: %d DID-uri au raspuns din %d\r\n", found, cnt);
        return;
    }
    if (!strcmp(cmd, "ID")) {
        if (nt < 2) { reply(owner, "ID <adr_logica_hex>  (ex: ID 03 = ABS)\r\n"); return; }
        uint8_t a = strtol(tok[1], nullptr, 16); String pn, err;
        if (tp20_ident(a, pn, err)) replyf(owner, "0x%02X ident: %s\r\n", a, pn.c_str());
        else replyf(owner, "0x%02X: %s\r\n", a, err.c_str());
        return;
    }
    if (!strcmp(cmd, "GRP")) {
        if (nt < 3) { reply(owner, "GRP <adr_logica_hex> <grup>  (ex: GRP 03 4 = ABS grupa 4)\r\n"); return; }
        uint8_t a = strtol(tok[1], nullptr, 16); uint8_t g = atoi(tok[2]); String dec, err;
        if (tp20_group(a, g, dec, err)) replyf(owner, "0x%02X %s\r\n", a, dec.c_str());
        else replyf(owner, "0x%02X grp %d: %s\r\n", a, g, err.c_str());
        return;
    }
    if (!strcmp(cmd, "DID")) { reply(owner, "foloseste: UDS <reqid> <did>  sau  GRP <adr> <grup>\r\n"); return; }
    if (!strcmp(cmd, "DTC")) {
        if (nt >= 3 && !strcasecmp(tok[1], "CLR")) {
            if (cfg_int("dtc.clear") < 1) { reply(owner, "stergere DTC dezactivata (SET dtc.clear 1)\r\n"); return; }
            reply(owner, "stergere DTC: neimplementata inca (siguranta) — se face din VCDS\r\n"); return;
        }
        if (nt < 2) { reply(owner, "DTC <adr>  (0x7xx=UDS 19, adr mica=TP2.0 18)\r\n"); return; }
        uint32_t a = strtol(tok[1], nullptr, 16);
        if (a >= 0x100) {   // UDS 19 02 FF
            iso_tp_allow(uds_resp_id(a), a);
            uint8_t req[3] = {0x19, 0x02, 0xFF};
            if (!await_resp(a, req, 3, 0x59, 1500)) { replyf(owner, "0x%03lX: fara raspuns\r\n", (unsigned long)a); return; }
            if (g_alen >= 3 && g_abuf[0] == 0x59) {
                int nd = (g_alen - 3) / 4; replyf(owner, "0x%03lX: %d DTC:", (unsigned long)a, nd);
                for (int k = 0; k < nd; k++) replyf(owner, " %02X%02X%02X/st%02X", g_abuf[3+4*k], g_abuf[4+4*k], g_abuf[5+4*k], g_abuf[6+4*k]);
                reply(owner, "\r\n");
            } else reply(owner, "raspuns 19 neasteptat\r\n");
        } else {            // TP2.0 KWP 18
            String codes, err;
            if (tp20_dtc((uint8_t)a, codes, err)) replyf(owner, "0x%02lX: %s\r\n", (unsigned long)a, codes.c_str());
            else replyf(owner, "0x%02lX: %s\r\n", (unsigned long)a, err.c_str());
        }
        return;
    }
}

// ==================== READEPROM (v2.5.0): citire memorie modul TP2.0 ca VAG CAN PRO ====================
// ==================== v2.5.5: CODING / CODINGALL - citire long coding (serviciul 22, DID 0600) ====================
// Simetric cu DTC / DTCALL, cerut de user: CODING <adr> citeste codarea unui modul, CODINGALL o citeste de pe TOATE
// modulele marcate ca CODATE in lista de instalare a gateway-ului (flags bit0 = GW_FLAG_CODED). DOAR CITIRE (SID 22).
// Transport, dupa marimea argumentului:
//   adr <= 0xFF  -> adresa logica TP2.0 (ca la READEPROM; ex. 20 = electrica centrala / BCM)
//   adr >= 0x100 -> ID de cerere UDS pe CAN            (ex. 714 = bord, 7E0 = ECU motor)
// DID implicit 0600, CONFIRMAT pe BCM prin sniff VAG CAN PRO (docs/research/sniff_20260921_bcm/README.md:
// `22 0600` -> `62 06 00` + 30 B long coding, in sesiunea implicita, FARA seed/key). Se poate da alt DID ca ultim argument.
// Cu <nume_byos>, rezultatul se urca si la BYOS pe acelasi handler ca SNIFF/READEPROM (type=can_log, kind=coding).
#define CODING_MAX_LEN   64

struct CodRes { char tr; char err[32]; uint8_t n; uint8_t d[CODING_MAX_LEN]; };

static void cr_reset(CodRes &r, char tr) { r.tr = tr; r.err[0] = 0; r.n = 0; }
static void cr_err(CodRes &r, const char *e) { strncpy(r.err, e, sizeof(r.err) - 1); r.err[sizeof(r.err) - 1] = 0; r.n = 0; }
static void cr_take(CodRes &r, const uint8_t *d, size_t n) {
    if (n > CODING_MAX_LEN) n = CODING_MAX_LEN;
    memcpy(r.d, d, n); r.n = (uint8_t)n;
}
static String cr_hex(const CodRes &r, bool spaced) {
    String s; s.reserve(r.n * 3 + 32);
    for (uint8_t i = 0; i < r.n; i++) { char b[4]; snprintf(b, sizeof(b), "%02X", r.d[i]); if (spaced && i) s += " "; s += b; }
    if (spaced && r.tr == 'k' && r.n == 3) {   // v2.5.9: codare scurta KWP, 7 cifre ca in VCDS
        char b[40]; snprintf(b, sizeof(b), "  = coding %07lu", (unsigned long)(((uint32_t)r.d[0] << 16) | ((uint32_t)r.d[1] << 8) | r.d[2]));
        s += b;
    }
    return s;
}
static const char *cr_trname(const CodRes &r)  { return r.tr == 'u' ? "uds" : (r.tr == 'k' ? "kwp" : "tp"); }
static const char *cr_trlabel(const CodRes &r) { return r.tr == 'u' ? "UDS" : (r.tr == 'k' ? "KWP 1A9B" : "TP2.0"); }

static bool coding_via_uds(uint32_t req, uint16_t did, CodRes &r) {
    cr_reset(r, 'u');
    uint8_t out[CODING_MAX_LEN]; size_t n = 0; String err;
    if (!uds_did(req, did, out, sizeof(out), &n, err)) {          // unele module cer sesiune extinsa (ca la DTC)
        if (!uds_session_ok(req) || !uds_did(req, did, out, sizeof(out), &n, err)) { cr_err(r, err.c_str()); return false; }
    }
    if (!n) { cr_err(r, "raspuns gol"); return false; }
    cr_take(r, out, n); return true;
}

static bool coding_via_tp(uint8_t pos, uint16_t did, CodRes &r) {
    uint8_t q[3] = {0x22, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF)};
    for (int att = 0; att < 2; att++) {        // 0 = canal simplu (ca VAG CAN PRO la BCM), 1 = cu 10 89 (ca VCDS)
        cr_reset(r, 't');
        String err;
        bool op = (att == 0) ? tp20_open_ex(pos, false, err) : tp20_open(pos, err);
        if (!op) { cr_err(r, err.c_str()); tp20_close(); continue; }
        uint8_t resp[CODING_MAX_LEN + 8]; size_t rl = 0;
        bool ok = tp20_kwp(q, 3, resp, sizeof(resp), &rl, err);
        if (ok && rl >= 4 && resp[0] == 0x62 && resp[1] == q[1] && resp[2] == q[2]) {
            cr_take(r, resp + 3, rl - 3); tp20_close(); return true;
        }
        cr_err(r, ok ? "raspuns 22 neasteptat" : err.c_str());
        tp20_close();
    }
    // v2.5.9: module KWP vechi (airbag...) nu au DID 0600; codarea scurta sta in identificarea 1A 9B, octetii 17..19
    // (u24, 7 cifre). Verificat pe airbag Altea 1K0909605T: 00 53 37 = 0021303, identic cu VCDS.
    if (did != CODING_DID_DEF) return false;
    {
        String err;
        if (!tp20_open(pos, err)) { tp20_close(); return false; }
        uint8_t q2[2] = {0x1A, 0x9B}; uint8_t resp[96]; size_t rl = 0;
        bool ok = tp20_kwp(q2, 2, resp, sizeof(resp), &rl, err);
        tp20_close();
        if (ok && rl >= 2 + 20 && resp[0] == 0x5A && resp[1] == 0x9B) {
            cr_reset(r, 'k'); cr_take(r, resp + 2 + 17, 3); return true;
        }
    }
    return false;
}

// modul din lista de instalare: transportul preferat din tabela dtcall.cpp, apoi alternativa (ca dtc_module)
static bool coding_module(const GwMod &m, uint16_t did, CodRes &r) {
    const ModRoute *rt = dtcall_route(m.adr);
    uint32_t ureq = rt ? rt->uds_req : 0;
    bool pref_uds = rt && rt->pref_uds && ureq;
    for (int pass = 0; pass < 2; pass++) {
        bool do_uds = (pass == 0) ? pref_uds : !pref_uds;
        if (do_uds) { if (!ureq) continue; if (coding_via_uds(ureq, did, r)) return true; }
        else        { if (coding_via_tp(m.pos, did, r)) return true; }
    }
    return false;
}

static String coding_item(const char *aid, const char *nm, uint16_t did, const CodRes &r, bool ok) {
    char h[128];
    if (!ok) {
        char e[72]; jl_escape(e, sizeof(e), r.err);
        snprintf(h, sizeof(h), "{\"a\":\"%s\",\"nm\":\"%s\",\"did\":\"%04X\",\"err\":\"%s\"}", aid, nm, did, e);
        return String(h);
    }
    snprintf(h, sizeof(h), "{\"a\":\"%s\",\"nm\":\"%s\",\"did\":\"%04X\",\"tr\":\"%s\",\"n\":%u,\"d\":\"",
             aid, nm, did, cr_trname(r), r.n);
    String s = h; s += cr_hex(r, false); s += "\"}";
    return s;
}
static String coding_json(const char *aid, const char *nm, uint16_t did, const CodRes &r, bool ok) {
    String s = "{\"t\":\"coding\","; String it = coding_item(aid, nm, did, r, ok);
    s += it.substring(1); s += "\n"; return s;
}

// upload BYOS: acelasi handler ca SNIFF / READEPROM (type=can_log), kind=coding, modulele in "list"
static void coding_upload(uint8_t owner, const char *name, const String &items, int n) {
    if (WiFi.status() != WL_CONNECTED) { reply(owner, "  upload: WiFi offline (datele sunt doar pe consola)\r\n"); return; }
    String url = sniff_cloud_url("can_log");
    String body = String("{\"type\":\"can_log\",\"label\":\"") + name + "\",\"chunk\":0,\"chunks\":1,\"ts\":" + (unsigned long)millis() +
                  ",\"kind\":\"coding\",\"fw\":\"" FW_VERSION "\",\"n\":" + n + ",\"data\":\"\",\"frames\":[],\"list\":[" + items + "]}";
    String err;
    if (sniff_post_json(url, body, err)) replyf(owner, "  upload OK (label '%s', %d modul%s)\r\n", name, n, n == 1 ? "" : "e");
    else replyf(owner, "  upload ESUAT: %s\r\n", err.c_str());
}

static void coding_sanitize(char *s) { for (char *p = s; *p; p++) if (!isalnum((unsigned char)*p) && *p != '_' && *p != '-') *p = '_'; }

// CODING <adr_tp20_hex | canid_uds_hex> [nume_byos] [did_hex]
static void cmd_coding(uint8_t owner, char *tok[], int nt) {
    if (nt < 2) { reply(owner, "CODING <adr_tp20_hex | canid_uds_hex> [nume_byos] [did_hex=0600]   (ex: CODING 20 bcm_coding | CODING 714)\r\n"); return; }
    if (sniff_active()) { reply(owner, "CODING: SNIFF activ (SNIFF STOP intai)\r\n"); return; }
    if (g_dtc_running)  { reply(owner, "CODING: DTCALL ruleaza acum, reia in cateva secunde\r\n"); return; }
    long adr = strtol(tok[1], nullptr, 16);
    if (adr < 1 || adr > 0x7FF) { reply(owner, "CODING: adresa invalida (01..FF = TP2.0, 100..7FF = UDS)\r\n"); return; }
    uint16_t did = (nt >= 4) ? (uint16_t)strtol(tok[3], nullptr, 16) : CODING_DID_DEF;
    char name[32]; name[0] = 0;
    if (nt >= 3) { strncpy(name, tok[2], sizeof(name) - 1); name[sizeof(name) - 1] = 0; coding_sanitize(name); }

    g_dtc_running = true;                                   // blocheaza scanul automat DTC cat tinem canalul
    char prev[8]; strncpy(prev, can_mode_name(), sizeof(prev) - 1); prev[sizeof(prev) - 1] = 0;
    if (strcmp(prev, "normal") != 0) can_set_mode("normal");

    CodRes r; bool ok; uint32_t t0 = millis();
    char aid[8];
    if (adr <= 0xFF) {
        snprintf(aid, sizeof(aid), "%02lX", adr);
        replyf(owner, "CODING modul TP2.0 0x%02lX (%s), DID %04X ...\r\n", adr, dtcall_mod_name((uint8_t)adr), did);
        ok = coding_via_tp((uint8_t)adr, did, r);
    } else {
        snprintf(aid, sizeof(aid), "%03lX", adr);
        replyf(owner, "CODING UDS 0x%03lX, DID %04X ...\r\n", adr, did);
        ok = coding_via_uds((uint32_t)adr, did, r);
    }
    const char *nm = (adr <= 0xFF) ? dtcall_mod_name((uint8_t)adr) : "?";

    if (strcmp(prev, "normal") != 0) can_set_mode(prev);
    osch_set_enabled(strcmp(can_mode_name(), "listen") != 0);
    g_dtc_running = false;

    if (!ok) replyf(owner, "CODING ESUAT: %s (%lu ms)\r\n", r.err, (unsigned long)(millis() - t0));
    else {
        replyf(owner, "  [%s] %u octeti, %lu ms:\r\n", cr_trlabel(r), r.n, (unsigned long)(millis() - t0));
        replyf(owner, "  %s\r\n", cr_hex(r, true).c_str());
    }
    String js = coding_json(aid, nm, did, r, ok); emitJson(js.c_str(), js.length());
    if (name[0]) coding_upload(owner, name, coding_item(aid, nm, did, r, ok), 1);
}

// CODINGALL [nume_byos] [did_hex]
static void codingall_run(uint8_t owner, uint16_t did, const char *name) {
    if (sniff_active()) { reply(owner, "CODINGALL: SNIFF activ (SNIFF STOP intai)\r\n"); return; }
    if (g_dtc_running)  { reply(owner, "CODINGALL: DTCALL ruleaza acum, reia in cateva secunde\r\n"); return; }
    g_dtc_running = true;
    char prev[8]; strncpy(prev, can_mode_name(), sizeof(prev) - 1); prev[sizeof(prev) - 1] = 0;
    if (strcmp(prev, "normal") != 0) can_set_mode("normal");

    uint32_t t0 = millis();
    GwMod mods[GW_MAX_MODS]; int n = 0; String err;
    bool ok = gw_read_install(mods, &n, err);
    if (!ok) { proto_pump(200); err = ""; ok = gw_read_install(mods, &n, err); }   // a doua incercare (ca la DTCALL)
    tp20_close();
    if (!ok) {
        replyf(owner, "CODINGALL: lista de instalare esuata: %s\r\n", err.c_str());
        if (strcmp(prev, "normal") != 0) can_set_mode(prev);
        osch_set_enabled(strcmp(can_mode_name(), "listen") != 0);
        g_dtc_running = false; return;
    }
    int coded = 0; for (int i = 0; i < n; i++) if (mods[i].flags & GW_FLAG_CODED) coded++;
    replyf(owner, "=== CODINGALL === %d module in lista, %d codate, DID %04X\r\n", n, coded, did);

    String items; int rd = 0, bad = 0;
    for (int i = 0; i < n; i++) {
        const GwMod &m = mods[i];
        if (!(m.flags & GW_FLAG_CODED)) continue;               // necodate: nu are ce citi
        CodRes r; bool okm = coding_module(m, did, r);
        char aid[8]; snprintf(aid, sizeof(aid), "%02X", m.adr);
        const char *nm = dtcall_mod_name(m.adr);
        if (items.length()) items += ",";
        items += coding_item(aid, nm, did, r, okm);
        String js = coding_json(aid, nm, did, r, okm); emitJson(js.c_str(), js.length());
        if (okm) { rd++;  replyf(owner, "  %02X %-14s [%s] %2u B: %s\r\n", m.adr, nm, cr_trlabel(r), r.n, cr_hex(r, true).c_str()); }
        else     { bad++; replyf(owner, "  %02X %-14s NU RASPUNDE: %s\r\n", m.adr, nm, r.err); }
        proto_pump(3); proto_keepalive();
    }
    tp20_close();
    if (strcmp(prev, "normal") != 0) can_set_mode(prev);
    osch_set_enabled(strcmp(can_mode_name(), "listen") != 0);
    g_dtc_running = false;

    replyf(owner, "=== gata: %d citite, %d esuate, %lu ms ===\r\n", rd, bad, (unsigned long)(millis() - t0));
    if (name && name[0]) coding_upload(owner, name, items, rd + bad);
}

// READEPROM <adr_modul_hex> <adresa_hex> <lungime_dec> <nume>
//   ex: READEPROM 20 000C01 47 bcm_cfg   = exact blocul pe care il citeste VCP la „citire EEPROM BCM" (J519, Altea)
// Secventa (docs/research/sniff_20260921_bcm/README.md): canal TP2.0 -> 22 F187 -> 27 03/04 -> 10 86 -> 35/36/37 -> 82.
// DOAR citire; manual; pe loc. Modul CAN de dinainte (listen/normal) e restaurat la final. Rezultatul: hex pe consola +
// POST la cloud.url?type=can_log {"kind":"eeprom",...,"data":"<hex>","frames":[]} (aceeasi tabela ca SNIFF, fara schimbari server).
static void cmd_readeprom(uint8_t owner, char *tok[], int nt) {
    if (nt < 5) { reply(owner, "READEPROM <adr_modul_hex> <adresa_hex> <lungime_dec> <nume>   ex: READEPROM 20 000C01 47 bcm_cfg\r\n"); return; }
    if (sniff_active()) { reply(owner, "READEPROM: SNIFF activ (SNIFF STOP intai)\r\n"); return; }
    if (tp20_sec_locked()) { reply(owner, "READEPROM: cheia a fost refuzata de modul in aceasta sesiune; REBOOT inainte de o noua incercare\r\n"); return; }
    long mod = strtol(tok[1], nullptr, 16); long addr = strtol(tok[2], nullptr, 16); long len = strtol(tok[3], nullptr, 10);
    if (mod < 1 || mod > 0xFF || addr < 0 || addr > 0xFFFFFF || len < 1 || len > READEPROM_MAX_LEN) {
        replyf(owner, "READEPROM: parametri invalizi (modul 01..FF, adresa 0..FFFFFF, lungime 1..%d)\r\n", READEPROM_MAX_LEN); return;
    }
    char name[32]; strncpy(name, tok[4], sizeof(name) - 1); name[sizeof(name) - 1] = 0;
    for (char *p = name; *p; p++) if (!isalnum((unsigned char)*p) && *p != '_' && *p != '-') *p = '_';

    char prev[8]; strncpy(prev, can_mode_name(), sizeof(prev) - 1); prev[sizeof(prev) - 1] = 0;
    if (strcmp(prev, "normal") != 0) can_set_mode("normal");
    replyf(owner, "READEPROM modul 0x%02lX adresa 0x%06lX %ld octeti -> '%s' ...\r\n", mod, addr, len, name);

    uint8_t *buf = (uint8_t *)malloc(len);
    if (!buf) { reply(owner, "READEPROM: fara memorie\r\n"); if (strcmp(prev, "normal") != 0) can_set_mode(prev); return; }
    Tp20MemResult r; String err;
    bool ok = tp20_read_mem((uint8_t)mod, (uint32_t)addr, (size_t)len, buf, (size_t)len, r, err, READEPROM_CHUNK);
    if (strcmp(prev, "normal") != 0) can_set_mode(prev);
    osch_set_enabled(strcmp(can_mode_name(), "listen") != 0);

    if (r.partno.length()) replyf(owner, "  modul: %s\r\n", r.partno.c_str());
    if (r.seed) replyf(owner, "  seed %08lX -> key %08lX, blockmax %u, %lu ms%s\r\n", (unsigned long)r.seed, (unsigned long)r.key, r.blockmax, (unsigned long)r.ms,
                       r.seed_retry ? " (27 03 a mers la a 2-a incercare)" : "");
    if (r.stop_nrc) replyf(owner, "  oprit la 0x%06lX: %s — citit %lu/%ld octeti (0x%06lX..0x%06lX)\r\n", (unsigned long)r.stop_addr,
                           r.stop_nrc == 0x31 ? "NRC 0x31 requestOutOfRange = margine memorie" : r.stop_nrc == 0x33 ? "NRC 0x33 zona protejata" :
                           r.stop_nrc == 0xFF ? "fara raspuns" : r.stop_nrc == 0xFE ? "raspuns neasteptat / fara date" : (String("NRC 0x") + String(r.stop_nrc, HEX)).c_str(),
                           (unsigned long)r.got, len, (unsigned long)addr, (unsigned long)(addr + (r.got ? r.got - 1 : 0)));
    // v2.5: cadrele TP2.0 (trace) - JSON pentru upload + text pe consola la esec
    String frames; frames.reserve(tp20_trace_count() * 48 + 4);
    for (size_t i = 0; i < tp20_trace_count(); i++) {
        const Tp20Frame *f = tp20_trace_get(i);
        char hx[20]; int p = 0; for (int k = 0; k < f->dlc; k++) p += snprintf(hx + p, sizeof(hx) - p, "%02X", f->d[k]);
        if (i) frames += ",";
        frames += String("{\"t\":") + f->t + ",\"id\":\"" + String(f->id, HEX) + "\",\"e\":0,\"x\":" + f->tx + ",\"d\":\"" + hx + "\"}";
        if (!ok) {
            char line[48]; int q = 0;
            for (int k = 0; k < f->dlc; k++) q += snprintf(line + q, sizeof(line) - q, " %02X", f->d[k]);
            replyf(owner, "  %5lu ms %c %03X%s\r\n", (unsigned long)f->t, f->tx ? '>' : '<', f->id, line);
        }
    }
    if (!ok && tp20_trace_count() >= TP20_TRACE_MAX) reply(owner, "  (trace plin, cadrele urmatoare nu au fost retinute)\r\n");
    if (!ok) {
        replyf(owner, "READEPROM ESUAT: %s\r\n", err.c_str()); free(buf);
        if (WiFi.status() == WL_CONNECTED) {   // urcam si esecul (cu cadre), ca sa-l putem analiza de pe server
            String url = sniff_cloud_url("can_log");
            String body = String("{\"type\":\"can_log\",\"label\":\"") + name + "_fail\",\"chunk\":0,\"ts\":" + (unsigned long)millis() +
                          ",\"kind\":\"eeprom\",\"fw\":\"" FW_VERSION "\",\"mod\":\"" + String((unsigned long)mod, HEX) +
                          "\",\"addr\":\"" + String((unsigned long)addr, HEX) + "\",\"len\":" + (unsigned long)len + ",\"got\":0,\"err\":\"" + err +
                          "\",\"seed\":\"" + String((unsigned long)r.seed, HEX) + "\",\"partno\":\"" + r.partno + "\",\"data\":\"\",\"frames\":[" + frames + "]}";
            String e2;
            if (sniff_post_json(url, body, e2)) replyf(owner, "  upload esec OK (label '%s_fail')\r\n", name);
            else replyf(owner, "  upload esec ESUAT: %s\r\n", e2.c_str());
        }
        return;
    }
    if (err.length() && !r.stop_nrc) replyf(owner, "  atentie: %s\r\n", err.c_str());

    // v2.6: hex pe consola doar pentru citiri mici (o citire de 32 KB ar insemna 2048 linii pe telnet)
    size_t show = r.got <= 512 ? r.got : 64;
    for (size_t i = 0; i < show; i++) {
        if ((i & 15) == 0) replyf(owner, "  %06lX:", (unsigned long)(addr + i));
        replyf(owner, " %02X", buf[i]);
        if ((i & 15) == 15 || i + 1 == show) reply(owner, "\r\n");
    }
    if (show < r.got) replyf(owner, "  ... inca %lu octeti (doar in BYOS)\r\n", (unsigned long)(r.got - show));

    // upload BYOS (aceeasi tabela/handler ca SNIFF: type=can_log), pe bucati de 4 KB; cadrele doar in chunk 0
    if (WiFi.status() != WL_CONNECTED) { reply(owner, "  upload: WiFi offline (datele sunt doar pe consola)\r\n"); free(buf); return; }
    String url = sniff_cloud_url("can_log");
    const size_t UPCH = 4096; size_t nch = (r.got + UPCH - 1) / UPCH; if (nch == 0) nch = 1; size_t okc = 0; String e2;
    for (size_t c = 0; c < nch; c++) {
        size_t off = c * UPCH, n = r.got - off; if (n > UPCH) n = UPCH;
        String hex; hex.reserve(n * 2 + 4);
        for (size_t i = 0; i < n; i++) { char b[4]; snprintf(b, sizeof(b), "%02X", buf[off + i]); hex += b; }
        String body = String("{\"type\":\"can_log\",\"label\":\"") + name + "\",\"chunk\":" + c + ",\"chunks\":" + nch + ",\"ts\":" + (unsigned long)millis() +
                      ",\"kind\":\"eeprom\",\"fw\":\"" FW_VERSION "\",\"mod\":\"" + String((unsigned long)mod, HEX) +
                      "\",\"addr\":\"" + String((unsigned long)(addr + off), HEX) + "\",\"addr0\":\"" + String((unsigned long)addr, HEX) +
                      "\",\"len\":" + (unsigned long)len + ",\"got\":" + (unsigned long)n + ",\"total\":" + (unsigned long)r.got +
                      ",\"stop_nrc\":" + r.stop_nrc + ",\"stop_addr\":\"" + String((unsigned long)r.stop_addr, HEX) +
                      "\",\"blockmax\":" + r.blockmax + ",\"seed\":\"" + String((unsigned long)r.seed, HEX) + "\",\"partno\":\"" + r.partno +
                      "\",\"data\":\"" + hex + "\",\"frames\":[" + (c == 0 ? frames : String("")) + "]}";
        if (sniff_post_json(url, body, e2)) okc++; else { replyf(owner, "  upload bucata %u ESUAT: %s\r\n", (unsigned)c, e2.c_str()); break; }
        proto_pump(5);
    }
    free(buf);
    if (okc == nch) replyf(owner, "  upload OK: %u bucat%s, %lu octeti (label '%s')\r\n", (unsigned)nch, nch == 1 ? "a" : "i", (unsigned long)r.got, name);
}

// ==================== TPQ (v2.5.6): cereri KWP de citire pe un canal TP2.0, cu trace ====================
// Diagnostic pentru module care refuza secventa standard (ABS Altea: NRC 0x21 la tot). Canalul se deschide FARA 10 89,
// cererile (max 4, hex lipit: 1089 2101 1A9B) se trimit in ordine pe ACELASI canal, apoi A8. Se afiseaza fiecare raspuns
// si toate cadrele. SIGURANTA: doar SID de citire 10 (81/89), 1A, 18, 21, 22, 3E — restul refuzate.
#define TPQ_REQ_MAX 16
static bool tpq_allowed(const uint8_t *q, size_t n) {
    if (n < 1) return false;
    switch (q[0]) {
        case 0x10: return n == 2 && (q[1] == 0x81 || q[1] == 0x89);
        case 0x1A: case 0x18: case 0x21: case 0x22: case 0x3E: return true;
        default: return false;
    }
}
static void cmd_tpq(uint8_t owner, char *tok[], int nt) {
    if (nt < 3) { reply(owner, "TPQ <adr_tp20_hex> <cerere_hex> [cerere2..4]   ex: TPQ 03 2101 | TPQ 03 1089 2101 | TPQ 03 1A9B\r\n"
                                "  doar citire: 1081 1089 1Axx 18.. 21xx 22xxxx 3E..; canalul se deschide fara 10 89\r\n"); return; }
    if (sniff_active()) { reply(owner, "TPQ: SNIFF activ (SNIFF STOP intai)\r\n"); return; }
    if (g_dtc_running)  { reply(owner, "TPQ: DTCALL ruleaza acum, reia in cateva secunde\r\n"); return; }
    long adr = strtol(tok[1], nullptr, 16);
    if (adr < 1 || adr > 0xFF) { reply(owner, "TPQ: adresa TP2.0 invalida (01..FF)\r\n"); return; }
    uint8_t reqs[4][TPQ_REQ_MAX]; size_t rl[4]; int nq = 0;
    for (int i = 2; i < nt && nq < 4; i++) {
        size_t L = strlen(tok[i]);
        if (L < 2 || (L & 1) || L / 2 > TPQ_REQ_MAX) { replyf(owner, "TPQ: cerere invalida '%s' (hex lipit, numar par de cifre)\r\n", tok[i]); return; }
        for (size_t k = 0; k < L / 2; k++) {
            char b[3] = {tok[i][2 * k], tok[i][2 * k + 1], 0}; char *e;
            reqs[nq][k] = (uint8_t)strtol(b, &e, 16);
            if (*e) { replyf(owner, "TPQ: hex invalid '%s'\r\n", tok[i]); return; }
        }
        rl[nq] = L / 2;
        if (!tpq_allowed(reqs[nq], rl[nq])) { replyf(owner, "TPQ: '%s' refuzat (doar citire: 1081 1089 1A 18 21 22 3E)\r\n", tok[i]); return; }
        nq++;
    }
    g_dtc_running = true;                                   // blocheaza scanul automat DTC cat tinem canalul
    char prev[8]; strncpy(prev, can_mode_name(), sizeof(prev) - 1); prev[sizeof(prev) - 1] = 0;
    if (strcmp(prev, "normal") != 0) can_set_mode("normal");
    if (tp20_is_open()) tp20_close();
    replyf(owner, "TPQ modul 0x%02lX (%s), %d cereri, canal fara 10 89 ...\r\n", adr, dtcall_mod_name((uint8_t)adr), nq);
    tp20_trace_begin();
    uint32_t t0 = millis(); String err;
    if (!tp20_open_ex((uint8_t)adr, false, err)) replyf(owner, "  canal: %s\r\n", err.c_str());
    else {
        reply(owner, "  canal: OK\r\n");
        for (int i = 0; i < nq; i++) {
            uint8_t resp[128]; size_t n = 0; String e;
            uint32_t tq = millis();
            bool ok = tp20_kwp(reqs[i], rl[i], resp, sizeof(resp), &n, e);
            String hx; for (size_t k = 0; k < n && k < 40; k++) { char b[4]; snprintf(b, 4, "%02X ", resp[k]); hx += b; }
            if (n > 40) hx += "...";
            String busy = tp20_busy_retries() ? String(", busy x") + tp20_busy_retries() : String("");
            replyf(owner, "  %s -> %s%s [%lu ms%s]\r\n", tok[2 + i], ok ? "OK " : "", ok ? hx.c_str() : e.c_str(),
                   (unsigned long)(millis() - tq), busy.c_str());
            proto_pump(5);
        }
        tp20_close();
    }
    tp20_trace_end();
    reply(owner, "  trace (> noi, < modul):\r\n");
    for (size_t i = 0; i < tp20_trace_count(); i++) {
        const Tp20Frame *f = tp20_trace_get(i);
        char line[48]; int q = 0;
        for (int k = 0; k < f->dlc; k++) q += snprintf(line + q, sizeof(line) - q, " %02X", f->d[k]);
        replyf(owner, "  %5lu ms %c %03X%s\r\n", (unsigned long)f->t, f->tx ? '>' : '<', f->id, line);
        if ((i & 7) == 7) proto_pump(2);
    }
    if (tp20_trace_count() >= TP20_TRACE_MAX) reply(owner, "  (trace plin: se vad doar ultimele cadre)\r\n");
    if (strcmp(prev, "normal") != 0) can_set_mode(prev);
    osch_set_enabled(strcmp(can_mode_name(), "listen") != 0);
    g_dtc_running = false;
    replyf(owner, "=== TPQ gata, %lu ms ===\r\n", (unsigned long)(millis() - t0));
}

// v2.5.7: UDSSCAN [nume_byos] [de_la_hex] [pana_la_hex] — cine raspunde UDS pe fiecare ID (cerere 22 F187 = cod piesa, doar citire).
// Raspunsul trebuie sa vina pe ID-ul de raspuns al cererii (veh_resp_id: +0x6A, +8 la 7E0-7E7), altfel e ignorat.
// Argumente: hex de max 3 cifre = interval; orice altceva = nume => upload BYOS (type=can_log, kind=udsscan, list=[{req,rsp,pn}]).
static void cmd_udsscan(uint8_t owner, char *tok[], int nt) {
    if (sniff_active()) { reply(owner, "UDSSCAN: SNIFF activ (SNIFF STOP intai)\r\n"); return; }
    if (g_dtc_running)  { reply(owner, "UDSSCAN: DTCALL/CODING ruleaza acum\r\n"); return; }
    uint32_t a = 0x700, b = 0x7DF; int nr = 0; char name[32]; name[0] = 0;
    for (int i = 1; i < nt; i++) {
        char *e; unsigned long v = strtoul(tok[i], &e, 16);
        if (*e == 0 && strlen(tok[i]) <= 3) { if (nr++ == 0) a = v; else b = v; }
        else { strncpy(name, tok[i], sizeof(name) - 1); name[sizeof(name) - 1] = 0; coding_sanitize(name); }
    }
    if (nr == 1) b = a;
    if (a < 0x100 || b > 0x7FF || a > b) { reply(owner, "UDSSCAN [nume_byos] [de_la=700] [pana_la=7DF]  (hex)\r\n"); return; }
    g_dtc_running = true;
    char prev[8]; strncpy(prev, can_mode_name(), sizeof(prev) - 1); prev[sizeof(prev) - 1] = 0;
    if (strcmp(prev, "normal") != 0) can_set_mode("normal");
    osch_set_enabled(false);
    replyf(owner, "UDSSCAN 0x%03lX..0x%03lX (22 F187) ...\r\n", (unsigned long)a, (unsigned long)b);
    uint32_t t0 = millis(); int found = 0; String items;
    for (uint32_t id = a; id <= b; id++) {
        uint32_t rid = uds_resp_id(id);
        if (rid > 0x7FF) continue;
        iso_tp_allow(rid, id);
        uint8_t q[3] = {0x22, 0xF1, 0x87};
        bool got = await_resp(id, q, 3, 0x62, 80) && g_await_id == rid;
        if (got) {
            found++;
            String pn; if (g_abuf[0] == 0x62) for (size_t i = 3; i < g_alen; i++) { char c = g_abuf[i]; pn += (c >= 32 && c < 127) ? c : '.'; }
            else pn = "NRC 0x" + String(g_alen >= 3 ? g_abuf[2] : 0, HEX);
            pn.trim();
            replyf(owner, "  0x%03lX -> 0x%03lX  %s\r\n", (unsigned long)id, (unsigned long)rid, pn.c_str());
            char e[48]; jl_escape(e, sizeof(e), pn.c_str()); char it[96];
            snprintf(it, sizeof(it), "%s{\"req\":\"%03lX\",\"rsp\":\"%03lX\",\"pn\":\"%s\"}", items.length() ? "," : "",
                     (unsigned long)id, (unsigned long)rid, e);
            items += it;
        }
        iso_tp_forget(rid);
        proto_pump(2); proto_keepalive();
    }
    if (strcmp(prev, "normal") != 0) can_set_mode(prev);
    osch_set_enabled(strcmp(can_mode_name(), "listen") != 0);
    g_dtc_running = false;
    replyf(owner, "=== gata: %d module UDS, %lu ms ===\r\n", found, (unsigned long)(millis() - t0));
    if (!name[0]) return;
    if (WiFi.status() != WL_CONNECTED) { reply(owner, "  upload: WiFi offline (datele sunt doar pe consola)\r\n"); return; }
    String body = String("{\"type\":\"can_log\",\"label\":\"") + name + "\",\"chunk\":0,\"chunks\":1,\"ts\":" + (unsigned long)millis() +
                  ",\"kind\":\"udsscan\",\"fw\":\"" FW_VERSION "\",\"n\":" + found + ",\"data\":\"\",\"frames\":[],\"list\":[" + items + "]}";
    String err;
    if (sniff_post_json(sniff_cloud_url("can_log"), body, err)) replyf(owner, "  upload OK (label '%s', %d module)\r\n", name, found);
    else replyf(owner, "  upload ESUAT: %s\r\n", err.c_str());
}

// v2.5.9: DIDSCAN <reqid> <did_de_la> <did_pana_la> [nume_byos] — citeste `22 <did>` pe fiecare DID din interval (DOAR SID 22).
// NRC 31 (DID inexistent) vine repede; fara raspuns = 200 ms. Afiseaza doar raspunsurile pozitive (62). Max 4096 DID-uri pe rulare.
// Scop initial: gasirea unei temperaturi de ulei MASURATE pe ECU motor, langa 10E6 (modelata).
static void cmd_didscan(uint8_t owner, char *tok[], int nt) {
    if (nt < 4) { reply(owner, "DIDSCAN <reqid_hex> <did_de_la_hex> <did_pana_la_hex> [nume_byos]   ex: DIDSCAN 7E0 1000 10FF ulei\r\n"); return; }
    if (sniff_active()) { reply(owner, "DIDSCAN: SNIFF activ (SNIFF STOP intai)\r\n"); return; }
    if (g_dtc_running)  { reply(owner, "DIDSCAN: DTCALL/CODING/UDSSCAN ruleaza acum\r\n"); return; }
    uint32_t req = strtoul(tok[1], nullptr, 16), a = strtoul(tok[2], nullptr, 16), b = strtoul(tok[3], nullptr, 16);
    if (req < 0x100 || req > 0x7FF || a > 0xFFFF || b > 0xFFFF || a > b || b - a >= 4096) {
        reply(owner, "DIDSCAN: reqid 100..7FF, DID 0000..FFFF, interval max 4096\r\n"); return;
    }
    char name[32]; name[0] = 0;
    if (nt >= 5) { strncpy(name, tok[4], sizeof(name) - 1); name[sizeof(name) - 1] = 0; coding_sanitize(name); }
    uint32_t rid = uds_resp_id(req);
    g_dtc_running = true;
    char prev[8]; strncpy(prev, can_mode_name(), sizeof(prev) - 1); prev[sizeof(prev) - 1] = 0;
    if (strcmp(prev, "normal") != 0) can_set_mode("normal");
    osch_set_enabled(false);
    iso_tp_allow(rid, req);
    replyf(owner, "DIDSCAN 0x%03lX -> 0x%03lX, DID %04lX..%04lX (22, doar citire) ...\r\n", (unsigned long)req, (unsigned long)rid, (unsigned long)a, (unsigned long)b);
    uint32_t t0 = millis(); int found = 0, nrc = 0, silent = 0; String items;
    for (uint32_t did = a; did <= b; did++) {
        uint8_t q[3] = {0x22, (uint8_t)(did >> 8), (uint8_t)did};
        bool got = await_resp(req, q, 3, 0x62, 200) && g_await_id == rid;
        if (!got) { silent++; }
        else if (g_abuf[0] == 0x7F) { nrc++; }
        else if (g_alen >= 3 && g_abuf[0] == 0x62 && g_abuf[1] == q[1] && g_abuf[2] == q[2]) {
            found++;
            size_t n = g_alen - 3; const uint8_t *d = g_abuf + 3;
            char hx[3 * 24 + 4]; size_t hl = 0;
            for (size_t i = 0; i < n && i < 24; i++) hl += snprintf(hx + hl, sizeof(hx) - hl, "%02X%s", d[i], i + 1 < n ? " " : "");
            if (n > 24) snprintf(hx + hl, sizeof(hx) - hl, " ..");
            if (n == 1) replyf(owner, "  %04lX  %-8s  (u8 %u, u8-40 %d, u8-60 %d)\r\n", (unsigned long)did, hx, d[0], d[0] - 40, d[0] - 60);
            else if (n == 2) replyf(owner, "  %04lX  %-8s  (u16 %u, /10 %.1f, /16 %.2f)\r\n", (unsigned long)did, hx, (d[0] << 8) | d[1], ((d[0] << 8) | d[1]) / 10.0, ((d[0] << 8) | d[1]) / 16.0);
            else replyf(owner, "  %04lX  %s\r\n", (unsigned long)did, hx);
            if (items.length() < 16000) {
                char it[3 * 24 + 48];
                char hj[2 * 24 + 1]; size_t jl = 0;
                for (size_t i = 0; i < n && i < 24; i++) jl += snprintf(hj + jl, sizeof(hj) - jl, "%02X", d[i]);
                snprintf(it, sizeof(it), "%s{\"did\":\"%04lX\",\"n\":%u,\"d\":\"%s\"}", items.length() ? "," : "", (unsigned long)did, (unsigned)n, hj);
                items += it;
            }
        }
        proto_pump(1); proto_keepalive();
    }
    iso_tp_forget(rid);
    if (strcmp(prev, "normal") != 0) can_set_mode(prev);
    osch_set_enabled(strcmp(can_mode_name(), "listen") != 0);
    g_dtc_running = false;
    replyf(owner, "=== gata: %d DID-uri cu date, %d refuzate (NRC), %d fara raspuns, %lu ms ===\r\n", found, nrc, silent, (unsigned long)(millis() - t0));
    if (!name[0]) return;
    if (WiFi.status() != WL_CONNECTED) { reply(owner, "  upload: WiFi offline (datele sunt doar pe consola)\r\n"); return; }
    char hdr[160];
    snprintf(hdr, sizeof(hdr), ",\"kind\":\"didscan\",\"req\":\"%03lX\",\"from\":\"%04lX\",\"to\":\"%04lX\",\"fw\":\"" FW_VERSION "\",\"n\":%d",
             (unsigned long)req, (unsigned long)a, (unsigned long)b, found);
    String body = String("{\"type\":\"can_log\",\"label\":\"") + name + "\",\"chunk\":0,\"chunks\":1,\"ts\":" + (unsigned long)millis() +
                  hdr + ",\"data\":\"\",\"frames\":[],\"list\":[" + items + "]}";
    String err;
    if (sniff_post_json(sniff_cloud_url("can_log"), body, err)) replyf(owner, "  upload OK (label '%s', %d DID-uri)\r\n", name, found);
    else replyf(owner, "  upload ESUAT: %s\r\n", err.c_str());
}

// ==================== v2.5.9: BYOSDEBUG <nume> | BYOSDEBUG STOP | BYOSDEBUG ====================
// Din momentul pornirii, fiecare linie de comanda (text) si tot raspunsul ei text se aduna si se urca la BYOS
// (type=can_log, kind=debug, acelasi label, chunk crescator) de catre task-ul de retea — consola si CAN nu asteapta.
void proto_console_line(uint8_t owner, const char *raw, bool /*pin_ok*/) {
    while (*raw == ' ') raw++;
    if (!strncasecmp(raw, "BYOSDEBUG", 9) && (raw[9] == 0 || raw[9] == ' ')) {
        const char *arg = raw + 9; while (*arg == ' ') arg++;
        if (!*arg) {
            replyf(owner, "BYOSDEBUG %s  label=%s  bucati trimise=%lu  octeti=%lu  pierduti=%lu\r\n", g_dbg_on ? "PORNIT" : "oprit",
                   g_dbg_label[0] ? g_dbg_label : "-", (unsigned long)g_dbg_chunk, (unsigned long)g_dbg_sent, (unsigned long)g_dbg_lost);
            return;
        }
        if (!strcasecmp(arg, "STOP")) {
            if (!g_dbg_on) { reply(owner, "BYOSDEBUG nu era pornit\r\n"); return; }
            g_dbg_cur += "> BYOSDEBUG STOP\n";
            dbg_move_ready();
            g_dbg_on = false; g_dbg_stop_pending = true;
            replyf(owner, "BYOSDEBUG oprit; ultimul text pleaca in cateva secunde (label '%s')\r\n", g_dbg_label);
            return;
        }
        if (!g_dbg_mx) g_dbg_mx = xSemaphoreCreateMutex();
        if (!g_dbg_mx) { reply(owner, "BYOSDEBUG: nu pot crea mutex\r\n"); return; }
        strncpy(g_dbg_label, arg, sizeof(g_dbg_label) - 1); g_dbg_label[sizeof(g_dbg_label) - 1] = 0;
        coding_sanitize(g_dbg_label);
        g_dbg_chunk = 0; g_dbg_sent = 0; g_dbg_lost = 0; g_dbg_cur = ""; g_dbg_stop_pending = false;
        xSemaphoreTake(g_dbg_mx, portMAX_DELAY); g_dbg_ready = ""; xSemaphoreGive(g_dbg_mx);
        g_dbg_on = true;
        g_dbg_cur = String("# BYOSDEBUG ") + g_dbg_label + " fw " FW_VERSION " uptime " + (millis() / 1000) + " s\n";
        replyf(owner, "BYOSDEBUG pornit: fiecare comanda + rezultat urca la BYOS cu label '%s' (BYOSDEBUG STOP = oprire)\r\n", g_dbg_label);
        return;
    }
    bool log = g_dbg_on && *raw && *raw != '{';
    if (log) { g_dbg_cur += "> "; g_dbg_cur += raw; g_dbg_cur += "\n"; }
    g_dbg_console = true;
    console_line_impl(owner, raw);
    g_dbg_console = false;
    if (log) dbg_move_ready();
}

// v2.5.10: jurnal AUTOMAT de evenimente — nu depinde de BYOSDEBUG. Scris din loop (core 1), urcat de task-ul de retea.
static String   g_ev_ready;
static uint32_t g_ev_chunk = 0, g_ev_next = 0, g_ev_win = 0, g_ev_n = 0, g_ev_boot = 0;
static SemaphoreHandle_t g_ev_mx = nullptr;
#define EV_MAX_BYTES   12288
#define EV_PER_MIN     30
void proto_event(const char *fmt, ...) {
    if (!g_ev_mx) { g_ev_mx = xSemaphoreCreateMutex(); if (!g_ev_mx) return; }
    uint32_t now = millis();
    if (now - g_ev_win >= 60000) { g_ev_win = now; g_ev_n = 0; }
    if (++g_ev_n > EV_PER_MIN) { if (g_ev_n == EV_PER_MIN + 1) fmt = "... prea multe evenimente, pauza pana la minutul urmator"; else return; }
    char m[200]; va_list ap; va_start(ap, fmt); vsnprintf(m, sizeof(m), fmt, ap); va_end(ap);
    char line[240]; snprintf(line, sizeof(line), "[%lu s] %s\n", (unsigned long)(now / 1000), m);
    Serial.print(line);
    if (g_dbg_on) { g_dbg_cur += "! "; g_dbg_cur += line; }
    xSemaphoreTake(g_ev_mx, portMAX_DELAY);
    if (g_ev_ready.length() + strlen(line) > EV_MAX_BYTES) g_ev_ready = "[... evenimente pierdute: server indisponibil ...]\n";
    g_ev_ready += line;
    xSemaphoreGive(g_ev_mx);
}
static String json_esc(const String &txt) {
    String esc; esc.reserve(txt.length() + 64);
    for (size_t i = 0; i < txt.length(); i++) {
        char c = txt[i];
        if (c == '"' || c == '\\') { esc += '\\'; esc += c; }
        else if (c == '\n') esc += "\\n";
        else if (c == '\r') continue;
        else if ((uint8_t)c < 0x20) esc += ' ';
        else esc += c;
    }
    return esc;
}
static void ev_service(uint32_t now) {
    if (!g_ev_mx || (int32_t)(now - g_ev_next) < 0) return;
    xSemaphoreTake(g_ev_mx, portMAX_DELAY);
    String txt = g_ev_ready; g_ev_ready = "";
    xSemaphoreGive(g_ev_mx);
    if (!txt.length()) return;
    if (!g_ev_boot) g_ev_boot = esp_random() | 1;
    bool ok = false; String err;
    if (WiFi.status() == WL_CONNECTED) {
        String mac = WiFi.macAddress(); mac.replace(":", "");
        String body = String("{\"type\":\"can_log\",\"label\":\"ev_") + mac + "\",\"kind\":\"event\",\"boot\":" + g_ev_boot +
                      ",\"chunk\":" + g_ev_chunk + ",\"ts\":" + (unsigned long)millis() + ",\"fw\":\"" FW_VERSION "\",\"frames\":[],\"text\":\"" + json_esc(txt) + "\"}";
        ok = sniff_post_json(sniff_cloud_url("can_log"), body, err);
    }
    if (ok) { g_ev_chunk++; g_ev_next = now + 5000; }   // cel mult o trimitere la 5 s: evenimentele se aduna
    else { xSemaphoreTake(g_ev_mx, portMAX_DELAY); g_ev_ready = txt + g_ev_ready; xSemaphoreGive(g_ev_mx); g_ev_next = now + 15000; }
}

// apelat de task-ul de retea (core 0): trimite ce s-a adunat. La eroare textul ramane si se reincearca.
static uint32_t g_dbg_next = 0;
void proto_debug_service(uint32_t now) {
    ev_service(now);
    if (!g_dbg_mx || (int32_t)(now - g_dbg_next) < 0) return;
    xSemaphoreTake(g_dbg_mx, portMAX_DELAY);
    String txt = g_dbg_ready; g_dbg_ready = "";
    xSemaphoreGive(g_dbg_mx);
    if (!txt.length()) { g_dbg_stop_pending = false; return; }
    if (WiFi.status() != WL_CONNECTED) { xSemaphoreTake(g_dbg_mx, portMAX_DELAY); g_dbg_ready = txt + g_dbg_ready; xSemaphoreGive(g_dbg_mx); g_dbg_next = now + 5000; return; }
    String esc; esc.reserve(txt.length() + 64);
    for (size_t i = 0; i < txt.length(); i++) {
        char c = txt[i];
        if (c == '"' || c == '\\') { esc += '\\'; esc += c; }
        else if (c == '\n') esc += "\\n";
        else if (c == '\r') continue;
        else if ((uint8_t)c < 0x20) esc += ' ';
        else esc += c;
    }
    String body = String("{\"type\":\"can_log\",\"label\":\"") + g_dbg_label + "\",\"kind\":\"debug\",\"chunk\":" + g_dbg_chunk +
                  ",\"ts\":" + (unsigned long)millis() + ",\"fw\":\"" FW_VERSION "\",\"frames\":[],\"text\":\"" + esc + "\"}";
    String err;
    if (sniff_post_json(sniff_cloud_url("can_log"), body, err)) { g_dbg_chunk++; g_dbg_sent += txt.length(); }
    else { xSemaphoreTake(g_dbg_mx, portMAX_DELAY); g_dbg_ready = txt + g_dbg_ready; xSemaphoreGive(g_dbg_mx); g_dbg_next = now + 5000; }
}
