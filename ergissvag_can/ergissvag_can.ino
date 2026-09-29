/**
 * ergissvag_can.ino — ergCAN ESP32: OBD/UDS master pe mufa OBD + senzori + abonamente + consola + OTA + cloud
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 2.0.0
 * @changes
 *   v2.5.13 2026-09-24 — calibrare magnetometru (sensors.cpp v1.4): heading corectat hard-iron/soft-iron + montaj, colectare automata
 *     fara calibrare, comanda MAGCAL; snapshot cloud primeste "mag" (brut x/y/z, h, cal, col).
 *   v2.5.10 2026-09-24 — CAN SE REPARA SINGUR: schimbarea modului asteapta sfarsitul recuperarii bus-off (altfel twai_stop/uninstall
 *     esuau, driverul ramanea agatat si orice reinstalare dadea DRIVER_FAIL pana la reboot — incident 24.09 dimineata);
 *     instalarea reincearca dupa uninstall la ESP_ERR_INVALID_STATE; driver cazut => reincercare automata la 5 s; bus-off care
 *     nu se recupereaza in 5 s => reinstalare. Codul exact de eroare in STAT (can_err) si in evenimente.
 *     JURNAL AUTOMAT (proto_event, BYOS kind=event, label ev_<MAC>): boot + motiv reset, driver CAN, bus-off/recuperare,
 *     „motorul nu mai raspunde desi bus-ul e viu", client F16 conectat/deconectat, scanare DTC automata.
 *   v2.0.0 2026-09-15 — REARHITECTURA (vezi memory/ERGCAN_2_PLAN.md):
 *     - Setari in NVS (cfg.*): WiFi, cloud, autoconfig, OTA, consola, diagnoza — editabile din F16/consola/server
 *     - OTA prin WiFi cu revenire automata (partitions.csv: 2 zone app) — ULTIMUL flash pe USB
 *     - Planificator abonamente OBD (obd_sched): F16 trimite lista o data (sub), ESP citeste si posteaza; multi-PID pe 7E0
 *     - Multi-client TCP (31416) + consola telnet (23) + serial, stil AT: HELP/STAT/VIN/KM/SELFTEST/PID/SUB/CFG/OTA/SNIFF/EXPLORE
 *     - SNIFF: captura pasiva CAN in LittleFS + upload (BYOS can_log) — ex. citire EEPROM BCM cu VAG CAN PRO
 *     - iso_tp v1.3: raspunsuri doar de la ID-uri cunoscute; nu mai confunda TP 2.0
 *     - OBD legacy (obd1/raw/stat) pastrat pt scripturile de test (obd_speed.py, vcds_sniff.py)
 *   v2.2.2 2026-09-16 — RECONECTARI (diagnoza pe datele din cloud: 5-8 reporniri/zi, sesiuni F16 rupte):
 *     - serverele TCP/telnet/mDNS/UDP NU se mai distrug la reconectarea WiFi daca IP-ul local a ramas
 *       acelasi (placa are IP fix prin DHCP) -> o clipire a AP-ului nu mai inchide sesiunea F16/telnet
 *       si nu mai opreste anuntul de discovery; rebind se face doar cand IP-ul chiar s-a schimbat;
 *     - AP fallback abia dupa WIFI_AP_FALLBACK_S (120 s) de STA pierdut, nu dupa 20 s — de aici veneau
 *       repornirile ("STA up dupa AP fallback -> restart pt bind curat"); la prima pornire pragul ramane scurt;
 *     - diagnostic in snapshot-ul cloud: `ip`, `wdrops` (cate pierderi de STA) si `rr` (motivul resetului).
 *   v2.2.3 2026-09-16 — COMBUSTIBIL: bordul da litrii pe UDS 22 2206 (si 2212, aceeasi valoare) ca u8.
 *     Confirmat pe masina: ambele = 20 cand rezervorul avea ~21 L si 16 cand avea 16 L, deci ipoteza
 *     „unul e temperatura" a picat. Adaugat `read_fuel()` (2206, rezerva 2212), comanda consolei `FUEL`,
 *     linia „Litri" din SELFTEST, citire in fundal la 30 s difuzata la F16 ca {"t":"fuel","v":L} si
 *     campul `fuel` in snapshot-ul cloud (langa vin/km).
 *     SELFTEST: linia „Litri" da acum valoarea reala, iar „Module" chiar sondeaza modulele
 *     (UDS 0x7xx + canale TP2.0, ca MODPROBE) in loc de textul „disponibil in 2.1".
 *   v2.2.4 2026-09-16 — „VALORI INGHETATE + REFRESH LENT" (raport user, dovedit cu datele din cloud + GPS:
 *     masina MERGEA 11:50-12:05 cu 15-60 km/h, dar cache-ul OBD era gol si can=0, iar WiFi-ul era intact —
 *     rssi -42..-51, wdrops=0, uptime continuu). Lantul: socketul F16 moare TACUT pe hotspot (fara FIN) ->
 *     placa il declara plecat si sterge abonamentele lui -> nimeni nu mai interogheaza -> nu mai vin cadre ->
 *     `bus_alive` fals -> setul intern (singurul care ar fi repornit interogarile) ramane blocat. Iesirea se
 *     facea doar intamplator, cand nimerea un `obd_probe`. Acum:
 *     - `ensure_internal_subs()` NU mai cere `bus_alive` (conditie auto-blocanta pe linia de diagnoza);
 *     - keepalive TCP pe clientii JSON (idle 20 s, 4 sonde la 5 s) -> socketul mort e prins in ~40 s;
 *     - snapshot cloud cu `nc`, `subs`, `req`, `resp`, `tmo`, `fage`, `eage` -> data viitoare se vede din server
 *       daca nu se interogheaza si de ce, fara sa fie nevoie de consola in masina.
 *   v2.2.5 2026-09-16 — trei defecte gasite in propriile date de dupa 2.2.4 (nu raportate de user):
 *     - `fage`/`eage` ieseau 4294967 s: snapshot-ul se face pe task-ul de retea, iar loop()-ul (celalalt nucleu)
 *       actualizeaza lastFrameMs intre timp => scaderea pe unsigned facea underflow. Acum diferente cu semn, taiate la 0.
 *     - obiectul `obd` disparea din snapshot exact cand F16 era conectat: acelasi underflow in testul de prospetime
 *       al cache-ului (`now - ts > 30000`) => PID-ul proaspat parea vechi de 49 de zile si era sarit TACUT.
 *     - `fuel` sarea 14 -> 6 -> 21 L: `await_resp` potriveste doar dupa SID (0x62), deci cu busul plin putea prinde
 *       raspunsul altei cereri. Acum se verifica DID-ul ecou (62 <hi> <lo>) si se publica `fdid` (de pe ce DID vin litrii).
 *   v2.2.6 2026-09-16 — „F16 conectat, 0 abonamente" (consola din masina, `stat` clients=1 + `sub` 0 abonamente):
 *     F16 cazut pe ELM tine socketul deschis dar NU retrimite `sub` (il trimite o data per sesiune), iar setul
 *     intern era gardat pe „niciun client conectat" => nimeni nu mai interoga OBD-ul. Acum decizia se ia dupa
 *     ABONAMENTELE ACTIVE ale altor owneri, nu dupa numarul de clienti.
 *     In plus: multi-PID (6 PID-uri/cerere) nu mai ramane oprit definitiv dupa cateva esecuri — se reincearca
 *     periodic; altfel se interogheaza cate un PID pe rand, exact „refresh lent" (`stat` arata multi=0).
 *     `multi` e publicat acum si in snapshot-ul cloud.
 *   v2.2.7 2026-09-16 — „unele PID lipsesc" la relanti (poza user, dupa 2.2.6): F16 arata ◈ doar pe RPM/MAP/LOAD,
 *     iar THR/PED/CHT/IAT/CAT/STFT/LTFT/EQUIV ramaneau „--". Cauza NU e busul (req 251 / resp 245 / tmo 6, multi=1):
 *     F16 tot nu se reaboneaza, deci asculta doar difuzarea SETULUI INTERN, iar el stocheaza push-urile dupa NUMELE
 *     cheii (`raw[k]=v`) — numele interne erau prescurtate (spd/clt/thr/iat/tadv/run/volt) si nu se potriveau.
 *     Acum cheile interne sunt identice cu cele din SUB_LIST-ul F16 + 5 PID-uri noi in set (pedal 0x49, catTemp 0x3C,
 *     stft 0x06, ltft 0x07, equivRatio 0x44). Cheile din cloud raman cele din obd_key_for_pid (schema neschimbata).
 *   v2.2.8 2026-09-16 — (cerinta user) setul intern = TOATE PID-urile cerute de F16, cu aceleasi chei si frecvente
 *     (24) + `maf`/`fuel` pentru cloud = 26 itemi (din 48). Dupa 2.2.7 se vedea ca F16 tot nu se aboneaza:
 *     `subs` ramanea 12 (doar setul intern) chiar si dupa repornirea aplicatiei, cu `nc`=2 clienti conectati.
 *     Cauza in app: abonamentul pleaca doar din `onHello`, iar `hello` se trimite o singura data, la acceptarea
 *     conexiunii TCP — socket agatat sau transport != „tcp" => nicio reabonare. Cu setul complet pe placa,
 *     F16 primeste tot prin difuzare, abonat sau nu.
 *   v2.2.9 2026-09-16 — TABELA PLINA + contoare care lipseau. Pe masina (contact pus, motor oprit) `subs` a ajuns
 *     la 48 = plafonul vechi, iar req/resp/tmo pareau inghetate. Explicatia, din aritmetica: 26 (setul intern) +
 *     pana la 24 de ONESHOT-uri de la F16 in fallback polling (`obd1` la 100 ms). Cu TWAI incapabil sa transmita
 *     (fara ACK pe bus), cererea nu pleaca, deci oneshot-ul nu expira NICIODATA si se aduna. Iar contorul care
 *     chiar crestea (`tx_refused`) nu era publicat, asa ca din cloud parea blocaj.
 *     - `OSCH_MAX_ITEMS` 48 -> 64 (26 interne + 24 abonate + oneshoturi incap acum);
 *     - oneshot-urile expira singure dupa ONESHOT_TTL_MS (3 s), indiferent daca cererea a plecat sau nu;
 *     - `unsupported` nu mai e definitiv: se reincearca la 5 min (UNSUP_RETRY_MS);
 *     - snapshot cloud: `nrc`, `p78`, `txref`, `txfail`, `unsup` — ca sa nu mai ghicesc data viitoare.
 *   NOTA (intrebare user): itemii interni si cei ai F16 pot avea acelasi PID, dar NU se cere de doua ori pe bus —
 *     duplicatele sunt sarite in aceeasi cerere, iar raspunsul alimenteaza TOTI itemii cu acel PID si le avanseaza
 *     `next_due`, deci raman in faza. Dublura costa doar sloturi in tabela (de aici plafonul marit).
 *   v2.3.0 2026-09-16 — INDEPENDENT DE MASINA (cerinta user): ce era specific Alteei sta acum intr-un PROFIL
 *     (veh.h/veh.cpp), nu in cod: bitrate, ID-ul bordului, regula de raspuns UDS (VAG +0x6A), DID-ul de km cu
 *     formatul lui (u16x10/u24/u16) si DID-urile de litri. PID-urile mode 01 raman standard OBD-II, deci merg
 *     pe orice masina fara profil.
 *     - profil COMPILAT = exact valorile de pana acum (PQ35) => fara /veh.json, comportament identic;
 *     - `SET mode <model>` (ex. `SET mode pq35`) sau `VEH GET <model>` aduce profilul de pe server si il
 *       salveaza in LittleFS (/veh.json); `VEH` = stare, `VEH LIST` = ce modele exista, `VEH CLEAR` = revenire;
 *     - catalogul e pe server (cell/ergcan/veh/<model>.json + veh.php) — masina noua NU cere firmware nou;
 *     - profilul e validat INAINTE de scrierea fisierului (o pagina de eroare nu ajunge pe placa), iar la
 *       schimbarea bitrate-ului driverul TWAI se reinstaleaza pe loc;
 *     - snapshot cloud: camp nou `veh` (ce profil ruleaza placa).
 *     ⚠ Flash: 97% din partitia app (~32 KB liberi). Urmatoarea functie mare cere repartitionare (NU prin OTA).
 *   v2.3.2 2026-09-17 — „SE BLOCHEAZA DUPA UN TIMP, TREBUIE RESETAT" (raport user; dovedit din cloud: 17.09 06:52-06:56
 *     masina in MERS, req aproape inghetat, fage/eage 17 s, can=0, apoi reset de alimentare la 06:57).
 *     BLOCARE CIRCULARA introdusa de 2.3.1: cu `can`=0 planificatorul nu mai emite cereri (idle_gate) si astepta
 *     revenirea PASIVA prin cadre de pe bus — dar pe mufa OBD nu circula nimic nesolicitat (aceeasi greseala ca
 *     `bus_alive` din 2.2.4), iar probe-ul de rezerva rula doar FARA abonamente, deci niciodata (setul intern = 26).
 *     Orice gol de raspunsuri > 15 s (ECU ocupat, rafala de timeout-uri) lasa placa muta pana la reset.
 *     - probe-ul `7DF 01 00` (single-shot) ruleaza acum si CU abonamente, cat timp magistrala e declarata moarta:
 *       la 5 s, sau la 30 s dupa TX esuat (contact luat) => revenire in <= 30 s, fara reset;
 *     - snapshot cloud: `prb` = cate probe-uri s-au trimis (creste in mers = s-a folosit reinvierea).
 *   v2.3.3 2026-09-17 — VALORI DE PE ECU MOTOR (UDS 22 pe 0x7E0), gasite cu SNIFF pe VCDS in aceeasi zi:
 *     generator load `2050` / performance `20A8` (u8 x100/256 %), temperatura ulei `10E6` (u8-60 °C), nivel ulei
 *     `2006` (u16/16 mm; 0x0780 = plafonul senzorului cu motorul pornit => ignorat). DID-urile stau in profilul
 *     vehiculului (veh.h v1.1). Citire in fundal (gen 3 s, temp 10 s, nivel 30 s; timeout 300 ms, backoff 60 s dupa
 *     3 esecuri), difuzare la F16 {"t":"genload|genperf|oilt|oill","v":x.x}, campuri cu acelasi nume in snapshot-ul
 *     cloud (doar cat sunt proaspete), comanda `ENG` pe consola (arata raw + NRC — pentru diagnoza pe masina).
 *   v2.6.0 2026-09-29 — PARAMETRI CU NUME + PROFIL GENERIC (cerinta user: „F16 sa ceara dupa denumiri; profil independent
 *     de masina, doar sa fie CAN").
 *     - `sub` pe NUME ({"n":"rpm","hz":5}); placa rezolva numele: profil -> valori de vehicul -> catalog standard OBD-II
 *       (params.cpp, 45 de nume). Raspunsul spune sursa fiecaruia. Forma veche cu PID ramane valabila (F16 <= 1.83).
 *     - toate valorile pleaca in aceeasi forma {"t":"d","k":nume,"v":..,"src":..}; `hello` anunta "proto":3 si profilul.
 *     - bitmap-ul PID-urilor suportate se citeste o data; ce nu declara ECU-ul nu se mai cere pe ghicite.
 *     - AUTODETECTIE pentru profilele cu "auto":1 (obd2): 500k/250k x 11/29 de biti, intai ASCULTARE (nu transmitem la o
 *       viteza la care se aud doar erori), apoi proba `01 00` single-shot; ECU-ul care raspunde devine tinta planificatorului.
 *       Ultima combinatie reusita se tine minte (veh.abr/veh.aext) si se incearca prima. NETESTAT pe masina cu 29 de biti.
 *     - snapshot cloud: `unsl` (cheile nesuportate), `nsup` (cate PID-uri declara ECU-ul), `abr`/`aext` (ce s-a detectat).
 *   Istoric 1.x: vezi git.
 *
 * Hardware: ESP32 DevKit v1 + SN65HVD230 (GPIO17->D, GPIO16<-R, RS=GND, R120 scos), CAN_H/L pe OBD pin 6/14, GND pin 5.
 *   + BMP280 (0x76) + GY-85 (ADXL345 0x53 / ITG3200 0x68 / HMC5883L 0x1E) pe I2C 21/22.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include "ca_isrg.h"
#include "driver/twai.h"
#include "esp_system.h"
#include <lwip/sockets.h>   // v2.2.4: SOL_SOCKET/SO_KEEPALIVE + TCP_KEEPIDLE/INTVL/CNT pt keepalive pe clientii JSON
#include "config.h"
#include "sensors.h"
#include "iso_tp.h"
#include "obd_cmd.h"
#include "obd_sched.h"
#include "obd_pid.h"
#include "cfg.h"
#include "ota.h"
#include "proto.h"
#include "sniff.h"
#include "cloud.h"
#include "veh.h"
#include "acfg.h"
#include "jsonlite.h"
#include "params.h"
#include <lwip/dns.h>

// ===== owner-i (endpoint-uri) =====
// 0 = serial; 1..MAX_JSON_CLIENTS = TCP JSON; MAX_JSON_CLIENTS+1.. = telnet
#define OWNER_SERIAL   0
#define OWNER_TCP0     1
#define OWNER_TELNET0  (1 + MAX_JSON_CLIENTS)
#define OWNER_MAX      (OWNER_TELNET0 + MAX_TELNET_CLIENTS)
#define OWNER_INTERNAL 250            // v2.2: abonamente interne (poluate mereu pt cloud); emit_to_owner le ignora

static WiFiServer tcpServer(TCP_PORT);
static WiFiServer telnetServer(TELNET_PORT);
static WiFiClient tcpClients[MAX_JSON_CLIENTS];
static WiFiClient telnetClients[MAX_TELNET_CLIENTS];
static char       telnetBuf[MAX_TELNET_CLIENTS][160];
static size_t     telnetLen[MAX_TELNET_CLIENTS] = {0};
static uint8_t    telnetIac[MAX_TELNET_CLIENTS] = {0};
static bool       telnetSkipLf[MAX_TELNET_CLIENTS] = {false};
static char       tcpBuf[MAX_JSON_CLIENTS][JSON_LINE_MAX];
static size_t     tcpLen[MAX_JSON_CLIENTS] = {0};
static bool       tcpOvf[MAX_JSON_CLIENTS] = {false};     // v2.6.0: linia curenta a depasit bufferul => o aruncam pana la \n
static char       serBuf[JSON_LINE_MAX]; static size_t serLen = 0; static bool serOvf = false;

static WiFiUDP  udpDisc;
static uint32_t lastUdpBcMs = 0, lastHbMs = 0, lastStatsMs = 0, lastSensorMs = 0;
static uint32_t rxTotal = 0, rxErr = 0, lastFrameMs = 0, lastObdRespMs = 0, ledOffAt = 0, bootMs = 0;
static bool     g_can_ok = false;
// v2.5.10: diagnostic + auto-reparare driver TWAI
static esp_err_t g_can_err = ESP_OK;
static char     g_can_err_at[12] = "";
static char     g_can_want[8] = "normal";      // modul cerut ultima data (la reparare revenim la el)
static uint32_t g_can_heal_at = 0, g_can_heals = 0, g_busoff_ms = 0;
static char     g_can_mode[8] = "normal";
volatile bool   g_can_bus_ok = true, g_can_tx_enabled = true;
uint32_t        g_tx_queued = 0, g_tx_refused = 0;
static uint32_t g_tx_failed = 0, lastProbeMs = 0, probeIntervalMs = CAN_PROBE_IDLE_MS, g_probe_n = 0;
static SensorData g_last_sd; static bool g_have_sd = false;

// ===== v2.2: cache valori OBD (populat de osch_value_hook indiferent de owner: intern SAU F16) =====
// Cheia = PID (0x00..0xFF) → ultima valoare decodata. Cloud + F16 folosesc acelasi cache (se "merge" automat).
struct ObdCacheE { bool has; float v; uint32_t ts; };
static ObdCacheE g_oc[256];
static bool      g_int_subs_added = false;
// nume prietenos pt PID-urile pe care le publicam in cloud (subset OBD mode 01). nullptr = nepublicat.
static const char *obd_key_for_pid(uint8_t pid) {
    switch (pid) {
        case 0x04: return "load";   case 0x05: return "clt";   case 0x0B: return "map";
        case 0x0C: return "rpm";    case 0x0D: return "spd";   case 0x0E: return "tadv";
        case 0x0F: return "iat";    case 0x10: return "maf";   case 0x11: return "thr";
        case 0x1F: return "run";    case 0x2F: return "fuel";  case 0x42: return "volt";
        case 0x46: return "amb";    case 0x5C: return "oilt";
        default:   return nullptr;
    }
}

// MON (o singura abonare live pe consola)
static int  g_mon_owner = -1; static long g_mon_id = 0;

enum WifiState : uint8_t { WIFI_STA_TRYING, WIFI_STA_OK, WIFI_AP_OK };
static WifiState g_wifi_state = WIFI_STA_TRYING;
static uint32_t  g_wifi_start_ms = 0; static bool g_net_up = false;
static bool      g_ap_fallback_used = false;   // am trecut prin AP fallback in acest ciclu de alimentare
// v2.2.2: rebind-ul serverelor se face doar cand IP-ul local s-a schimbat (placa are IP fix prin DHCP),
// ca o clipire a AP-ului sa nu mai omoare sesiunile F16/telnet. Contoarele merg in snapshot-ul cloud.
static IPAddress g_last_sta_ip;                // IP-ul pe care sunt legate serverele acum
static uint32_t  g_sta_down_since_ms = 0;      // de cand e STA pierdut (0 = e sus)
static bool      g_had_sta = false;            // am fost conectati macar o data in acest ciclu de alimentare
static uint16_t  g_wifi_drops = 0;             // de cate ori s-a pierdut STA de la pornire
// v2.1.3: supravietuieste ESP.restart() (soft reset), se sterge la power-on. Marcheaza ca am repornit
// deja o data pt. bind curat -> nu intram in boot-loop daca HU tot nu e gata dupa restart.
RTC_DATA_ATTR static uint32_t g_reboot_rebind_magic = 0;
#define REBIND_REBOOT_MAGIC 0xE16CA2A3UL

// ===== declaratii =====
static const char *wifiStateName();
static String wifiCurrentIp();
static void emit_to_owner(uint8_t owner, const char *line, size_t len);
static void broadcast_json(const char *line, size_t len);

// ===== LED =====
static void ledPulse() { digitalWrite(LED_PIN, HIGH); ledOffAt = millis() + LED_BLINK_MS; }
static void ledService() { if (ledOffAt && (int32_t)(millis() - ledOffAt) >= 0) { digitalWrite(LED_PIN, LOW); ledOffAt = 0; } }

// ===== TWAI =====
static bool twaiTimingFor(uint32_t bps, twai_timing_config_t &out) {
    switch (bps) {
        case 500000: out = TWAI_TIMING_CONFIG_500KBITS(); return true;
        case 250000: out = TWAI_TIMING_CONFIG_250KBITS(); return true;
        case 125000: out = TWAI_TIMING_CONFIG_125KBITS(); return true;
        case 100000: out = TWAI_TIMING_CONFIG_100KBITS(); return true;
        default:     return false;
    }
}
static twai_mode_t twaiModeFor(const char *m) {
    if (!strcmp(m, "listen")) return TWAI_MODE_LISTEN_ONLY;
    if (!strcmp(m, "noack"))  return TWAI_MODE_NO_ACK;
    return TWAI_MODE_NORMAL;
}
// v2.5.10: tine minte ultima eroare a driverului TWAI si o trece in jurnalul automat
static void can_note_err(const char *where, esp_err_t e) {
    g_can_err = e; strncpy(g_can_err_at, where, sizeof(g_can_err_at) - 1); g_can_err_at[sizeof(g_can_err_at) - 1] = 0;
    proto_event("CAN %s ESUAT: %s (heap %lu)", where, esp_err_to_name(e), (unsigned long)ESP.getFreeHeap());
}
// v2.5.10: opreste si dezinstaleaza driverul SIGUR. In timpul recuperarii bus-off twai_stop/twai_driver_uninstall
// intorc INVALID_STATE; inainte le ignoram => driverul ramanea instalat, g_can_ok=false, iar orice reinstalare esua (DRIVER_FAIL).
static void can_driver_down() {
    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_RECOVERING) {
        uint32_t t0 = millis();
        while (millis() - t0 < 1500 && twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_RECOVERING) delay(5);
    }
    if (twai_get_status_info(&st) == ESP_OK && st.state == TWAI_STATE_RUNNING) twai_stop();
    esp_err_t e = twai_driver_uninstall();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) can_note_err("uninstall", e);
    else if (e == ESP_ERR_INVALID_STATE && twai_get_status_info(&st) == ESP_OK) can_note_err("uninstall", e);   // inca instalat
    g_can_ok = false;
}
static bool twaiStart(const char *modeName) {
    twai_mode_t mode = twaiModeFor(modeName);
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, mode);
    g.rx_queue_len = TWAI_RX_QUEUE_LEN;
    g.tx_queue_len = (mode == TWAI_MODE_LISTEN_ONLY) ? 0 : TWAI_TX_QUEUE_LEN_NORMAL;
    g.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL |
                       TWAI_ALERT_ERR_PASS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_RECOVERED;
    twai_timing_config_t t;
    if (!twaiTimingFor(veh_bitrate(), t)) { can_note_err("bitrate", ESP_ERR_INVALID_ARG); return false; }   // v2.3.0: bitrate din profil
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    esp_err_t e = twai_driver_install(&g, &t, &f);
    if (e == ESP_ERR_INVALID_STATE) {           // v2.5.10: driver ramas instalat (uninstall esuat anterior) => il scoatem si reincercam
        twai_stop(); twai_driver_uninstall(); e = twai_driver_install(&g, &t, &f);
    }
    if (e != ESP_OK) { can_note_err("install", e); return false; }
    e = twai_start();
    if (e != ESP_OK) { can_note_err("start", e); twai_driver_uninstall(); return false; }
    g_can_err = ESP_OK; g_can_err_at[0] = 0;
    strncpy(g_can_mode, (mode == TWAI_MODE_LISTEN_ONLY) ? "listen" : (mode == TWAI_MODE_NO_ACK ? "noack" : "normal"), sizeof(g_can_mode) - 1);
    g_can_tx_enabled = (mode != TWAI_MODE_LISTEN_ONLY);
    g_can_bus_ok = true;
    return true;
}

// ===== hooks pt proto.cpp / sniff.cpp / obd_sched =====
const char *can_mode_name() { return g_can_mode; }
bool can_set_mode(const char *mode) {
    if (strcmp(mode, "normal") && strcmp(mode, "listen") && strcmp(mode, "noack")) return false;
    strncpy(g_can_want, mode, sizeof(g_can_want) - 1); g_can_want[sizeof(g_can_want) - 1] = 0;
    if (g_can_ok) can_driver_down();
    g_can_ok = twaiStart(mode);
    g_can_heal_at = g_can_ok ? 0 : millis() + 5000;   // v2.5.10: esec => reincercare automata din loop
    g_busoff_ms = 0;
    lastProbeMs = 0; probeIntervalMs = CAN_PROBE_IDLE_MS;
    return g_can_ok;
}
bool can_send_iso(uint32_t id, const uint8_t *data, size_t len) { return iso_tp_send_sf(id, data, len, false); }
// v2.3.1: „magistrala e vie?” — aceeasi definitie cu flag-ul `can` din snapshot, intr-un singur loc.
// O foloseste si planificatorul OBD, si cloud-ul, ca sa nu lucreze degeaba cu masina oprita.
bool can_is_live() {
    uint32_t now = millis();
    return g_can_ok && g_can_bus_ok
        && ((lastObdRespMs && now - lastObdRespMs < CAN_OK_WINDOW_MS)
         || (lastFrameMs   && now - lastFrameMs   < 10000));
}
uint32_t can_last_frame_age() { return lastFrameMs ? millis() - lastFrameMs : 0xFFFFFFFF; }
uint32_t can_last_ecu_age() { return lastObdRespMs ? millis() - lastObdRespMs : 0xFFFFFFFF; }
bool net_online() { return g_wifi_state == WIFI_STA_OK && WiFi.status() == WL_CONNECTED; }
String net_ip() { return wifiCurrentIp(); }

bool can_send_raw_frame(uint32_t id, const uint8_t *data, size_t len, bool pad) {
    if (!g_can_ok || !g_can_tx_enabled || !g_can_bus_ok) return false;
    twai_message_t m; memset(&m, 0, sizeof(m));
    m.identifier = id;
    size_t dlc = len > 8 ? 8 : len;
    memcpy(m.data, data, dlc);
    if (pad) { for (size_t i = dlc; i < 8; i++) m.data[i] = 0xAA; dlc = 8; }
    m.data_length_code = dlc;
    bool ok = twai_transmit(&m, pdMS_TO_TICKS(50)) == ESP_OK;
    if (ok) g_tx_queued++; else g_tx_refused++;
    char b[96];
    int n = snprintf(b, sizeof(b), "{\"ts\":%lu,\"type\":\"tx\",\"id\":\"0x%lX\",\"len\":%u,\"ok\":%d}\n",
                     (unsigned long)millis(), (unsigned long)id, (unsigned)dlc, ok);
    if (n > 0) broadcast_json(b, n);
    return ok;
}

static twai_mode_t s_prevmode;
void can_selftest_wiring(char *verdict, size_t vsz, char *hint, size_t hsz) {
    char prev[8]; strncpy(prev, g_can_mode, sizeof(prev));
    if (!can_set_mode("noack")) { snprintf(verdict, vsz, "DRIVER_FAIL"); snprintf(hint, hsz, "twai_driver_install esuat"); return; }
    twai_message_t m; while (twai_receive(&m, 0) == ESP_OK) {}
    uint32_t a = 0; twai_read_alerts(&a, 0);
    memset(&m, 0, sizeof(m)); m.identifier = 0x7DF; m.data_length_code = 8;
    uint8_t d[8] = {0x02, 0x01, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}; memcpy(m.data, d, 8); m.flags = TWAI_MSG_FLAG_SELF;
    bool txOk = twai_transmit(&m, pdMS_TO_TICKS(200)) == ESP_OK;
    bool rxSelf = false; uint32_t t0 = millis();
    while (millis() - t0 < 300) { twai_message_t r; if (twai_receive(&r, pdMS_TO_TICKS(20)) == ESP_OK && r.identifier == 0x7DF && !memcmp(r.data, d, 8)) { rxSelf = true; break; } }
    a = 0; twai_read_alerts(&a, 0);
    bool txFailed = (a & TWAI_ALERT_TX_FAILED) != 0;
    if (txOk && rxSelf && !txFailed) { snprintf(verdict, vsz, "OK"); snprintf(hint, hsz, "TX->transceiver->RX in bucla OK"); }
    else if (!txOk || txFailed) { snprintf(verdict, vsz, "TX_FAIL"); snprintf(hint, hsz, "GPIO17->D? RS pe GND? transceiver alimentat?"); }
    else { snprintf(verdict, vsz, "NO_LOOP"); snprintf(hint, hsz, "R(CRX)->GPIO16?"); }
    can_set_mode(prev);
    (void)s_prevmode;
}

size_t can_stat_json(char *buf, size_t sz) {
    uint32_t now = millis();
    twai_status_info_t st; memset(&st, 0, sizeof(st));
    bool haveSt = g_can_ok && twai_get_status_info(&st) == ESP_OK;
    bool canOk = can_is_live();
    int nc = 0; for (int i = 0; i < MAX_JSON_CLIENTS; i++) if (tcpClients[i] && tcpClients[i].connected()) nc++;
    int n = snprintf(buf, sz,
        "{\"ts\":%lu,\"type\":\"stat\",\"fw\":\"%s\",\"uptime\":%lu,\"can\":%d,\"mode\":\"%s\",\"twai\":\"%s\",\"bitrate\":%u,"
        "\"tec\":%u,\"rec\":%u,\"rx_total\":%lu,\"rx_err\":%lu,\"tx_queued\":%lu,\"tx_failed\":%lu,\"tx_refused\":%lu,"
        "\"last_frame_age\":%ld,\"last_ecu_age\":%ld,\"wifi\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"clients\":%d,\"heap\":%lu,"
        "\"sniff\":%d,\"can_err\":\"%s%s%s\",\"can_heals\":%lu,\"osch\":%d,\"osch_busy\":%d,\"osch_to\":%lu,\"osch_req\":%lu,\"osch_resp\":%lu,"
        "\"sensors\":{\"bmp\":%d,\"accel\":%d,\"gyro\":%d,\"mag\":%d}}\n",
        (unsigned long)now, FW_VERSION, (unsigned long)((now - bootMs) / 1000UL), canOk, g_can_mode,
        g_can_ok ? (st.state == TWAI_STATE_RUNNING ? "running" : st.state == TWAI_STATE_BUS_OFF ? "bus_off" : "other") : "off",
        (unsigned)veh_bitrate(), haveSt ? st.tx_error_counter : 0, haveSt ? st.rx_error_counter : 0,
        (unsigned long)rxTotal, (unsigned long)rxErr, (unsigned long)g_tx_queued, (unsigned long)g_tx_failed, (unsigned long)g_tx_refused,
        lastFrameMs ? (long)(now - lastFrameMs) : -1L, lastObdRespMs ? (long)(now - lastObdRespMs) : -1L,
        wifiStateName(), wifiCurrentIp().c_str(), g_wifi_state == WIFI_STA_OK ? WiFi.RSSI() : 0, nc, (unsigned long)ESP.getFreeHeap(),
        sniff_is_active() ? 1 : 0, g_can_err == ESP_OK ? "" : esp_err_to_name(g_can_err), g_can_err_at[0] ? "@" : "", g_can_err_at,
        (unsigned long)g_can_heals, osch_enabled() ? 1 : 0, osch_busy() ? 1 : 0, (unsigned long)osch_stats().timeout,
        (unsigned long)osch_stats().req, (unsigned long)osch_stats().resp,
        g_have_sd && g_last_sd.bmp_ok, g_have_sd && g_last_sd.accel_ok, g_have_sd && g_last_sd.gyro_ok, g_have_sd && g_last_sd.mag_ok);
    return (n > 0 && (size_t)n < sz) ? (size_t)n : 0;
}

bool sniff_start(const char *label, const char *url, uint8_t owner) { bool ok = sniff_begin(label, url, owner); if (ok) osch_set_enabled(false); return ok; }
String sniff_stop_upload() { String r = sniff_end_and_upload(); osch_set_enabled(strcmp(g_can_mode, "listen") != 0); return r; }
bool sniff_active() { return sniff_is_active(); }
uint32_t sniff_count() { return sniff_frames(); }

// ===== emitere =====
static void emit_to_owner(uint8_t owner, const char *line, size_t len) {
    if (owner == OWNER_SERIAL) { Serial.write((const uint8_t *)line, len); return; }
    if (owner >= OWNER_TCP0 && owner < OWNER_TCP0 + MAX_JSON_CLIENTS) {
        WiFiClient &c = tcpClients[owner - OWNER_TCP0];
        if (c && c.connected()) c.write((const uint8_t *)line, len);
        return;
    }
    if (owner >= OWNER_TELNET0 && owner < OWNER_TELNET0 + MAX_TELNET_CLIENTS) {
        WiFiClient &c = telnetClients[owner - OWNER_TELNET0];
        if (c && c.connected()) c.write((const uint8_t *)line, len);
    }
}
static void broadcast_json(const char *line, size_t len) {
    if (cfg_int("ser.json")) Serial.write((const uint8_t *)line, len);
    for (int i = 0; i < MAX_JSON_CLIENTS; i++) if (tcpClients[i] && tcpClients[i].connected()) tcpClients[i].write((const uint8_t *)line, len);
}

// ===== v2.6.0: AUTODETECTIE (profil generic, "auto":1) — viteza magistralei + adresare pe 11/29 de biti =====
static bool     g_ad_done = false;        // s-a gasit combinatia pe masina asta
static bool     g_ad_was_auto = false;    // profilul curent e (a fost) unul cu autodetectie
static uint32_t g_ad_next = 0;
static uint8_t  g_ad_fail = 0;

// O combinatie: intai ASCULTAM (mod listen, nimic emis). Daca se aud doar erori => acolo circula trafic la alta viteza
// si NU transmitem. Daca e liniste sau se aud cadre bune => proba `01 00` functional, single-shot.
static bool autodet_try(uint32_t br, bool ext, uint32_t *resp_out) {
    veh_set_detected(br, ext);                       // twaiStart ia viteza din veh_bitrate()
    if (g_can_ok) can_driver_down();
    g_can_ok = twaiStart("listen");
    if (!g_can_ok) return false;
    twai_message_t m; uint32_t a = 0; int heard = 0, errs = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 120) {
        if (twai_receive(&m, pdMS_TO_TICKS(10)) == ESP_OK) heard++;
        a = 0;
        if (twai_read_alerts(&a, 0) == ESP_OK && (a & TWAI_ALERT_BUS_ERROR)) errs++;
    }
    if (errs > 2 && heard == 0) return false;        // trafic la ALTA viteza: nu emitem nimic aici

    can_driver_down();
    g_can_ok = twaiStart("normal");
    if (!g_can_ok) return false;
    while (twai_receive(&m, 0) == ESP_OK) {}
    a = 0; twai_read_alerts(&a, 0);
    uint8_t rq[2] = {0x01, 0x00};
    if (!iso_tp_send_sf(ext ? 0x18DB33F1UL : 0x7DFUL, rq, 2, true)) return false;
    uint32_t best = 0, first_ms = 0; t0 = millis();
    while (millis() - t0 < 300) {
        if (best && millis() - first_ms > 60) break;  // dupa primul raspuns mai asteptam putin si celelalte ECU
        if (twai_receive(&m, pdMS_TO_TICKS(10)) != ESP_OK) continue;
        rxTotal++;
        bool e = (m.flags & TWAI_MSG_FLAG_EXTD) != 0;
        if (e != ext || !iso_tp_is_obd_resp(m.identifier) || m.data_length_code < 3) continue;
        if (m.data[1] != 0x41 || m.data[2] != 0x00) continue;
        if (!best || m.identifier < best) best = m.identifier;   // ID-ul cel mai mic = motorul (7E8 / 18DAF110)
        if (!first_ms) first_ms = millis();
    }
    if (!best) return false;
    *resp_out = best;
    return true;
}

bool can_autodetect(String &msg) {
    if (!veh().autodet) { msg = String("profilul ") + veh().model + " are valori fixe; pentru cel generic: SET mode obd2"; return false; }
    if (sniff_is_active()) { msg = "SNIFF in curs"; return false; }
    struct Cand { uint32_t br; bool ext; };
    Cand c[4] = {{500000, false}, {500000, true}, {250000, false}, {250000, true}};   // ISO 15765-4, in ordinea raspandirii
    long sbr = cfg_int("veh.abr"); bool sext = cfg_int("veh.aext") != 0;
    for (int i = 1; i < 4 && sbr; i++)
        if ((long)c[i].br == sbr && c[i].ext == sext) { Cand t = c[i]; for (int k = i; k > 0; k--) c[k] = c[k - 1]; c[0] = t; break; }

    osch_set_enabled(false);
    for (int i = 0; i < 4; i++) {
        uint32_t resp = 0;
        if (!autodet_try(c[i].br, c[i].ext, &resp)) continue;
        uint32_t req = iso_tp_req_for(resp);
        obd_set_func_id(c[i].ext ? 0x18DB33F1UL : 0x7DFUL);
        osch_set_ids(req, resp);
        String e2;
        if ((long)c[i].br != sbr || c[i].ext != sext) {
            cfg_set("veh.abr", String((unsigned long)c[i].br).c_str(), e2);
            cfg_set("veh.aext", c[i].ext ? "1" : "0", e2);
        }
        g_ad_done = true; g_ad_fail = 0; g_ad_next = 0;
        proto_sup_restart();                          // alt ECU => alt bitmap de PID-uri
        lastFrameMs = lastObdRespMs = millis();
        strncpy(g_can_want, "normal", sizeof(g_can_want) - 1);
        osch_set_enabled(true);
        msg = String((unsigned long)c[i].br) + " bps, " + (c[i].ext ? "29" : "11") + " biti, ECU 0x" + String((unsigned long)resp, HEX) +
              " (cerere 0x" + String((unsigned long)req, HEX) + ")";
        proto_event("AUTODET: %s", msg.c_str());
        return true;
    }
    // nimic n-a raspuns: ramanem pe ultima combinatie stiuta (sau pe cea din profil), in modul cerut
    veh_set_detected(sbr > 0 ? (uint32_t)sbr : 0, sext);
    if (g_can_ok) can_driver_down();
    g_can_ok = twaiStart(g_can_want);
    if (!g_can_ok) g_can_heal_at = millis() + 5000;
    // planificatorul ramane pornit, dar nu emite nimic pana la o detectie reusita (vezi osch_send_hook)
    osch_set_enabled(g_can_ok && strcmp(g_can_mode, "listen") != 0);
    msg = "niciun ECU nu a raspuns la 01 00 pe 500k/250k, 11/29 de biti (contact pus?)";
    return false;
}

// din loop(): porneste autodetectia cand profilul o cere; la revenirea pe un profil fix reface adresarea standard
static void autodet_service() {
    if (!veh().autodet) {
        if (g_ad_was_auto) {
            g_ad_was_auto = false; g_ad_done = false; g_ad_fail = 0; g_ad_next = 0;
            veh_set_detected(0, false);
            obd_set_func_id(OBD_FUNCTIONAL_REQ_ID);
            osch_set_ids(OBD_PHYSICAL_REQ_BASE, OBD_PHYSICAL_RESP_BASE);
            proto_sup_restart();
            osch_set_enabled(g_can_ok && strcmp(g_can_mode, "listen") != 0);
            proto_event("profil fix (%s): adresare OBD standard 7DF/7E0/7E8, %lu bps", veh().model, (unsigned long)veh_bitrate());
        }
        return;
    }
    g_ad_was_auto = true;
    if (g_ad_done || sniff_is_active() || !strcmp(g_can_want, "listen")) return;
    uint32_t now = millis();
    if (g_ad_next && (int32_t)(now - g_ad_next) < 0) return;
    String msg;
    if (!can_autodetect(msg)) {
        if (g_ad_fail < 255) g_ad_fail++;
        g_ad_next = millis() + (g_ad_fail <= 3 ? 10000UL : 30000UL);   // masina oprita: nu insistam
        if (!g_ad_next) g_ad_next = 1;
    }
}

// ===== obd_sched hooks =====
static bool osch_send_hook(uint32_t id, const uint8_t *pl, size_t len, void *) {
    if (veh().autodet && !g_ad_done) return false;   // v2.6.0: profil generic inca nedetectat => nu stim cui sa cerem
    return iso_tp_send_sf(id, pl, len, false);
}
static void osch_value_hook(const OschItem &it, const uint8_t *data, size_t len, uint32_t now, void *) {
    if (!it.key[0]) return;   // oneshot legacy: raw JSON emis separat
    double v; const char *u;
    char b[96];
    int n;
    if (obd_pid_decode(it.pid, data, len, &v, &u)) {
        g_oc[it.pid].has = true; g_oc[it.pid].v = (float)v; g_oc[it.pid].ts = now;   // v2.2: cache (cloud + merge)
        n = snprintf(b, sizeof(b), "{\"t\":\"d\",\"k\":\"%s\",\"v\":%.3f,\"u\":\"%s\",\"pid\":%u,\"ts\":%lu}\n", it.key, v, u, it.pid, (unsigned long)now);
    } else {
        char hx[20]; int p = 0; for (size_t i = 0; i < len && i < 8; i++) p += snprintf(hx + p, sizeof(hx) - p, "%02X", data[i]);
        n = snprintf(b, sizeof(b), "{\"t\":\"d\",\"k\":\"%s\",\"raw\":\"%s\",\"pid\":%u,\"ts\":%lu}\n", it.key, hx, it.pid, (unsigned long)now);
    }
    // v2.2: push la TOTI clientii JSON (F16 se aboneaza o data si doar asculta). Owner intern -> doar cache+broadcast.
    if (n > 0) broadcast_json(b, n);
}
static void osch_unsup_hook(const OschItem &it, void *) {
    char b[80]; int n = snprintf(b, sizeof(b), "{\"t\":\"unsup\",\"k\":\"%s\",\"pid\":%u}\n", it.key, it.pid);
    if (n > 0) emit_to_owner(it.owner, b, n);
}

// v2.2: set intern de abonamente OBD — poluate mereu (owner intern) ca sa avem date motor pt cloud chiar fara F16.
// osch dedup dupa cheie; cheile coincid cu cele publicate (obd_key_for_pid) => F16 primeste aceleasi push-uri.
static void add_internal_subs() {
    uint32_t now = millis();
    // v2.2.7: NUMELE CHEILOR = exact cele pe care le asteapta F16 (`raw[k]=v` in onRawLine, aceleasi ca in
    // SUB_LIST din ErgCanObdReader). Inainte setul intern folosea prescurtari proprii (spd/clt/thr/iat/tadv/
    // run/volt), asa ca F16 arata ◈ doar pe cele trei care se potriveau intamplator (rpm/map/load), iar THR,
    // CHT, IAT, TIM ramaneau „--" cand F16 asculta difuzarea setului intern fara sa se fi reabonat.
    // Cheile din cloud NU depind de astea (vin din obd_key_for_pid), deci schema din up_vag_ergcan_log ramane.
    // v2.2.8 (cerinta user): setul intern contine ACUM TOT ce cere F16 — aceleasi chei si aceleasi frecvente ca
    // in SUB_LIST din ErgCanObdReader.kt — plus cele doua care exista doar pentru cloud (`maf`, `fuel`).
    // Motiv: F16 isi trimite abonamentul DOAR din `onHello` (mesaj trimis o singura data, la acceptarea
    // conexiunii TCP); daca socketul lui ramane agatat sau transportul nu e „tcp", nu se mai aboneaza niciodata,
    // iar placa nu mai interogheaza decat ce e in setul intern. Cu setul complet, F16 primeste tot prin difuzare,
    // abonat sau nu. PID-urile nesuportate de motor se marcheaza singure `unsupported` dupa cateva rateuri.
    struct { const char *k; uint8_t pid; uint32_t per; } S[] = {
        {"rpm", 0x0C, 200},         {"speed", 0x0D, 200},       {"map", 0x0B, 333},
        {"throttle", 0x11, 333},    {"load", 0x04, 333},        {"coolant", 0x05, 1000},
        {"intake", 0x0F, 2000},     {"timing", 0x0E, 1000},     {"ambient", 0x46, 5000},
        {"runtime", 0x1F, 2000},    {"battery", 0x42, 2000},    {"baro", 0x33, 5000},
        {"catTemp", 0x3C, 5000},    {"oilTemp", 0x5C, 5000},    {"stft", 0x06, 1000},
        {"ltft", 0x07, 1000},       {"absLoad", 0x43, 1000},    {"relThrottle", 0x45, 1000},
        {"pedal", 0x49, 500},       {"cmdThrottle", 0x4C, 1000},{"purge", 0x2E, 5000},
        {"fuelRail", 0x23, 2000},   {"equivRatio", 0x44, 1000}, {"o2v", 0x15, 1000},
        // doar pentru cloud (F16 nu le cere): debit aer + nivel rezervor prin PID standard
        {"maf", 0x10, 2000},        {"fuel", 0x2F, 5000},
    };
    for (size_t i = 0; i < sizeof(S) / sizeof(S[0]); i++)
        osch_add(S[i].k, S[i].pid, S[i].per, OWNER_INTERNAL, false, now);
    g_int_subs_added = true;
}
// Porneste/opreste setul intern: DOAR cand exista consumator (cloud on sau client TCP) SI bus-ul e viu
// (contact pus = trafic CAN recent). Altfel nu trimitem cereri degeaba (masina oprita = TX fara ACK).
static void ensure_internal_subs() {
    // v2.2.6: criteriul NU mai e „exista client conectat", ci „exista abonamente ale altcuiva".
    // Incident 16.09 (consola din masina): F16 cazut pe ELM tinea socketul deschis FARA sa retrimita `sub`
    // (il trimite o singura data per sesiune) => `stat` arata clients=1, dar `sub` zicea 0 abonamente, iar
    // setul intern nu pornea fiindca „e cineva conectat" => nimeni nu mai interoga nimic. Un client conectat
    // dar neabonat nu alimenteaza cache-ul, deci nu mai conteaza la decizie.
    int foreign = 0;
    for (size_t i = 0; i < osch_capacity(); i++) {
        const OschItem *it = osch_item(i);
        if (it->active && !it->oneshot && it->owner != OWNER_INTERNAL) foreign++;
    }
    // Setul intern ruleaza DOAR pt cloud cand NU e niciun client conectat. Cand F16 e conectat, abonamentul
    // lui alimenteaza cache-ul (osch_value_hook cache-uieste dupa PID indiferent de owner) => cloud ia de acolo,
    // fara dubla poluare a bus-ului.
    // v2.2.4: SCOASA conditia `bus_alive` (lastFrameMs < 8 s) — era AUTO-BLOCANTA. Pe linia de diagnoza nu
    // circula nimic nesolicitat: daca nimeni nu interogheaza, nu vin cadre, deci busul "pare mort" si setul
    // intern nu mai pornea niciodata. Incident 16.09 (dovedit cu datele din cloud + GPS): socketul F16 a murit
    // tacut in mers, abonamentele lui au fost sterse la deconectare, iar setul intern a ramas blocat de
    // `bus_alive` => ~15 minute de mers FARA nicio interogare OBD (cache gol, can=0, valori inghetate in F16).
    // Daca ECU-ul chiar nu raspunde, planificatorul marcheaza singur itemii (timeout/unsupported) — vizibil acum
    // in `req`/`resp`/`tmo` din snapshot — fara sa mai blocheze repornirea interogarilor.
    bool want = cfg_int("cloud.on") && foreign == 0 && g_can_bus_ok && strcmp(g_can_mode, "listen") != 0;
    if (want && !g_int_subs_added) add_internal_subs();
    else if (!want && g_int_subs_added) { osch_remove_owner(OWNER_INTERNAL); g_int_subs_added = false; }
}

// ===== ISO-TP drain callback: scheduler + await + raw JSON legacy =====
extern bool proto_consume_obd(uint32_t src_id, const uint8_t *data, size_t len);
static void on_obd_msg(uint32_t src_id, const uint8_t *data, size_t len) {
    proto_consume_obd(src_id, data, len);   // scheduler + await
    obd_process_response(src_id, data, len); // raw JSON pt scripturi (broadcast via emitJson)
}

// emitJson: folosit de obd_cmd.cpp/sensors → broadcast la clientii JSON
void emitJson(const char *line, size_t len) { broadcast_json(line, len); }

// ===== proto reply =====
static void proto_reply(uint8_t owner, const char *line, size_t len) { emit_to_owner(owner, line, len); }

void proto_pump(uint32_t ms) {
    uint32_t t0 = millis();
    do {
        if (g_can_ok) {
            twai_message_t m; int burst = 0;
            while (burst++ < 24 && twai_receive(&m, 0) == ESP_OK) {
                rxTotal++; lastFrameMs = millis();
                if (iso_tp_is_obd_resp(m.identifier)) lastObdRespMs = lastFrameMs;
                if (!((m.flags & TWAI_MSG_FLAG_EXTD) && m.identifier <= 0x7FF)) iso_tp_on_frame(m.identifier, m.data, m.data_length_code);
            }
            iso_tp_drain(on_obd_msg);
        }
        delay(1);
    } while (millis() - t0 < ms);
}

// v2.4.0: cat timp o operatie lunga (DTCALL: citirea DTC pe module dureaza secunde) tine loop() blocat, clientii TCP n-ar mai primi
// nimic si F16 ar crede ca placa a murit (timeout pe date). Apelat intre module: trimite eșantionul de senzori la timp.
void proto_keepalive() {
    uint32_t now = millis();
    if (now - lastSensorMs >= SENSOR_SAMPLE_MS) {
        lastSensorMs = now; SensorData sd; sensors_read(sd); g_last_sd = sd; g_have_sd = true;
        char b[JSON_BUF_SZ]; size_t n = sensors_to_json(sd, b, sizeof(b)); if (n) broadcast_json(b, n);
    }
}

// ===== WiFi =====
static const char *wifiStateName() { return g_wifi_state == WIFI_STA_OK ? "STA" : g_wifi_state == WIFI_AP_OK ? "AP" : "STA-trying"; }
static String wifiCurrentIp() { return g_wifi_state == WIFI_STA_OK ? WiFi.localIP().toString() : g_wifi_state == WIFI_AP_OK ? WiFi.softAPIP().toString() : String("0.0.0.0"); }

static void wifiStaBeginAsync() {
    WiFi.mode(WIFI_STA); WiFi.setSleep(false); WiFi.setHostname(MDNS_HOSTNAME);
    WiFi.begin(cfg_str("wifi.ssid"), cfg_str("wifi.pass"));
    g_wifi_state = WIFI_STA_TRYING; g_wifi_start_ms = millis();
}
// DNS fix (cfg net.dns) aplicat peste DHCP prin lwip — nu schimba IP-ul. HU-ul poate sa nu dea DNS clientilor.
static void apply_fixed_dns() {
    const char *d = cfg_str("net.dns");
    if (!d || !*d) return;
    IPAddress ip;
    if (!ip.fromString(d)) return;
    ip_addr_t a;
    IP_ADDR4(&a, ip[0], ip[1], ip[2], ip[3]);
    dns_setserver(0, &a);
}
// v2.1.2: opreste serverele ca sa le pot relega pe interfata noua (fix: dupa AP fallback -> STA,
// serverele ramaneau legate pe AP-ul vechi -> telnet/F16/discovery inbound nu functionau pe hotspot HU)
static void netServicesDown() {
    if (!g_net_up) return;
    for (int i = 0; i < MAX_JSON_CLIENTS; i++) if (tcpClients[i]) tcpClients[i].stop();
    for (int i = 0; i < MAX_TELNET_CLIENTS; i++) if (telnetClients[i]) telnetClients[i].stop();
    tcpServer.end();
    telnetServer.end();
    udpDisc.stop();
    MDNS.end();
    g_net_up = false;
}
static void netServicesUp() {
    if (g_net_up) return;
    if (MDNS.begin(MDNS_HOSTNAME)) { MDNS.addService("ergcan", "tcp", TCP_PORT); MDNS.addServiceTxt("ergcan", "tcp", "ver", FW_VERSION); }
    tcpServer.begin(); tcpServer.setNoDelay(true);
    if (cfg_int("con.telnet")) { telnetServer.begin(); telnetServer.setNoDelay(true); }
    udpDisc.begin(UDP_DISC_PORT); lastUdpBcMs = 0;
    g_net_up = true;
}
static void sendUdpDiscovery() {
    IPAddress ip = (g_wifi_state == WIFI_AP_OK) ? WiFi.softAPIP() : WiFi.localIP();
    uint32_t now = millis();
    int canOk = can_is_live() ? 1 : 0;
    char buf[192];
    int n = snprintf(buf, sizeof(buf), "{\"app\":\"ergCAN\",\"ip\":\"%s\",\"tcp_port\":%d,\"ts\":%lu,\"fw\":\"%s\",\"can\":%d,\"mode\":\"%s\"}",
                     ip.toString().c_str(), TCP_PORT, (unsigned long)now, FW_VERSION, canOk, g_can_mode);
    if (n <= 0) return;
    if (g_wifi_state == WIFI_STA_OK) {
        IPAddress gw = WiFi.gatewayIP(); if (gw[0]) { udpDisc.beginPacket(gw, UDP_DISC_PORT); udpDisc.write((uint8_t *)buf, n); udpDisc.endPacket(); }
        IPAddress bc = WiFi.broadcastIP(); if (bc[0]) { udpDisc.beginPacket(bc, UDP_DISC_PORT); udpDisc.write((uint8_t *)buf, n); udpDisc.endPacket(); }
    }
    udpDisc.beginPacket(IPAddress(255, 255, 255, 255), UDP_DISC_PORT); udpDisc.write((uint8_t *)buf, n); udpDisc.endPacket();
}
static void wifiServiceTick() {
    static uint32_t lastRetry = 0;
    switch (g_wifi_state) {
    case WIFI_STA_TRYING:
        if (WiFi.status() == WL_CONNECTED) {
            // Daca am trecut prin AP fallback, rebind-ul serverelor/mDNS pe interfata noua e nesigur pe ESP32
            // (raman legate pe netif-ul AP distrus: telnet refuzat, F16 "connected apoi timeout", .local mort).
            // Repornim curat o singura data -> boot direct pe STA, bind corect (ca scenariul hotspot telefon).
            if (g_ap_fallback_used && g_reboot_rebind_magic != REBIND_REBOOT_MAGIC) {
                g_reboot_rebind_magic = REBIND_REBOOT_MAGIC;
                Serial.println("[wifi] STA up dupa AP fallback -> restart pt bind curat");
                Serial.flush(); delay(200); ESP.restart();
            }
            WiFi.setSleep(false);   // v2.1.3: modem-sleep OFF si dupa retry (AP-ul HU pierde unicast la station sleep -> TCP "connected apoi timeout", mDNS mort)
            IPAddress ip = WiFi.localIP();
            g_wifi_state = WIFI_STA_OK; apply_fixed_dns();
            // v2.2.2: rebind DOAR daca IP-ul s-a schimbat (sau serviciile sunt jos). Cu IP fix in DHCP,
            // reconectarea nu mai inchide clientii TCP (F16/telnet) si nu mai opreste discovery-ul.
            if (!g_net_up || ip != g_last_sta_ip) {
                Serial.printf("[wifi] STA %s (rebind servere)\n", ip.toString().c_str());
                netServicesDown(); netServicesUp();
            } else {
                Serial.printf("[wifi] STA %s (acelasi IP -> serverele raman sus)\n", ip.toString().c_str());
            }
            lastUdpBcMs = 0;                                    // anunta discovery imediat, fara sa astepte 3 s
            g_last_sta_ip = ip; g_had_sta = true; g_sta_down_since_ms = 0;
            // v2.1.3: sincronizare ceas (NTP) - placa nu avea ceas (rula la 1970). Pt timestamp-uri corecte in cloud
            // si validare TLS pe termen (majoritatea hotspoturilor permit UDP 123 chiar cand alte porturi sunt limitate).
            configTime(0, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
        }
        else if (millis() - g_wifi_start_ms > (uint32_t)WIFI_STA_TIMEOUT_S * 1000UL) {
            // v2.2.2: AP-ul de rezerva porneste abia dupa WIFI_AP_FALLBACK_S de STA pierdut (inainte 20 s:
            // orice clipire a hotspotului ducea la AP fallback si apoi la restart "pt bind curat").
            // La prima pornire (niciodata conectat in acest ciclu) pastram pragul scurt.
            uint32_t down_ms  = g_sta_down_since_ms ? (millis() - g_sta_down_since_ms) : (millis() - g_wifi_start_ms);
            uint32_t ap_after = g_had_sta ? (uint32_t)WIFI_AP_FALLBACK_S * 1000UL : (uint32_t)WIFI_STA_TIMEOUT_S * 1000UL;
            if (cfg_int("wifi.ap") && down_ms >= ap_after) { g_ap_fallback_used = true; WiFi.disconnect(true); WiFi.mode(WIFI_AP); WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN); g_wifi_state = WIFI_AP_OK; netServicesUp(); }
            else { g_wifi_start_ms = millis(); WiFi.begin(cfg_str("wifi.ssid"), cfg_str("wifi.pass")); }
        }
        break;
    case WIFI_STA_OK:
        if (WiFi.status() != WL_CONNECTED) {
            g_wifi_state = WIFI_STA_TRYING; g_wifi_start_ms = millis();
            if (!g_sta_down_since_ms) g_sta_down_since_ms = millis();     // v2.2.2: de cand e jos (pt AP fallback)
            if (g_wifi_drops < 0xFFFF) g_wifi_drops++;
            Serial.println("[wifi] STA pierdut -> reconectare (serverele raman sus)");
        }
        break;
    case WIFI_AP_OK:
        if (millis() - lastRetry >= 30000UL) { lastRetry = millis(); WiFi.disconnect(true, true); WiFi.mode(WIFI_STA); WiFi.setSleep(false); WiFi.setHostname(MDNS_HOSTNAME); WiFi.begin(cfg_str("wifi.ssid"), cfg_str("wifi.pass")); g_wifi_state = WIFI_STA_TRYING; g_wifi_start_ms = millis(); }
        break;
    }
}

// ===== clienti: acceptare + citire linii =====
// v2.6.0: `cap` = marimea bufferului. O linie prea lunga NU mai e taiata in tacere (inainte `len = 0` pastra doar coada
// liniei, fara "cmd", care era apoi ignorata): o aruncam intreaga pana la \n si raspundem cu eroare.
static void feed_line(char *buf, size_t &len, size_t cap, bool &ovf, int b, uint8_t owner, bool json_default) {
    if (b == '\r') return;
    if (b == '\n') {
        if (ovf) {
            char e[96]; int n = json_default
                ? snprintf(e, sizeof(e), "{\"type\":\"err\",\"err\":\"linie prea lunga (max %u caractere)\"}\n", (unsigned)(cap - 1))
                : snprintf(e, sizeof(e), "EROARE: linie prea lunga (max %u caractere)\r\n", (unsigned)(cap - 1));
            if (n > 0) emit_to_owner(owner, e, (size_t)n);
            ovf = false; len = 0;
            return;
        }
        if (len > 0) { buf[len] = 0;
            if (json_default) proto_json_line(owner, buf);
            else proto_console_line(owner, buf, false);
            len = 0;
        }
        return;
    }
    if (ovf) return;
    if (len < cap - 1) buf[len++] = (char)b;
    else { ovf = true; len = 0; }
}

static void serviceTelnet() {
    if (cfg_int("con.telnet") && telnetServer.hasClient()) {
        int slot = -1; for (int i = 0; i < MAX_TELNET_CLIENTS; i++) if (!(telnetClients[i] && telnetClients[i].connected())) { slot = i; break; }
        WiFiClient nc = telnetServer.available();
        if (slot < 0) nc.stop();
        else { telnetClients[slot] = nc; telnetClients[slot].setNoDelay(true); telnetLen[slot] = 0; telnetIac[slot] = 0;
            // v2.1.1: negociere telnet — serverul face echo si suprima go-ahead (char mode).
            // Fara asta, clientii Android (ConnectBot) asteapta si nu trimit/afiseaza nimic; PuTTY mergea din echo local.
            const uint8_t neg[] = {0xFF, 0xFB, 0x01, 0xFF, 0xFB, 0x03};   // IAC WILL ECHO, IAC WILL SGA
            telnetClients[slot].write(neg, sizeof(neg));
            const char *w = "ergCAN " FW_VERSION " - HELP pt comenzi\r\nergCAN> ";
            telnetClients[slot].write((const uint8_t *)w, strlen(w));
        }
    }
    for (int i = 0; i < MAX_TELNET_CLIENTS; i++) {
        WiFiClient &c = telnetClients[i];
        if (!c) continue;
        if (!c.connected()) { c.stop(); proto_client_gone(OWNER_TELNET0 + i); if (g_mon_owner == OWNER_TELNET0 + i) g_mon_owner = -1; continue; }
        while (c.available()) {
            int b = c.read(); if (b < 0) break;
            // filtreaza negocierea telnet (IAC 0xFF + 2 octeti) si orice non-ASCII
            if (b == 0xFF) { telnetIac[i] = 2; continue; }
            if (telnetIac[i] > 0) { telnetIac[i]--; continue; }
            if (b >= 0x80 || b == 0) continue;
            // v2.1.1: ECHO (am anuntat WILL ECHO) — altfel ConnectBot nu arata ce tastezi
            if (b == '\b' || b == 0x7F) {                     // backspace
                if (telnetLen[i] > 0) { telnetLen[i]--; c.write((const uint8_t *)"\b \b", 3); }
                continue;
            }
            if (b == '\n' && telnetSkipLf[i]) { telnetSkipLf[i] = false; continue; }   // \n dupa \r = acelasi Enter
            if (b == '\r' || b == '\n') {
                telnetSkipLf[i] = (b == '\r');
                c.write((const uint8_t *)"\r\n", 2);
                if (telnetLen[i] > 0) { telnetBuf[i][telnetLen[i]] = 0; proto_console_line(OWNER_TELNET0 + i, telnetBuf[i], false); telnetLen[i] = 0; }
                char p[16]; proto_console_prompt(OWNER_TELNET0 + i, p, sizeof(p)); c.write((const uint8_t *)p, strlen(p));
                continue;
            }
            telnetSkipLf[i] = false;
            if (b >= 0x20 && telnetLen[i] < 159) { telnetBuf[i][telnetLen[i]++] = (char)b; c.write((uint8_t *)&b, 1); }
        }
    }
}

// v2.2.4: keepalive TCP pe clientii JSON. Pe hotspot socketul F16 poate muri TACUT (fara FIN): placa il
// credea in continuare conectat, F16 credea ca asculta o sesiune vie => valori inghetate minute in sir.
// Cu keepalive peer-ul mort e detectat in ~40 s (idle 20 s + 4 sonde la 5 s) si slotul se elibereaza,
// iar `proto_client_gone` curata abonamentele lui.
static void tcp_enable_keepalive(WiFiClient &c) {
    int on = 1, idle = 20, intvl = 5, cnt = 4;
    c.setSocketOption(SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
#ifdef TCP_KEEPIDLE
    c.setOption(TCP_KEEPIDLE, &idle);
    c.setOption(TCP_KEEPINTVL, &intvl);
    c.setOption(TCP_KEEPCNT, &cnt);
#else
    (void)idle; (void)intvl; (void)cnt;
#endif
}

static void serviceTcp() {
    if (tcpServer.hasClient()) {
        int slot = -1; for (int i = 0; i < MAX_JSON_CLIENTS; i++) if (!(tcpClients[i] && tcpClients[i].connected())) { slot = i; break; }
        WiFiClient nc = tcpServer.available();
        if (slot < 0) nc.stop();
        else { tcpClients[slot] = nc; tcpClients[slot].setNoDelay(true); tcp_enable_keepalive(tcpClients[slot]); tcpLen[slot] = 0; tcpOvf[slot] = false;
            proto_event("client JSON %d conectat (%s)", slot, tcpClients[slot].remoteIP().toString().c_str());   // v2.5.10
            char h[JSON_BUF_SZ];
            // v2.6.0: "proto":3 = placa stie abonarea pe nume; "veh" = profilul de vehicul pe care ruleaza
            int n = snprintf(h, sizeof(h), "{\"ts\":%lu,\"type\":\"hello\",\"app\":\"ergCAN\",\"fw\":\"%s\",\"proto\":%d,\"veh\":\"%s\",\"vehver\":%u,\"bitrate\":%u,\"port\":%d,\"can\":%d,\"mode\":\"%s\"}\n",
                             (unsigned long)millis(), FW_VERSION, PARAMS_PROTO, veh().model, (unsigned)veh().ver, (unsigned)veh_bitrate(), TCP_PORT, g_can_ok ? 1 : 0, g_can_mode);
            if (n > 0) tcpClients[slot].write((const uint8_t *)h, n);
        }
    }
    for (int i = 0; i < MAX_JSON_CLIENTS; i++) {
        WiFiClient &c = tcpClients[i];
        if (!c) continue;
        if (!c.connected()) { c.stop(); proto_client_gone(OWNER_TCP0 + i); proto_event("client JSON %d deconectat", i); continue; }
        int cnt = 0;
        while (c.available() && cnt < TCP_RX_PER_LOOP) { int b = c.read(); if (b < 0) break; cnt++; feed_line(tcpBuf[i], tcpLen[i], sizeof(tcpBuf[i]), tcpOvf[i], b, OWNER_TCP0 + i, true); }
    }
    while (Serial.available()) { int b = Serial.read(); if (b < 0) break;
        // serial: linie {..} = JSON, altfel consola
        if (serLen == 0 && (b == ' ')) continue;
        bool json = (serLen > 0 && serBuf[0] == '{');
        if (serLen == 0 && b == '{') json = true;
        feed_line(serBuf, serLen, sizeof(serBuf), serOvf, b, OWNER_SERIAL, json);
    }
}

// ===== setup/loop =====
// v2.1.3: task dedicat pt retea blocanta (cloud BYOS + autoconfigurare). Ruleaza pe core 0 cu stiva mare
// (TLS/mbedTLS). Chiar daca do_post atarna (retea fara internet, ex HU), loop-ul de pe core 1 ramane viu:
// telnet, F16, mDNS, discovery si CAN continua sa functioneze.
static void netWorkerTask(void *pv) {
    for (;;) {
        cloud_service(millis());
        acfg_service(millis());
        proto_debug_service(millis());   // v2.5.9: jurnal BYOSDEBUG
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void setup() {
    Serial.begin(115200); delay(150); bootMs = millis();
    ota_boot_check();          // devreme: revenire daca firmware-ul nou nu s-a confirmat
    cfg_init();
    pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
    Serial.println("\n ergCAN v" FW_VERSION " — OBD/UDS + abonamente + consola + OTA");

    OschHooks oh{osch_send_hook, osch_value_hook, osch_unsup_hook, nullptr};
    osch_init(oh); osch_set_ids(OBD_PHYSICAL_REQ_BASE, OBD_PHYSICAL_RESP_BASE);
    proto_init(proto_reply);
    iso_tp_allow(0x77E, 0x714);   // bord UDS (KM) — pt SELFTEST/await

    sensors_init();
    proto_event("BOOT fw " FW_VERSION " reset=%d heap %lu", (int)esp_reset_reason(), (unsigned long)ESP.getFreeHeap());   // v2.5.10
    strncpy(g_can_want, cfg_str("can.boot"), sizeof(g_can_want) - 1);
    // v2.6.0: profil generic => pornim direct pe viteza gasita ultima data (autodetectia o confirma din loop)
    if (veh().autodet && cfg_int("veh.abr") > 0) veh_set_detected((uint32_t)cfg_int("veh.abr"), cfg_int("veh.aext") != 0);
    g_can_ok = twaiStart(cfg_str("can.boot"));
    if (!g_can_ok) g_can_heal_at = millis() + 5000;
    osch_set_enabled(g_can_ok && strcmp(g_can_mode, "listen") != 0);
    wifiStaBeginAsync();

    // v2.1.3: cloud + acfg pe task dedicat (core 0) ca do_post-ul blocant sa NU inghete loop-ul (core 1)
    xTaskCreatePinnedToCore(netWorkerTask, "netw", 20480, NULL, 1, NULL, 0);
}

void loop() {
    ledService();
    wifiServiceTick();
    if (g_net_up) {
        serviceTcp();
        serviceTelnet();
        if (millis() - lastUdpBcMs >= UDP_BC_INTERVAL_MS) { lastUdpBcMs = millis(); sendUdpDiscovery(); }
    } else {
        serviceTcp();   // serial merge si fara retea
    }
    iso_tp_tick(millis());
    ota_service(millis() - bootMs);
    // v2.1.3: cloud + acfg NU mai ruleaza in loop (do_post blocant: DNS/TCP/TLS pe retea fara internet -
    // ex hotspot HU - ingheta loop-ul => telnet/F16/mDNS moarte, "connected apoi timeout").
    // Ruleaza pe task separat (netWorkerTask, core 0). Vezi setup().

    autodet_service();   // v2.6.0: doar pentru profilele cu "auto":1; altfel nu face nimic

    if (g_can_ok) {
        uint32_t alerts = 0;
        if (twai_read_alerts(&alerts, 0) == ESP_OK && alerts) {
            if (alerts & TWAI_ALERT_BUS_OFF) { g_can_bus_ok = false; twai_clear_transmit_queue(); twai_initiate_recovery();
                g_busoff_ms = millis() ? millis() : 1; proto_event("CAN BUS-OFF -> recuperare (tx_failed %lu)", (unsigned long)g_tx_failed);
                char b[96]; int n = snprintf(b, sizeof(b), "{\"ts\":%lu,\"type\":\"buststate\",\"ok\":0}\n", (unsigned long)millis()); broadcast_json(b, n); }
            if (alerts & TWAI_ALERT_BUS_RECOVERED) { esp_err_t e = twai_start(); g_can_bus_ok = true; g_busoff_ms = 0;
                proto_event("CAN recuperat din bus-off, twai_start=%s", esp_err_to_name(e)); }
            if (alerts & (TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL)) rxErr++;
            if (alerts & TWAI_ALERT_TX_FAILED) { g_tx_failed++; probeIntervalMs = CAN_PROBE_BACKOFF_MS; }
        }
        // probe idle: fara abonamente (nimeni nu tine bus-ul viu) SAU cand idle_gate a oprit planificatorul.
        // v2.3.2: a doua conditie lipsea => cu `can`=0 planificatorul tacea, probe-ul nu rula (erau abonamente),
        // pe mufa OBD nu vine nimic nesolicitat => placa ramanea muta pana la reset. Probe-ul e SINGURA cale
        // de revenire din starea „mort" — nu-l conditiona de nimic ce depinde de trafic.
        bool have_subs = false;
        for (size_t i = 0; i < osch_capacity(); i++) if (osch_item(i)->active) { have_subs = true; break; }
        bool gated = cfg_int("can.idle_gate") && !can_is_live();
        if (g_can_tx_enabled && g_can_bus_ok && (!have_subs || gated) && !sniff_is_active() && !(veh().autodet && !g_ad_done)) {
            uint32_t nowP = millis();
            bool idle = (lastFrameMs == 0) || (int32_t)(nowP - lastFrameMs) >= (int32_t)CAN_PROBE_IDLE_MS;
            if (idle && (lastProbeMs == 0 || nowP - lastProbeMs >= probeIntervalMs)) { lastProbeMs = nowP; g_probe_n++; obd_probe(); }
        }
        // v2.5.10: bus-off care nu se recupereaza in 5 s (alerta pierduta / controller blocat) => reinstalam driverul
        if (g_busoff_ms && millis() - g_busoff_ms > 5000) {
            proto_event("CAN bus-off nerecuperat in 5 s -> reinstalez driverul");
            g_busoff_ms = 0; can_set_mode(g_can_want); osch_set_enabled(g_can_ok && strcmp(g_can_mode, "listen") != 0);
        }
        // v2.5.10: „motorul nu mai raspunde desi magistrala e vie" — tranzitiile ajung in jurnal, cu starea planificatorului
        {
            static bool s_eng = false; static uint32_t s_chk = 0;
            uint32_t nw = millis();
            if (nw - s_chk >= 2000) { s_chk = nw;
                bool bus = lastFrameMs && nw - lastFrameMs < 3000;
                bool eng = lastObdRespMs && nw - lastObdRespMs < 15000;
                if (s_eng && !eng && bus) {
                    const OschStats &os = osch_stats();
                    proto_event("MOTOR TACE (bus viu): osch=%d busy=%d req=%lu resp=%lu to=%lu nrc=%lu txref=%lu multi=%d mode=%s",
                                osch_enabled() ? 1 : 0, osch_busy() ? 1 : 0, (unsigned long)os.req, (unsigned long)os.resp, (unsigned long)os.timeout,
                                (unsigned long)os.nrc, (unsigned long)g_tx_refused, os.multi_ok ? 1 : 0, g_can_mode);
                } else if (!s_eng && eng) proto_event("motor raspunde (0x7E8)");
                if (eng || !bus) s_eng = eng;
            }
        }
        ensure_internal_subs();   // v2.2: set intern activ doar cu consumator + bus viu
        osch_tick(millis());

        twai_message_t m; int burst = 0;
        while (burst++ < 32 && twai_receive(&m, 0) == ESP_OK) {
            rxTotal++; lastFrameMs = millis();
            if (iso_tp_is_obd_resp(m.identifier)) { lastObdRespMs = lastFrameMs; probeIntervalMs = CAN_PROBE_IDLE_MS; }
            ledPulse();
            if (sniff_is_active()) sniff_frame(m.identifier, (m.flags & TWAI_MSG_FLAG_EXTD), m.data, m.data_length_code, lastFrameMs);
            // raw frame JSON: broadcast la clientii JSON + MON pe consola
            char b[JSON_BUF_SZ]; char hx[3 * 8 + 1]; int p = 0; int dlc = m.data_length_code > 8 ? 8 : m.data_length_code;
            for (int i = 0; i < dlc; i++) { if (i) hx[p++] = ' '; static const char H[] = "0123456789ABCDEF"; hx[p++] = H[(m.data[i] >> 4) & 0xF]; hx[p++] = H[m.data[i] & 0xF]; }
            hx[p] = 0;
            int n = snprintf(b, sizeof(b), "{\"ts\":%lu,\"type\":\"can\",\"id\":\"0x%lX\",\"ext\":%d,\"rtr\":%d,\"len\":%u,\"data\":\"%s\"}\n",
                             (unsigned long)millis(), (unsigned long)m.identifier, (m.flags & TWAI_MSG_FLAG_EXTD) ? 1 : 0, (m.flags & TWAI_MSG_FLAG_RTR) ? 1 : 0, (unsigned)m.data_length_code, hx);
            if (n > 0) { broadcast_json(b, n);
                if (g_mon_owner >= 0 && (g_mon_id == 0 || (long)m.identifier == g_mon_id)) emit_to_owner(g_mon_owner, b, n);
            }
            // v2.6.0: un cadru extended cu ID mic nu e raspunsul standard cu acelasi numar
            if (!((m.flags & TWAI_MSG_FLAG_EXTD) && m.identifier <= 0x7FF)) iso_tp_on_frame(m.identifier, m.data, m.data_length_code);
        }
        iso_tp_drain(on_obd_msg);
        // v2.2: citire lenta VIN/KM in fundal, DOAR cand motorul raspunde (altfel await ar bloca pana la timeout)
        if (g_can_bus_ok && lastObdRespMs && millis() - lastObdRespMs < CAN_OK_WINDOW_MS) proto_slow_read_tick(millis());
    }

    // v2.5.10: driver CAN cazut => reincercam singuri la 5 s (inainte ramanea mort pana la reboot)
    if (!g_can_ok && g_can_heal_at && (int32_t)(millis() - g_can_heal_at) >= 0 && !sniff_is_active()) {
        g_can_heals++;
        g_can_ok = twaiStart(g_can_want);
        if (g_can_ok) { g_can_heal_at = 0; proto_event("CAN reparat automat (incercarea %lu), mod %s", (unsigned long)g_can_heals, g_can_mode);
                        osch_set_enabled(strcmp(g_can_mode, "listen") != 0); lastProbeMs = 0; }
        else g_can_heal_at = millis() + 5000;
    }

    uint32_t now = millis();
    if (now - lastSensorMs >= SENSOR_SAMPLE_MS) { lastSensorMs = now; SensorData sd; sensors_read(sd); g_last_sd = sd; g_have_sd = true;
        char b[JSON_BUF_SZ]; size_t n = sensors_to_json(sd, b, sizeof(b)); if (n) broadcast_json(b, n); }
    if (now - lastHbMs >= HB_INTERVAL_MS && now - lastFrameMs >= HB_INTERVAL_MS) { lastHbMs = now;
        char b[160]; int n = snprintf(b, sizeof(b), "{\"ts\":%lu,\"type\":\"hb\",\"uptime\":%lu,\"wifi\":\"%s\",\"ip\":\"%s\"}\n",
            (unsigned long)now, (unsigned long)((now - bootMs) / 1000UL), wifiStateName(), wifiCurrentIp().c_str()); if (n > 0) broadcast_json(b, n); }
    delay(1);
}

// MON hook din consola
void con_set_mon(int owner, long id, bool on) { g_mon_owner = on ? owner : -1; g_mon_id = id; }

// NETDIAG — diagnoza retea pas cu pas (DNS -> TCP -> TLS -> HTTP) pt „connection refused” in masina
static void parse_url(const String &url, String &host, int &port, String &path) {
    port = 443; path = "/";
    int a = url.indexOf("://"); a = (a < 0) ? 0 : a + 3;
    int slash = url.indexOf('/', a); if (slash < 0) slash = url.length();
    String hp = url.substring(a, slash);
    path = slash < (int)url.length() ? url.substring(slash) : "/";
    int c = hp.indexOf(':');
    if (c >= 0) { host = hp.substring(0, c); port = hp.substring(c + 1).toInt(); }
    else { host = hp; }
}
String net_diag() {
    String o = "=== NETDIAG ===\r\n";
    o += "WiFi   : " + String(wifiStateName()) + " ip " + wifiCurrentIp() +
         " gw " + WiFi.gatewayIP().toString() + " mask " + WiFi.subnetMask().toString() + " rssi " + WiFi.RSSI() + "\r\n";
    o += "DNS    : " + WiFi.dnsIP(0).toString() + " / " + WiFi.dnsIP(1).toString() + "\r\n";
    { time_t nowt = time(nullptr);
      if (nowt > 1700000000) { struct tm tmv; gmtime_r(&nowt, &tmv); char tb[32]; strftime(tb, sizeof(tb), "%Y-%m-%d %H:%M:%SZ", &tmv); o += "Ceas   : " + String(tb) + " (NTP OK)\r\n"; }
      else o += "Ceas   : NESETAT (epoch " + String((long)nowt) + ") - NTP nereusit (HU blocheaza UDP 123?)\r\n"; }
    String url = cfg_str("cloud.on") ? cfg_str("cloud.url") : cfg_str("ota.url");
    String host; int port; String path; parse_url(url, host, port, path);
    o += "Tinta  : " + host + ":" + port + "\r\n";
    // 1) DNS
    IPAddress ip; uint32_t t0 = millis();
    bool dns = WiFi.hostByName(host.c_str(), ip);
    o += "1 DNS  : " + String(dns ? (ip.toString() + " (" + (millis() - t0) + "ms)") : ("ESUAT (" + String(millis() - t0) + "ms) — HU nu da DNS bun clientilor")) + "\r\n";
    if (!dns) { o += ">> cauza: rezolvarea DNS. Fix: SET net.dns <gateway HU> (sau 8.8.8.8), SAVE, reboot.\r\n"; return o; }
    // 2) TCP brut
    WiFiClient tcp; t0 = millis();
    bool tcpok = tcp.connect(ip, port, 5000);
    o += "2 TCP  : " + String(tcpok ? ("conectat (" + String(millis() - t0) + "ms)") : ("ESUAT/refuzat (" + String(millis() - t0) + "ms)")) + "\r\n";
    tcp.stop();
    if (!tcpok) { o += ">> cauza: TCP la " + host + ":" + port + " refuzat/blocat (port " + port + " blocat pe LTE? NAT HU?).\r\n"; return o; }
    // 3) TLS
    NetworkClientSecure tls; tls.setCACert(CA_ISRG_ROOT_X1); tls.setHandshakeTimeout(8); tls.setTimeout(8000); t0 = millis();
    bool tlsok = tls.connect(host.c_str(), port);
    o += "3 TLS  : " + String(tlsok ? ("handshake OK (" + String(millis() - t0) + "ms)") : "ESUAT (certificat/handshake)") + "\r\n";
    tls.stop();
    if (!tlsok) { o += ">> cauza: TLS. Ceas gresit sau certificat.\r\n"; return o; }
    // 4) HTTP
    NetworkClientSecure hc; hc.setCACert(CA_ISRG_ROOT_X1); hc.setHandshakeTimeout(8); hc.setTimeout(8000);
    HTTPClient http; http.setConnectTimeout(6000); http.setTimeout(8000); http.setReuse(false);
    String u2 = "https://" + host + ":" + port + path;
    o += "4 HTTP : ";
    if (http.begin(hc, u2)) { int code = http.GET(); o += (code > 0 ? ("cod " + String(code)) : ("err " + http.errorToString(code))); http.end(); }
    else o += "begin esuat";
    o += "\r\n>> DNS+TCP+TLS OK => reteaua e buna; daca aici pica, e serverul/URL.\r\n";
    return o;
}

// snapshot telemetrie pt cloud (BYOS autonom): mac, fw, uptime, can, senzori
String cloud_snapshot_json() {
    uint32_t now = millis();
    bool canOk = g_can_ok && g_can_bus_ok && ((lastObdRespMs && now - lastObdRespMs < CAN_OK_WINDOW_MS) || (lastFrameMs && now - lastFrameMs < 10000));
    String mac = WiFi.macAddress(); mac.replace(":", "");
    char b[512];
    snprintf(b, sizeof(b),
        "{\"mac\":\"%s\",\"fw\":\"%s\",\"ts\":%lu,\"uptime\":%lu,\"can\":%d,\"rssi\":%d,\"heap\":%lu,"
        "\"bmp\":{\"ok\":%d,\"t\":%.2f,\"p\":%.2f},\"acc\":{\"ok\":%d,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},"
        "\"gyr\":{\"ok\":%d,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}",
        mac.c_str(), FW_VERSION, (unsigned long)now, (unsigned long)((now - bootMs) / 1000UL), canOk ? 1 : 0,
        g_wifi_state == WIFI_STA_OK ? WiFi.RSSI() : 0, (unsigned long)ESP.getFreeHeap(),
        g_have_sd && g_last_sd.bmp_ok, g_have_sd ? g_last_sd.bmp_temp_c : 0.0f, g_have_sd ? g_last_sd.bmp_press_hpa : 0.0f,
        g_have_sd && g_last_sd.accel_ok, g_have_sd ? g_last_sd.accel_x_g : 0.0f, g_have_sd ? g_last_sd.accel_y_g : 0.0f, g_have_sd ? g_last_sd.accel_z_g : 0.0f,
        g_have_sd && g_last_sd.gyro_ok, g_have_sd ? g_last_sd.gyro_x_dps : 0.0f, g_have_sd ? g_last_sd.gyro_y_dps : 0.0f, g_have_sd ? g_last_sd.gyro_z_dps : 0.0f);
    String o = b;
    if (g_have_sd) o += sensors_mag_json(g_last_sd);   // v2.5.13: magnetometru brut + heading + stare calibrare
    // v2.2: VIN + KM (citite in fundal) + date motor din cache (proaspete < 30s)
    String vin = proto_vin_cached(); if (vin.length()) { o += ",\"vin\":\""; o += vin; o += "\""; }
    long km = proto_km_cached();     if (km >= 0)       { o += ",\"km\":"; o += String(km); }
    long fuel = proto_fuel_cached(); if (fuel >= 0)     { o += ",\"fuel\":"; o += String(fuel);      // v2.2.3: litri (UDS 2206/2212)
                                                          o += ",\"fdid\":"; o += String(proto_fuel_did()); }   // v2.2.5: de pe ce DID
    o += proto_eng_json();   // v2.3.3: genload/genperf/oilt/oill (ECU motor), doar valorile proaspete
    o += proto_dtc_json();   // v2.4.0: rezumat DTCALL (nr. coduri + module cu erori)
    o += proto_dtc_list_json();  // v2.5.4: si CODURILE pe module, o singura data dupa fiecare scanare
    // v2.2.2: diagnoza reconectari — IP-ul local, cate pierderi de STA de la pornire, motivul ultimului reset
    if (g_wifi_state == WIFI_STA_OK) { o += ",\"ip\":\""; o += WiFi.localIP().toString(); o += "\""; }
    o += ",\"wdrops\":"; o += String((unsigned)g_wifi_drops);
    o += ",\"rr\":";     o += String((int)esp_reset_reason());
    // v2.2.4: diagnoza „nu se mai interogheaza OBD" direct din server — clienti TCP, abonamente active,
    // contoarele planificatorului si varsta (secunde) a ultimului cadru CAN / ultim raspuns ECU.
    { int nc2 = 0; for (int i = 0; i < MAX_JSON_CLIENTS; i++) if (tcpClients[i] && tcpClients[i].connected()) nc2++;
      int subs = 0; for (size_t i = 0; i < osch_capacity(); i++) if (osch_item(i)->active) subs++;
      const OschStats &os = osch_stats();
      o += ",\"nc\":";    o += String(nc2);
      o += ",\"subs\":";  o += String(subs);
      o += ",\"req\":";   o += String((unsigned long)os.req);
      o += ",\"resp\":";  o += String((unsigned long)os.resp);
      o += ",\"tmo\":";   o += String((unsigned long)os.timeout);
      o += ",\"multi\":"; o += String(os.multi_ok ? 1 : 0);   // v2.2.6: 0 = s-a cazut pe cate un PID (refresh lent)
      // v2.2.9: contoarele care lipseau si m-au pus sa ghicesc. `txref` creste cand TWAI nu poate transmite
      // (motor oprit = fara ACK pe bus) — exact cazul in care req/resp/tmo par „inghetate", desi placa e sanatoasa.
      int unsup = 0; for (size_t i = 0; i < osch_capacity(); i++) if (osch_item(i)->active && osch_item(i)->unsupported) unsup++;
      o += ",\"nrc\":";   o += String((unsigned long)os.nrc);
      o += ",\"p78\":";   o += String((unsigned long)os.pending78);
      o += ",\"txref\":"; o += String((unsigned long)g_tx_refused);
      o += ",\"txfail\":";o += String((unsigned long)g_tx_failed);
      o += ",\"prb\":";   o += String((unsigned long)g_probe_n);   // v2.3.2: probe-uri trimise (reinvierea din `can`=0)
      o += ",\"unsup\":"; o += String(unsup);
      o += ",\"veh\":\"";  o += veh().model; o += "\"";   // v2.3.0: ce profil de vehicul ruleaza placa
      o += proto_unsup_json();                              // v2.6.0: CE chei sunt nesuportate, nu doar cate
      { uint32_t abr = 0; bool aext = false;
        if (veh().autodet && veh_detected(&abr, &aext)) { o += ",\"abr\":"; o += String((unsigned long)abr); o += ",\"aext\":"; o += String(aext ? 1 : 0); o += ",\"adok\":"; o += String(g_ad_done ? 1 : 0); } }
      // v2.2.5: varstele se calculeaza cu diferenta CU SEMN si se taie la 0. Snapshot-ul e construit pe
      // task-ul de retea, iar loop()-ul de pe celalalt nucleu actualizeaza lastFrameMs intre timp: `now`
      // putea ramane in urma => (now - lastFrameMs) facea underflow si ieseau 4294967 s in cloud.
      int32_t d_f = lastFrameMs   ? (int32_t)(now - lastFrameMs)   : -1;
      int32_t d_e = lastObdRespMs ? (int32_t)(now - lastObdRespMs) : -1;
      o += ",\"fage\":";  o += String(lastFrameMs   ? (long)(d_f > 0 ? d_f / 1000 : 0) : -1L);
      o += ",\"eage\":";  o += String(lastObdRespMs ? (long)(d_e > 0 ? d_e / 1000 : 0) : -1L);
    }
    String obd = "";
    for (int pid = 0; pid < 256; pid++) {
        // v2.2.5: comparatie CU SEMN — valoarea poate fi scrisa de loop() (celalalt nucleu) dupa ce am citit
        // `now`, iar `now - ts` facea underflow => PID-ul parea vechi de 49 de zile si era sarit TACUT.
        // Exact asa disparea obiectul `obd` din snapshot tocmai cand F16 era conectat si veneau valori des.
        if (!g_oc[pid].has || (int32_t)(now - g_oc[pid].ts) > 30000) continue;
        const char *k = obd_key_for_pid((uint8_t)pid); if (!k) continue;
        if (obd.length()) obd += ",";
        char e[48]; snprintf(e, sizeof(e), "\"%s\":%.2f", k, g_oc[pid].v); obd += e;
    }
    if (obd.length()) { o += ",\"obd\":{"; o += obd; o += "}"; }
    o += "}";
    return o;
}
