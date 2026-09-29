/**
 * tp20.cpp — VW TP 2.0 + KWP2000 (vezi tp20.h). Validat pe Altea: ABS grupuri, gateway 04A1, BCM (sniff VCP 2026-09-21).
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.6
 * @changes
 *   v1.6 2026-09-23 — fw 2.5.7, dupa sniff VCDS (docs/research/sniff_20260923_vcds): VCDS asteapta ~125 ms dupa A1 inainte de
 *                     prima cerere (ABS: 10 89 -> 50 89) si ~50-100 ms intre cereri; noi trimiteam imediat => ABS NRC 0x21.
 *                     (1) prima cerere pe canal la >= TP20_FIRST_REQ_MS dupa A1; (2) >= TP20_REQ_GAP_MS intre cereri;
 *                     (3) pauza de retry la 0x21 fara rafala de A3 (era A3 la ~20 ms => ABS inchidea canalul cu A8).
 *   v1.5 2026-09-23 — fw 2.5.6: ABS (0x03) raspundea NRC 0x21 busyRepeatRequest la ORICE cerere si noi renuntam din prima.
 *                     tp20_kwp repeta cererea la 0x21 (pana la 5 ori, 250 ms, A3 intre ele ca sa tina canalul viu);
 *                     setup respins cu D7/D8 (temporar ocupat) reincercat de 3 ori la 300 ms; tp20_trace_begin/end
 *                     publice pentru comanda de consola TPQ.
 *   v1.0 2026-09-15 — creat (2.1), dupa docs/research/04_vw_tp20_kwp2000.md
 *   v1.1 2026-09-19 — buffer de reasamblare 256 -> 384: raspunsul `22 04A1` al gateway-ului (lista de instalare) are 259 octeti
 *   v1.4 2026-09-21 — fw 2.5.3: tp20_read_mem pe BUCATI (35 addr+len_chunk / 36.. / 37, repetat) intr-o singura sesiune de
 *                     securitate; se opreste la primul NRC (31 = requestOutOfRange = margine) si raporteaza stop_nrc/stop_addr;
 *                     partial = succes daca s-a citit ceva. Trace = ring (ultimele 96 cadre), ca sa vedem sfarsitul, nu inceputul.
 *   v1.3 2026-09-21 — fw 2.5.2: READEPROM pe masina a dat `27 03: fara raspuns TP2.0` dupa ce 22 F187 mersese. Nu se vedea
 *                     NIMIC din ce a facut BCM-ul => (1) TRACE cadre TX/RX pe canal in tp20_read_mem (tp20_trace_*), afisat de
 *                     proto la esec + urcat in `frames`; (2) NAK (op 9) detectat explicit in loc de "fara raspuns";
 *                     (3) pauza T3 (din A1) intre cadrul precedent si o cerere noua (ACK-ul nostru era urmat imediat de 27 03);
 *                     (4) 27 03 (cerere seed, inofensiva) reincercata o data dupa 250 ms.
 *   v1.2 2026-09-21 — fw 2.5.0: (1) cereri MULTI-FRAME (op 2x = urmeaza, op 1x = ultimul cu ACK), pana la 40 octeti;
 *                     (2) raspunsul e validat pe SID (req+0x40 sau 7F) — un cadru intarziat/retransmis de la tranzactia
 *                     anterioara (ex. `50 86` dublat in sniff-ul VCP) nu mai e luat drept raspuns; (3) tp20_open_ex()
 *                     fara 10 89; (4) tp20_read_mem() = secventa VAG CAN PRO de citire memorie (27/10 86/35/36/37/82),
 *                     cu blocare dupa prima cheie refuzata (tp20_sec_locked) ca sa nu blocam accesul in modul.
 */
#include "tp20.h"
#include "config.h"
#include "driver/twai.h"
#include "iso_tp.h"
#include <cstring>

extern bool sniff_is_active();
extern void sniff_frame(uint32_t id, bool ext, const uint8_t *d, uint8_t dlc, uint32_t ts);

#define TP20_SETUP_ID   0x200
#define TP20_SETUP_TO   1000
#define TP20_ACK_TO     300
#define TP20_DATA_TO    600
#define TP20_PENDING_TO 5000
#define TP20_REQ_MAX    40      // octeti cerere (1 cadru de 5 + 5 cadre de 7)
#define TP20_FIRST_REQ_MS 130   // v1.6: VCDS: ~125 ms intre A1 si prima cerere
#define TP20_REQ_GAP_MS   50    // v1.6: VCDS: 50-100 ms intre ACK-ul raspunsului si cererea urmatoare

struct Tp20Chan { bool open; uint8_t logical; uint32_t tx_id, rx_id; uint8_t tx_seq, rx_seq; uint16_t t3ms; uint32_t last_act; uint32_t ready_at; };
static Tp20Chan ch = {false, 0, 0, 0, 0, 0, 10, 0, 0};
static uint8_t  g_last_nrc = 0;
static uint8_t  g_busy_retries = 0;   // v1.5: cate repetari 0x21 a facut ultima tranzactie
static bool     g_sec_locked = false;

// ---- trace (v1.3) ----
static Tp20Frame g_tr[TP20_TRACE_MAX];
static size_t    g_trn = 0;
static bool      g_tr_on = false;
static uint32_t  g_tr_t0 = 0;
static void tr_add(bool tx, uint32_t id, const uint8_t *d, uint8_t dlc) {
    if (!g_tr_on) return;
    Tp20Frame &f = g_tr[g_trn % TP20_TRACE_MAX]; g_trn++;   // v1.4: ring
    f.t = millis() - g_tr_t0; f.id = (uint16_t)id; f.tx = tx ? 1 : 0; f.dlc = dlc > 8 ? 8 : dlc;
    memset(f.d, 0, 8); memcpy(f.d, d, f.dlc);
}
void tp20_trace_begin() { g_trn = 0; g_tr_t0 = millis(); g_tr_on = true; }   // v1.5
void tp20_trace_end()   { g_tr_on = false; }
uint8_t tp20_busy_retries() { return g_busy_retries; }
size_t tp20_trace_count() { return g_trn < TP20_TRACE_MAX ? g_trn : TP20_TRACE_MAX; }
const Tp20Frame *tp20_trace_get(size_t i) {
    if (i >= tp20_trace_count()) return nullptr;
    size_t start = g_trn > TP20_TRACE_MAX ? g_trn % TP20_TRACE_MAX : 0;
    return &g_tr[(start + i) % TP20_TRACE_MAX];
}

// ---- low level ----
static bool tp_tx(uint32_t id, const uint8_t *d, uint8_t len) {
    twai_message_t m; memset(&m, 0, sizeof(m));
    m.identifier = id; m.data_length_code = len;
    memcpy(m.data, d, len);
    bool ok = twai_transmit(&m, pdMS_TO_TICKS(50)) == ESP_OK;
    if (ok) { ch.last_act = millis(); tr_add(true, id, d, len); }
    return ok;
}
// asteapta un cadru cu identifier==want (want=0 => oricare din canal). Alte cadre => iso_tp/sniff (best effort).
static bool tp_rx(uint32_t want, uint32_t timeout, twai_message_t &out) {
    uint32_t t0 = millis();
    while (millis() - t0 < timeout) {
        twai_message_t m;
        if (twai_receive(&m, pdMS_TO_TICKS(5)) == ESP_OK) {
            if (sniff_is_active()) sniff_frame(m.identifier, m.flags & TWAI_MSG_FLAG_EXTD, m.data, m.data_length_code, millis());
            if (m.identifier == want || (want == 0 && (m.identifier == ch.rx_id))) { tr_add(false, m.identifier, m.data, m.data_length_code); out = m; return true; }
            if (m.identifier >= 0x7E8 && m.identifier <= 0x7EF) iso_tp_on_frame(m.identifier, m.data, m.data_length_code);
        }
    }
    return false;
}
static uint16_t decode_time(uint8_t t) {   // biti 7-6 unitate (0:.1ms 1:1ms 2:10ms 3:100ms), 5-0 mult
    uint16_t mult = t & 0x3F;
    switch ((t >> 6) & 3) { case 0: return mult / 10; case 1: return mult; case 2: return mult * 10; default: return mult * 100; }
}

// ---- setup canal ----
bool tp20_open_ex(uint8_t logical, bool start_session, String &err) {
    if (ch.open && ch.logical == logical) return true;
    if (ch.open) tp20_close();
    // 1) setup pe 0x200
    uint8_t setup[7] = {logical, 0xC0, 0x00, 0x10, 0x00, 0x03, 0x01};
    twai_message_t r;
    for (int att = 0; ; att++) {   // v1.5: D7/D8 = temporar ocupat (ex. canal anterior inca neexpirat) -> reincercam
        if (!tp_tx(TP20_SETUP_ID, setup, 7)) { err = "TX setup esuat (bus?)"; return false; }
        if (!tp_rx(TP20_SETUP_ID + logical, TP20_SETUP_TO, r)) { err = "modul 0x" + String(logical, HEX) + " nu raspunde la setup (nu exista / nu e TP2.0)"; return false; }
        if (r.data_length_code >= 7 && r.data[1] == 0xD0) break;
        bool temp = r.data_length_code >= 2 && (r.data[1] == 0xD7 || r.data[1] == 0xD8);
        if (!temp || att >= 2) { err = "setup respins b1=0x" + String(r.data[1], HEX) + (r.data[1] == 0xD6 ? " (aplicatie nesuportata)" : r.data[1] == 0xD7 ? " (temporar nesuportat)" : r.data[1] == 0xD8 ? " (fara resurse / ocupat)" : ""); return false; }
        delay(300);
    }
    ch.rx_id = r.data[2] | ((uint32_t)(r.data[3] & 0x0F) << 8);
    ch.tx_id = r.data[4] | ((uint32_t)(r.data[5] & 0x0F) << 8);
    if ((r.data[3] & 0x10) || (r.data[5] & 0x10)) { err = "ID invalid (V=1)"; return false; }
    // 2) parametri canal
    uint8_t par[6] = {0xA0, 0x0F, 0x8A, 0xFF, 0x4A, 0xFF};
    if (!tp_tx(ch.tx_id, par, 6)) { err = "TX params esuat"; return false; }
    if (!tp_rx(ch.rx_id, TP20_ACK_TO, r) || r.data[0] != 0xA1) { err = "fara A1 (params)"; return false; }
    ch.t3ms = decode_time(r.data[4]); if (ch.t3ms < 2) ch.t3ms = 5;
    ch.tx_seq = 0; ch.rx_seq = 0; ch.open = true; ch.logical = logical; ch.last_act = millis();
    ch.ready_at = millis() + TP20_FIRST_REQ_MS;   // v1.6
    if (!start_session) return true;
    // 3) StartDiagnosticSession KWP 10 89
    uint8_t resp[8]; size_t rl; String e2;
    uint8_t ss[2] = {0x10, 0x89};
    if (!tp20_kwp(ss, 2, resp, sizeof(resp), &rl, e2) || rl < 1 || resp[0] != 0x50) {
        err = "StartSession esuat (" + e2 + ")"; tp20_close(); return false;
    }
    return true;
}
bool tp20_open(uint8_t logical, String &err) { return tp20_open_ex(logical, true, err); }

void tp20_close() {
    if (ch.open) { uint8_t a8 = 0xA8; tp_tx(ch.tx_id, &a8, 1); twai_message_t r; tp_rx(ch.rx_id, 200, r); }
    ch.open = false;
}
bool tp20_is_open() { return ch.open; }
uint8_t tp20_addr() { return ch.logical; }
uint8_t tp20_last_nrc() { return g_last_nrc; }
bool tp20_sec_locked() { return g_sec_locked; }

static void keep_alive() {
    if (!ch.open) return;
    if (millis() - ch.last_act < ch.t3ms * 40u) return;   // A3 doar cand canalul e liber
    uint8_t a3 = 0xA3; tp_tx(ch.tx_id, &a3, 1);
    twai_message_t r; tp_rx(ch.rx_id, 100, r);
}

// colecteaza UN mesaj KWP complet de pe canal (reasamblare pe seq, ACK la op 0/1). `pre` = cadru deja citit (sau null).
static bool collect_msg(const twai_message_t *pre, uint32_t timeout, uint8_t *buf, size_t bufsz, size_t *out_len) {
    twai_message_t r; size_t total = 0, need = 0; bool have_len = false;
    uint32_t deadline = millis() + timeout;
    bool first = (pre != nullptr);
    if (pre) r = *pre;
    while (millis() < deadline) {
        if (!first) { if (!tp_rx(ch.rx_id, timeout, r)) break; }
        first = false;
        uint8_t op = (r.data[0] & 0xF0) >> 4;
        if (op >= 0x9) continue;                       // A1/A3 keepalive, B ACK, 9 NAK
        uint8_t seq = r.data[0] & 0x0F;
        size_t off = 1;
        if (!have_len) { need = (((size_t)r.data[1] << 8) | r.data[2]) & 0x0FFF; off = 3; have_len = true; }
        for (size_t i = off; i < r.data_length_code && total < bufsz; i++) buf[total++] = r.data[i];
        if (op == 0 || op == 1) { uint8_t ack = 0xB0 | ((seq + 1) & 0x0F); tp_tx(ch.tx_id, &ack, 1); }
        if (total >= need) { *out_len = need < bufsz ? need : bufsz; return true; }
        deadline = millis() + timeout;
    }
    *out_len = total;
    return false;
}

// ---- tranzactie KWP peste TP2.0 ----
static bool tp20_kwp_once(const uint8_t *req, size_t rlen, uint8_t *resp, size_t resp_sz, size_t *resp_len, String &err) {
    g_last_nrc = 0;
    if (!ch.open) { err = "canal inchis"; return false; }
    if (rlen == 0 || rlen > TP20_REQ_MAX) { err = "cerere prea lunga (max 40)"; return false; }
    // v1.2: multi-frame. Primul cadru: [op|seq, lenHi, lenLo, 5 octeti]; urmatoarele: [op|seq, 7 octeti].
    // op 2 = mai urmeaza (fara ACK), op 1 = ultimul (asteapta ACK) — exact ca VAG CAN PRO in sniff.
    // v1.3: respecta T3 (timpul minim intre cadrele NOASTRE, din A1) - ACK-ul la raspunsul precedent era urmat imediat de cererea noua
    // v1.6: plus pauzele VCDS: prima cerere la >= 130 ms dupa A1, apoi >= 50 ms dupa ultimul cadru de pe canal
    { uint32_t mingap = ch.t3ms > TP20_REQ_GAP_MS ? ch.t3ms : TP20_REQ_GAP_MS;
      uint32_t gap = millis() - ch.last_act; if (gap < mingap) delay(mingap - gap);
      int32_t w = (int32_t)(ch.ready_at - millis()); if (w > 0) delay((uint32_t)w); }
    size_t sent = 0; bool firstf = true;
    while (sent < rlen) {
        uint8_t f[8]; size_t cap = firstf ? 5 : 7; size_t n = rlen - sent; if (n > cap) n = cap;
        bool last = (sent + n >= rlen);
        f[0] = (last ? 0x10 : 0x20) | (ch.tx_seq & 0x0F);
        size_t off = 1;
        if (firstf) { f[1] = (uint8_t)(rlen >> 8); f[2] = (uint8_t)rlen; off = 3; }
        memcpy(f + off, req + sent, n);
        if (!tp_tx(ch.tx_id, f, off + n)) { err = "TX kwp esuat"; return false; }
        ch.tx_seq = (ch.tx_seq + 1) & 0x0F;
        sent += n; firstf = false;
    }
    twai_message_t r;
    // asteapta ACK (B) de la ECU; unele module incep direct raspunsul
    uint32_t t0 = millis(); bool have_pre = false;
    while (millis() - t0 < TP20_ACK_TO) {
        if (!tp_rx(ch.rx_id, TP20_ACK_TO, r)) break;
        if ((r.data[0] & 0xF0) == 0xB0) break;
        if ((r.data[0] & 0xF0) == 0x90) { err = "NAK TP2.0 0x" + String(r.data[0], HEX) + " (modulul astepta alt numar de secventa)"; return false; }   // v1.3
        if ((r.data[0] & 0xF0) <= 0x30) { have_pre = true; break; }
    }

    uint8_t buf[384]; size_t total = 0;
    uint8_t want = req[0] + 0x40;
    // pana la 3 mesaje: sarim peste cele straine (retransmisii intarziate ale tranzactiei anterioare) si peste 7F xx 78
    for (int tries = 0; tries < 3; tries++) {
        uint32_t to = (tries == 0) ? TP20_DATA_TO : TP20_PENDING_TO;
        if (!collect_msg(have_pre ? &r : nullptr, to, buf, sizeof(buf), &total)) {
            err = (total == 0 && tries == 0) ? "fara raspuns TP2.0" : ("raspuns TP2.0 incomplet (" + String(total) + ")");
            return false;
        }
        have_pre = false;
        if (total >= 3 && buf[0] == 0x7F && buf[1] == req[0] && buf[2] == 0x78) continue;   // responsePending: mai asteptam
        if (total >= 3 && buf[0] == 0x7F && buf[1] == req[0]) { g_last_nrc = buf[2]; err = "NRC 0x" + String(buf[2], HEX); *resp_len = 0; return false; }
        if (buf[0] == want) break;
        if (tries == 2) { err = "raspuns strain (0x" + String(buf[0], HEX) + " la SID 0x" + String(req[0], HEX) + ")"; return false; }
        // altfel: mesaj din alta tranzactie — il ignoram si mai citim unul
    }
    size_t n = total < resp_sz ? total : resp_sz;
    memcpy(resp, buf, n); *resp_len = n;
    keep_alive();
    return true;
}

// v1.5: NRC 0x21 busyRepeatRequest = „repeta cererea" (KWP2000) - ABS-ul Altea raspunde asa; inainte renuntam din prima.
bool tp20_kwp(const uint8_t *req, size_t rlen, uint8_t *resp, size_t resp_sz, size_t *resp_len, String &err) {
    g_busy_retries = 0;
    for (;;) {
        bool ok = tp20_kwp_once(req, rlen, resp, resp_sz, resp_len, err);
        if (ok || g_last_nrc != 0x21 || g_busy_retries >= 5) {
            if (!ok && g_last_nrc == 0x21) err += " (busy, repetat de " + String(g_busy_retries) + " ori)";
            return ok;
        }
        g_busy_retries++;
        // v1.6: pauza 250 ms FARA rafala de A3 (VCDS trimite A3 la ~570 ms; A3 la 20 ms => ABS inchidea canalul cu A8)
        { twai_message_t r; tp_rx(ch.rx_id, 250, r); }   // golim eventualele retransmisii ale raspunsului 7F
    }
}

// ---- helpers ----
bool tp20_ident(uint8_t logical, String &partno, String &err) {
    bool opened_here = !ch.open;
    if (!tp20_open(logical, err)) return false;
    uint8_t req[2] = {0x1A, 0x9B}; uint8_t resp[64]; size_t rl;
    bool ok = tp20_kwp(req, 2, resp, sizeof(resp), &rl, err);
    if (ok && rl >= 2 && resp[0] == 0x5A) { partno = ""; for (size_t i = 2; i < rl; i++) { char c = resp[i]; partno += (c >= 32 && c < 127) ? c : '.'; } partno.trim(); }
    else if (ok) { ok = false; err = "raspuns 1A neasteptat"; }
    if (opened_here) tp20_close();
    return ok;
}
bool tp20_group(uint8_t logical, uint8_t grp, String &decoded, String &err) {
    bool opened_here = !ch.open;
    if (!tp20_open(logical, err)) return false;
    uint8_t req[2] = {0x21, grp}; uint8_t resp[64]; size_t rl;
    bool ok = tp20_kwp(req, 2, resp, sizeof(resp), &rl, err);
    if (ok && rl >= 2 && resp[0] == 0x61) {
        decoded = "grp " + String(grp) + ":";
        for (size_t i = 2; i + 2 < rl; i += 3) {
            const char *u; double v = vag_decode(resp[i], resp[i + 1], resp[i + 2], &u);
            decoded += " [" + String(v, 2) + u + " (F" + resp[i] + " a" + resp[i + 1] + " b" + resp[i + 2] + ")]";
        }
    } else if (ok) { ok = false; err = "raspuns 21 neasteptat (0x" + String(resp[0], HEX) + ")"; }
    if (opened_here) tp20_close();
    return ok;
}
bool tp20_dtc(uint8_t logical, String &codes, String &err) {
    bool opened_here = !ch.open;
    if (!tp20_open(logical, err)) return false;
    uint8_t req[4] = {0x18, 0x02, 0xFF, 0x00}; uint8_t resp[128]; size_t rl;
    bool ok = tp20_kwp(req, 4, resp, sizeof(resp), &rl, err);
    if (ok && rl >= 2 && resp[0] == 0x58) {
        int n = resp[1]; codes = String(n) + " coduri:";
        for (int k = 0; k < n && (size_t)(2 + 3 * k + 1) < rl; k++) {
            uint16_t code = (resp[2 + 3 * k] << 8) | resp[3 + 3 * k];
            codes += " " + String(code) + "/st" + String(resp[4 + 3 * k], HEX);
        }
    } else if (ok) { ok = false; err = "raspuns 18 neasteptat"; }
    if (opened_here) tp20_close();
    return ok;
}

// ---- READEPROM: citire memorie ca VAG CAN PRO (docs/research/sniff_20260921_bcm/README.md) ----
bool tp20_read_mem(uint8_t logical, uint32_t addr, size_t len, uint8_t *out, size_t out_sz, Tp20MemResult &r, String &err, size_t chunk) {
    r.partno = ""; r.seed = r.key = 0; r.blockmax = 0; r.got = 0; r.ms = 0; r.seed_retry = false; r.stop_nrc = 0; r.stop_addr = 0;
    if (chunk == 0 || chunk > 0xFFF) chunk = 256;
    uint32_t t0 = millis();
    if (g_sec_locked) { err = "cheie refuzata anterior de modul; REBOOT inainte de o noua incercare"; return false; }
    if (len == 0 || len > out_sz || addr > 0xFFFFFF) { err = "adresa/lungime invalida"; return false; }
    if (ch.open) tp20_close();
    g_trn = 0; g_tr_t0 = millis(); g_tr_on = true;                // v1.3: trace de la setup pana la A8
    if (!tp20_open_ex(logical, false, err)) { g_tr_on = false; return false; }      // VCP nu trimite 10 89
    uint8_t resp[384]; size_t rl; bool ok = false;
    do {
        // identificare (informativ; nu blocheaza)
        uint8_t q1[3] = {0x22, 0xF1, 0x87};
        String e1;
        if (tp20_kwp(q1, 3, resp, sizeof(resp), &rl, e1) && rl > 3 && resp[0] == 0x62) {
            for (size_t i = 3; i < rl; i++) { char c = resp[i]; r.partno += (c >= 32 && c < 127) ? c : '.'; }
            r.partno.trim();
        }
        // 27 03 -> seed
        uint8_t q2[2] = {0x27, 0x03};
        bool s_ok = tp20_kwp(q2, 2, resp, sizeof(resp), &rl, err);
        if (!s_ok && err.startsWith("fara raspuns")) {              // v1.3: cererea de seed e inofensiva - o mai incercam o data
            delay(250); String e2;
            s_ok = tp20_kwp(q2, 2, resp, sizeof(resp), &rl, e2);
            if (!s_ok) err = err + "; reincercare: " + e2; else r.seed_retry = true;
        }
        if (!s_ok) { err = "27 03: " + err; break; }
        if (rl < 6 || resp[0] != 0x67 || resp[1] != 0x03) { err = "27 03: raspuns neasteptat"; break; }
        r.seed = ((uint32_t)resp[2] << 24) | ((uint32_t)resp[3] << 16) | ((uint32_t)resp[4] << 8) | resp[5];
        r.key = r.seed + (uint32_t)TP20_SEC_KEY_ADD;
        // 27 04 + cheie (6 octeti = 2 cadre)
        uint8_t q3[6] = {0x27, 0x04, (uint8_t)(r.key >> 24), (uint8_t)(r.key >> 16), (uint8_t)(r.key >> 8), (uint8_t)r.key};
        if (!tp20_kwp(q3, 6, resp, sizeof(resp), &rl, err)) {
            uint8_t nrc = tp20_last_nrc();
            if (nrc == 0x35 || nrc == 0x36 || nrc == 0x37) g_sec_locked = true;   // nu mai insistam pana la reboot
            err = "27 04: " + err + (g_sec_locked ? " (cheia gresita/blocata: TP20_SEC_KEY_ADD de reverificat cu un nou sniff VCP; REBOOT pt reincercare)" : "");
            break;
        }
        if (rl < 2 || resp[0] != 0x67 || resp[1] != 0x04) { err = "27 04: raspuns neasteptat"; break; }
        // 10 86 (sesiune dezvoltare)
        uint8_t q4[2] = {0x10, 0x86};
        if (!tp20_kwp(q4, 2, resp, sizeof(resp), &rl, err)) { err = "10 86: " + err; break; }
        if (rl < 2 || resp[0] != 0x50) { err = "10 86: raspuns neasteptat"; break; }
        // v1.4: pe BUCATI — 35 <addr24> 00 <len24> / 36.. / 37, repetat in aceeasi sesiune pana la len sau pana refuza modulul
        uint8_t q6[1] = {0x36}, q7[1] = {0x37};
        while (r.got < len) {
            uint32_t a = addr + r.got; size_t want = len - r.got; if (want > chunk) want = chunk;
            uint8_t q5[8] = {0x35, (uint8_t)(a >> 16), (uint8_t)(a >> 8), (uint8_t)a, 0x00,
                             (uint8_t)(want >> 16), (uint8_t)(want >> 8), (uint8_t)want};
            String e5;
            if (!tp20_kwp(q5, 8, resp, sizeof(resp), &rl, e5)) {
                r.stop_nrc = tp20_last_nrc() ? tp20_last_nrc() : 0xFF; r.stop_addr = a;
                if (r.got == 0) { err = "35 requestUpload la 0x" + String(a, HEX) + ": " + e5; }
                else err = "oprit la 0x" + String(a, HEX) + ": " + e5;
                break;
            }
            if (rl < 1 || resp[0] != 0x75) { err = "35: raspuns neasteptat"; r.stop_nrc = 0xFE; r.stop_addr = a; break; }
            r.blockmax = rl >= 2 ? resp[1] : 0;
            size_t end = r.got + want; int guard = (int)(want / 8) + 4; bool cut = false;
            while (r.got < end && guard-- > 0) {
                if (!tp20_kwp(q6, 1, resp, sizeof(resp), &rl, err)) { err = "36 transferData la 0x" + String(addr + r.got, HEX) + ": " + err; cut = true; break; }
                if (rl < 1 || resp[0] != 0x76) { err = "36: raspuns neasteptat"; cut = true; break; }
                if (rl == 1) { cut = true; break; }                       // modulul nu mai are date
                size_t n = rl - 1; if (r.got + n > end) n = end - r.got;
                memcpy(out + r.got, resp + 1, n); r.got += n;
            }
            { String e7; tp20_kwp(q7, 1, resp, sizeof(resp), &rl, e7); }   // 37 transferExit, best effort, dupa fiecare bucata
            if (cut) { if (!r.stop_nrc) { r.stop_nrc = tp20_last_nrc() ? tp20_last_nrc() : 0xFE; r.stop_addr = addr + r.got; } break; }
        }
        ok = (r.got > 0);
    } while (false);
    // 82 stopCommunication + A8 — intotdeauna
    { uint8_t q8[1] = {0x82}; String e8; size_t rl8; tp20_kwp(q8, 1, resp, sizeof(resp), &rl8, e8); }
    tp20_close();
    g_tr_on = false;
    r.ms = millis() - t0;
    if (ok && r.got < len && !err.length()) err = "partial: " + String(r.got) + "/" + String(len) + " octeti";
    return ok;
}

// ---- formule VAG (research 04 §2.1) ----
double vag_decode(uint8_t f, uint8_t a, uint8_t b, const char **unit) {
    const char *u = ""; double v = 0;
    switch (f) {
        case 1: v = a * b / 5.0; u = "rpm"; break;
        case 2: v = 0.002 * a * b; u = "%"; break;
        case 3: v = 0.002 * a * b; u = "deg"; break;
        case 4: v = fabs(b - 127.0) * 0.01 * a; u = "degATDC"; break;
        case 5: v = a * (b - 100.0) * 0.1; u = "C"; break;
        case 6: case 21: v = 0.001 * a * b; u = "V"; break;
        case 7: v = 0.01 * a * b; u = "km/h"; break;
        case 8: v = 0.1 * a * b; u = ""; break;
        case 9: v = (b - 127.0) * 0.02 * a; u = "deg"; break;
        case 10: v = b; u = (b == 0) ? "COLD" : "WARM"; break;
        case 11: v = 0.0001 * a * (b - 128.0) + 1.0; u = ""; break;
        case 12: v = 0.001 * a * b; u = "ohm"; break;
        case 13: v = (b - 127.0) * 0.001 * a; u = "mm"; break;
        case 14: v = 0.005 * a * b; u = "bar"; break;
        case 15: v = 0.01 * a * b; u = "ms"; break;
        case 18: v = a * b / 25.6; u = "mbar"; break;
        case 19: v = 0.01 * a * b; u = "l"; break;
        case 20: v = a * (b - 128.0) / 128.0; u = "%"; break;
        case 22: v = 0.001 * a * b; u = "ms"; break;
        case 23: v = b / 256.0 * a; u = "%"; break;
        case 24: v = 0.001 * a * b; u = "A"; break;
        case 25: v = b * 1.421 + a / 182.0; u = "g/s"; break;
        case 26: case 28: v = b - a; u = "C"; break;
        case 27: v = fabs(b - 128.0) * 0.01 * a; u = "deg"; break;
        case 33: v = a ? 100.0 * b / a : 100.0 * b; u = "%"; break;
        case 34: v = (b - 128.0) * 0.01 * a; u = "kW"; break;
        case 35: v = 0.01 * a * b; u = "l/h"; break;
        case 36: v = a * b; u = "km"; break;
        case 48: case 54: v = a * 256.0 + b; u = ""; break;
        case 65: v = 0.01 * a * (b - 127.0); u = "mm"; break;
        // grupuri MK60 (F67-70 posibil cu semn -> a/b brute in output)
        case 67: v = 640.0 * a + 2.5 * b; u = "deg"; break;
        case 68: v = (a * 256.0 + b) / 7.365; u = "deg/s"; break;
        case 69: v = (a * 256.0 + b) * 0.3254; u = "bar"; break;
        case 70: v = (a * 256.0 + b) * 0.192; u = "m/s2"; break;
        default: v = a * 256.0 + b; u = "?"; break;
    }
    if (unit) *unit = u;
    return v;
}
