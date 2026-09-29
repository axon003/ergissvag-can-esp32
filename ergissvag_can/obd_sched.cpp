/**
 * obd_sched.cpp — planificator abonamente OBD mode 01 (vezi obd_sched.h)
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.1
 * @changes
 *   v1.1 2026-09-29 — fw 2.6.0: osch_req_id/osch_resp_id, osch_set_unsupported
 *   v1.0 2026-09-15 — creat initial
 */
#include "obd_sched.h"
#include "obd_pid.h"
#include <cstring>

// v1.1: doar declaratia (cfg.h trage Arduino.h, iar fisierul asta se compileaza si pe PC, in tests/)
extern long cfg_int(const char *key);

// v2.3.1: definite in ergissvag_can.ino — „magistrala e vie?” (acelasi criteriu cu flag-ul `can`).
extern bool can_is_live();

#define REQ_TIMEOUT_MS      100     // P2 nominal 50 ms; latenta masurata pe Altea: med 9 ms, p95 13 ms, max 30 ms
#define PENDING_TIMEOUT_MS  5000    // P2* dupa 0x78
#define RETRY_TX_MS         20      // coada TX plina → reincearca
#define MISS_LIMIT          3       // PID lipsa din raspuns de atatea ori → nesuportat
#define MULTI_FAIL_LIMIT    2       // cereri multi-PID esuate la rand → trecem pe cate unul
// v2.2.6: ...dar NU pentru totdeauna. Pana acum `multi_ok=false` ramanea asa pana la repornire, iar pe masina
// (`stat` → multi=0) se interoga cate un PID pe rand = „refresh lent". Motorul Altea accepta 6 PID-uri/cerere;
// esecurile sunt de obicei trecatoare (bus incarcat, ECU ocupat), deci reincercam periodic.
#define MULTI_RETRY_MS      60000
#define UNSUP_RETRY_MS      300000   // v2.2.9: la 5 min reincercam PID-urile marcate nesuportate (vezi osch_tick)
// v2.2.9: un oneshot (cerere legacy `obd1`) traia pana cand o cerere care il continea primea raspuns sau expira.
// Daca TWAI nu poate transmite deloc (contact pus, motor oprit = fara ACK pe bus), cererea NU pleaca, deci
// oneshot-ul nu expira niciodata: F16 in fallback polling (100 ms) umplea tabela cu pana la 24 de itemi morti.
#define ONESHOT_TTL_MS      3000

static OschHooks g_h;
static OschItem  g_items[OSCH_MAX_ITEMS];
static OschStats g_st;
static uint32_t  g_req_id = 0x7E0, g_resp_id = 0x7E8;
static bool      g_enabled = true;

static struct {
    bool     active;
    uint8_t  pids[OSCH_MAX_PER_REQ];
    uint8_t  n;
    uint32_t sent_ms;
    uint32_t deadline;
} g_out;
static uint32_t g_retry_at = 0;
static uint8_t  g_multi_fail = 0;
static uint32_t g_multi_retry_at = 0;   // v2.2.6: cand reincercam multi-PID dupa ce a fost oprit (0 = nu e oprit)
static uint32_t g_unsup_retry_at = 0;   // v2.2.9: cand reincercam PID-urile marcate nesuportate

void osch_init(const OschHooks &h) {
    g_h = h;
    memset(g_items, 0, sizeof(g_items));
    memset(&g_st, 0, sizeof(g_st));
    memset(&g_out, 0, sizeof(g_out));
    g_st.multi_ok = true;
    g_multi_fail = 0;
    g_multi_retry_at = 0;
    g_retry_at = 0;
}

void osch_set_ids(uint32_t req_id, uint32_t resp_id) { g_req_id = req_id; g_resp_id = resp_id; }
uint32_t osch_req_id() { return g_req_id; }
uint32_t osch_resp_id() { return g_resp_id; }
void osch_set_unsupported(size_t i, bool unsupported) {
    if (i >= OSCH_MAX_ITEMS || !g_items[i].active || g_items[i].oneshot) return;
    g_items[i].unsupported = unsupported;
    if (!unsupported) { g_items[i].n_miss = 0; }
}
void osch_set_enabled(bool en) { g_enabled = en; if (!en) g_out.active = false; }
bool osch_enabled() { return g_enabled; }
size_t osch_capacity() { return OSCH_MAX_ITEMS; }
const OschItem *osch_item(size_t i) { return i < OSCH_MAX_ITEMS ? &g_items[i] : nullptr; }
const OschStats &osch_stats() { return g_st; }
bool osch_busy() { return g_out.active; }

int osch_add(const char *key, uint8_t pid, uint32_t period_ms, uint8_t owner, bool oneshot, uint32_t now) {
    // oneshot duplicat (acelasi PID deja asteptat) → nu mai adaugam (F16 1.75 trimite aceleasi obd1 la 100 ms)
    for (int i = 0; i < OSCH_MAX_ITEMS; i++) {
        OschItem &it = g_items[i];
        if (!it.active) continue;
        if (oneshot && it.oneshot && it.pid == pid) return i;
        if (!oneshot && !it.oneshot && it.owner == owner && strncmp(it.key, key, OSCH_KEY_MAX) == 0) {
            it.pid = pid; it.period_ms = period_ms ? period_ms : 1000; it.next_due = now; it.unsupported = false;
            return i;
        }
    }
    for (int i = 0; i < OSCH_MAX_ITEMS; i++) {
        OschItem &it = g_items[i];
        if (it.active) continue;
        memset(&it, 0, sizeof(it));
        strncpy(it.key, key ? key : "", OSCH_KEY_MAX - 1);
        it.pid = pid;
        it.owner = owner;
        it.oneshot = oneshot;
        it.period_ms = oneshot ? 0 : (period_ms ? period_ms : 1000);
        it.next_due = now;
        it.active = true;
        return i;
    }
    return -1;
}

void osch_remove_owner(uint8_t owner) {
    for (auto &it : g_items) if (it.active && it.owner == owner) it.active = false;
}

void osch_clear() {
    for (auto &it : g_items) it.active = false;
    g_out.active = false;
}

// intarziere relativa: cat de depasit e termenul fata de perioada (oneshot = foarte urgent)
static double lateness(const OschItem &it, uint32_t now) {
    double late = (double)(int32_t)(now - it.next_due);
    if (it.oneshot) return 1e9 + late;
    return late / (double)(it.period_ms ? it.period_ms : 1);
}

static bool due(const OschItem &it, uint32_t now) {
    return it.active && !it.unsupported && (int32_t)(now - it.next_due) >= 0;
}

static void finish_items(uint32_t now) {
    // itemii din cererea fara raspuns: oneshot scosi; periodicii raman scadenti si se recer imediat
    // (timeout-ul de 100 ms limiteaza oricum ritmul cand motorul nu raspunde)
    (void)now;
    for (uint8_t k = 0; k < g_out.n; k++)
        for (auto &it : g_items)
            if (it.active && it.oneshot && it.pid == g_out.pids[k]) it.active = false;
}

void osch_on_message(uint32_t src_id, const uint8_t *pl, size_t len, uint32_t now) {
    if (!g_out.active || src_id != g_resp_id || !pl || len < 1) return;

    if (pl[0] == 0x7F) {
        if (len >= 3 && pl[1] == 0x01 && pl[2] == 0x78) {   // responsePending
            g_st.pending78++;
            g_out.deadline = now + PENDING_TIMEOUT_MS;
            return;
        }
        if (len >= 2 && pl[1] != 0x01) return;               // NRC pt alt serviciu
        g_st.nrc++;
        if (g_out.n > 1) {
            if (++g_multi_fail >= MULTI_FAIL_LIMIT) { g_st.multi_ok = false; g_multi_retry_at = now + MULTI_RETRY_MS; }
        } else if (g_out.n == 1) {
            for (auto &it : g_items)
                if (it.active && it.pid == g_out.pids[0]) {
                    if (it.oneshot) it.active = false;
                    else { it.unsupported = true; if (g_h.unsupported) g_h.unsupported(it, g_h.ctx); }
                }
        }
        g_out.active = false;
        finish_items(now);
        return;
    }
    if (pl[0] != 0x41) return;

    g_st.resp++;
    g_st.last_lat_ms = now - g_out.sent_ms;
    bool got[OSCH_MAX_PER_REQ] = {false};
    size_t i = 1;
    while (i < len) {
        uint8_t pid = pl[i];
        int L = obd_pid_len(pid);
        if (L < 0 || i + 1 + (size_t)L > len) break;
        for (uint8_t k = 0; k < g_out.n; k++) if (g_out.pids[k] == pid) got[k] = true;
        for (auto &it : g_items) {
            if (!it.active || it.pid != pid) continue;
            it.n_ok++;
            it.last_ok_ms = now;
            if (g_h.value) g_h.value(it, pl + i + 1, (size_t)L, now, g_h.ctx);
            if (it.oneshot) it.active = false;
            else {
                it.next_due += it.period_ms;
                if ((int32_t)(now - it.next_due) > (int32_t)it.period_ms) it.next_due = now + it.period_ms;  // nu recuperam restante
            }
        }
        i += 1 + (size_t)L;
    }
    if (g_out.n > 1) g_multi_fail = 0;
    for (uint8_t k = 0; k < g_out.n; k++) {
        if (got[k]) continue;
        for (auto &it : g_items) {
            if (!it.active || it.pid != g_out.pids[k]) continue;
            if (it.oneshot) { it.active = false; continue; }
            if (++it.n_miss >= MISS_LIMIT) { it.unsupported = true; if (g_h.unsupported) g_h.unsupported(it, g_h.ctx); }
            else it.next_due = now + it.period_ms;
        }
    }
    g_out.active = false;
}

void osch_tick(uint32_t now) {
    if (!g_enabled) return;
    // v2.2.6: multi-PID oprit dupa esecuri → il reincercam dupa MULTI_RETRY_MS (inainte ramanea oprit pana la reboot)
    if (!g_st.multi_ok && g_multi_retry_at && (int32_t)(now - g_multi_retry_at) >= 0) {
        g_st.multi_ok = true; g_multi_fail = 0; g_multi_retry_at = 0;
    }
    // v2.2.9: `unsupported` era DEFINITIV (se stergea doar la re-abonare). Un PID poate lipsi temporar —
    // motor oprit, ECU in alta sesiune, raspunsuri pierdute pe bus incarcat — si atunci ramanea scos pentru
    // totdeauna, iar setul intern (care nu se mai re-adauga) se golea incet. Le reincercam o data la UNSUP_RETRY_MS;
    // daca intr-adevar nu sunt suportate, se marcheaza la loc dupa MISS_LIMIT rateuri si costul e neglijabil.
    // v2.2.9: oneshot-urile mai vechi de ONESHOT_TTL_MS se sterg singure (vezi comentariul de la define).
    for (auto &it : g_items)
        if (it.active && it.oneshot && (int32_t)(now - it.next_due) > (int32_t)ONESHOT_TTL_MS) it.active = false;
    if (g_unsup_retry_at == 0) g_unsup_retry_at = now + UNSUP_RETRY_MS;
    else if ((int32_t)(now - g_unsup_retry_at) >= 0) {
        g_unsup_retry_at = now + UNSUP_RETRY_MS;
        for (auto &it : g_items)
            if (it.active && it.unsupported) { it.unsupported = false; it.n_miss = 0; it.next_due = now; }
    }
    if (g_out.active) {
        if ((int32_t)(now - g_out.deadline) < 0) return;
        g_st.timeout++;
        if (g_out.n > 1 && ++g_multi_fail >= MULTI_FAIL_LIMIT) { g_st.multi_ok = false; g_multi_retry_at = now + MULTI_RETRY_MS; }
        g_out.active = false;
        finish_items(now);
        return;
    }
    if (g_retry_at && (int32_t)(now - g_retry_at) < 0) return;
    g_retry_at = 0;

    // v2.3.1: cu magistrala moarta (contact luat) nu mai emitem CERERI NOI — pe 2.3.0, in 934 s cu
    // `can`=0 au iesit 699 cereri si 0 raspunsuri. Gate-ul e pus AICI, dupa tratarea cererii in zbor
    // si dupa intretinerea de mai sus: un timeout in curs se inchide normal, oneshot-urile expira,
    // PID-urile „nesuportate” se reincearca la timp. Revenirea e pasiva (lastFrameMs din bucla RX).
    if (cfg_int("can.idle_gate") && !can_is_live()) return;

    // cel mai intarziat item scadent
    int best = -1; double bl = -1e18;
    for (int i = 0; i < OSCH_MAX_ITEMS; i++) {
        if (!due(g_items[i], now)) continue;
        double l = lateness(g_items[i], now);
        if (l > bl) { bl = l; best = i; }
    }
    if (best < 0) return;

    uint8_t pids[OSCH_MAX_PER_REQ];
    uint8_t n = 0;
    pids[n++] = g_items[best].pid;
    bool best_known = obd_pid_len(g_items[best].pid) > 0;

    // completam cererea cu alti itemi scadenti sau aproape scadenti (in ultima jumatate de perioada)
    if (g_st.multi_ok && best_known) {
        for (int pass = 0; pass < 2 && n < OSCH_MAX_PER_REQ; pass++) {
            for (int i = 0; i < OSCH_MAX_ITEMS && n < OSCH_MAX_PER_REQ; i++) {
                const OschItem &it = g_items[i];
                if (!it.active || it.unsupported || obd_pid_len(it.pid) <= 0 || it.pid == 0x00) continue;
                bool dup = false;
                for (uint8_t k = 0; k < n; k++) if (pids[k] == it.pid) dup = true;
                if (dup) continue;
                int32_t until = (int32_t)(it.next_due - now);
                bool ok = pass == 0 ? until <= 0 : (!it.oneshot && until <= (int32_t)(it.period_ms / 2));
                if (ok) pids[n++] = it.pid;
            }
        }
    }

    uint8_t req[1 + OSCH_MAX_PER_REQ];
    req[0] = 0x01;
    memcpy(req + 1, pids, n);
    if (!g_h.send || !g_h.send(g_req_id, req, 1 + n, g_h.ctx)) {
        g_st.tx_refused++;
        g_retry_at = now + RETRY_TX_MS;
        return;
    }
    g_st.req++;
    g_out.active = true;
    memcpy(g_out.pids, pids, n);
    g_out.n = n;
    g_out.sent_ms = now;
    g_out.deadline = now + REQ_TIMEOUT_MS;
    for (uint8_t k = 0; k < n; k++)
        for (auto &it : g_items) if (it.active && it.pid == pids[k]) it.n_req++;
}
