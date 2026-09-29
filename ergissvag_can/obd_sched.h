/**
 * obd_sched.h — planificator abonamente OBD mode 01 (ergCAN 2.0)
 *   O singura cerere in lucru pe bus (motorul raspunde la una pe rand — masurat 15.09: rafala F16 pierdea 66-71%).
 *   Cerere fizica pe 7E0 → raspuns 7E8, pana la 6 PID-uri intr-o cerere (acceptat de motorul Altea), revenire la
 *   cate un PID daca ECU refuza, 0x78 → asteapta pana la 5 s, PID-uri nesuportate scoase, echitate dupa intarziere.
 *   C++ portabil (fara Arduino): testat pe PC in tests/test_sched.cpp.
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.1
 * @changes
 *   v1.1 2026-09-29 — fw 2.6.0: osch_req_id()/osch_resp_id() (ECU-ul poate fi altul decat 7E0/7E8 dupa autodetectie),
 *                     osch_set_unsupported() (PID absent din bitmap-ul ECU-ului: nu-l mai cerem pe ghicite)
 *   v1.0 2026-09-15 — creat initial
 */
#ifndef ERGCAN_OBD_SCHED_H
#define ERGCAN_OBD_SCHED_H

#include <cstdint>
#include <cstddef>

// v2.2.9: 48 -> 64. Setul intern complet (26, din 2.2.8) + abonamentul F16 (24) = 50 > 48, deci tabela se
// umplea si ultimele cereri ale clientului erau respinse tacut (`osch_add` -> -1, doar `skipped` in raspuns).
#define OSCH_MAX_ITEMS     64
#define OSCH_MAX_PER_REQ   6
#define OSCH_KEY_MAX       16

struct OschItem {
    char     key[OSCH_KEY_MAX];
    uint8_t  pid;
    uint8_t  owner;           // 0 = intern (VIN/probe), 1..n = client TCP/serial
    bool     active;
    bool     oneshot;         // cerere legacy (obd1) sau citire unica: dispare dupa raspuns/timeout
    bool     unsupported;
    uint32_t period_ms;
    uint32_t next_due;
    uint32_t n_req, n_ok, n_miss;
    uint32_t last_ok_ms;
};

struct OschStats {
    uint32_t req, resp, timeout, nrc, pending78, tx_refused;
    uint32_t last_lat_ms;
    bool     multi_ok;
};

struct OschHooks {
    // trimite payload ISO-TP Single Frame (ex. {0x01,0x0C,0x0D}) pe can_id; false = coada plina / bus absent
    bool (*send)(uint32_t can_id, const uint8_t *payload, size_t len, void *ctx);
    // valoare primita pentru un item (octetii de date ai PID-ului, fara 41/PID)
    void (*value)(const OschItem &it, const uint8_t *data, size_t len, uint32_t now, void *ctx);
    // eveniment: un item a fost marcat nesuportat
    void (*unsupported)(const OschItem &it, void *ctx);
    void *ctx;
};

void osch_init(const OschHooks &h);
void osch_set_ids(uint32_t req_id, uint32_t resp_id);     // implicit 0x7E0 / 0x7E8
uint32_t osch_req_id();
uint32_t osch_resp_id();
void osch_set_unsupported(size_t i, bool unsupported);    // v1.1: marcat din bitmap-ul PID-urilor suportate
void osch_set_enabled(bool en);                           // false: nu trimite (listen / bus off)
bool osch_enabled();
int  osch_add(const char *key, uint8_t pid, uint32_t period_ms, uint8_t owner, bool oneshot, uint32_t now);
void osch_remove_owner(uint8_t owner);
void osch_clear();
size_t osch_capacity();
const OschItem *osch_item(size_t i);
const OschStats &osch_stats();
bool osch_busy();

// payload ISO-TP reasamblat, primit de la src_id
void osch_on_message(uint32_t src_id, const uint8_t *pl, size_t len, uint32_t now);
void osch_tick(uint32_t now);

#endif
