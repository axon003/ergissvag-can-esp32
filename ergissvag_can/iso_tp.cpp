/**
 * iso_tp.cpp — implementare ISO 15765-2
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.4
 * @changes
 *   v1.4 2026-09-29 — fw 2.6.0: adresare pe 29 de biti (ID > 0x7FF => cadru extended; raspunsuri 18DAF1xx acceptate,
 *                     Flow Control pe 18DAxxF1). Pe 11 biti nimic nu se schimba.
 *   v1.0 2026-05-23 — creat initial
 *   v1.1 2026-09-13 — respecta g_can_tx_enabled (mod listen-only la cald) + contoare TX (g_tx_queued/g_tx_refused)
 *   v1.3 2026-09-15 — ergCAN 2.0: sesiuni doar pt raspunsuri de la ID-uri cunoscute (7E8..7EF + iso_tp_allow),
 *                     Flow Control pe cererea perechii (77E→714), nu mecanic src-8; nu mai confunda cadre TP 2.0
 *   v1.2 2026-09-14 — twai_transmit NON-BLOCANT (timeout 0): fara ACK pe bus coada se umplea si fiecare cerere
 *                     obd1 de la F16 (10+/s) bloca loop-ul 100 ms → UDP discovery nu mai pleca → F16 pierdea
 *                     ergCAN + CAN-A. Parametru single_shot (TWAI_MSG_FLAG_SS) pt probe: fara retransmisie.
 */
#include "iso_tp.h"
#include "driver/twai.h"
#include <string.h>

// Bus state flag — definit in main.cpp; cand false (bus_off / recovery in progress),
// blocheaza orice TX pentru a preveni acumulare de erori si BUS_OFF loop.
extern volatile bool g_can_bus_ok;
// false in modul "listen" (zero emisie pe bus) — definit in .ino
extern volatile bool g_can_tx_enabled;
// contoare pt "stat" — definite in .ino
extern uint32_t g_tx_queued, g_tx_refused;

static IsoTpRxSession g_sessions[ISO_TP_MAX_SESSIONS];

// v2.0: raspunsuri ISO-TP acceptate DOAR de la ID-uri cunoscute (altfel cadrele TP 2.0 de pe 0x3xx / 0x7xx dinamic
// erau luate drept First Frame si firmware-ul trimitea Flow Control nesolicitat). Pereche raspuns → cerere pt FC.
struct IsoTpPair { uint32_t resp, req; };
static IsoTpPair g_pairs[16];
static int       g_npairs = 0;

// v1.1 (fw 2.5.7): scoate o pereche (UDSSCAN parcurge ~220 ID-uri; tabela ar umple-se)
void iso_tp_forget(uint32_t resp_id) {
    for (int i = 0; i < g_npairs; i++) if (g_pairs[i].resp == resp_id) { g_pairs[i] = g_pairs[--g_npairs]; return; }
}

void iso_tp_allow(uint32_t resp_id, uint32_t req_id) {
    for (int i = 0; i < g_npairs; i++) if (g_pairs[i].resp == resp_id) { g_pairs[i].req = req_id; return; }
    if (g_npairs < (int)(sizeof(g_pairs) / sizeof(g_pairs[0]))) g_pairs[g_npairs++] = {resp_id, req_id};
}

bool iso_tp_is_obd_resp(uint32_t id) {
    return (id >= 0x7E8 && id <= 0x7EF) || ((id & 0x1FFFFF00UL) == 0x18DAF100UL);
}
uint32_t iso_tp_req_for(uint32_t id) {
    if (id >= 0x7E8 && id <= 0x7EF) return id - 8;
    if ((id & 0x1FFFFF00UL) == 0x18DAF100UL) return 0x18DA00F1UL | ((id & 0xFFUL) << 8);
    return 0;
}

static bool allowed_resp(uint32_t id, uint32_t *fc_to) {
    if (iso_tp_is_obd_resp(id)) { *fc_to = iso_tp_req_for(id); return true; }
    for (int i = 0; i < g_npairs; i++) if (g_pairs[i].resp == id) { *fc_to = g_pairs[i].req; return true; }
    return false;
}

// Helper: send raw CAN frame (8 bytes, standard ID)
static bool send_can_frame(uint32_t id, const uint8_t *data, size_t dlc, bool single_shot = false) {
    if (!g_can_bus_ok || !g_can_tx_enabled) { g_tx_refused++; return false; }   // bus absent / listen-only
    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.identifier = id;
    m.data_length_code = dlc;
    if (single_shot) m.flags = TWAI_MSG_FLAG_SS;
    if (id > 0x7FF) m.flags |= TWAI_MSG_FLAG_EXTD;        // v1.4: adresare pe 29 de biti
    memcpy(m.data, data, dlc);
    // Timeout 0: coada plina = refuz imediat (contorizat), NU blocam loop-ul (UDP discovery, TCP, senzori).
    bool ok = twai_transmit(&m, 0) == ESP_OK;
    if (ok) g_tx_queued++; else g_tx_refused++;
    return ok;
}

bool iso_tp_send_sf(uint32_t can_id, const uint8_t *data, size_t len, bool single_shot) {
    if (len < 1 || len > 7) return false;
    if (!g_can_bus_ok) return false;
    uint8_t frame[8];
    frame[0] = (uint8_t)len;                              // PCI: high nibble 0, low nibble = length
    memcpy(frame + 1, data, len);
    for (size_t i = 1 + len; i < 8; i++) frame[i] = 0xAA; // padding standard CAN_DBC ISO15765
    return send_can_frame(can_id, frame, 8, single_shot);
}

static IsoTpRxSession* find_or_create_session(uint32_t src_id, uint32_t fc_to) {
    for (int i = 0; i < ISO_TP_MAX_SESSIONS; i++) {
        if (g_sessions[i].active && g_sessions[i].src_id == src_id) return &g_sessions[i];
    }
    for (int i = 0; i < ISO_TP_MAX_SESSIONS; i++) {
        if (!g_sessions[i].active) {
            memset(&g_sessions[i], 0, sizeof(g_sessions[i]));
            g_sessions[i].src_id     = src_id;
            // v2.0: Flow Control pe ID-ul cererii din perechea cunoscuta (7E8→7E0, 77E→714), nu mecanic src-8
            g_sessions[i].fc_can_id  = fc_to;
            g_sessions[i].active     = true;
            g_sessions[i].last_rx_ms = millis();
            return &g_sessions[i];
        }
    }
    return nullptr; // pool full — drop frame
}

// Trimite Flow Control: continue, block size = 0 (continuu), ST_min = 0
static void send_flow_control(uint32_t target_id) {
    uint8_t fc[8] = { 0x30, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };
    send_can_frame(target_id, fc, 8);
}

IsoTpRxSession* iso_tp_on_frame(uint32_t can_id, const uint8_t *data, size_t dlc) {
    if (dlc < 1) return nullptr;
    uint32_t fc_to;
    if (!allowed_resp(can_id, &fc_to)) return nullptr;   // v2.0: doar raspunsuri de la ID-uri cunoscute
    uint8_t pci = data[0];
    uint8_t pci_type = (pci >> 4) & 0x0F;

    if (pci_type == 0) {
        // === Single Frame ===
        uint8_t len = pci & 0x0F;
        if (len < 1 || len > 7 || dlc < 1u + len) return nullptr;
        IsoTpRxSession *s = find_or_create_session(can_id, fc_to);
        if (!s) return nullptr;
        memcpy(s->buf, data + 1, len);
        s->total_len   = len;
        s->got_len     = len;
        s->complete    = true;
        s->last_rx_ms  = millis();
        return s;
    }
    else if (pci_type == 1) {
        // === First Frame === (PCI 1L LL → 12-bit total length)
        if (dlc < 8) return nullptr;
        size_t total = ((size_t)(pci & 0x0F) << 8) | data[1];
        if (total > ISO_TP_MSG_MAX) total = ISO_TP_MSG_MAX;
        IsoTpRxSession *s = find_or_create_session(can_id, fc_to);
        if (!s) return nullptr;
        memcpy(s->buf, data + 2, 6);
        s->got_len    = 6;
        s->total_len  = total;
        s->next_seq   = 1;
        s->complete   = false;
        s->last_rx_ms = millis();
        send_flow_control(s->fc_can_id);
        return nullptr;
    }
    else if (pci_type == 2) {
        // === Consecutive Frame ===
        uint8_t seq = pci & 0x0F;
        IsoTpRxSession *s = nullptr;
        for (int i = 0; i < ISO_TP_MAX_SESSIONS; i++) {
            if (g_sessions[i].active && g_sessions[i].src_id == can_id) { s = &g_sessions[i]; break; }
        }
        if (!s || seq != s->next_seq) return nullptr;
        s->next_seq = (uint8_t)((s->next_seq + 1) & 0x0F);
        size_t copy = 7;
        if (dlc < 1u + copy) copy = dlc - 1;
        if (s->got_len + copy > s->total_len) copy = s->total_len - s->got_len;
        if (copy > 0) {
            memcpy(s->buf + s->got_len, data + 1, copy);
            s->got_len += copy;
        }
        s->last_rx_ms = millis();
        if (s->got_len >= s->total_len) {
            s->complete = true;
            return s;
        }
    }
    // PCI=3 (Flow Control inbound) — ignoram (suntem doar receiver pentru ISO-TP)
    return nullptr;
}

void iso_tp_drain(iso_tp_msg_cb cb) {
    for (int i = 0; i < ISO_TP_MAX_SESSIONS; i++) {
        if (g_sessions[i].active && g_sessions[i].complete) {
            if (cb) cb(g_sessions[i].src_id, g_sessions[i].buf, g_sessions[i].got_len);
            g_sessions[i].active   = false;
            g_sessions[i].complete = false;
        }
    }
}

void iso_tp_tick(uint32_t now_ms) {
    for (int i = 0; i < ISO_TP_MAX_SESSIONS; i++) {
        if (g_sessions[i].active && (now_ms - g_sessions[i].last_rx_ms) > ISO_TP_TIMEOUT_MS) {
            g_sessions[i].active = false;
        }
    }
}
