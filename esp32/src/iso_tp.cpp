/**
 * iso_tp.cpp — implementare ISO 15765-2
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-05-23 — creat initial
 */
#include "iso_tp.h"
#include "driver/twai.h"
#include <string.h>

// Bus state flag — definit in main.cpp; cand false (bus_off / recovery in progress),
// blocheaza orice TX pentru a preveni acumulare de erori si BUS_OFF loop.
extern volatile bool g_can_bus_ok;

static IsoTpRxSession g_sessions[ISO_TP_MAX_SESSIONS];

// Helper: send raw CAN frame (8 bytes, standard ID)
static bool send_can_frame(uint32_t id, const uint8_t *data, size_t dlc) {
    if (!g_can_bus_ok) return false;   // bus absent — no point in TX
    twai_message_t m;
    memset(&m, 0, sizeof(m));
    m.identifier = id;
    m.data_length_code = dlc;
    memcpy(m.data, data, dlc);
    return twai_transmit(&m, pdMS_TO_TICKS(100)) == ESP_OK;
}

bool iso_tp_send_sf(uint32_t can_id, const uint8_t *data, size_t len) {
    if (len < 1 || len > 7) return false;
    if (!g_can_bus_ok) return false;
    uint8_t frame[8];
    frame[0] = (uint8_t)len;                              // PCI: high nibble 0, low nibble = length
    memcpy(frame + 1, data, len);
    for (size_t i = 1 + len; i < 8; i++) frame[i] = 0xAA; // padding standard CAN_DBC ISO15765
    return send_can_frame(can_id, frame, 8);
}

static IsoTpRxSession* find_or_create_session(uint32_t src_id) {
    for (int i = 0; i < ISO_TP_MAX_SESSIONS; i++) {
        if (g_sessions[i].active && g_sessions[i].src_id == src_id) return &g_sessions[i];
    }
    for (int i = 0; i < ISO_TP_MAX_SESSIONS; i++) {
        if (!g_sessions[i].active) {
            memset(&g_sessions[i], 0, sizeof(g_sessions[i]));
            g_sessions[i].src_id     = src_id;
            // Standard OBD/UDS: ECU response ID = request ID + 8 (7E8 raspuns la 7E0)
            // Pentru reverse (Flow Control catre ECU): src_id - 8
            g_sessions[i].fc_can_id  = (src_id >= 8) ? (src_id - 8) : src_id;
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
    uint8_t pci = data[0];
    uint8_t pci_type = (pci >> 4) & 0x0F;

    if (pci_type == 0) {
        // === Single Frame ===
        uint8_t len = pci & 0x0F;
        if (len < 1 || len > 7 || dlc < 1u + len) return nullptr;
        IsoTpRxSession *s = find_or_create_session(can_id);
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
        IsoTpRxSession *s = find_or_create_session(can_id);
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
