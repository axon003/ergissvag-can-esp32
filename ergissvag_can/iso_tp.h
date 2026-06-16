/**
 * iso_tp.h — ISO 15765-2 (ISO-TP) transport layer for OBD-II / UDS over CAN
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-05-23 — implementare SF + FF/CF/FC pentru multi-ECU broadcast collection
 *
 * Pattern de utilizare:
 *   - iso_tp_send_sf(0x7DF, req, len) → trimite Single Frame ISO-TP (OBD broadcast)
 *   - In handler frame primit: iso_tp_on_frame(canId, data, dlc)
 *   - In loop periodic: iso_tp_drain(cb) si iso_tp_tick(now_ms)
 *   - cb primeste mesajul complet (assembled) per ECU
 *
 * Suporta pana la ISO_TP_MAX_SESSIONS sesiuni simultane (broadcast → multi-ECU).
 */
#ifndef ERGCAN_ISO_TP_H
#define ERGCAN_ISO_TP_H

#include <Arduino.h>

#define ISO_TP_MAX_SESSIONS  8       // suficient pentru 0x7E8..0x7EF (8 ECU)
#define ISO_TP_MSG_MAX       256     // mesaj max assembled (suficient pentru DTC list lung, VIN, etc.)
#define ISO_TP_TIMEOUT_MS    1500    // session timeout daca nu mai vin CF-uri

struct IsoTpRxSession {
    uint32_t src_id;            // CAN ID sursa (ex: 0x7E8)
    uint32_t fc_can_id;         // CAN ID pe care trimitem Flow Control (de obicei src_id - 8)
    uint8_t  buf[ISO_TP_MSG_MAX];
    size_t   total_len;         // lungime mesaj declarata in First Frame
    size_t   got_len;           // octeti primiti pana acum
    uint8_t  next_seq;          // urmatorul sequence number asteptat in CF
    uint32_t last_rx_ms;
    bool     active;
    bool     complete;
};

// Trimite ISO-TP Single Frame (1-7 bytes data). Returneaza true daca transmis OK pe TWAI bus.
bool iso_tp_send_sf(uint32_t can_id, const uint8_t *data, size_t len);

// Procesare frame CAN primit. Returneaza sesiunea daca s-a completat un mesaj (caller va consuma).
IsoTpRxSession* iso_tp_on_frame(uint32_t can_id, const uint8_t *data, size_t dlc);

// Drain mesaje complete; cb se cheama pentru fiecare. Apoi sesiunea se elibereaza.
typedef void (*iso_tp_msg_cb)(uint32_t src_id, const uint8_t *data, size_t len);
void iso_tp_drain(iso_tp_msg_cb cb);

// Cleanup sesiuni timeout. Apel periodic in loop.
void iso_tp_tick(uint32_t now_ms);

#endif
