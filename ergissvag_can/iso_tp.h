/**
 * iso_tp.h — ISO 15765-2 (ISO-TP) transport layer for OBD-II / UDS over CAN
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.2
 * @changes
 *   v1.2 2026-09-29 — fw 2.6.0: adresare OBD pe 29 de biti (ISO 15765-4: cerere 18DB33F1 / 18DAxxF1, raspuns 18DAF1xx).
 *                     Un ID peste 0x7FF se trimite ca „extended"; iso_tp_is_obd_resp() / iso_tp_req_for().
 *   v1.1 2026-09-23 — iso_tp_forget() pt UDSSCAN (fw 2.5.7)
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

// Trimite ISO-TP Single Frame (1-7 bytes data). Returneaza true daca a intrat in coada TWAI (non-blocant).
// single_shot=true → TWAI_MSG_FLAG_SS: fara retransmisie daca nu da nimeni ACK (pt probe pe bus posibil mort).
bool iso_tp_send_sf(uint32_t can_id, const uint8_t *data, size_t len, bool single_shot = false);

// v2.0: inregistreaza o pereche raspuns→cerere (ex. 0x77E→0x714 bord UDS). 7E8..7EF sunt implicite (→ -8).
void iso_tp_allow(uint32_t resp_id, uint32_t req_id);
void iso_tp_forget(uint32_t resp_id);   // v1.1: scoate perechea (UDSSCAN)

// v1.2: raspuns OBD „legislat" = 0x7E8..0x7EF (11 biti) sau 0x18DAF1xx (29 de biti), indiferent de profil
bool     iso_tp_is_obd_resp(uint32_t can_id);
// v1.2: ID-ul cererii fizice pentru un raspuns OBD (7E8 -> 7E0, 18DAF110 -> 18DA10F1); 0 daca nu e raspuns OBD
uint32_t iso_tp_req_for(uint32_t resp_id);

// Procesare frame CAN primit. Returneaza sesiunea daca s-a completat un mesaj (caller va consuma).
IsoTpRxSession* iso_tp_on_frame(uint32_t can_id, const uint8_t *data, size_t dlc);

// Drain mesaje complete; cb se cheama pentru fiecare. Apoi sesiunea se elibereaza.
typedef void (*iso_tp_msg_cb)(uint32_t src_id, const uint8_t *data, size_t len);
void iso_tp_drain(iso_tp_msg_cb cb);

// Cleanup sesiuni timeout. Apel periodic in loop.
void iso_tp_tick(uint32_t now_ms);

#endif
