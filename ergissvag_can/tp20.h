/**
 * tp20.h — VW TP 2.0 + KWP2000 pe TWAI (ergCAN 2.1) — citire module non-OBD (bord 17, electrica 09, gateway 19, ABS 03...)
 *   Model BLOCANT per-tranzactie (apelat din consola, pompeaza RX cu timeout) — usor de reglat pe masina.
 *   Adrese LOGICE TP2.0 (NU VCDS): 01 motor, 02 cutie, 03 ABS, 05 airbag, 07 bord, 09 EPS, 13 unghi volan,
 *   1F gateway, 20 electrica centrala, 21 confort, 2C clima. Setup pe 0x200, canal dinamic (0x300/0x740...).
 *   SIGURANTA: doar SID de CITIRE (10/1A/21/18/22/3E) + pentru READEPROM 27 (seed/key), 35/36/37 (upload = ECU -> tester)
 *   si 82 (stopCommunication). Scrieri (2E/3B/34/3D), rutine (31) si basic settings REFUZATE.
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.4
 * @changes
 *   v1.4 2026-09-23 — fw 2.5.6: repetare la NRC 0x21 / setup D7-D8; tp20_trace_begin/end + tp20_busy_retries (comanda TPQ).
 *   v1.0 2026-09-15 — creat (2.1), dupa docs/research/04_vw_tp20_kwp2000.md. NEVALIDAT PE MASINA.
 *   v1.1 2026-09-21 — fw 2.5.0: cereri KWP MULTI-FRAME (27 04 + cheie, 35 + adresa), tp20_open_ex() fara 10 89,
 *                     tp20_read_mem() = secventa VAG CAN PRO „citire EEPROM" (docs/research/sniff_20260921_bcm/README.md).
 */
#ifndef ERGCAN_TP20_H
#define ERGCAN_TP20_H

#include <Arduino.h>

// deschide canal catre adresa logica; intoarce false + err. La succes retine canalul intern.
bool tp20_open(uint8_t logical, String &err);                       // + StartDiagnosticSession 10 89 (ca VCDS)
bool tp20_open_ex(uint8_t logical, bool start_session, String &err); // start_session=false: doar canalul (ca VAG CAN PRO)
void tp20_close();
bool tp20_is_open();
uint8_t tp20_addr();

// trimite o cerere KWP (bytes, pana la 40 = multi-frame) si primeste raspunsul complet (fara 0x78 intermediare).
// resp/resp_sz de apelant. La NRC intoarce false cu err="NRC 0x.." si *resp_len=0 (codul NRC in tp20_last_nrc()).
bool tp20_kwp(const uint8_t *req, size_t rlen, uint8_t *resp, size_t resp_sz, size_t *resp_len, String &err);
uint8_t tp20_last_nrc();

// helper-e uzuale (deschid+inchid singure daca canalul nu e deja deschis)
bool tp20_ident(uint8_t logical, String &partno, String &err);           // 1A 9B -> ASCII part number
bool tp20_group(uint8_t logical, uint8_t grp, String &decoded, String &err); // 21 grp -> valori decodate VAG
bool tp20_dtc(uint8_t logical, String &codes, String &err);              // 18 02 FF 00 -> coduri VAG

// READEPROM (fw 2.5.0): exact secventa VAG CAN PRO pe BCM: canal (fara 10 89) -> 22 F187 (part) -> 27 03 seed ->
// 27 04 key(seed+TP20_SEC_KEY_ADD) -> 10 86 -> 35 <addr24> 00 <len24> -> 36 (repetat) -> 37 -> 82 -> A8.
// Doar citire (upload = modulul trimite). Canalul se inchide intotdeauna la iesire. MANUAL, pe loc (nu in mers).
struct Tp20MemResult {
    String   partno;     // 22 F187 (gol daca nu raspunde)
    uint32_t seed, key;  // u32
    uint8_t  blockmax;   // din 75 <blockmax>
    size_t   got;        // octeti primiti efectiv
    uint32_t ms;         // durata totala
    bool     seed_retry; // v1.2: 27 03 a mers doar la a doua incercare
    uint8_t  stop_nrc;   // v1.3: NRC la care s-a oprit citirea pe bucati (0 = tot ce s-a cerut; 0x31 = margine memorie); 0xFF = fara raspuns
    uint32_t stop_addr;  // v1.3: adresa la care modulul a refuzat (valabil daca stop_nrc != 0)
};
// v1.3: `chunk` = octeti per 35 requestUpload; citirea continua pe bucati in ACEEASI sesiune pana la len sau pana modulul refuza
// (NRC in r.stop_nrc / r.stop_addr). Intoarce true daca s-a citit macar un octet.
bool tp20_read_mem(uint8_t logical, uint32_t addr, size_t len, uint8_t *out, size_t out_sz, Tp20MemResult &r, String &err, size_t chunk = 256);
bool tp20_sec_locked();   // true = o cheie a fost refuzata (NRC 35/36/37) => nu mai incercam pana la reboot

// v1.2 (fw 2.5.2): TRACE cadre TP2.0 in timpul tp20_read_mem(): toate cadrele TX/RX de pe canal (setup 0x200/0x2xx + date),
// ca sa vedem exact ce a raspuns modulul (ex. NAK, retransmisie, nimic). Se goleste la fiecare READEPROM.
#define TP20_TRACE_MAX 96
struct Tp20Frame { uint32_t t; uint16_t id; uint8_t tx, dlc, d[8]; };   // t = ms de la inceputul citirii; tx=1 trimis de noi
size_t tp20_trace_count();                   // v1.3: ring — ultimele TP20_TRACE_MAX cadre, in ordine cronologica
const Tp20Frame *tp20_trace_get(size_t i);   // nullptr daca i >= count
void tp20_trace_begin();                     // v1.4: porneste trace-ul (golit) - si in afara READEPROM
void tp20_trace_end();
uint8_t tp20_busy_retries();                 // v1.4: repetari NRC 0x21 in ultima tp20_kwp

// decodare formula VAG (F, a, b) -> valoare+unitate (din research 04 §2.1)
double vag_decode(uint8_t f, uint8_t a, uint8_t b, const char **unit);

#endif
