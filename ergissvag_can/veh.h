/**
 * veh.h — profilul vehiculului (ergCAN 2.3): ce e specific masinii sta INTR-UN PROFIL, nu in cod
 *
 *   Pana la 2.2.9 bordul PQ35 era hardcodat in proto.cpp (0x714/0x77E, DID 2203 pentru km, 2206/2212 pentru
 *   litri) si bitrate-ul in config.h. Acum toate astea vin dintr-un profil:
 *     - implicit: profilul compilat (PQ35) => comportament identic cu 2.2.9 daca nu se face nimic;
 *     - `VEH GET <model>` / `SET mode <model>` aduce profilul de pe server si il salveaza in LittleFS (/veh.json);
 *     - la pornire, daca exista /veh.json, el castiga in fata celui compilat.
 *   Catalogul de modele sta pe server (cell/ergcan/veh/<model>.json) — o masina noua nu cere firmware nou.
 *
 *   PID-urile mode 01 (rpm, viteza, temperaturi...) sunt OBD-II standard, deci NU fac parte din profil:
 *   merg pe orice masina. Profilul acopera doar „extra"-ul de producator + bitrate-ul.
 *
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.3
 * @changes
 *   v1.3 2026-09-29 — fw 2.6.0: profil GENERIC — cheia "auto":1 = placa isi gaseste singura viteza magistralei
 *                     (500k/250k) si adresarea (11/29 de biti) prin proba OBD `01 00`; rezultatul se aplica peste
 *                     `bitrate` din profil (veh_set_detected). Pentru profilele fara "auto" nimic nu se schimba.
 *   v1.0 2026-09-16 — creat initial (cerinta user: „sa fie independent de masina, SET mode pq35")
 *   v1.2 2026-09-21 — fw 2.5.2: KM din ECU MOTOR (ekm_req/ekm_did/ekm_fmt, PQ35: 0x7E0 22 121C u32, cum citeste VCDS) —
 *                     read_km() il incearca INTAI, bordul (km_did, u16x10) ramane rezerva. Format nou VEH_KM_U32.
 *   v1.1 2026-09-17 — DID-uri de pe ECU MOTOR (gasite cu SNIFF pe VCDS): generator load/performance, temperatura si
 *                     nivel ulei. `eng_req`=0 sau DID=0 => valoarea nu se citeste.
 */
#ifndef ERGCAN_VEH_H
#define ERGCAN_VEH_H

#include <Arduino.h>

// format kilometraj (octetii de date din raspunsul 62 <did>)
#define VEH_KM_NONE    0
#define VEH_KM_U16X10  1    // 2 octeti, valoarea x10 (PQ35: 0x7AAE = 31406 -> 314060 km)
#define VEH_KM_U24     2    // 3 octeti, km direct
#define VEH_KM_U16     3    // 2 octeti, km direct
#define VEH_KM_U32     4    // 4 octeti, km direct (v1.2: ECU motor PQ35 DID 121C = 00 04 CB 80)

// format nivel combustibil
#define VEH_FUEL_NONE  0
#define VEH_FUEL_U8L   1    // 1 octet = litri
#define VEH_FUEL_U8PCT 2    // 1 octet = procent din rezervor

struct VehProfile {
    char     model[20];
    char     name[48];
    uint32_t bitrate;        // 500000 / 250000 / 125000 / 100000
    uint16_t cluster_req;    // ID cerere bord (0 = masina nu are bord interogabil => fara km/litri)
    uint16_t resp_off;       // offset raspuns pt ID-uri in afara 0x7E0-0x7E7 (VAG: 0x6A, 0x714 -> 0x77E)
    uint16_t km_did;
    uint8_t  km_fmt;
    uint16_t fuel_did;       // DID principal
    uint16_t fuel_alt;       // DID de rezerva (0 = niciunul)
    uint8_t  fuel_fmt;
    // v1.1: DID-uri UDS pe ECU motor (formate fixe, documentate in docs/research/sniff_vcds_20260917.md)
    uint16_t eng_req;        // ID cerere ECU motor (0x7E0); 0 = fara valori de motor pe UDS
    uint16_t genload_did;    // u8 x 100/256 = % („Generator: load signal")
    uint16_t genperf_did;    // u8 x 100/256 = % („Generator performance")
    uint16_t oilt_did;       // u8 - 60 = °C
    uint16_t oill_did;       // u16 / 16 = mm; 0x0780 (120,0 mm) = plafon cu motorul pornit => respins
    // v1.2: KM din ECU motor (sursa PRIMARA; bordul = rezerva). 0 = nu se incearca.
    uint16_t ekm_req;        // 0x7E0
    uint16_t ekm_did;        // 0x121C
    uint8_t  ekm_fmt;        // VEH_KM_U32 / U24
    uint16_t ver;
    bool     from_file;      // true = din /veh.json, false = profilul compilat
    uint8_t  autodet;        // v1.3: 1 = profil generic, viteza + adresarea se detecteaza pe masina
};

const VehProfile &veh();                       // initializare lenesa (citeste /veh.json la prima folosire)
uint32_t veh_bitrate();                        // folosit de twaiStart()
// v1.3: rezultatul autodetectiei (0 = renunta la el si revino la valoarea din profil)
void     veh_set_detected(uint32_t bitrate, bool ext29);
bool     veh_detected(uint32_t *bitrate, bool *ext29);
uint32_t veh_resp_id(uint32_t req);            // regula de raspuns UDS, dupa profil

bool   veh_fetch(const char *model, String &err);   // GET de pe server -> /veh.json -> aplica
bool   veh_clear(String &err);                      // sterge /veh.json -> revine la profilul compilat
bool   veh_list(String &out, String &err);          // lista modelelor de pe server
String veh_status();                                // o linie pentru consola / STAT

#endif
