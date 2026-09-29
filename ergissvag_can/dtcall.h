/**
 * dtcall.h — DTCALL: lista de instalare a gateway-ului (22 04A1) + erori pe fiecare modul (ergCAN 2.4.0)
 *
 *   Logica PURA (fara Arduino/CAN) ca sa poata fi testata pe PC (tests/test_dtcall.cpp):
 *     - parsare lista de instalare: 64 x [adresa][pozitie][flags][bus]  (flags bit0 = codat, bit2 = ARE ERORI)
 *     - tabela adresa VCDS -> transport (UDS 0x7xx sau TP2.0) + nume
 *     - formatare coduri: KWP VAG (numar 16 biti, 5 cifre) si UDS (3 octeti J2012 + FTB)
 *   Transportul (TP2.0/UDS) si JSON-ul sunt in proto.cpp.
 *
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-09-19 — creat (fw 2.4.0): cerinta user „DTCALL: citeste gw inst list si erorile pe care exista, atasate automat la sub"
 */
#ifndef ERGCAN_DTCALL_H
#define ERGCAN_DTCALL_H

#include <stdint.h>
#include <stddef.h>

#define GW_MAX_MODS     64
#define GW_FLAG_CODED   0x01
#define GW_FLAG_FAULT   0x04   // CONFIRMAT de user (09-17): modulele 25/09/56/19 apareau cu erori in VCDS

struct GwMod { uint8_t adr; uint8_t pos; uint8_t flags; uint8_t bus; };

// d = octetii DUPA `62 04 A1`. Sare peste intrarile goale (adr==0 sau flags==0). Intoarce nr. de module.
int dtcall_parse_install(const uint8_t *d, size_t n, GwMod *out, int max);

// Cum se ajunge la modul. TP2.0: adresa logica = coloana `pos` din lista de instalare (04->0x13, 44->0x09, 15->0x05 ... �
// VERIFICAT pe captura reala GWIL), deci NU se tine in tabela. UDS pe CAN: `uds_req` (0 = necunoscut/inexistent).
// pref_uds = 1 => se incearca INTAI UDS 19 02 FF, apoi TP2.0; 0 => intai TP2.0, apoi UDS (daca uds_req != 0).
// tp_uds = 1 => pe canalul TP2.0 serviciile sunt UDS (19 02 FF), ca la gateway; 0 => KWP2000 (18 02 FF 00).
// conf = 1 => transport CONFIRMAT pe masina (MODPROBE 09-16); 0 => presupus (ID UDS ghicit), se incearca alternativa.
struct ModRoute { uint8_t adr; const char *name; uint16_t uds_req; uint8_t pref_uds; uint8_t tp_uds; uint8_t conf; uint8_t tp; };
// v1.3 (fw 2.5.8): `tp` = adresa logica TP2.0 pt MODPROBE (0 = nu se sondeaza pe TP2.0; DTCALL ia `pos` din lista de instalare).
// Tabela vine din PROFILUL vehiculului (cheia "mods" din veh/<model>.json); tabela compilata = doar rezerva.
const ModRoute *dtcall_route(uint8_t adr);      // nullptr daca modulul nu e in tabela
int  dtcall_load_routes(const char *profile_json); // "mods":[{adr,name,uds,pref_uds,tp_uds,conf,tp}] -> nr module; -1 = profil fara "mods"
bool dtcall_routes_from_profile();                 // true = tabela activa vine din profil
int  dtcall_route_count();
const ModRoute *dtcall_route_at(int i);
const char *dtcall_mod_name(uint8_t adr);       // "Motor" ... sau "?" (nu e in tabela)

// coduri de eroare
void dtcall_fmt_vag(uint16_t code, char out[8]);              // KWP: 5 cifre zecimale, ex "01314", "16684"
bool dtcall_vag_to_pcode(uint16_t code, char out[8]);         // 16384+PPPP (cifre zecimale) -> "P0300"; false daca nu e cod P
void dtcall_fmt_uds(const uint8_t *d3, char out[12]);         // 3 octeti UDS: "P0300-00" (J2012 + FTB hex)

#endif
