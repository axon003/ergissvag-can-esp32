/**
 * dtcall.cpp — logica pura DTCALL (vezi dtcall.h)
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.4
 * @changes
 *   v1.4 2026-09-23 — CORECTIE: 0x711 (5K0953234) = 25 Imobilizator, nu 16 Volan (log VCDS Altea); 16 fara ruta UDS.
 *   v1.3 2026-09-23 — fw 2.5.8: tabela de rute vine din PROFILUL vehiculului (cheia "mods"); cea compilata ramane rezerva.
 *   v1.2 2026-09-23 — fw 2.5.8: rute UDS din UDSSCAN pe Altea (09-23 14:46, 5 module): 16 Volan = 0x711 (5K0953234),
 *                     52 Usa pasager = 0x74B (1T0959702AB); 04/09/15/25 NU raspund UDS in 0x700..0x7DF => uds_req 0 (raman TP2.0).
 *   v1.1 2026-09-23 — fw 2.5.7: 08 Clima = UDS 0x746 (rsp 0x7B0), 42 Usa sofer = UDS 0x74A (rsp 0x7B4) — VAZUTE in sniff VCDS 09-23
 *                     (VCDS incearca TP2.0 0x2C/0x22 de 10 ori, fara raspuns, apoi UDS). Erau presupuse 0x70E / 0x717.
 *   v1.0 2026-09-19 — creat (fw 2.4.0)
 */
#include "dtcall.h"
#include <stdio.h>
#include <string.h>
#include "jsonlite.h"

int dtcall_parse_install(const uint8_t *d, size_t n, GwMod *out, int max) {
    int k = 0;
    for (size_t i = 0; i + 4 <= n && k < max; i += 4) {
        if (d[i] == 0 || d[i + 2] == 0) continue;     // adresa 0 sau flags 0 = intrare goala
        out[k].adr = d[i]; out[k].pos = d[i + 1]; out[k].flags = d[i + 2]; out[k].bus = d[i + 3];
        k++;
    }
    return k;
}

// Harta Altea XL (PQ35), din Auto-Scan VCDS + MODPROBE pe masina (docs/research/sniff_vcds_20260917.md).
// conf=1 doar unde raspunsul a fost VAZUT pe masina: 01 (UDS 7E0), 17 (UDS 714), 15/44/19 (TP2.0 05/09/1F), 03 (TP2.0 03, „busy").
// Restul sunt presupuneri de pornire: daca nu raspund, se incearca alternativa, iar rezultatul „nc" apare in raport.
static const ModRoute ROUTES[] = {
    {0x01, "Motor",          0x7E0, 1, 0, 1},
    {0x02, "Cutie",          0x7E1, 1, 0, 0},
    {0x03, "ABS",            0,     0, 0, 1},
    {0x04, "Unghi volan",    0,     0, 0, 0},
    {0x08, "Clima",          0x746, 1, 0, 1},
    {0x09, "Electrica",      0,     0, 0, 1},
    {0x15, "Airbag",         0,     0, 0, 1},
    {0x16, "Volan",          0,     0, 0, 0},
    {0x17, "Bord",           0x714, 1, 0, 1},
    {0x19, "Gateway",        0,     0, 1, 1},
    {0x25, "Imobilizator",   0x711, 1, 0, 1},
    {0x37, "Navigatie",      0,     0, 0, 0},
    {0x42, "Usa sofer",      0x74A, 1, 0, 1},
    {0x44, "Servodirectie",  0,     0, 0, 1},
    {0x46, "Confort",        0,     0, 0, 0},
    {0x52, "Usa pasager",    0x74B, 1, 0, 1},
    {0x56, "Radio",          0,     0, 0, 0},
};

// v1.3: rute din profil ("mods" in veh/<model>.json) — daca exista, inlocuiesc complet tabela compilata
#define DTC_PROF_MAX 32
static ModRoute g_pr[DTC_PROF_MAX];
static char     g_prn[DTC_PROF_MAX][16];
static int      g_npr = -1;          // -1 = profilul nu are "mods" => tabela compilata

int dtcall_load_routes(const char *json) {
    g_npr = -1;
    if (!json) return -1;
    const char *k = strstr(json, "\"mods\"");
    if (!k) return -1;
    const char *arr = strchr(k, '[');
    if (!arr) return -1;
    size_t pos = 1; char obj[224]; int n = 0; long v;
    while (n < DTC_PROF_MAX && jl_next_object(arr, &pos, obj, sizeof(obj))) {
        if (!jl_int(obj, "adr", &v) || v <= 0 || v > 0xFF) continue;
        ModRoute &r = g_pr[n];
        r.adr = (uint8_t)v;
        g_prn[n][0] = 0; jl_str(obj, "name", g_prn[n], sizeof(g_prn[n]));
        r.name     = g_prn[n][0] ? g_prn[n] : "?";
        r.uds_req  = jl_int(obj, "uds", &v) ? (uint16_t)v : 0;
        r.pref_uds = jl_int(obj, "pref_uds", &v) ? (uint8_t)(v != 0) : (uint8_t)(r.uds_req != 0);
        r.tp_uds   = jl_int(obj, "tp_uds", &v) ? (uint8_t)(v != 0) : 0;
        r.conf     = jl_int(obj, "conf", &v) ? (uint8_t)(v != 0) : 0;
        r.tp       = jl_int(obj, "tp", &v) ? (uint8_t)v : 0;
        n++;
    }
    g_npr = n;
    return n;
}
bool dtcall_routes_from_profile() { return g_npr >= 0; }
int  dtcall_route_count() { return g_npr >= 0 ? g_npr : (int)(sizeof(ROUTES) / sizeof(ROUTES[0])); }
const ModRoute *dtcall_route_at(int i) {
    if (i < 0 || i >= dtcall_route_count()) return nullptr;
    return g_npr >= 0 ? &g_pr[i] : &ROUTES[i];
}
const ModRoute *dtcall_route(uint8_t adr) {
    for (int i = 0; i < dtcall_route_count(); i++) { const ModRoute *r = dtcall_route_at(i); if (r->adr == adr) return r; }
    return nullptr;
}
const char *dtcall_mod_name(uint8_t adr) { const ModRoute *r = dtcall_route(adr); return r ? r->name : "?"; }

void dtcall_fmt_vag(uint16_t code, char out[8]) { snprintf(out, 8, "%05u", (unsigned)code); }

bool dtcall_vag_to_pcode(uint16_t code, char out[8]) {
    if (code < 16384) return false;
    unsigned n = (unsigned)code - 16384u;
    if (n > 9999) return false;
    snprintf(out, 8, "P%04u", n);      // ex 16684 -> P0300 (Ross-Tech: 16684/P0300/000768)
    return true;
}

void dtcall_fmt_uds(const uint8_t *d3, char out[12]) {
    static const char T[4] = {'P', 'C', 'B', 'U'};
    snprintf(out, 12, "%c%u%X%02X-%02X", T[(d3[0] >> 6) & 3], (unsigned)((d3[0] >> 4) & 3), (unsigned)(d3[0] & 0x0F), (unsigned)d3[1], (unsigned)d3[2]);
}
