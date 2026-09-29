/**
 * params.h — parametri cu NUME (ergCAN 2.6): aplicatia cere „rpm", „coolant", „odoKm"..., iar placa stie de unde ii ia
 *
 *   Pana la 2.5.x clientul (F16) trimitea in abonament PID-ul numeric, iar valorile de producator (ESP, volan)
 *   le cerea cu DID-uri scrise in aplicatie — adica aplicatia era legata de masina. Din 2.6 clientul trimite doar
 *   NUMELE; rezolvarea se face pe placa, in ordinea:
 *     1. profilul vehiculului (cheia "dids": DID-uri de producator, cu scara lor);
 *     2. valorile „de vehicul" citite de firmware (km, litri, generator, ulei — campurile fixe din profil);
 *     3. catalogul STANDARD de mai jos (SAE J1979 mode 01) — acelasi pe orice masina cu OBD-II pe CAN.
 *   Un nume pe care nu-l stie nimeni e raportat „none" si nu produce trafic pe magistrala.
 *
 *   C++ portabil (fara Arduino): testat pe PC in tests/test_params.cpp.
 *
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-09-29 — creat initial (fw 2.6.0; cerinta user: F16 cere parametri dupa nume, nu dupa PID/DID)
 */
#ifndef ERGCAN_PARAMS_H
#define ERGCAN_PARAMS_H

#include <cstdint>
#include <cstddef>

#define PARAMS_PROTO   3      // versiunea protocolului JSON anuntata in `hello` si in raspunsul la `sub`

struct StdParam {
    const char *n;        // numele cerut de client (= cheia din push-ul {"t":"d","k":...})
    uint8_t     pid;      // PID mode 01
    const char *u;        // unitate, informativ
    uint16_t    per_ms;   // perioada implicita cand clientul nu da `hz`
};

size_t          std_param_count();
const StdParam *std_param_at(size_t i);
const StdParam *std_param_find(const char *name);        // nullptr = numele nu e in catalogul standard
const char     *std_param_name_for_pid(uint8_t pid);     // nullptr = PID fara nume in catalog

// ---- PID-uri suportate de ECU (raspunsurile la 01 00 / 20 / 40 / ... ) ----
void    param_sup_reset();
// raspuns la `01 <base>`: 4 octeti de bitmap. Intoarce urmatoarea baza de interogat sau 0 cand s-a terminat.
uint8_t param_sup_feed(uint8_t base, const uint8_t *d, size_t len);
void    param_sup_done();                                 // inchide citirea (chiar daca lantul s-a rupt)
bool    param_sup_known();
bool    param_sup_has(uint8_t pid);                       // true si cat timp bitmap-ul e necunoscut
size_t  param_sup_count();

#endif
