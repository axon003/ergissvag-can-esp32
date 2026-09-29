// test_params.cpp — test pe PC pentru catalogul de parametri cu nume + bitmap-ul PID-urilor suportate (ergCAN 2.6.0)
// compilare (MSYS2 mingw64): g++ -std=c++17 -O1 -I../ergissvag_can test_params.cpp ../ergissvag_can/params.cpp ../ergissvag_can/obd_pid.cpp -o test_params.exe
#include "params.h"
#include "obd_pid.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <set>
#include <string>

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL linia %d: %s\n", __LINE__, #c); } } while (0)

int main() {
    // --- catalogul: nume unice, PID-uri unice, fiecare PID are lungime si formula ---
    std::set<std::string> names; std::set<int> pids;
    for (size_t i = 0; i < std_param_count(); i++) {
        const StdParam *p = std_param_at(i);
        CHECK(p != nullptr);
        CHECK(strlen(p->n) > 0 && strlen(p->n) < 16);            // incape in OSCH_KEY_MAX
        CHECK(names.insert(p->n).second);
        CHECK(pids.insert(p->pid).second);
        CHECK(p->per_ms >= 50);
        int L = obd_pid_len(p->pid);
        if (L <= 0) printf("  fara lungime: %s pid %02X\n", p->n, p->pid);
        CHECK(L > 0);
        uint8_t d[8] = {0x80, 0x40, 0x20, 0x10, 0, 0, 0, 0}; double v; const char *u;
        bool dec = obd_pid_decode(p->pid, d, (size_t)L, &v, &u);
        if (!dec) printf("  fara formula: %s pid %02X\n", p->n, p->pid);
        CHECK(dec);
    }
    CHECK(std_param_at(std_param_count()) == nullptr);

    // --- cheile pe care le foloseste F16 de la 1.75 incoace trebuie sa ramana, cu aceleasi PID-uri ---
    struct { const char *n; int pid; } F16[] = {
        {"rpm", 0x0C}, {"speed", 0x0D}, {"map", 0x0B}, {"throttle", 0x11}, {"load", 0x04}, {"coolant", 0x05},
        {"intake", 0x0F}, {"timing", 0x0E}, {"ambient", 0x46}, {"runtime", 0x1F}, {"battery", 0x42}, {"baro", 0x33},
        {"catTemp", 0x3C}, {"oilTemp", 0x5C}, {"stft", 0x06}, {"ltft", 0x07}, {"absLoad", 0x43}, {"relThrottle", 0x45},
        {"pedal", 0x49}, {"cmdThrottle", 0x4C}, {"purge", 0x2E}, {"fuelRail", 0x23}, {"equivRatio", 0x44}, {"o2v", 0x15},
    };
    for (auto &f : F16) {
        const StdParam *p = std_param_find(f.n);
        CHECK(p != nullptr);
        if (p) CHECK(p->pid == f.pid);
    }
    CHECK(std_param_find("espYawRate") == nullptr);      // de producator: doar din profil
    CHECK(std_param_find("") == nullptr);
    CHECK(std_param_find(nullptr) == nullptr);
    CHECK(std_param_find("RPM") == nullptr);             // numele sunt exacte
    CHECK(strcmp(std_param_name_for_pid(0x0C), "rpm") == 0);
    CHECK(std_param_name_for_pid(0xFE) == nullptr);

    // --- PID-urile noi ---
    { uint8_t d[4] = {0x00, 0x30, 0x0B, 0x58}; double v; const char *u;      // 0x00300B58 = 3148632 -> 314863.2 km
      CHECK(obd_pid_len(0xA6) == 4);
      CHECK(obd_pid_decode(0xA6, d, 4, &v, &u) && fabs(v - 314863.2) < 0.01 && !strcmp(u, "km"));
      CHECK(!obd_pid_decode(0xA6, d, 3, &v, &u)); }
    { uint8_t d[2] = {0x7D, 0x00}; double v; const char *u;
      CHECK(obd_pid_decode(0x62, d, 1, &v, &u) && v == 0.0);                 // 125 = 0 %
      uint8_t e[2] = {0x00, 0xC8};
      CHECK(obd_pid_decode(0x63, e, 2, &v, &u) && v == 200.0); }
    CHECK(obd_pid_len(0x64) == -1);                       // nedeclarat: nu intra in cereri multi-PID
    CHECK(obd_pid_len(0x0C) == 2);                        // tabelul vechi neatins

    // --- bitmap: cat timp nu e cunoscut, totul e permis ---
    param_sup_reset();
    CHECK(!param_sup_known());
    CHECK(param_sup_has(0x5C));
    // raspuns invalid la 01 00 => ramane necunoscut, nimic blocat
    CHECK(param_sup_feed(0x00, nullptr, 0) == 0);
    param_sup_done();
    CHECK(!param_sup_known());
    CHECK(param_sup_has(0x0C));

    // Bitmap de exemplu: bloc 00 = BE 1F A8 13 (are 0x20), bloc 20 = 90 05 B0 15 (are 0x40), bloc 40 = FA DC 20 00 (fara 0x60)
    param_sup_reset();
    { uint8_t b0[4] = {0xBE, 0x1F, 0xA8, 0x13};
      CHECK(param_sup_feed(0x00, b0, 4) == 0x20);
      CHECK(!param_sup_known());                          // lantul nu s-a incheiat
      uint8_t b2[4] = {0x90, 0x05, 0xB0, 0x15};
      CHECK(param_sup_feed(0x20, b2, 4) == 0x40);
      uint8_t b4[4] = {0xFA, 0xDC, 0x20, 0x00};
      CHECK(param_sup_feed(0x40, b4, 4) == 0);
      CHECK(param_sup_known());
      CHECK(param_sup_has(0x0C));                         // rpm
      CHECK(param_sup_has(0x0D));                         // viteza
      CHECK(param_sup_has(0x05));                         // antigel
      CHECK(!param_sup_has(0x0A));                        // presiune combustibil: bit 0 in BE 1F..
      CHECK(!param_sup_has(0x5C));                        // temperatura ulei: blocul 40 se opreste inainte
      CHECK(!param_sup_has(0xA6));                        // odometru: blocul A0 nici nu exista
      CHECK(param_sup_has(0x42));                         // tensiune
      CHECK(param_sup_count() > 10); }

    // lant rupt la mijloc (blocul 20 nu raspunde): ce s-a citit ramane valabil
    param_sup_reset();
    { uint8_t b0[4] = {0xBE, 0x1F, 0xA8, 0x13};
      CHECK(param_sup_feed(0x00, b0, 4) == 0x20);
      param_sup_done();
      CHECK(param_sup_known());
      CHECK(param_sup_has(0x0C));
      CHECK(!param_sup_has(0x42)); }

    // baza gresita (nu e multiplu de 0x20) e ignorata
    param_sup_reset();
    { uint8_t b[4] = {0xFF, 0xFF, 0xFF, 0xFF};
      CHECK(param_sup_feed(0x0C, b, 4) == 0);
      param_sup_done();
      CHECK(!param_sup_known()); }

    printf("%s: %d verificari, %d esecuri\n", fails ? "ESEC" : "OK", checks, fails);
    return fails ? 1 : 0;
}
