/**
 * params.cpp — catalogul standard de parametri cu nume + bitmap-ul PID-urilor suportate (vezi params.h)
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-09-29 — creat initial (fw 2.6.0)
 */
#include "params.h"
#include <cstring>

// Numele primelor 24 = exact cheile folosite de F16 pana la 1.83 (SUB_LIST), ca difuzarea sa ramana compatibila.
// Restul sunt tot PID-uri standard J1979; ce nu are masina se vede in bitmap si nu se mai interogheaza.
static const StdParam CAT[] = {
    {"rpm",         0x0C, "rpm",    200},
    {"speed",       0x0D, "km/h",   200},
    {"map",         0x0B, "kPa",    333},
    {"throttle",    0x11, "%",      333},
    {"load",        0x04, "%",      333},
    {"coolant",     0x05, "C",     1000},
    {"intake",      0x0F, "C",     2000},
    {"timing",      0x0E, "deg",   1000},
    {"ambient",     0x46, "C",     5000},
    {"runtime",     0x1F, "s",     2000},
    {"battery",     0x42, "V",     2000},
    {"baro",        0x33, "kPa",   5000},
    {"catTemp",     0x3C, "C",     5000},
    {"oilTemp",     0x5C, "C",     5000},
    {"stft",        0x06, "%",     1000},
    {"ltft",        0x07, "%",     1000},
    {"absLoad",     0x43, "%",     1000},
    {"relThrottle", 0x45, "%",     1000},
    {"pedal",       0x49, "%",      500},
    {"cmdThrottle", 0x4C, "%",     1000},
    {"purge",       0x2E, "%",     5000},
    {"fuelRail",    0x23, "kPa",   2000},
    {"equivRatio",  0x44, "lambda",1000},
    {"o2v",         0x15, "V",     1000},
    // --- in plus fata de F16 1.83 ---
    {"maf",         0x10, "g/s",   2000},
    {"fuelLevel",   0x2F, "%",     5000},
    {"fuelRate",    0x5E, "L/h",   2000},
    {"fuelPress",   0x0A, "kPa",   2000},
    {"egr",         0x2C, "%",     2000},
    {"stft2",       0x08, "%",     1000},
    {"ltft2",       0x09, "%",     1000},
    {"catTemp2",    0x3D, "C",     5000},
    {"throttleB",   0x47, "%",     1000},
    {"pedal2",      0x4A, "%",      500},
    {"ethanol",     0x52, "%",    30000},
    {"hybridBatt",  0x5B, "%",     5000},
    {"torqueDem",   0x61, "%",      500},
    {"torqueAct",   0x62, "%",      500},
    {"torqueRef",   0x63, "Nm",   30000},
    {"distMil",     0x21, "km",   60000},
    {"distClr",     0x31, "km",   60000},
    {"timeMil",     0x4D, "min",  60000},
    {"timeClr",     0x4E, "min",  60000},
    {"warmups",     0x30, "",     60000},
    {"odoKm",       0xA6, "km",   60000},
};
#define CAT_N (sizeof(CAT) / sizeof(CAT[0]))

size_t std_param_count() { return CAT_N; }
const StdParam *std_param_at(size_t i) { return i < CAT_N ? &CAT[i] : nullptr; }

const StdParam *std_param_find(const char *name) {
    if (!name || !*name) return nullptr;
    for (size_t i = 0; i < CAT_N; i++) if (strcmp(CAT[i].n, name) == 0) return &CAT[i];
    return nullptr;
}

const char *std_param_name_for_pid(uint8_t pid) {
    for (size_t i = 0; i < CAT_N; i++) if (CAT[i].pid == pid) return CAT[i].n;
    return nullptr;
}

// ---- bitmap PID-uri suportate ----
static uint8_t g_sup[32];
static bool    g_sup_known = false;
static bool    g_sup_fed   = false;     // s-a primit macar blocul 01 00 (altfel „cunoscut" ar insemna „nimic suportat")

void param_sup_reset() { memset(g_sup, 0, sizeof(g_sup)); g_sup_known = false; g_sup_fed = false; }

static void sup_set(uint8_t pid) { g_sup[pid >> 3] |= (uint8_t)(1u << (pid & 7)); }
static bool sup_get(uint8_t pid) { return (g_sup[pid >> 3] >> (pid & 7)) & 1u; }

uint8_t param_sup_feed(uint8_t base, const uint8_t *d, size_t len) {
    if (!d || len < 4 || (base & 0x1F) != 0) return 0;      // raspuns invalid: nu schimbam nimic
    g_sup_fed = true;
    uint32_t m = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) | ((uint32_t)d[2] << 8) | d[3];
    for (int i = 0; i < 32; i++) {
        unsigned pid = (unsigned)base + (unsigned)i + 1u;
        if (pid > 0xFF) break;
        if (m & (1UL << (31 - i))) sup_set((uint8_t)pid);
    }
    // ultimul bit al fiecarui bloc = „exista si blocul urmator" (PID base+0x20)
    unsigned next = (unsigned)base + 0x20u;
    if (next <= 0xE0 && sup_get((uint8_t)next)) return (uint8_t)next;
    g_sup_known = true;
    return 0;
}

void param_sup_done() { g_sup_known = g_sup_fed; }
bool param_sup_known() { return g_sup_known; }

bool param_sup_has(uint8_t pid) {
    if (!g_sup_known) return true;      // nu stim inca => nu blocam nimic pe ghicite
    return sup_get(pid);
}

size_t param_sup_count() {
    size_t n = 0;
    for (unsigned p = 1; p <= 0xFF; p++) if ((p & 0x1F) != 0 && sup_get((uint8_t)p)) n++;
    return n;
}
