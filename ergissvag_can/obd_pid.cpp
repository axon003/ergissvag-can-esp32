/**
 * obd_pid.cpp — lungimi si formule PID-uri OBD-II mode 01 (SAE J1979)
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.1
 * @changes
 *   v1.0 2026-09-15 — creat initial (ergCAN 2.0)
 *   v1.1 2026-09-29 — fw 2.6.0: PID-uri peste 0x60 folosite de catalogul cu nume (cuplu 0x61-0x63, odometru 0xA6)
 */
#include "obd_pid.h"

// lungime date per PID 0x00..0x60 (J1979). 0 = neclar in standard → tratat ca necunoscut.
static const int8_t PID_LEN[0x61] = {
/*00*/ 4, 4, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1,
/*10*/ 2, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 2,
/*20*/ 4, 2, 2, 2, 4, 4, 4, 4, 4, 4, 4, 4, 1, 1, 1, 1,
/*30*/ 1, 2, 2, 1, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2, 2, 2,
/*40*/ 4, 4, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 4,
/*50*/ 4, 1, 1, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 2, 2, 1,
/*60*/ 4
};

int obd_pid_len(uint8_t pid) {
    // v1.1: peste 0x60 tabelul din standard are multe PID-uri cu lungimi compuse; le declaram doar pe cele folosite
    switch (pid) {
        case 0x61: case 0x62: return 1;      // cuplu cerut / cuplu efectiv (% din referinta)
        case 0x63: return 2;                 // cuplu de referinta (Nm)
        case 0x80: case 0xA0: case 0xC0: return 4;   // bitmap PID-uri suportate
        case 0xA6: return 4;                 // odometru (0,1 km)
        default: break;
    }
    if (pid > 0x60) return -1;
    return PID_LEN[pid] > 0 ? PID_LEN[pid] : -1;
}

bool obd_pid_decode(uint8_t pid, const uint8_t *d, size_t len, double *out, const char **unit) {
    if (!d || len < 1) return false;
    double A = d[0], B = len > 1 ? d[1] : 0, AB = A * 256.0 + B;
    const char *u = "";
    double v;
    switch (pid) {
        case 0x04: v = A * 100.0 / 255.0; u = "%"; break;
        case 0x05: v = A - 40; u = "C"; break;
        case 0x06: case 0x07: case 0x08: case 0x09: v = (A - 128) * 100.0 / 128.0; u = "%"; break;
        case 0x0A: v = A * 3; u = "kPa"; break;
        case 0x0B: v = A; u = "kPa"; break;
        case 0x0C: if (len < 2) return false; v = AB / 4.0; u = "rpm"; break;
        case 0x0D: v = A; u = "km/h"; break;
        case 0x0E: v = A / 2.0 - 64; u = "deg"; break;
        case 0x0F: v = A - 40; u = "C"; break;
        case 0x10: if (len < 2) return false; v = AB / 100.0; u = "g/s"; break;
        case 0x11: v = A * 100.0 / 255.0; u = "%"; break;
        case 0x14: case 0x15: case 0x16: case 0x17: case 0x18: case 0x19: case 0x1A: case 0x1B:
                   v = A / 200.0; u = "V"; break;
        case 0x1F: if (len < 2) return false; v = AB; u = "s"; break;
        case 0x21: case 0x31: if (len < 2) return false; v = AB; u = "km"; break;
        case 0x22: if (len < 2) return false; v = AB * 0.079; u = "kPa"; break;
        case 0x23: if (len < 2) return false; v = AB * 10; u = "kPa"; break;
        case 0x2C: case 0x2E: case 0x2F: case 0x45: case 0x47: case 0x48: case 0x49: case 0x4A:
        case 0x4B: case 0x4C: case 0x52: case 0x5A: case 0x5B:
                   v = A * 100.0 / 255.0; u = "%"; break;
        case 0x2D: v = (A - 128) * 100.0 / 128.0; u = "%"; break;
        case 0x30: v = A; u = ""; break;
        case 0x33: v = A; u = "kPa"; break;
        case 0x3C: case 0x3D: case 0x3E: case 0x3F: if (len < 2) return false; v = AB / 10.0 - 40; u = "C"; break;
        case 0x42: if (len < 2) return false; v = AB / 1000.0; u = "V"; break;
        case 0x43: if (len < 2) return false; v = AB * 100.0 / 255.0; u = "%"; break;
        case 0x44: if (len < 2) return false; v = AB * 2.0 / 65536.0; u = "lambda"; break;
        case 0x46: case 0x5C: v = A - 40; u = "C"; break;
        case 0x4D: case 0x4E: if (len < 2) return false; v = AB; u = "min"; break;
        case 0x51: v = A; u = ""; break;
        case 0x59: if (len < 2) return false; v = AB * 10; u = "kPa"; break;
        case 0x5E: if (len < 2) return false; v = AB / 20.0; u = "L/h"; break;
        case 0x61: case 0x62: v = A - 125; u = "%"; break;
        case 0x63: if (len < 2) return false; v = AB; u = "Nm"; break;
        case 0xA6: if (len < 4) return false;
                   v = ((double)d[0] * 16777216.0 + (double)d[1] * 65536.0 + (double)d[2] * 256.0 + (double)d[3]) / 10.0; u = "km"; break;
        default: return false;
    }
    if (out) *out = v;
    if (unit) *unit = u;
    return true;
}

void obd_pid_bitmap(uint8_t base, const uint8_t *d, size_t len, bool *supported) {
    if (!d || len < 4 || !supported) return;
    uint32_t m = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) | ((uint32_t)d[2] << 8) | d[3];
    for (int i = 0; i < 32; i++)
        if (m & (1UL << (31 - i))) supported[(base + i + 1) & 0xFF] = true;
}
