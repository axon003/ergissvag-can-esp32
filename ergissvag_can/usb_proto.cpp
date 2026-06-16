/**
 * usb_proto.cpp — JSON line parser pe Serial USB
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-05-23 — creat initial
 */
#include "usb_proto.h"
#include "obd_cmd.h"
#include <string.h>
#include <stdlib.h>

extern void emitJson(const char *line, size_t len);

#define USB_LINE_MAX 256

static char g_buf[USB_LINE_MAX];
static size_t g_len = 0;

// Cauta "<key>" si returneaza pointer dupa cele 2 ghilimele de key
static const char* find_key(const char *line, const char *key) {
    char needle[40];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    return strstr(line, needle);
}

// Extrage int dupa "key": ... ; intoarce 1 daca gasit (parseaza zecimal sau hex 0x)
static bool json_int(const char *line, const char *key, long *out) {
    const char *p = find_key(line, key);
    if (!p) return false;
    p = strchr(p, ':');
    if (!p) return false;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '"') p++;
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
        *out = strtol(p + 2, nullptr, 16);
    } else {
        *out = strtol(p, nullptr, 10);
    }
    return true;
}

// Extrage string copiat in out (max out_sz)
static bool json_str(const char *line, const char *key, char *out, size_t out_sz) {
    const char *p = find_key(line, key);
    if (!p) return false;
    p = strchr(p, ':');
    if (!p) return false;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '"') return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i < out_sz - 1) out[i++] = *p++;
    out[i] = 0;
    return true;
}

static void dispatch(const char *line) {
    char cmd[24];
    if (!json_str(line, "cmd", cmd, sizeof(cmd))) return;
    long pid = -1, ecu = -1, did = -1, sub = -1;
    bool has_pid = json_int(line, "pid", &pid);
    bool has_ecu = json_int(line, "ecu", &ecu);
    bool has_did = json_int(line, "did", &did);
    bool has_sub = json_int(line, "sub", &sub);

    if (strcmp(cmd, "obd1") == 0 && has_pid) {
        obd_query_pid01((uint8_t)pid);
    }
    else if (strcmp(cmd, "obd3") == 0) {
        obd_query_dtcs();
    }
    else if (strcmp(cmd, "obd4") == 0) {
        obd_clear_dtcs();
    }
    else if (strcmp(cmd, "obd7") == 0) {
        obd_query_pending_dtcs();
    }
    else if (strcmp(cmd, "obd9") == 0 && has_pid) {
        obd_query_info09((uint8_t)pid);
    }
    else if (strcmp(cmd, "uds10") == 0) {
        if (!has_ecu) ecu = 0x7E0;
        if (!has_sub) sub = 0x03;
        uds_diag_session((uint32_t)ecu, (uint8_t)sub);
    }
    else if (strcmp(cmd, "uds22") == 0 && has_did) {
        if (!has_ecu) ecu = 0x7E0;
        uds_read_did((uint32_t)ecu, (uint16_t)did);
    }
    else if (strcmp(cmd, "uds19") == 0) {
        uds_read_dtc_info_all();
    }
    else if (strcmp(cmd, "ping") == 0) {
        char buf[80];
        int n = snprintf(buf, sizeof(buf),
            "{\"ts\":%lu,\"type\":\"pong\"}\n", (unsigned long)millis());
        if (n > 0) emitJson(buf, (size_t)n);
    }
}

void usb_proto_init() {
    g_len = 0;
}

void usb_proto_service() {
    while (Serial.available() > 0) {
        int b = Serial.read();
        if (b < 0) break;
        if (b == '\r') continue;
        if (b == '\n') {
            if (g_len > 0) {
                g_buf[g_len] = 0;
                dispatch(g_buf);
                g_len = 0;
            }
        } else {
            if (g_len < USB_LINE_MAX - 1) {
                g_buf[g_len++] = (char)b;
            } else {
                g_len = 0;   // overflow, abandon line
            }
        }
    }
}
