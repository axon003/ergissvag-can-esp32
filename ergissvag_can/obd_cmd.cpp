/**
 * obd_cmd.cpp — implementare OBD2 + UDS command/response handling
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-05-23 — creat initial
 */
#include "obd_cmd.h"
#include "iso_tp.h"
#include <stdio.h>
#include <string.h>

// Forward decl — emitJson din main.cpp/main.ino (Serial + TCP)
extern void emitJson(const char *line, size_t len);

// ---- Query builders (toate Single Frame, ≤7 bytes payload) ----

bool obd_query_pid01(uint8_t pid) {
    uint8_t req[2] = { 0x01, pid };
    return iso_tp_send_sf(OBD_FUNCTIONAL_REQ_ID, req, 2);
}

bool obd_query_dtcs() {
    uint8_t req[1] = { 0x03 };
    return iso_tp_send_sf(OBD_FUNCTIONAL_REQ_ID, req, 1);
}

bool obd_clear_dtcs() {
    uint8_t req[1] = { 0x04 };
    return iso_tp_send_sf(OBD_FUNCTIONAL_REQ_ID, req, 1);
}

bool obd_query_pending_dtcs() {
    uint8_t req[1] = { 0x07 };
    return iso_tp_send_sf(OBD_FUNCTIONAL_REQ_ID, req, 1);
}

bool obd_query_info09(uint8_t pid) {
    uint8_t req[2] = { 0x09, pid };
    return iso_tp_send_sf(OBD_FUNCTIONAL_REQ_ID, req, 2);
}

bool uds_diag_session(uint32_t ecu_can_id, uint8_t session_type) {
    uint8_t req[2] = { 0x10, session_type };
    return iso_tp_send_sf(ecu_can_id, req, 2);
}

bool uds_read_did(uint32_t ecu_can_id, uint16_t did) {
    uint8_t req[3] = { 0x22, (uint8_t)(did >> 8), (uint8_t)(did & 0xFF) };
    return iso_tp_send_sf(ecu_can_id, req, 3);
}

bool uds_read_dtc_info_all() {
    // 0x19 0x02 0xFF = ReportDTCByStatusMask, mask 0xFF (toate statusurile)
    uint8_t req[3] = { 0x19, 0x02, 0xFF };
    return iso_tp_send_sf(OBD_FUNCTIONAL_REQ_ID, req, 3);
}

// ---- Response parsers ----

// Codifica 2 bytes DTC raw (J2012) in string ascii: "P0420", "U0100", etc.
static void dtc_2b_to_str(uint8_t hi, uint8_t lo, char out[6]) {
    char prefix;
    switch ((hi >> 6) & 0x03) {
        case 0: prefix = 'P'; break;
        case 1: prefix = 'C'; break;
        case 2: prefix = 'B'; break;
        case 3: prefix = 'U'; break;
        default: prefix = 'P';
    }
    snprintf(out, 6, "%c%X%X%X%X", prefix, (hi >> 4) & 0x03, hi & 0x0F, (lo >> 4) & 0x0F, lo & 0x0F);
}

// Append "AB CD EF ..." hex dump in buf
static void hex_dump(const uint8_t *data, size_t len, char *out, size_t out_sz) {
    out[0] = 0;
    for (size_t i = 0; i < len; i++) {
        char tmp[4];
        snprintf(tmp, sizeof(tmp), "%02X ", data[i]);
        strncat(out, tmp, out_sz - strlen(out) - 1);
    }
}

void obd_process_response(uint32_t src_id, const uint8_t *data, size_t len) {
    if (len < 1) return;
    char buf[768];
    uint8_t sid = data[0];

    // === Negative Response (0x7F SID NRC) ===
    if (sid == 0x7F && len >= 3) {
        int n = snprintf(buf, sizeof(buf),
            "{\"ts\":%lu,\"type\":\"obd\",\"resp\":\"nrc\",\"ecu\":\"0x%lX\",\"sid\":\"0x%02X\",\"nrc\":\"0x%02X\"}\n",
            (unsigned long)millis(), (unsigned long)src_id, data[1], data[2]);
        if (n > 0) emitJson(buf, (size_t)n);
        return;
    }

    // === Mode 01 response (current data) ===
    if (sid == 0x41 && len >= 2) {
        uint8_t pid = data[1];
        char hex[64];
        hex_dump(data + 2, len - 2, hex, sizeof(hex));
        int n = snprintf(buf, sizeof(buf),
            "{\"ts\":%lu,\"type\":\"obd\",\"resp\":\"01\",\"ecu\":\"0x%lX\",\"pid\":%u,\"data\":\"%s\"}\n",
            (unsigned long)millis(), (unsigned long)src_id, pid, hex);
        if (n > 0) emitJson(buf, (size_t)n);
        return;
    }

    // === Mode 03 / 07 response (stored / pending DTCs) ===
    if ((sid == 0x43 || sid == 0x47) && len >= 1) {
        // Format SAE J1979: 43 [NN] DTC1_HI DTC1_LO ... (NN byte optional)
        size_t off = 1;
        // Daca payload size dupa SID e par → primul byte = count; daca e impar → e direct DTC.
        if ((len - 1) >= 1 && ((len - 1) % 2 == 1)) off = 2;
        int dtc_count = (int)((len - off) / 2);
        char codes[512] = {0};
        for (int i = 0; i < dtc_count && i < 32; i++) {
            char code[6];
            dtc_2b_to_str(data[off + 2*i], data[off + 2*i + 1], code);
            if (codes[0]) strncat(codes, ",", sizeof(codes) - strlen(codes) - 1);
            strncat(codes, "\"", sizeof(codes) - strlen(codes) - 1);
            strncat(codes, code, sizeof(codes) - strlen(codes) - 1);
            strncat(codes, "\"", sizeof(codes) - strlen(codes) - 1);
        }
        int n = snprintf(buf, sizeof(buf),
            "{\"ts\":%lu,\"type\":\"obd\",\"resp\":\"%02X\",\"ecu\":\"0x%lX\",\"count\":%d,\"codes\":[%s]}\n",
            (unsigned long)millis(), sid, (unsigned long)src_id, dtc_count, codes);
        if (n > 0) emitJson(buf, (size_t)n);
        return;
    }

    // === Mode 04 response (clear DTCs OK) ===
    if (sid == 0x44) {
        int n = snprintf(buf, sizeof(buf),
            "{\"ts\":%lu,\"type\":\"obd\",\"resp\":\"04\",\"ecu\":\"0x%lX\",\"ok\":1}\n",
            (unsigned long)millis(), (unsigned long)src_id);
        if (n > 0) emitJson(buf, (size_t)n);
        return;
    }

    // === Mode 09 response (vehicle info; VIN PID 02, ECU name PID 0A) ===
    if (sid == 0x49 && len >= 2) {
        uint8_t pid = data[1];
        size_t off = 2;
        if (pid == 0x02 && len > 3) off = 3;     // skip "number of items" byte pentru VIN
        char asc[64] = {0};
        size_t a = 0;
        for (size_t i = off; i < len && a < sizeof(asc) - 1; i++) {
            char c = (char)data[i];
            asc[a++] = (c >= 0x20 && c < 0x7F) ? c : '.';
        }
        asc[a] = 0;
        char hex[128];
        hex_dump(data + 2, (len > 2) ? len - 2 : 0, hex, sizeof(hex));
        int n = snprintf(buf, sizeof(buf),
            "{\"ts\":%lu,\"type\":\"obd\",\"resp\":\"09\",\"ecu\":\"0x%lX\",\"pid\":%u,\"ascii\":\"%s\",\"data\":\"%s\"}\n",
            (unsigned long)millis(), (unsigned long)src_id, pid, asc, hex);
        if (n > 0) emitJson(buf, (size_t)n);
        return;
    }

    // === UDS 10 response (session control OK) ===
    if (sid == 0x50 && len >= 2) {
        int n = snprintf(buf, sizeof(buf),
            "{\"ts\":%lu,\"type\":\"obd\",\"resp\":\"10\",\"ecu\":\"0x%lX\",\"sub\":\"0x%02X\",\"ok\":1}\n",
            (unsigned long)millis(), (unsigned long)src_id, data[1]);
        if (n > 0) emitJson(buf, (size_t)n);
        return;
    }

    // === UDS 22 response (Read DID) ===
    if (sid == 0x62 && len >= 3) {
        uint16_t did = ((uint16_t)data[1] << 8) | data[2];
        char hex[256];
        hex_dump(data + 3, (len > 3) ? len - 3 : 0, hex, sizeof(hex));
        int n = snprintf(buf, sizeof(buf),
            "{\"ts\":%lu,\"type\":\"obd\",\"resp\":\"22\",\"ecu\":\"0x%lX\",\"did\":\"0x%04X\",\"data\":\"%s\"}\n",
            (unsigned long)millis(), (unsigned long)src_id, did, hex);
        if (n > 0) emitJson(buf, (size_t)n);
        return;
    }

    // === UDS 19 response (DTC info) ===
    if (sid == 0x59 && len >= 2) {
        uint8_t sub = data[1];
        if (sub == 0x02 && len >= 3) {
            // 59 02 STATUS_AVAIL_MASK [DTC_24bit STATUS]*
            size_t off = 3;
            int dtc_count = (int)((len - off) / 4);
            char codes[512] = {0};
            for (int i = 0; i < dtc_count && i < 32; i++) {
                // UDS DTC = 3 bytes (24-bit); folosim primii 2 ca J2012 base + byte 3 ca FTB (skip)
                char code[6];
                dtc_2b_to_str(data[off + 4*i], data[off + 4*i + 1], code);
                if (codes[0]) strncat(codes, ",", sizeof(codes) - strlen(codes) - 1);
                strncat(codes, "\"", sizeof(codes) - strlen(codes) - 1);
                strncat(codes, code, sizeof(codes) - strlen(codes) - 1);
                strncat(codes, "\"", sizeof(codes) - strlen(codes) - 1);
            }
            int n = snprintf(buf, sizeof(buf),
                "{\"ts\":%lu,\"type\":\"obd\",\"resp\":\"19\",\"ecu\":\"0x%lX\",\"sub\":\"0x%02X\",\"count\":%d,\"codes\":[%s]}\n",
                (unsigned long)millis(), (unsigned long)src_id, sub, dtc_count, codes);
            if (n > 0) emitJson(buf, (size_t)n);
            return;
        }
        // sub-functions necunoscute → raw fallback
    }

    // === Fallback raw (orice response neidentificat) ===
    char hex[64];
    hex_dump(data, (len > 16) ? 16 : len, hex, sizeof(hex));
    int n = snprintf(buf, sizeof(buf),
        "{\"ts\":%lu,\"type\":\"obd\",\"resp\":\"raw\",\"ecu\":\"0x%lX\",\"sid\":\"0x%02X\",\"data\":\"%s\"}\n",
        (unsigned long)millis(), (unsigned long)src_id, sid, hex);
    if (n > 0) emitJson(buf, (size_t)n);
}
