/**
 * jsonlite.cpp — extragere campuri JSON simple (vezi jsonlite.h)
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-09-15 — creat initial (ergCAN 2.0)
 */
#include "jsonlite.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>

static const char *find_val(const char *json, const char *key) {
    char needle[48];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = json;
    while ((p = strstr(p, needle)) != nullptr) {
        const char *q = p + strlen(needle);
        while (*q == ' ' || *q == '\t') q++;
        if (*q == ':') {
            q++;
            while (*q == ' ' || *q == '\t') q++;
            return q;
        }
        p = q;
    }
    return nullptr;
}

bool jl_int(const char *json, const char *key, long *out) {
    const char *p = find_val(json, key);
    if (!p) return false;
    if (*p == '"') p++;
    char *end = nullptr;
    long v;
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) v = strtol(p + 2, &end, 16);
    else v = strtol(p, &end, 10);
    if (end == p) return false;
    *out = v;
    return true;
}

bool jl_num(const char *json, const char *key, double *out) {
    const char *p = find_val(json, key);
    if (!p) return false;
    if (*p == '"') p++;
    char *end = nullptr;
    double v = strtod(p, &end);
    if (end == p) return false;
    *out = v;
    return true;
}

bool jl_str(const char *json, const char *key, char *out, size_t out_sz) {
    const char *p = find_val(json, key);
    if (!p || *p != '"' || out_sz == 0) return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"') {
        if (i >= out_sz - 1) return false;   // nu trunchiem tacut
        out[i++] = *p++;
    }
    if (*p != '"') return false;
    out[i] = 0;
    return true;
}

size_t jl_hex(const char *s, uint8_t *out, size_t max) {
    size_t n = 0;
    int hi = -1;
    for (; *s; s++) {
        int v;
        if (*s >= '0' && *s <= '9') v = *s - '0';
        else if (*s >= 'a' && *s <= 'f') v = *s - 'a' + 10;
        else if (*s >= 'A' && *s <= 'F') v = *s - 'A' + 10;
        else { if (hi >= 0) return 0; continue; }
        if (hi < 0) hi = v;
        else { if (n >= max) return 0; out[n++] = (uint8_t)((hi << 4) | v); hi = -1; }
    }
    return hi >= 0 ? 0 : n;
}

size_t jl_escape(char *out, size_t out_sz, const char *s) {
    size_t n = 0;
    if (out_sz == 0) return 0;
    for (; *s && n + 2 < out_sz; s++) {
        unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\') { out[n++] = '\\'; out[n++] = (char)c; }
        else if (c < 0x20) out[n++] = ' ';
        else out[n++] = (char)c;
    }
    out[n] = 0;
    return n;
}

bool jl_next_object(const char *json, size_t *pos, char *out, size_t out_sz) {
    const char *p = json + *pos;
    while (*p && *p != '{') { if (*p == ']') return false; p++; }
    if (*p != '{') return false;
    int depth = 0;
    bool in_str = false;
    const char *start = p;
    for (; *p; p++) {
        if (in_str) { if (*p == '\\' && p[1]) p++; else if (*p == '"') in_str = false; continue; }
        if (*p == '"') in_str = true;
        else if (*p == '{') depth++;
        else if (*p == '}') {
            if (--depth == 0) {
                size_t len = (size_t)(p - start + 1);
                if (len >= out_sz) return false;
                memcpy(out, start, len);
                out[len] = 0;
                *pos = (size_t)(p - json + 1);
                return true;
            }
        }
    }
    return false;
}
