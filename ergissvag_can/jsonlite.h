/**
 * jsonlite.h — extragere campuri dintr-o linie JSON simpla (fara alocari), comuna pt protocol, OTA, abonamente
 *   C++ portabil. Valorile numerice accepta zecimal sau hex "0x..". Nu interpreteaza escape-uri in texte.
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-09-15 — creat initial (ergCAN 2.0), mutat din usb_proto.cpp
 */
#ifndef ERGCAN_JSONLITE_H
#define ERGCAN_JSONLITE_H

#include <cstdint>
#include <cstddef>

bool   jl_int(const char *json, const char *key, long *out);
bool   jl_num(const char *json, const char *key, double *out);
bool   jl_str(const char *json, const char *key, char *out, size_t out_sz);
// "02 01 0C" / "02010C" -> octeti; intoarce numarul lor (0 = invalid)
size_t jl_hex(const char *s, uint8_t *out, size_t max);
// copiaza s in out cu ghilimele/backslash escapate; intoarce lungimea scrisa
size_t jl_escape(char *out, size_t out_sz, const char *s);
// urmatorul obiect {...} dintr-un array JSON, incepand de la *pos; copiaza obiectul in out
bool   jl_next_object(const char *json, size_t *pos, char *out, size_t out_sz);

#endif
