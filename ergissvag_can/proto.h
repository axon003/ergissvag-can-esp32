/**
 * proto.h — dispecer comenzi (JSON pe TCP/serial + consola text telnet/serial) ergCAN 2.0
 *   Toate interfetele trec prin lista de comenzi permise (doar citiri). Scrierile/calibrarile refuzate in firmware.
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 2.2
 * @changes
 *   v2.2 2026-09-29 — fw 2.6.0: parametri cu nume (proto_sup_restart, proto_unsup_json)
 *   v2.0 2026-09-15 — creat (inlocuieste usb_proto.cpp): abonamente (sub), cfg, ota, consola (SELFTEST/SNIFF/EXPLORE), multi-client
 *   v2.1 2026-09-19 — fw 2.4.0: DTCALL (proto_dtc_json)
 */
#ifndef ERGCAN_PROTO_H
#define ERGCAN_PROTO_H

#include <Arduino.h>

// emitToClient: trimite un raspuns catre clientul care a cerut (owner). Implementat in .ino.
typedef void (*proto_reply_fn)(uint8_t owner, const char *line, size_t len);

void proto_init(proto_reply_fn reply);
void proto_json_line(uint8_t owner, const char *line);   // o linie JSON (fara \n) de la un client
void proto_console_line(uint8_t owner, const char *line, bool explore_pin_ok);  // o linie text (consola)
bool proto_console_prompt(uint8_t owner, char *out, size_t sz);  // prompt curent (ex "ergCAN> " / "EXPLORE# ")
void proto_client_gone(uint8_t owner);
void proto_event(const char *fmt, ...);   // v2.5.10: eveniment in jurnalul automat (BYOS kind=event), apelat din loop (core 1)
void proto_debug_service(uint32_t now);   // v2.5.9: urca jurnalul BYOSDEBUG (apelat din task-ul de retea, core 0)                   // clientul s-a deconectat → curata abonamentele lui

// v2.2: citire lenta VIN/KM in fundal (din loop, cand bus-ul raspunde). Rezultatul e memorat si folosit de cloud.
void   proto_slow_read_tick(uint32_t now);   // citeste VIN (o data) + KM (periodic) daca bus-ul raspunde si osch e liber
String proto_vin_cached();                    // VIN memorat ("" daca inca necitit)
long   proto_km_cached();                     // KM memorat (-1 daca inca necitit)
long   proto_fuel_cached();                   // litri in rezervor (-1 daca inca necititi)
String proto_eng_json();                     // v2.3.3: fragment JSON cu valorile proaspete de pe ECU motor (,"genload":..)
int    proto_fuel_did();                      // DID-ul de pe care vin litrii (0x2206/0x2212; 0 = inca niciunul)
String proto_dtc_json();                      // v2.4.0: fragment JSON cu rezumatul ultimului DTCALL (,"dtcn":..,"dtcm":"25,09","dtca":s)
String proto_dtc_list_json();                 // v2.5.4: fragment ,"dtcl":[...] cu CODURILE pe module; doar in primul snapshot dupa o scanare
void   proto_sup_restart();                   // v2.2: reciteste bitmap-ul PID-urilor suportate (alt ECU dupa autodetectie)
String proto_unsup_json();                    // v2.2: fragment ,"unsl":"chei nesuportate","nsup":<PID-uri declarate de ECU>

#endif
