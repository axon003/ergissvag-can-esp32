/**
 * usb_proto.h — line-delimited JSON command parser pe USB Serial (F16 → ESP32)
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-05-23 — creat initial; comenzi obd1/3/4/7/9, uds10/19/22, ping, mode
 *
 * Format comenzi (1 linie JSON terminat cu \n):
 *   {"cmd":"obd1","pid":12}              ← mode 01 PID 0x0C (RPM)
 *   {"cmd":"obd3"}                       ← mode 03 stored DTCs broadcast
 *   {"cmd":"obd4"}                       ← mode 04 clear DTCs broadcast
 *   {"cmd":"obd7"}                       ← mode 07 pending DTCs broadcast
 *   {"cmd":"obd9","pid":2}               ← mode 09 PID 0x02 (VIN)
 *   {"cmd":"uds10","ecu":2016,"sub":3}   ← UDS DiagSessionControl extended (ecu 0x7E0)
 *   {"cmd":"uds22","ecu":2016,"did":4444} ← UDS Read DID 0x115C
 *   {"cmd":"uds19"}                      ← UDS read DTC info broadcast 7DF
 *   {"cmd":"ping"}                       ← health check (raspuns "pong")
 *
 * Valori numerice accepta zecimal sau hex cu "0x" prefix.
 */
#ifndef ERGCAN_USB_PROTO_H
#define ERGCAN_USB_PROTO_H

#include <Arduino.h>

void usb_proto_init();
void usb_proto_service();    // apel in fiecare loop()

#endif
