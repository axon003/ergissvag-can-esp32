/**
 * obd_cmd.h — OBD2 + UDS command builders / response parsers
 * @author Gabriel Diaconu (ERGISS Media)
 * @version 1.0
 * @changes
 *   v1.0 2026-05-23 — creat initial; mode 01/03/04/07/09 OBD2 + UDS 10/19/22
 */
#ifndef ERGCAN_OBD_CMD_H
#define ERGCAN_OBD_CMD_H

#include <Arduino.h>

// Functional addressing (broadcast — toate ECU-urile cu OBD2 raspund)
#define OBD_FUNCTIONAL_REQ_ID  0x7DF

// Standard OBD2 physical addressing range
//   request 0x7E0..0x7E7  →  response 0x7E8..0x7EF (ECU 0..7)
#define OBD_PHYSICAL_REQ_BASE  0x7E0
#define OBD_PHYSICAL_RESP_BASE 0x7E8

// --- OBD2 queries (always broadcast / functional, raspuns multi-ECU) ---
bool obd_query_pid01(uint8_t pid);            // mode 01 cur data
bool obd_query_dtcs();                        // mode 03 stored DTCs (broadcast)
bool obd_clear_dtcs();                        // mode 04 clear DTCs (broadcast)
bool obd_query_pending_dtcs();                // mode 07 pending DTCs (broadcast)
bool obd_query_info09(uint8_t pid);           // mode 09 vehicle info (VIN, ECU name)

// --- UDS queries (functional broadcast sau physical per-ECU) ---
bool uds_diag_session(uint32_t ecu_can_id, uint8_t session_type);  // mode 10 sub: 01=default, 03=extended
bool uds_read_did(uint32_t ecu_can_id, uint16_t did);              // mode 22 Read Data by ID
bool uds_read_dtc_info_all();                                       // mode 19 sub 02 mask FF broadcast

// Procesare mesaj complet primit din ISO-TP — emite JSON pe Serial + TCP
void obd_process_response(uint32_t src_id, const uint8_t *data, size_t len);

#endif
