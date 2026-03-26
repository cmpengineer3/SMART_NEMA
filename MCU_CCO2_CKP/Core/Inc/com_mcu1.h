/*
 * com_mcu1.h
 *
 *  Created on: Jan 27, 2026
 *      Author: firza
 */

#ifndef INC_COM_MCU1_H_
#define INC_COM_MCU1_H_

#include "config.h"
#include <stdbool.h>
#include <stdint.h>

// ============================================================================
//                    BUFFER DAN VARIABEL (untuk Live Expression)
// ============================================================================

// Buffer untuk terima data dari MCU1
#define MCU1_RX_BUFFER_SIZE  512
extern volatile char mcu1_rx_buffer[MCU1_RX_BUFFER_SIZE];
extern volatile uint16_t mcu1_rx_index;
extern volatile bool mcu1_data_ready;

// Variabel monitoring (Live Expression)
extern volatile uint16_t node_total_count;      // Jumlah total node dari MCU1
extern volatile uint16_t node_current_index;    // Index node yang sedang diproses
extern volatile uint16_t node_success_count;    // Jumlah node berhasil kirim ke MQTT
extern volatile uint16_t node_fail_count;       // Jumlah node gagal
extern volatile bool is_requesting;             // Flag sedang proses request
extern volatile char jml_node [10];

// ============================================================================
//                    FUNCTION PROTOTYPES
// ============================================================================

// Inisialisasi
void MCU1_Request_Init(void);

// Callback UART (panggil dari HAL_UART_RxCpltCallback untuk UART3)
void MCU1_UART_RxCallback(void);

// Buffer management
void MCU1_ClearBuffer(void);

// Request functions
bool MCU1_SendRequest(const char *request, uint32_t timeout_ms);
bool MCU1_RequestNodeCount(void);
bool MCU1_RequestNodeData(uint16_t index, char *json_out, size_t json_size);

// Main function - request semua node dari MCU1 dan kirim ke MQTT
void MCU2_RequestAllNodes(void);
void MCU2_RegisCCO(void);
void MCU2_RegisAllNodes(void);
void TEST_SendReqCnt(void);
int Parse_JSON_Int(const char *json, const char *key);
bool Parse_JSON_String(const char *json, const char *key, char *output, size_t output_size);
bool MCU2_SendControlGroup(int pwm_value);
bool MCU2_SendControlIndividu(const char *sta_mac, int pwm_value);
bool MCU2_RequestMacCCO(char *mac_out, size_t mac_size);


// Command handler (update dari sebelumnya)
void Command_Handler(void);

#endif /* INC_COM_MCU1_H_ */
