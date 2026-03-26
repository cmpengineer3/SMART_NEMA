/*
 * uart.h (SIMPLIFIED VERSION)
 *
 * Fitur:
 * 1. Semua data masuk ke rxBuffer (termasuk +QMTRECV)
 * 2. Saat +QMTRECV terdeteksi, buffer di-CLEAR dulu, baru data baru masuk
 * 3. Tidak ada parsing, tidak ada buffer terpisah
 * 4. Bisa dilihat langsung di Live Expression
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "config.h"

/* ============== BUFFER SIZE ============== */
#define RX_BUFFER_SIZE       512
#define PAY_LOAD_SIZE       512

/* ============== CRITICAL SECTION ============== */
#define ENTER_CRITICAL()    __disable_irq()
#define EXIT_CRITICAL()     __enable_irq()

/* ============== BUFFER - VOLATILE untuk Live Expression ============== */
extern volatile char rxBuffer[RX_BUFFER_SIZE];
extern volatile char payload_data[PAY_LOAD_SIZE];
extern volatile uint16_t rx_index;
extern volatile bool mqtt_disconnected;

// Flag untuk menandakan ada data MQTT baru (opsional, untuk monitoring)
extern volatile bool mqtt_data_ready;

// Debug counter - untuk monitoring di Live Expression
extern volatile uint32_t mqtt_msg_count;
extern volatile uint8_t hlw_rx_byte;

/* ============== TIMING ============== */
#define POLLING_INTERVAL_MS    30000

/* ============== FUNCTION PROTOTYPES ============== */

// Inisialisasi
void UART_Init_Buffer(void);

// Kirim data
void UART_SendString(const char *str);
void UART_SendATCommand(const char *cmd_no_crlf);

// Tunggu response
bool UART_WaitForOK(uint32_t timeout_ms);
bool UART_WaitForURC(const char *urc_expected, uint32_t timeout_ms);
bool UART_WaitForPrompt(const char *prompt, uint32_t timeout_ms);
bool UART_WaitFor_OK_Then_URC(const char *urc_expected,
                              uint32_t timeout_ok_ms,
                              uint32_t timeout_urc_ms);
bool Extract_Payload(void);

// Buffer management
void UART_ClearBuffer(void);
void UART_FlushRx(uint32_t wait_ms);

#endif /* INC_UART_H_ */
