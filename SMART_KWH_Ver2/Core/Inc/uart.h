/*
 * uart.h — Quectel modem UART driver (modular, no project dependencies)
 *
 * Usage:
 *   1. Call UART_Init_Buffer(&huartX) once after MX_USARTx_UART_Init().
 *   2. Use UART_SendATCommand / UART_WaitFor* helpers for AT command flow.
 *   3. Check mqtt_data_ready / mqtt_disconnected flags from main loop.
 *
 * Copy uart.h + uart.c to any project — only dependency is stm32f4xx_hal.h.
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ── Buffer sizes ─────────────────────────────────────────────────────────── */
#define RX_BUFFER_SIZE      512
#define PAY_LOAD_SIZE       512

/* ── Critical section helpers ─────────────────────────────────────────────── */
#define ENTER_CRITICAL()    __disable_irq()
#define EXIT_CRITICAL()     __enable_irq()

/* ── Shared state (read-only from application) ─────────────────────────────── */
extern volatile char        rxBuffer[RX_BUFFER_SIZE];
extern volatile char        payload_data[PAY_LOAD_SIZE];
extern volatile uint16_t    rx_index;
extern volatile bool        mqtt_disconnected;  /* set by ISR on +QMTSTAT URC  */
extern volatile bool        mqtt_data_ready;    /* set by ISR on +QMTRECV line  */
extern volatile uint32_t    mqtt_msg_count;     /* incremented on each +QMTRECV */

/* ── Init ─────────────────────────────────────────────────────────────────── */
/*
 * Register the UART handle and start interrupt-driven reception.
 * Call ONCE after the HAL UART init function (e.g. MX_USART1_UART_Init).
 */
void UART_Init_Buffer(UART_HandleTypeDef *huart);

/*
 * Watchdog refresh hook — called inside every UART wait loop every ~5 seconds.
 * Default implementation is empty (no-op). Override in main.c if IWDG is used:
 *
 *   void UART_WatchdogRefresh(void) { HAL_IWDG_Refresh(&hiwdg); }
 *
 * This prevents the IWDG from firing during long AT command waits (e.g. the
 * 90-second QMTOPEN URC timeout) without coupling uart.c to the IWDG handle.
 */
void UART_WatchdogRefresh(void);

/* ── Buffer helpers ───────────────────────────────────────────────────────── */
void UART_ClearBuffer(void);
void UART_FlushRx(uint32_t wait_ms);

/* ── Transmit helpers ─────────────────────────────────────────────────────── */
void UART_SendString(const char *str);

/*
 * Clear rx buffer then send "cmd\r\n".
 * Use for AT commands where you want a clean response window.
 */
void UART_SendATCommand(const char *cmd_no_crlf);

/*
 * Send raw bytes without any modification (no \r\n appended, no buffer clear).
 * Used by mqtt.c to send QMTPUBEX payload bytes after the '>' prompt.
 */
void UART_TransmitRaw(const uint8_t *data, uint16_t len);

/* ── Response polling (call from main/task context, NOT from ISR) ─────────── */
bool UART_WaitForOK(uint32_t timeout_ms);
bool UART_WaitForURC(const char *urc_expected, uint32_t timeout_ms);
bool UART_WaitForPrompt(const char *prompt, uint32_t timeout_ms);

/*
 * Wait for "OK" first, then wait for a URC.
 * Used for QMTOPEN / QMTCONN / QMTSUB which respond with OK then a URC.
 */
bool UART_WaitFor_OK_Then_URC(const char *urc_expected,
                               uint32_t timeout_ok_ms,
                               uint32_t timeout_urc_ms);

/* ── Payload extraction ───────────────────────────────────────────────────── */
/*
 * Extract JSON object after "DATA": key from rxBuffer into payload_data[].
 * Returns true if extraction succeeded.
 */
bool Extract_Payload(void);

#endif /* INC_UART_H_ */
