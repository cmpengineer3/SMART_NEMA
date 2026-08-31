#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ── Buffer sizes ─────────────────────────────────────────────────────────── */
/* [FIX #1] 512 -> 1024: memperkecil peluang rxBuffer penuh sebelum sempat
 * di-clear (mis. saat payload MQTT besar / beberapa URC menumpuk). Dipadukan
 * dengan strategi overflow "geser separuh terakhir" (bukan wipe total) di
 * HAL_UART_RxCpltCallback — lihat uart.c. */
#define RX_BUFFER_SIZE      1024
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
extern volatile bool        urc_line_pending;   /* [STEP6] set di ISR, dibersihkan UART_ProcessURC */
extern volatile uint32_t    rx_overflow_count;  /* [FIX #1] jumlah kejadian rxBuffer penuh (harusnya jarang/nol) */

/* ── Init ─────────────────────────────────────────────────────────────────── */
void UART_Init_Buffer(UART_HandleTypeDef *huart);


void UART_WatchdogRefresh(void);

/*
 * Debug UART poll hook — dipanggil setiap iterasi polling di semua wait loop.
 * Default __weak implementation kosong (no-op). Override di main.c untuk:
 *
 *   void UART_DebugPoll(void) { UID_Process(); }
 *
 * Ini membuat perintah debug (SETUID/GETUID/STATUS/dsb.) TETAP diproses saat
 * firmware menunggu respons modem — termasuk selama MQTT_Open yang bisa
 * memakan beberapa detik. Byte-nya sendiri sudah selalu masuk buffer lewat
 * ISR huart1; hook ini yang membuat pemrosesannya juga tidak macet.
 */
void UART_DebugPoll(void);

/* ── Buffer helpers ───────────────────────────────────────────────────────── */
/* Scan URC (+QMTRECV / +QMTSTAT) — dipanggil dari superloop. strstr()
 * DIKELUARKAN dari ISR agar ISR selalu ringan (<10 µs). */
void UART_ProcessURC(void);

/* Hook byte USART1 (debug) — override di main.c. Weak no-op by default. */
void UART_OnDebugByte(uint8_t b);

/* Arm HAL_UART_Receive_IT untuk USART1 (debug) pertama kali. */
void UART_StartDebugRx(void);

void UART_ClearBuffer(void);
void UART_FlushRx(uint32_t wait_ms);

/* ── Transmit helpers ─────────────────────────────────────────────────────── */
void UART_SendString(const char *str);

void UART_SendATCommand(const char *cmd_no_crlf);


void UART_TransmitRaw(const uint8_t *data, uint16_t len);

/* ── Response polling (call from main/task context, NOT from ISR) ─────────── */
bool UART_WaitForOK(uint32_t timeout_ms);
bool UART_WaitForURC(const char *urc_expected, uint32_t timeout_ms);
bool UART_WaitForPrompt(const char *prompt, uint32_t timeout_ms);

bool UART_WaitFor_OK_Then_URC(const char *urc_expected,
                               uint32_t timeout_ok_ms,
                               uint32_t timeout_urc_ms);

/* ── Payload extraction ───────────────────────────────────────────────────── */

bool Extract_Payload(void);

/* ─────────────────────────────────────────────────────────────────────────── */
/*  [RAWMON] Monitor byte mentah UART3 (modem) → dicetak ke UART1 (debug)     */
/*                                                                             */
/*  Tujuan: bisa lihat langsung apa yang benar-benar masuk ke huart3 —        */
/*  termasuk byte rusak/corrupt — TANPA mengganggu rxBuffer/parsing modem     */
/*  yang sudah ada. Pakai ring buffer terpisah, diisi di ISR (murah, tanpa    */
/*  strstr/HAL_UART_Transmit), lalu dikuras & dicetak dari superloop.         */
/* ─────────────────────────────────────────────────────────────────────────── */

/* Status monitor & statistik — boleh dibaca (read-only) dari luar uart.c */
extern volatile bool     uart3_rawmon_enabled;
extern volatile uint32_t uart3_raw_rx_count;   /* total byte huart3 diterima sejak monitor ON */
extern volatile uint32_t uart3_raw_dropped;    /* byte hilang krn ring buffer RAWMON penuh    */
extern volatile uint32_t uart3_raw_last_rx_tick; /* HAL_GetTick() byte huart3 terakhir masuk  */

/* Nyala/matikan monitor. Saat dinyalakan, statistik & ring buffer direset. */
void UART3_RawMon_Enable(bool enable);

/* Panggil sesering mungkin dari context non-ISR: superloop utama, DAN idealnya
 * juga dari titik-titik yang menunggu lama (mis. delay boot modem), supaya
 * ring buffer tidak keburu penuh saat traffic UART3 padat. Aman dipanggil
 * terus-menerus walau monitor sedang OFF (langsung return, murah). */
void UART3_RawMon_Poll(void);

#endif /* INC_UART_H_ */
