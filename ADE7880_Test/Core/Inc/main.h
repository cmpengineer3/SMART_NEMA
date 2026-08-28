/**
  ******************************************************************************
  * @file    main.h
  * @brief   Header untuk firmware diagnostik ADE7880
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdarg.h>

/* ── Peta pin (identik dengan project New_KWH_EC25) ───────────────────────── */
#define LED_Pin             GPIO_PIN_5
#define LED_GPIO_Port       GPIOA

/* ── [STEP 4] Relay DO1 + pinout pendukung ────────────────────────────────
 *
 *  Skematik Smartsiklon_Quectel_EC25_miniPCIE sheet 2/4 —
 *
 *      PA1 --[ R6 220R ]--> GATE Q1 (BSS138L, N-channel low-side)
 *                              |
 *                              +- drain --> koil relay K1 --> +5V
 *                              +- source --> GND
 *
 *  Jadi gate HIGH = MOSFET ON = koil dialiri arus = RELAY ON.
 *  → ACTIVE HIGH (bukan active-low seperti asumsi lama).
 *
 *  Pin modem (EN3V8, LTE_PWR, LTE_RST) juga di-define supaya bisa dipaksa
 *  LOW di boot — memastikan modem TETAP MATI di Step 4. Modem baru
 *  dinyalakan di Step 6. */
#define RELAY_Pin           GPIO_PIN_1
#define RELAY_GPIO_Port     GPIOA

#define EN10_Pin            GPIO_PIN_6      /* tidak ada di skematik, NC — aman */
#define EN10_GPIO_Port      GPIOA
#define EN3V8_Pin           GPIO_PIN_7      /* enable 3.8V modem — HARUS LOW    */
#define EN3V8_GPIO_Port     GPIOA
#define LTE_PWR_Pin         GPIO_PIN_0
#define LTE_PWR_GPIO_Port   GPIOB
#define LTE_RST_Pin         GPIO_PIN_1
#define LTE_RST_GPIO_Port   GPIOB
#define MEM_WP_Pin          GPIO_PIN_5      /* FRAM write protect — LOW=enable  */
#define MEM_WP_GPIO_Port    GPIOB

/* Input diskret (belum dipakai, dideklarasikan supaya kelak konsisten). */
#define IN1_Pin             GPIO_PIN_2      /* skematik: PC2 (BUKAN PC1)        */
#define IN1_GPIO_Port       GPIOC
#define IN2_Pin             GPIO_PIN_1      /* skematik: PC1 (BUKAN PC2)        */
#define IN2_GPIO_Port       GPIOC

/* Pin ADE7880 didefinisikan di powermeter_ade7880.h:
 *   CS   PB12    SCK  PB13    MISO PB14    MOSI PB15
 *   PM1  PC6     PM0  PC7     RST  PC8
 *   IRQ0 PA8     IRQ1 PC9
 * Pin USART1 debug: PA9 (TX) / PA10 (RX)
 */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;   /* [STEP 5] modem — dipakai stm32f1xx_it.c */
extern UART_HandleTypeDef huart4;   /* [STEP 5] header J5 */
extern IWDG_HandleTypeDef hiwdg;

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
