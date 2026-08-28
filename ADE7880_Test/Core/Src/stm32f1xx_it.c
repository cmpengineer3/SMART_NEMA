/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt handler — firmware diagnostik ADE7880
  *
  *  [STEP 5] USART3 + UART4 SEKARANG DI-INIT — IRQn di-enable, tapi
  *  HAL_UART_Receive_IT() belum dipanggil untuk keduanya, jadi RXNE
  *  interrupt disable, ISR di bawah tidak akan fire meski byte datang.
  ******************************************************************************
  */

#include "main.h"
#include "stm32f1xx_it.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;    /* [STEP 5] */
extern UART_HandleTypeDef huart4;    /* [STEP 5] */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/

void NMI_Handler(void)        { while (1) { } }
void HardFault_Handler(void)  { while (1) { } }
void MemManage_Handler(void)  { while (1) { } }
void BusFault_Handler(void)   { while (1) { } }
void UsageFault_Handler(void) { while (1) { } }
void SVC_Handler(void)        { }
void DebugMon_Handler(void)   { }
void PendSV_Handler(void)     { }

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F1xx Peripheral Interrupt Handlers                    */
/******************************************************************************/

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/* [STEP 5] USART3 (modem) — belum ada RX di-arm, jadi RXNE tidak akan fire.
 * Handler tetap disediakan supaya vektor sesuai kalau nanti kita mulai
 * memakai HAL_UART_Receive_IT(&huart3, ...) di Step 6/7. */
void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

/* [STEP 5] UART4 (header J5) — belum dipakai */
void UART4_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart4);
}
