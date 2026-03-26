/*
 * com_mcu2.h
 *
 *  Created on: Jan 27, 2026
 *      Author: firza
 */

#ifndef INC_COM_MCU2_H_
#define INC_COM_MCU2_H_

#include "uart_cco.h"

// Flag request dari MCU2
extern volatile bool mcu2_request_ready;

// Inisialisasi
void MCU2_Comm_Init(void);

// Callback untuk UART2 (panggil dari HAL_UART_RxCpltCallback)
void MCU2_UART_RxCallback(void);

// Kirim data ke MCU2
void MCU1_SendNodeCount(void);
void MCU1_SendNodeByIndex(uint16_t index);
void MCU1_SendRegByIndex(uint16_t index);
bool MCU1_SendPWM_Group(int pwm_value);
bool MCU1_SendPWM_Individu(const char *sta_mac, int pwm_value);
void MCU1_SendOK(void);
void MCU1_SendError(const char *msg);

// Proses request dari MCU2 (panggil di main loop)
void MCU1_ProcessMCU2Request(void);

#endif /* INC_COM_MCU2_H_ */
