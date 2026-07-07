/*
 * com_mcu2.h
 *
 *  Created on: Jan 27, 2026
 *      Author: firza
 */

#ifndef INC_COM_MCU2_H_
#define INC_COM_MCU2_H_

#include "uart_cco.h"

extern volatile bool mcu2_request_ready;
void MCU2_Comm_Init(void);
void MCU2_UART_RxCallback(void);
void MCU1_SendNodeCount(void);
void MCU1_SendNodeByIndex(uint16_t index);
void MCU1_SendRegByIndex(uint16_t index);
bool MCU1_SendPWM_Group(int pwm_value);
bool MCU1_SendPWM_Individu(const char *sta_mac, int pwm_value);
void MCU1_SendOK(void);
void MCU1_SendError(const char *msg);

void MCU1_ProcessMCU2Request(void);

#endif /* INC_COM_MCU2_H_ */
