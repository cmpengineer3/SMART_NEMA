/*
 * uart_cco.h
 *
 *  Created on: Nov 20, 2025
 *      Author: firza
 */

#ifndef INC_UART_CCO_H_
#define INC_UART_CCO_H_

#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stdlib.h"
#include <stdint.h>

#define RX_BUFFER_CCO 512

// ========== KONFIGURASI POLLING ==========
#define POLLING_TIMEOUT_MS      10000
#define POLLING_MAX_RETRY       1
#define POLLING_DELAY_BETWEEN   300
#define POLLING_INTERVAL_MS    50000

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;


// ========== BUFFER VARIABLES ==========
extern char rxBuffer[];
extern char recvBuffer[];
extern char txBuffer[200];
extern char mac_cco[];

extern volatile bool recv_data_ready;

extern const int max_retry;
extern int attempt_debug;
extern uint32_t delays;

// -------------------- Struct STA --------------------
typedef struct {
    char mac_sta[13];
    double La;
    double Lo;
    uint16_t v;
    uint16_t i;
    uint16_t p;
    uint8_t relay;
    uint8_t dim;
    bool cond;
    uint8_t retry_count;
} STA_Node;


extern STA_Node mac_sta_list[250];
extern uint16_t mac_sta_count;

extern char mac_sta_new_list[250][13];
extern uint16_t mac_sta_new_count;

extern char mac_wh_list[250][13];
extern uint16_t mac_wh_count;

extern char mac_wh_new_list[250][13];
extern uint16_t mac_wh_new_count;

// ========== Fungsi UART Dasar ==========
void UART_CCO_Init(void);
void UART_CCO_ClearBuffer(void);
void UART_CCO_ClearRecvBuffer(void);
void UART_CCO_ClearNewTopoList(void);

void UART_CCO_SendRaw(const char *s);
void UART_CCO_SendAT(const char *cmd);

bool UART_CCO_WaitFor(const char *keyword, uint32_t timeout);
bool UART_CCO_SendCmdWait(const char *cmd, const char *urc, uint32_t t1, uint32_t t2);

// ========== Fungsi AT+SENDEX ==========
bool CCO_SendData_SENDEX(const char *mac_dest, const char *payload);
bool CCO_WaitAndParse_RECV(const char *mac_expected, char *payload_out, size_t payload_size, uint32_t timeout);

// ========== Fungsi Parsing ==========
void CCO_ParseRECV_Line(const char *line, char *mac_out, char *payload_out, size_t payload_max);
void CCO_ParseSTAResponse(const char *json);
bool ExtractJsonValue(const char *json, const char *key, char *value, size_t max_len);

// ========== Fungsi Topology ==========
void UART_CCO_ParseTopoInfo(char *line);
void UART_CCO_AddTopoMAC(const char *mac);
void TOPOINFO(void);
void UART_CCO_AddNewTopoMAC(const char *mac);
void UART_CCO_SyncTopoList(void);

// ========== Fungsi Whitelist ==========
void UART_CCO_ClearNewWHList(void);
void WHINFO(void);
void UART_CCO_ParseWHInfo(char *line);
void UART_CCO_AddNewWHMAC(const char *mac);
void UART_CCO_SyncWHList(void);

// ========== Fungsi Polling ==========
void CCO_PollAllSTA_SENDEX(void);
void CCO_PollAllSTA_PWM(void);
bool CCO_FindNodeByMAC(const char *mac, uint16_t *index);
void CCO_UpdateNodeData(uint16_t index, uint16_t v, uint16_t i, uint16_t dim,
                        const char *lat, const char *lng);

// ========== Fungsi JSON Output ==========
void USART2_SendNodeAsJSON(STA_Node* node);
void USART2_SendAllNodes_JSON(void);

// ========== Fungsi Konfigurasi ==========
void config(void);
void searching(void);
void Save_Node(void);
void Send_GPIO_Toggle(void);
#endif /* INC_UART_CCO_H_ */
