/*
 * function.h
 *
 *  Created on: Nov 25, 2025
 *      Author: firza
 */

#ifndef INC_FUNCTION_H_
#define INC_FUNCTION_H_

#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>

// ==================== BUFFER CONFIGURATION ====================
#define RX_BUFFER_STA       256
#define RECV_BUFFER_SIZE    256
#define TX_BUFFER_STA       256

// ==================== LED PINS (sesuaikan dengan main.h) ====================
#define LED_IND_Pin      GPIO_PIN_0
#define LED_IND_Port     GPIOB
#define LED3_MAC_Pin      GPIO_PIN_4
#define LED3_MAC_Port     GPIOA
#define RELAY_Pin        GPIO_PIN_5
#define RELAY_Port       GPIOA

// ==================== VAC CONFIGURATION ====================
extern uint32_t adc_value;
extern float vout;
extern float vin_ac;
extern float vac_fix;

// Sensor CT Arus
extern float ct_current_rms;
extern float ct_offsetVoltage;
extern float ct_fix;

// ===== DEBUG — tambahkan semua ini ke Live Expressions STM32CubeIDE =====
extern uint32_t dbg_ct_offset_counts;  // Target: ~2040
extern uint32_t dbg_ct_raw_min;        // Target: jauh dari 0
extern uint32_t dbg_ct_raw_max;        // Target: jauh dari 4095
extern float    dbg_ct_peak_voltage;   // Amplitudo sinyal CT (Volt)
extern float    dbg_ct_vrms;           // Vrms mentah sebelum CURRENT_CAL
extern uint16_t dbg_ct_clipped;        // Target: 0 (tidak ada clipping)
extern uint16_t dbg_sensor_calls_ps;   // Berapa kali Sensor_ReadAll per detik
extern uint32_t dbg_ct_total_samples;  // Total sampel CT per detik. Target: >50.000
// =========================================================================

//void VAC_Read(void);
void CT_CalculateOffset(void);
void Sensor_ReadAll(void);

extern uint32_t lastSensorTick;
void Sensor_Update(void);
// ==================== DATA STRUCTURES ====================

// Header Types
typedef enum {
    HEADER_UNKNOWN = 0,
    HEADER_RQ,
    HEADER_G,
    HEADER_S
} HeaderType_t;

typedef struct {
    HeaderType_t header;
    char sta_mac[16];
    int pwm_value;
    bool valid;
} ParsedCommand_t;


//typedef struct {
//    uint32_t data1;
//    uint32_t data2;
//} dataMerge;

// ==================== EXTERNAL VARIABLES ====================
extern uint32_t send_interval;
extern uint32_t lastSendTime;

extern char rxBuffer[RX_BUFFER_STA];
extern char txBuffer[TX_BUFFER_STA];
extern char recvBuffer[RECV_BUFFER_SIZE];
extern volatile bool recv_data_ready;

extern char mac_sta[13];
extern char mac_cco[13];
//extern char Parsebuffer[512];

extern volatile uint16_t rx_index;
extern volatile bool rx_complete;
extern volatile bool data_ready_to_process;

extern int dutytest, relaytest;
extern int dimming;
extern int pulse;

extern float sendLat;
extern float sendLng;

//extern char sendLat[15], sendLng[15];

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;

// ==================== UART STA FUNCTIONS ====================
// ==================== UART1 (STA) FUNCTIONS ====================
void UART1_StartUart(void);
void UART_STA_ClearBuffer(void);
void UART_TX_ClearBuffer(void);
void UART_STA_SendRaw(const char *s);
void UART_STA_SendAT(const char *cmd);
bool UART_STA_WaitFor(const char *keyword, uint32_t timeout);
bool UART_STA_SendCmdWait(const char *cmd, const char *urc, uint32_t t_ok, uint32_t t_urc);

// ==================== JSON PARSING ====================
char* ExtractJsonValue(const char *json, const char *key);

// ==================== RECV PROCESSING ====================
bool ParsePayload(const char *payload, ParsedCommand_t *cmd);
void ProcessRecv(void);

// ==================== COMMAND HANDLERS ====================
void Handle_RQ(void);
void Handle_G(int pwm_value);
void Handle_S(const char *target_mac, int pwm_value);

// ==================== SEND RESPONSE ====================
void SendDataResponse(void);

// ==================== MAC FUNCTIONS ====================
bool Save_Mac(void);
bool Get_CCO_Mac(void);

// ==================== GPS FUNCTIONS ====================
void UART_GPS_Init(void);
void GPS_ProcessData(bool auto_stop_on_fix);
void GPS_Stop(void);
void GPS_Start(void);
bool GPS_HasFix(void);
uint8_t GPS_GetFixQuality(void);
void GPS_GetCoordinates(float *lat, float *lng);
const char* GPS_GetTimeString(void);

// ==================== NVIC PRIORITY ====================
void UART_ConfigurePriorities(void);

// ==================== UTILITY FUNCTIONS ====================
void blinkLED(GPIO_TypeDef* port, uint16_t pin, uint8_t times, uint16_t delayMs);
void Set_PWM1_Duty(uint8_t percent);
#endif
