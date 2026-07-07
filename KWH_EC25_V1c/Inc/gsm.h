#ifndef __GSM_H
#define __GSM_H
#include "main.h"
#include "KWH.h"

void clrRXBuff(void);
void sendCmd(const char* cmds);
uint8_t cekSendCmd(const char* cmds, const char* resp);
void gsmStatus(char gps_bypass);
void gsmON(void);
void gsmInit(void);
int cekSignal(void);
int HTTP_POST(char *pURL,char *pdata,uint16_t data_length,char *pRxbuff);
//uint8_t getGPS(float *lat, float *lon, float *speed_kph, float *heading, float *altitude);
uint8_t getGPS(float *lat, float *lon);
char getData(char* pV,daily_schedule_t* pSch,int* ns);
void gsmReset(void);
uint8_t setZone();
//void updateTime(RTC_TimeTypeDef* pTime,RTC_DateTypeDef* pDate);
void gsmResetDev();
void gsmOFF(void);
void getIMSI(char *pRxbuff);
void getIMEI(char *pRxbuff);

char GSMRXbuff[1024];
uint16_t GSM_rx_index;

#endif
