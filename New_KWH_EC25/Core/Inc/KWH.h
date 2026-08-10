/*
 * PJU.h
 *
 *  Created on: Jul 30, 2019
 *      Author: user
 */

#ifndef KWH_H_
#define KWH_H_

#include "main.h"

typedef struct pwm_schedule_t{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint16_t pwm_val;
}pwm_schedule_t;

typedef struct daily_schedule_t{
	uint16_t pwm_force;
	uint16_t pwm_force_val;
	pwm_schedule_t pwm_schedule[5];
}daily_schedule_t;


typedef struct pwr_value_t{
	float VRms_R;
	float VRms_S;
	float VRms_T;
	float IRms_R;
	float IRms_S;
	float IRms_T;
	float Pow_R;      /* Daya AKTIF / Watt  fasa R (dari ADE7880_getDataPOW) */
	float Pow_S;      /* Daya AKTIF / Watt  fasa S                            */
	float Pow_T;      /* Daya AKTIF / Watt  fasa T                            */
	float VA_R;       /* Daya SEMU  / VA    fasa R (= VRms x IRms, manual)    */
	float VA_S;       /* Daya SEMU  / VA    fasa S                            */
	float VA_T;       /* Daya SEMU  / VA    fasa T                            */
	float Pf_R;
	float Pf_S;
	float Pf_T;
	float Freq_R;
	float Freq_S;
	float Freq_T;
	float WH_R;
	float WH_S;
	float WH_T;
	int WHC_R;
	int WHC_S;
	int WHC_T;
}pwr_value_t;

typedef struct WH_T{
	float WH_R;
	float WH_S;
	float WH_T;
}WH_T;

uint8_t updatePwmVal(RTC_TimeTypeDef* cTime,daily_schedule_t* pSch,int max);
uint32_t getElectricValue(pwr_value_t* pPwrVal);
/* Akumulasi energi (Wh) berdasarkan daya semu (VA) terakhir.
 * elapsed_ms = selang waktu sejak pemanggilan sebelumnya (dalam ms).
 * WAJIB dipanggil tiap siklus baca sensor, SETELAH getElectricValue(). */
void updateWh(uint32_t elapsed_ms);
void resetWH(void);

/* getWH : ambil akumulator internal  → pval
 * setWH : muat nilai pval            → akumulator internal (mis. restore FRAM) */
void getWH(WH_T* pval);
void setWH(WH_T* pval);
#endif /* PJU_H_ */
