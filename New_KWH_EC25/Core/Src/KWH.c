/*
 * PJU.c
 *
 *  Created on: Jul 30, 2019
 *      Author: user
 */
#include <KWH.h>
#include "main.h"
#include "usart.h"
#include <stdio.h>
/* #include "gsm.h"  -- dihapus: KWH.c tidak memakai fungsi GSM */
#include "rtc.h"
#include "powermeter_ade7880.h"

extern float GAIN_VRMSR, GAIN_VRMSS, GAIN_VRMST,
						 OFFS_VRMSR, OFFS_VRMSS, OFFS_VRMST,
						 GAIN_IRMSR, GAIN_IRMSS, GAIN_IRMST, GAIN_IRMSN,
						 OFFS_IRMSR, OFFS_IRMSS, OFFS_IRMST, OFFS_IRMSN,
						 GAIN_PR, GAIN_PS, GAIN_PT,
						 OFFS_PR, OFFS_PS, OFFS_PT;

//float Wh,Pow;
float P_R,P_S,P_T,VA_R,VA_S,VA_T,VARR,VARS,VART;
float Wh_R=0, Wh_S=0, Wh_T=0;

/* Akumulasi energi WH dari daya SEMU (VA = V x I), sesuai permintaan.
 * Kalau nanti mau dari daya aktif, ganti VA_R/S/T di sini dengan P_R/S/T. */
void updateWh(){
	Wh_R+=VA_R/3600.0;
	Wh_S+=VA_S/3600.0;
	Wh_T+=VA_T/3600.0;
}

void getWH(WH_T* pval){
	pval->WH_R = Wh_R;
	pval->WH_S = Wh_S;
	pval->WH_T = Wh_T;

	//	return Wh;
}

void setWH(WH_T* pval){
	pval->WH_R = Wh_R;
	pval->WH_S = Wh_S;
	pval->WH_T = Wh_T;
//	Wh=val;
}


uint32_t getElectricValue(pwr_value_t* pPwrVal)
{
	float Vrms_R, Vrms_S, Vrms_T;
	float Vrms_R_cal, Vrms_S_cal, Vrms_T_cal;
	float Irms_R, Irms_S, Irms_T, Irms_N;
	float Irms_R_cal, Irms_S_cal, Irms_T_cal;
	float pF_R,pF_S,pF_T,f_R,f_S,f_T;

	uint32_t start_conv = HAL_GetTick();

	ADE7880_getData(&Vrms_R,&Vrms_S,&Vrms_T,&Irms_R,&Irms_S,&Irms_T,&Irms_N);
	//Mengubah ke Data ASLI untuk dikirim ke Server
	Vrms_R_cal=(Vrms_R*GAIN_VRMSR)+OFFS_VRMSR;
	Vrms_S_cal=(Vrms_S*GAIN_VRMSS)+OFFS_VRMSS;
	Vrms_T_cal=(Vrms_T*GAIN_VRMST)+OFFS_VRMST;

	if(Vrms_R_cal<120) Vrms_R_cal=0;
	if(Vrms_S_cal<120) Vrms_S_cal=0;
	if(Vrms_T_cal<120) Vrms_T_cal=0;

	Irms_R_cal=(Irms_R*GAIN_IRMSR)+OFFS_IRMSR-0.16;
	Irms_S_cal=(Irms_S*GAIN_IRMSS)+OFFS_IRMSS-0.16;
	Irms_T_cal=(Irms_T*GAIN_IRMST)+OFFS_IRMST-0.16;

//	if(Irms_R_cal<0.5) Irms_R_cal=0;
//	if(Irms_S_cal<0.5) Irms_S_cal=0;
//	if(Irms_T_cal<0.5) Irms_T_cal=0;


//	Vrms_R_cal=Vrms_R;//(Vrms_R*GAIN_VRMSR)+OFFS_VRMSR;
//	Vrms_S_cal=Vrms_S;//(Vrms_S*GAIN_VRMSS)+OFFS_VRMSS;
//	Vrms_T_cal=Vrms_T;//(Vrms_T*GAIN_VRMST)+OFFS_VRMST;
//
//	Irms_R_cal=Irms_R;//(Irms_R*GAIN_IRMSR)+OFFS_IRMSR-0.16;
//	Irms_S_cal=Irms_S;//(Irms_S*GAIN_IRMSS)+OFFS_IRMSS-0.16;
//	Irms_T_cal=Irms_T;//(Irms_T*GAIN_IRMST)+OFFS_IRMST-0.16;

//	HAL_Delay(10);
	ADE7880_getDataPFf(&pF_R,&pF_S,&pF_T,&f_R,&f_S,&f_T);

	/* Daya AKTIF (Watt) murni dari ADE7880 */
	ADE7880_getDataPOW(&P_R, &P_S, &P_T);

	if(pF_R<0.10){
		pF_R=0;
		Irms_R_cal = 0;
	}
	if(pF_S<0.10){
		pF_S=0;
		Irms_S_cal = 0;
	}
	if(pF_T<0.10){
		pF_T=0;
		Irms_T_cal = 0;
	}

	/* Daya SEMU (VA) = V x I per fasa (hitung manual) */
	VA_R = Vrms_R_cal * Irms_R_cal;
	VA_S = Vrms_S_cal * Irms_S_cal;
	VA_T = Vrms_T_cal * Irms_T_cal;

	pPwrVal->VRms_R = Vrms_R_cal;
	pPwrVal->VRms_S = Vrms_S_cal;
	pPwrVal->VRms_T = Vrms_T_cal;
	pPwrVal->IRms_R = Irms_R_cal;
	pPwrVal->IRms_S = Irms_S_cal;
	pPwrVal->IRms_T = Irms_T_cal;
	pPwrVal->Pf_R = pF_R;
	pPwrVal->Pf_S = pF_S;
	pPwrVal->Pf_T = pF_T;
	pPwrVal->Freq_R = f_R;
	pPwrVal->Freq_S = f_S;
	pPwrVal->Freq_T = f_T;
	pPwrVal->Pow_R = P_R;      /* daya AKTIF (Watt) dari ADE       */
	pPwrVal->Pow_S = P_S;
	pPwrVal->Pow_T = P_T;
	pPwrVal->VA_R  = VA_R;     /* daya SEMU (VA) = V x I           */
	pPwrVal->VA_S  = VA_S;
	pPwrVal->VA_T  = VA_T;
	pPwrVal->WH_R = Wh_R;
	pPwrVal->WH_S = Wh_S;
	pPwrVal->WH_T = Wh_T;
	pPwrVal->WHC_R = 1;
	pPwrVal->WHC_S = 1;
	pPwrVal->WHC_T = 1;
//	pPwrVal->IRms = Irms;//3;//Irms;
//	pPwrVal->Pow = Pow;//600;//Pow;
//	pPwrVal->Pf = Pf;//0.8;//Pf;
//	pPwrVal->Freq = f;//50;//f;
//	pPwrVal->WH = Wh;
//	pPwrVal->WHC = 0;
//	Wh=0;
	uint32_t t_conv = HAL_GetTick() - start_conv;

	return t_conv;
}

void resetWH(){
	Wh_R=0;
	Wh_S=0;
	Wh_T=0;
}

uint8_t updatePwmVal(RTC_TimeTypeDef* cTime,daily_schedule_t* pSch,int max)
{
	char buff[30];
	uint16_t n;
	pwm_schedule_t cSch;
	uint8_t	PWMV;
	if(pSch->pwm_force==1){
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,(pSch->pwm_force_val));
		if(pSch->pwm_force_val==0){
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1);
		}else{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,0);
		}

		PWMV = pSch->pwm_force_val;

		cSch.hour = 0;
		cSch.min = 0;

//		PWMV = (pSch->pwm_force_val==1)? 0 : 1;
	}else{
		for(int i=max-1; i>=0; i--)   /* FIX: int, bukan uint8_t (i>=0 dulu selalu true) */
		{
//			n= sprintf(buff,"pSchedule[%d] = %2d : %2d\r\n",i,pSch->pwm_schedule[i].hour,pSch->pwm_schedule[i].min);
//			HAL_UART_Transmit(&huart1,buff,n,1000);

			uint16_t sch = (pSch->pwm_schedule[i].hour*100 )+pSch->pwm_schedule[i].min;
			uint16_t time = (cTime->Hours*100 )+cTime->Minutes;

			if(i==0 && time<sch){							//Bila counter habis dan time masih < schedule, berarti antara jam 00-jadwal terkecil
				cSch.hour = pSch->pwm_schedule[max-1].hour;
				cSch.min = pSch->pwm_schedule[max-1].min;
				if(pSch->pwm_schedule[max-1].pwm_val==0){
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1);
				}else{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,0);
				}
				PWMV = pSch->pwm_schedule[max-1].pwm_val;
//				PWMV = pSch->pwm_schedule[1].pwm_val;
//				PWMV = (pSch->pwm_schedule[1].pwm_val==1) ? 0 : 1;

				break;
			}else if(time>=sch){
				cSch.hour = pSch->pwm_schedule[i].hour;
				cSch.min = pSch->pwm_schedule[i].min;
				if(pSch->pwm_schedule[i].pwm_val==0){
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1);
				}else{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,0);
				}

				PWMV = pSch->pwm_schedule[i].pwm_val;
//				PWMV = pSch->pwm_schedule[i].pwm_val;
//				PWMV = (pSch->pwm_schedule[i].pwm_val==1) ? 0 : 1;

				break;
			}
		}
	}

//	PWMV = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);

	n= sprintf(buff,"CTime = %2d : %2d\r\n",cTime->Hours,cTime->Minutes);
	HAL_UART_Transmit(&huart1,(uint8_t*)buff,n,1000);
	n= sprintf(buff,"CSchedule = %2d : %2d\r\n",cSch.hour,cSch.min);
	HAL_UART_Transmit(&huart1,(uint8_t*)buff,n,1000);
	n= sprintf(buff,"Relay Status = %d\r\n",PWMV);
	HAL_UART_Transmit(&huart1,(uint8_t*)buff,n,1000);

	return PWMV;
}
