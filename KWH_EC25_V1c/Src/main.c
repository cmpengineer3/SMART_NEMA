/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//#include "fram.h"
#include "gsm.h"
#include "encrypt.h"
#include "powermeter_ade7880.h"
#include "KWH.h"
#include "fram.h"
#include "dwt_delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
6.0.1.123	B873187C193B9D7D029B836873DAC239C790A918

6.0.1.143	C9288782F26CB829EDC4DE0C8C4800695A6F3E22
6.0.0.232	76350B760454AB811E12F0CD0400C0BA9EE4D56D
6.0.1.154	F0D6829CB2BE7747AE528214C2DAD76C7C304565
6.0.1.100	4F0816967D80FB0AF550234791E47FFBA93BFBC8
6.0.1.172	2125929F7768F13F51BA6DC87484D3597B208594
6.0.1.145	9BB1DFAA9B203F547E81ACB3B1374AED1059FA56
6.0.0.247	052F911C3502536AD696E3707B6A849EEFCC4428

KWH01310000018	B75B4C0825573E172A4B3DC8B80AD84169C74345

 * */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Update 17-05-2020
 * -ADE
hitung waktu pembacaan 1 batch

3 vrms
3 irms
3 pF
3 freq

hitung waktu total perhitungan	-> fixed

-character aneh setelah decrypt -> fixed

-filter tanggal					-> fixed
-cek timezone					-> fixed
-test tanpa NTP server			-> fixed
-CTZU di save pakai AT&W		-> fixed
 *
 * */


#define DBG_UART	huart1
#define GSM_UART	huart3

#define MAXDATA	512

#define UPDATE_VALUE_INTERVAL_MS	10000

//char i[] = "KWH01310000018";		//KWH01310000018
//char i[] = "KWH01310000019";		//KWH01310000019
//char i[] = "KWH01310000020";	 	//KWH01310000020
//char i[] = "KWH01310000021";	 	//KWH01310000021
char i[] = "KWH01310000022";	 	//KWH01310000022
//char i[] = "KWH01310000023";	 	//KWH01310000023
//char i[] = "KWH01310000024";	 	//KWH01310000024
//char i[] = "KWH01310000025";	 	//KWH01310000025
//char i[] = "KWH01310000026";	 	//KWH01310000026
//char i[] = "KWH01310000027";	 	//KWH01310000027
//char i[] = "KWH03200000028";	 	//KWH01310000028

//char i[] = "6.0.1.123";		//Demo Smart KWH EG95 ID1
//char i[] = "Smart Kwh EG95 Beta Test 01";		//Demo Smart KWH EG95 ID2
//char i[] = "Smart Kwh EG95 Beta Test 02";		//Demo Smart KWH EG95 SIKLON
//char i[] = "Demo Smart KWH 02";
//char i[] = "Demo Smart KWH 01";
//char i[] = "KWH-1908-0.0.0.0.0.8";
//char i[] = "KWH-1908-0.0.0.0.0.7";
//char i[] = "KWH-1908-0.0.0.0.0.6";
//char i[] = "KWH-1908-0.0.0.0.0.5";
//char i[] = "KWH-1908-0.0.0.0.0.4";
//char i[] = "KWH-1908-0.0.0.0.0.3";
//char i[] = "KWH-1908-0.0.0.0.0.2";
//char i[] = "KWH-1908-0.0.0.0.0.1";

const char VER[] = "1.0";
//const char T[] = "Demo Smart KWH 02";
const char T[] = "KWH";
const char GSM[] = "0123456789";
char v[MAXDATA];

char RXDataBuff[MAXDATA];
char RX_k[20];
char RX_v[MAXDATA];
char v_updated[MAXDATA];
char data[MAXDATA];

char TX_k[20];
char TX_v[MAXDATA];

uint32_t last_update_server_minutes;
uint32_t last_update_val_ms;

const char URL[] = "http://io.smartsiklon.co.id";//\r\n";


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t buff1[20],buff3[20],buff4[20];


void HAL_RstTick(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance == USART1){

//		HAL_UART_Transmit(&huart3,buff1,1,1000);
		HAL_UART_Transmit(&GSM_UART,buff1,1,10);

		HAL_UART_Receive_IT(&huart1, (uint8_t *)buff1, 1);
	}
	else if(UartHandle->Instance == UART4){

//		HAL_UART_Transmit(&GSM_UART,buff4,1,100);

		HAL_UART_Receive_IT(&DBG_UART, (uint8_t *)buff4, 1);
	}
	else if(UartHandle->Instance == USART3){

		GSMRXbuff[GSM_rx_index] = buff3[0];
		++GSM_rx_index;

		HAL_UART_Transmit(&DBG_UART,buff3,1,10);

		HAL_UART_Receive_IT(&GSM_UART, (uint8_t *)buff3, 1);
	}


}

float baca_suhuIC()
{
		uint16_t sensorValue;
		float temperature;
		HAL_ADC_Start(&hadc1);
		HAL_Delay(10);
		HAL_ADC_PollForConversion(&hadc1, 100);
		sensorValue = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		temperature=((1.43 - (float)sensorValue*(3.3/4095.0)) / 4.3) + 25;
		return temperature;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	/* USER CODE BEGIN 1 */
	char buff[400],ctime_s[20];
	char IMSI[15];
	char IMEI[15];
	uint16_t n;
	uint16_t data_length;
	uint16_t errcode;
	uint8_t retry;
//	uint16_t g;
	uint8_t PWMV;
	uint16_t ADEStuck_count;

	RTC_TimeTypeDef cTime;
	RTC_DateTypeDef cDate;

	daily_schedule_t	day_schedule;
	pwr_value_t pwr_val;
	pwr_value_t last_pwr_val;
	WH_T tWH;
	char isSGM_Init=0;
	float WH;
	int ns=1;

	char UPDATE_SERVER_INTERVAL_MINUTE;

	uint32_t t_conv;
	uint32_t t_update;

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&DBG_UART, (uint8_t *)buff1, 1);
  HAL_UART_Receive_IT(&GSM_UART, (uint8_t *)buff3, 1);

  DWT_Init();
  HAL_RstTick();

  __HAL_RCC_RTC_ENABLE();
  ADE7880_Config();
  HAL_GPIO_WritePin(RELAY_GPIO_Port,RELAY_Pin,0);

  gsmON();
  HAL_IWDG_Refresh(&hiwdg);
  getIMSI(IMSI);
  HAL_IWDG_Refresh(&hiwdg);
  getIMEI(IMEI);
  HAL_IWDG_Refresh(&hiwdg);
  setZone();
  HAL_IWDG_Refresh(&hiwdg);
  gsmStatus(BYPASS);
  HAL_IWDG_Refresh(&hiwdg);
  updateTime(&cTime,&cDate);
  HAL_IWDG_Refresh(&hiwdg);

  day_schedule.pwm_force=0;
  day_schedule.pwm_force_val=0;
  for(int i=0; i<2; i++){
	  day_schedule.pwm_schedule[i].hour = 5;
	  day_schedule.pwm_schedule[i].min = 30;
  	  day_schedule.pwm_schedule[i].sec = 0;
  	  day_schedule.pwm_schedule[i].pwm_val = 0;
   }
   isSGM_Init=1;
   PWMV =0;
   UPDATE_SERVER_INTERVAL_MINUTE=0;
   errcode=0;
   retry=0;
   ADEStuck_count=0;
   t_conv =0;

   //Get Latest WH Value
   WH = FRAM_Read_WH();
   //save it to temporary 3 phase variable
   tWH.WH_R = WH/3;
   tWH.WH_S = WH/3;
   tWH.WH_T = WH/3;
   //Set it to Current WH
   setWH(&tWH);

   /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if((HAL_GetTick() - last_update_val_ms) >= UPDATE_VALUE_INTERVAL_MS - t_conv){
		//save start conversion time
		t_conv = HAL_GetTick();

		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

//		HAL_RTC_GetTime(&hrtc,&cTime,RTC_FORMAT_BIN);
//	  	HAL_RTC_GetDate(&hrtc,&cDate, RTC_FORMAT_BIN);

	  	//Update Parameter Electric
	  	getElectricValue(&pwr_val);

	  	WH = pwr_val.WH_R + pwr_val.WH_S + pwr_val.WH_T;

	  	if(pwr_val.VRms_R == last_pwr_val.VRms_R && pwr_val.VRms_S == last_pwr_val.VRms_S && pwr_val.VRms_T == last_pwr_val.VRms_T ){
	  		ADEStuck_count++;
	  	}else{
	  		ADEStuck_count=0;
	  	}

	  	//Re-Initialize ADE7880 if Stuck
	  	if(ADEStuck_count>10){
	  		ADE7880_Config();
	  		ADEStuck_count=0;
	  	}

//	  	//get WH Value
//	  	getWH(&tWH);

	  	//Save WH Data to FRAM
	  	WattH.m_float = WH;
	  	WritemByte_FRAM(addr_energy,WattH.m_bytes);

	  	//Get current pwr value for next state compare
//	  	getElectricValue(&last_pwr_val);

	  	n = sprintf(buff,"VR=%3.2f IR=%2.2f PFR=%2.2f PR=%2.2f WHR=%3.5f\r\n",pwr_val.VRms_R,pwr_val.IRms_R,pwr_val.Pf_R,pwr_val.Pow_R,pwr_val.WH_R);
	  	HAL_UART_Transmit(&huart1,(uint8_t*)&buff,n,10000);

	  	n = sprintf(buff,"VS=%3.2f IS=%2.2f PFS=%2.2f PS=%2.2f WHS=%3.5f\r\n",pwr_val.VRms_S,pwr_val.IRms_S,pwr_val.Pf_S,pwr_val.Pow_S,pwr_val.WH_S);
	  	HAL_UART_Transmit(&huart1,(uint8_t*)&buff,n,10000);

	  	n = sprintf(buff,"VT=%3.2f IT=%2.2f PFT=%2.2f PT=%2.2f WHT=%3.5f\r\n",pwr_val.VRms_T,pwr_val.IRms_T,pwr_val.Pf_T,pwr_val.Pow_T,pwr_val.WH_T);
	  	HAL_UART_Transmit(&huart1,(uint8_t*)&buff,n,10000);

	  	n = sprintf(buff,"WH=%3.5f\r\n",WH);//pwr_val.WH_R + pwr_val.WH_S + pwr_val.WH_T);
	  	HAL_UART_Transmit(&huart1,(uint8_t*)&buff,n,10000);

	  	last_pwr_val.VRms_R = pwr_val.VRms_R;
	  	last_pwr_val.VRms_S = pwr_val.VRms_S;
	  	last_pwr_val.VRms_T = pwr_val.VRms_T;


	  	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

		//accumulate conversion time
	  	t_conv = HAL_GetTick()-t_conv;

//	  	n = sprintf(buff,"tconv=%d\r\n",t_conv);
	  	n = sprintf(buff,"tick= %d ms\r\n",HAL_GetTick());
	  	HAL_UART_Transmit(&huart1,(uint8_t*)&buff,n,10000);
//	  	n = sprintf(buff,"Min =%d last =%d\r\n",cTime.Minutes,last_update_server_minutes);
//	  	HAL_UART_Transmit(&huart1,(uint8_t*)&buff,n,10000);

//	  	HAL_RstTick();

	  	last_update_val_ms= HAL_GetTick();
	}

	if((HAL_GetTick() - last_update_server_minutes > UPDATE_SERVER_INTERVAL_MINUTE*60000 /*- t_update*/) || retry==1){

		//get start time
		uint32_t t_update_mils = HAL_GetTick();

		//Turn On GSM Module if it's off
		gsmReset();

		//Get Updated Current Time from GSM
		updateTime(&cTime,&cDate);
		HAL_IWDG_Refresh(&hiwdg);

		float latitude=LATITUDE;//Read_latitude();    //Latitude akan diisi dengan last position latitude
		float longitude=LONGITUDE;//Read_longitude();  //Longitude akan diisi dengan last position longitude


		//Construct CTIME
		n=sprintf(ctime_s,"%4d-%02d-%02d %02d:%02d:%02d",cDate.Year+2000,cDate.Month,cDate.Date,cTime.Hours,cTime.Minutes,cTime.Seconds);

		//construct V string
		n = sprintf(v,"VER=%s;T=%s;UID=%s;GSM=%s;IMEI=%s;LAT=%f;LNG=%f;CTIME=%s;VR=%3.2f;VS=%3.2f;VT=%3.2f;IR=%3.2f;IS=%3.2f;IT=%3.2f;PWR=%3.2f;PWS=%3.2f;PWT=%3.2f;PFR=%3.2f;PFS=%3.2f;PFT=%3.2f;WHC=%d;WH=%3.2f;FREQ=%3.2f;INTERVAL=%d;PWMV1=%d;PWMV2=%d;PWM1=%02d:%02d;PWM2=%02d:%02d;PWMFORCE=%d;PWMFORCEV=%d;PWMV=%d;TEMP=%3.2f;SIGNAL=%d;ERRCODE=%d",
			VER,T,i,IMSI,IMEI,latitude,longitude,ctime_s,pwr_val.VRms_R,pwr_val.VRms_S,pwr_val.VRms_T,pwr_val.IRms_R,pwr_val.IRms_S,pwr_val.IRms_T,pwr_val.Pow_R,pwr_val.Pow_S,pwr_val.Pow_T,pwr_val.Pf_R,
			pwr_val.Pf_S,pwr_val.Pf_T,pwr_val.WHC_T,WH,pwr_val.Freq_T,UPDATE_SERVER_INTERVAL_MINUTE,
			day_schedule.pwm_schedule[0].pwm_val,day_schedule.pwm_schedule[1].pwm_val,day_schedule.pwm_schedule[0].hour,day_schedule.pwm_schedule[0].min,
			day_schedule.pwm_schedule[1].hour,day_schedule.pwm_schedule[1].min,day_schedule.pwm_force,day_schedule.pwm_force_val,PWMV,baca_suhuIC(),cekSignal(),errcode);


		//Temporary save WH
		getWH(&tWH);

		//reset WH value
		resetWH();

		HAL_UART_Transmit(&huart1,(uint8_t*)&v,n,30000);
		HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,1000);

		//encrypt Data
		encrypt(v,TX_k,TX_v,n);
		data_length = sprintf(data,"i=%s&k=%s&v=%s",i,TX_k,TX_v);
		HAL_UART_Transmit(&huart1,(uint8_t*)&data,data_length,30000);
		HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,1000);

		//Push Data to Server
		int dl = HTTP_POST(URL,data,data_length,RXDataBuff);
		n=sprintf(buff,"datalength %d\r\n",dl);
		HAL_UART_Transmit(&huart1,(uint8_t*)&buff,n,30000);

		//update 17-05-2020
		/*
		 * test pakai malloc
		 * */
//		for(int z=0; z<dl-11;z++)
//		{
//			RX_v[z]=' ';
//		}
		//Allocate memory in size of dl bytes
		char *pRXv = (char *)malloc(dl-10);

		if(dl >= 30){
			int next=0,j=0;
			for (int i=0; i<dl; i++) {
				if (RXDataBuff[i] == '!') {
					next = 1;
					j = 0;
				}else {
					if (next == 0)
						RX_k[j] = RXDataBuff[i];
					else
						pRXv[j] = RXDataBuff[i];
					j++;
				}
			}


//		for(int z=0; z<MAXDATA;z++)
//		{
//			RX_v[z]=' ';
//		}
//
//			if(dl >=30){
//				retry=0;
//				errcode=0;
//				int next=0,j=0;
//				for (int i=0; i<dl; i++) {
//					if (RXDataBuff[i] == '!') {
//						next = 1;
//						j = 0;
//					}else {
//						if (next == 0)
//							RX_k[j] = RXDataBuff[i];
//						else
//							RX_v[j] = RXDataBuff[i];
//
//						j++;
//					}
//			}

			int kl = strlen(RX_k);	//get RX_K length

			//Decrypt Data
//			decrypt(RX_k,pRXv,v_updated,dl);
			decrypt(RX_k,pRXv,v_updated,dl,kl);

			n = sprintf(data,"RX Key = %s \r\n",RX_k);
			HAL_UART_Transmit(&huart1,(uint8_t*)&data,n,10000);
			n = sprintf(data,"RX Data \r\n");
			HAL_UART_Transmit(&huart1,(uint8_t*)&data,n,10000);
			n = sprintf(data,"%s",RX_v);
			HAL_UART_Transmit(&huart1,(uint8_t*)&data,dl-kl-1,10000);

			n = sprintf(data,"\r\nHasil Decrypt \r\n");
			HAL_UART_Transmit(&huart1,(uint8_t*)&data,n,10000);
			n = sprintf(data,"%s\r\n",v_updated);
			HAL_UART_Transmit(&huart1,(uint8_t*)&data,dl-9,10000);

			//Get Updated Schedule from Server
//			UPDATE_SERVER_INTERVAL_MINUTE = getData(v_updated,&day_schedule);
			UPDATE_SERVER_INTERVAL_MINUTE = getData(v_updated,&day_schedule, &ns);

			n = sprintf(data,"\r\nUpdate %d\r\n",UPDATE_SERVER_INTERVAL_MINUTE);
			HAL_UART_Transmit(&huart1,(uint8_t*)&data,n,10000);

			//Update PWMValue
			PWMV = updatePwmVal(&cTime,&day_schedule,ns);
			errcode=0;

			//Matikan GSM module
//			gsmOFF();
			isSGM_Init=0;

			free(pRXv);
		}else{
			retry=1;
			errcode=1;

			//If data isn't valid, WH will continue to meassure it's last value before being reset
			WH_T temp;
			getWH(&temp);

			temp.WH_R += tWH.WH_R;
			temp.WH_S += tWH.WH_S;
			temp.WH_T += tWH.WH_T;

			setWH(&temp);

			n = sprintf(data,"Data Invalid, Not Updated, GSM Reset\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t*)&data,n,10000);
			gsmReset();

			//Matikan GSM module
			gsmOFF();
			isSGM_Init=0;
		}

		n= sprintf(buff,"mode : %d pwm_val= %d interval=%d\r\n",day_schedule.pwm_force ,day_schedule.pwm_force_val,UPDATE_SERVER_INTERVAL_MINUTE);
		HAL_UART_Transmit(&huart1,(uint8_t*)&buff,n,10000);

		for(int j=0; j<ns; j++){
			n = sprintf(data,"H%d = %2d:%2d:%2d pwm= %3d\r\n",j+1,day_schedule.pwm_schedule[j].hour,day_schedule.pwm_schedule[j].min,day_schedule.pwm_schedule[j].sec,day_schedule.pwm_schedule[j].pwm_val);
			HAL_UART_Transmit(&huart1,(uint8_t*)&data,n,10000);
		}

		HAL_IWDG_Refresh(&hiwdg);

		//time need for sending data to server / 60
		t_update_mils = (HAL_GetTick() - t_update_mils);

//		if(t_update_mils%1000 > 500)
//			t_update = (t_update_mils + 1000)/1000;
//		else
//			t_update = t_update_mils/1000;

		n = sprintf(buff,"time update=%d ms\r\n",t_update_mils);
		HAL_UART_Transmit(&huart1,(uint8_t*)&buff,n,10000);

		HAL_RstTick();
		last_update_server_minutes = HAL_GetTick();//cTime.Minutes;
	}

//	cekSendCmd("AT+CCLK?","OK");
//    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//	HAL_Delay(1000);

	//Stuck lebih dari 10x berarti nunggu direset
	if(UPDATE_SERVER_INTERVAL_MINUTE<60){
		HAL_IWDG_Refresh(&hiwdg);
	}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint32_t wTick;
void HAL_RstTick(void)
{
  wTick = 0;
}

void HAL_IncTick(void)
{
  wTick += uwTickFreq;
}

uint32_t HAL_GetTick(void)
{
  return wTick;
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
