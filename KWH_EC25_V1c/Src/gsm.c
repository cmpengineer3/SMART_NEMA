/*
 * GSM.c
 *
 *  Created on: Jul 28, 2019
 *      Author: user
 */
#include <KWH.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "gsm.h"
//#include "fram.h"
#include "usart.h"
#include "iwdg.h"
#include "rtc.h"

extern char i[];

void clrRXBuff(void)
{
	for(uint16_t i=0; i<=GSM_rx_index; i++)
		GSMRXbuff[i]=0;

	GSM_rx_index = 0;
}

void sendCmd(const char* cmds)
{
	char buff[60];
	clrRXBuff();
	uint16_t n=sprintf(buff,"%s\r\n",cmds);
	HAL_UART_Transmit(&huart1, (uint8_t *)buff, n, 100);
	HAL_UART_Transmit(&huart3, (uint8_t *)buff, n, 500);
}

uint8_t cekSendCmd(const char* cmds, const char* resp)
{
	char buff[60];
	uint8_t try1=0, try2=0;
	do
	{
		clrRXBuff();
		uint16_t n=sprintf(buff,"%s\r\n",cmds);
//		HAL_UART_Transmit(&huart1, (uint8_t *)buff, n, 100);
		HAL_UART_Transmit(&huart3, (uint8_t *)buff, n, 100);

		//Refresh IDWG sebelum 20 detik
		if(try2<20)
			HAL_IWDG_Refresh(&hiwdg);

		try2++;
		HAL_Delay(1000);
	}
	while(strstr((char *)GSMRXbuff,resp) == NULL && try2 <= 20);
	if(strstr((char *)GSMRXbuff,resp) != NULL)
	{
			return  1;
	}else
	{
			return  0;
	}
}

uint8_t setZone()
{
	char buff[60];
	uint8_t try1=0, try2=0,n;


	if(cekSendCmd("AT+CTZU?","3")==NULL)
	{
		cekSendCmd("AT+CTZR=2","2");
		cekSendCmd("AT+CTZU=3","3");
		cekSendCmd("AT&W","OK");
		gsmReset();
	}


//	//Activate PDP Context
//	clrRXBuff();
//	HAL_Delay(500);
//	n=sprintf(buff,"AT+QIACT=1\r\n");
//	HAL_UART_Transmit(&huart1, (uint8_t *)buff, n, 100*n);
//	HAL_UART_Transmit(&huart3, (uint8_t *)buff, n, 100*n);
//	HAL_IWDG_Refresh(&hiwdg);
//	int i=0;
//	while(i<10){
//		if(strstr((char *)GSMRXbuff,"OK") != NULL)
//			break;
//		i++;
//		HAL_Delay(1000);
//	}

//	cekSendCmd("AT+QNTP=1,\"128.138.140.50\",28","OK");

	cekSendCmd("AT+CCLK?","OK");

	HAL_IWDG_Refresh(&hiwdg);
//	clrRXBuff();
//	HAL_Delay(500);
//	n=sprintf(buff,"AT+QIDEACT=1");
//	if(cekSendCmd(buff,"OK")==NULL)gsmResetDev();

}

void gsmResetDev(){
	HAL_GPIO_WritePin(EN3V8_GPIO_Port, EN3V8_Pin, GPIO_PIN_RESET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(LTE_RST_GPIO_Port, LTE_RST_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN3V8_GPIO_Port, EN3V8_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, GPIO_PIN_SET);
	HAL_Delay(500);

	//Cek AT Command Setelah ResetDev, Bila masih tidak respon langsung Reset Mikro
	if(cekSendCmd("AT","OK")==NULL)HAL_NVIC_SystemReset();
}

void gsmReset(){
//	HAL_GPIO_WritePin(LTE_RST_GPIO_Port, LTE_RST_Pin, GPIO_PIN_RESET);
//	HAL_IWDG_Refresh(&hiwdg);
//	HAL_GPIO_WritePin(EN3V8_GPIO_Port, EN3V8_Pin, GPIO_PIN_RESET);
//	HAL_Delay(2000);
	HAL_IWDG_Refresh(&hiwdg);
	gsmON();
//	HAL_IWDG_Refresh(&hiwdg);
//	setZone();
	HAL_IWDG_Refresh(&hiwdg);
	gsmStatus(1);
	HAL_IWDG_Refresh(&hiwdg);
}

void gsmON(void)
{
	HAL_GPIO_WritePin(LTE_RST_GPIO_Port, LTE_RST_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN3V8_GPIO_Port, EN3V8_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, GPIO_PIN_RESET);
	HAL_Delay(600);
	HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, GPIO_PIN_SET);
	HAL_Delay(500);


	uint8_t try1=0, try2=0;
	do
	{
		clrRXBuff();
		//Refresh IDWG sebelum 20 detik
		if(try2++<15)
			HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(1000);
	}
	while(strstr((char *)GSMRXbuff,"RDY") == NULL && try2<15);

	if(strstr((char *)GSMRXbuff,"RDY") == NULL){

		//Check Baudrate after RDY being read
//		if(huart3.Init.BaudRate ==115200){

			huart3.Init.BaudRate = 115200;
			HAL_UART_Init(&huart3);
			HAL_Delay(100);

//			cekSendCmd("AT","OK");

			cekSendCmd("AT+IPR=9600","OK");
			huart3.Init.BaudRate = 9600;
			HAL_UART_Init(&huart3);
			cekSendCmd("AT&W","OK");

			HAL_NVIC_SystemReset();
//		}
	}
//	else{
//		cekSendCmd("AT+IPR=9600","OK");
//		huart3.Init.BaudRate = 9600;
//		HAL_UART_Init(&huart3);
//		cekSendCmd("AT&W","OK");
//	}

	if(cekSendCmd("AT","OK")==NULL)HAL_NVIC_SystemReset();//gsmResetDev();//goto RESETdev;

//	cekSendCmd("AT+CFUN?","OK");
//	cekSendCmd("AT+CPIN?","OK");

}

void gsmOFF(void)
{

	HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, GPIO_PIN_RESET);
	HAL_Delay(1500);
	HAL_GPIO_WritePin(EN3V8_GPIO_Port, EN3V8_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);

}


void gsmStatus(char gps_bypass)
{
	float latitude=0, longitude;
	uint16_t try1=0,n;
	char buff[100];


	if(cekSendCmd("AT+CREG?","1")==NULL)gsmResetDev();//HAL_NVIC_SystemReset();//gsmResetDev();		//Verify that CGATT = 1

	/* for bypass GPS */
	if(gps_bypass)
	{
			latitude = LATITUDE;
			longitude = LONGITUDE;
	}

	n=sprintf(buff,"Latitude: %f\r\n",latitude);
	HAL_UART_Transmit(&huart1, (uint8_t*)buff, n, 1000);
	n=sprintf(buff,"Longitude: %f\r\n",longitude);
	HAL_UART_Transmit(&huart1, (uint8_t*)buff, n, 1000);

//	if(cekSendCmd("AT+CCLK?","OK")==NULL)gsmResetDev();//HAL_NVIC_SystemReset();//
//	HAL_Delay(200);

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
}

int HTTP_POST(char *pURL,char *pdata,uint16_t data_length,char *pRxbuff){
	uint16_t n,RxLength=0;
	char buff[600];
	char timeout_s=50;
	int i;

	//Activate PS DOMAIN
//	clrRXBuff();
//	HAL_Delay(500);
//	n=sprintf(buff,"AT+CGATT?\r\n");
//	HAL_UART_Transmit(&huart3, (uint8_t *)buff, n, 100*n);
//	if(cekSendCmd(buff,"OK")==NULL)gsmResetDev();

	//Activate PS DOMAIN
//	clrRXBuff();
//	HAL_Delay(500);
//	n=sprintf(buff,"AT+CGREG?\r\n");
//	HAL_UART_Transmit(&huart3, (uint8_t *)buff, n, 100*n);
//	if(cekSendCmd(buff,"OK")==NULL)gsmResetDev();

//	//Comtext ID
//	clrRXBuff();
//	HAL_Delay(500);
//	n=sprintf(buff,"AT+QHTTPCFG=\"contextid\",1");
//	if(cekSendCmd(buff,"OK")==NULL)gsmResetDev();


	//Configure PDP Context
	clrRXBuff();
	HAL_Delay(500);
	n=sprintf(buff,"AT+QICSGP=1,1,\"%s\",\"\" ,\"\" ,1",PROVIDER);
	if(cekSendCmd(buff,"OK")==NULL)gsmResetDev();

	//Activate PDP Context
//	clrRXBuff();
//	HAL_Delay(500);
//	n=sprintf(buff,"AT+QIACT=1\r\n");
//	HAL_UART_Transmit(&huart1, (uint8_t *)buff, n, 100*n);
//	HAL_UART_Transmit(&huart3, (uint8_t *)buff, n, 100*n);
//	i=0;
//	while(i<timeout_s){
//		if(strstr((char *)GSMRXbuff,"OK") != NULL)
//			break;
//		i++;
//		HAL_Delay(1000);
//	}
//	if(cekSendCmd(buff,"OK")==NULL)gsmResetDev();
//	HAL_IWDG_Refresh(&hiwdg);

	//Query the state Context
//	clrRXBuff();
//	HAL_Delay(500);
//	n=sprintf(buff,"AT+QIACT?");
//	if(cekSendCmd(buff,"+QIACT")==NULL)gsmResetDev();
//	HAL_IWDG_Refresh(&hiwdg);

	//HTTPURL
	clrRXBuff();
	HAL_Delay(500);
	n=sprintf(buff,"%s",pURL);
	n=sprintf(buff,"AT+QHTTPURL=%d,%d\r\n",n,timeout_s);
	HAL_UART_Transmit(&huart1, (uint8_t *)buff, n, 100*n);
	HAL_UART_Transmit(&huart3, (uint8_t *)buff, n, 100*n);
//	HAL_Delay(4000);
	i=0;
	while(i<timeout_s){
		if(strstr((char *)GSMRXbuff,"CONNECT") != NULL)
			break;
		i++;

		if(i<(timeout_s/2))
			HAL_IWDG_Refresh(&hiwdg);

		HAL_Delay(1000);
	}

	HAL_IWDG_Refresh(&hiwdg);
	clrRXBuff();
	HAL_Delay(500);
	n=sprintf(buff,"%s",pURL);
	HAL_UART_Transmit(&huart1, (uint8_t *)buff, n, 100*n);
	HAL_UART_Transmit(&huart3, (uint8_t *)buff, n, 100*n);
	i=0;
	while(i<timeout_s){
		if(strstr((char *)GSMRXbuff,"OK") != NULL)
			break;
		i++;
		if(i<(timeout_s/2))
			HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(1000);
	}
//	HAL_Delay(4000);
	HAL_IWDG_Refresh(&hiwdg);

	//HTTP POST
	clrRXBuff();
	HAL_Delay(500);
	n=sprintf(buff,"AT+QHTTPPOST=%d,%d,%d\r\n",data_length,timeout_s,timeout_s);
	HAL_UART_Transmit(&huart1, (uint8_t *)buff, n, 100*n);
	HAL_UART_Transmit(&huart3, (uint8_t *)buff, n, 100*n);
	i=0;
	while(i<timeout_s){
		if(strstr((char *)GSMRXbuff,"CONNECT") != NULL)
			break;
		i++;
		if(i<(timeout_s/2))
			HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(1000);
	}
//	HAL_Delay(6000);
	HAL_IWDG_Refresh(&hiwdg);

//	clrRXBuff();
	HAL_Delay(500);
	n=sprintf(buff,"%s",pdata);
	HAL_UART_Transmit(&huart1, (uint8_t *)buff, n, 10*n);
	HAL_UART_Transmit(&huart3, (uint8_t *)buff, n, 10*n);
	HAL_IWDG_Refresh(&hiwdg);
	i=0;
	while(i<timeout_s){
		if(strstr((char *)GSMRXbuff,"200") != NULL)
			break;
		i++;
		if(i<(timeout_s/2))
			HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(1000);
	}
//	HAL_Delay(15000);
	HAL_IWDG_Refresh(&hiwdg);

	if(strstr((char *)GSMRXbuff,"OK") != NULL)
	{
		if(strstr((char *)GSMRXbuff,"200")){

			char *str= strtok((char *)GSMRXbuff, "\n");
			for(int i=0; i<3; i++){
				str = strtok(NULL, "\n");

			}
//			n=sprintf(buff,"tok = %s",str);
//			HAL_UART_Transmit(&huart1, (uint8_t *)buff, n, 10*n);


			for(int i=0; i<3; i++){
				str = strtok(NULL, ",");
			}

			char n_str = atoi(str);

			HAL_Delay(500);
			HAL_IWDG_Refresh(&hiwdg);
			clrRXBuff();
			n=sprintf(buff,"AT+QHTTPREAD=80");
			if(cekSendCmd(buff,"CONNECT")==NULL)gsmResetDev();
			HAL_IWDG_Refresh(&hiwdg);
//			HAL_Delay(3000);

			uint16_t timeout = 1000;
			do
			{
				HAL_Delay(50);
				timeout--;
			}while(strstr((char *)GSMRXbuff,"OK") == NULL && timeout != 0);

			char *tok= strtok((char *)GSMRXbuff, "\n");
			for(int i=0; i<2; i++){
				tok = strtok(NULL, "\n");
			}
			strncpy (pRxbuff,tok,n_str);

			RxLength = strlen(pRxbuff);

//			n=sprintf(buff,"%s datalength=%d",tok,n_str);
//			HAL_UART_Transmit(&huart1, (uint8_t *)buff, n, 10*n);

			n=sprintf(buff,"%s",pRxbuff);
			HAL_UART_Transmit(&huart1, (uint8_t *)buff, n, 10*n);

		}
		else{
			RxLength=20;
		}

	}else{
		RxLength=30;
	}

	HAL_IWDG_Refresh(&hiwdg);

	return RxLength;
}

uint8_t getGPS(float *lat, float *lon)
{
	if(cekSendCmd("AT+CGNSINF", "OK"))
	{
		// skip GPS run status
		char *tok = strtok((char *)GSMRXbuff, ",");
		if (! tok) return 0;

		// skip fix status
		tok = strtok(NULL, ",");
		if (! tok) return 0;


      // skip date
		tok = strtok(NULL, ",");
      	if (! tok) return 0;

    // grab the latitude
      	char *latp = strtok(NULL, ",");
      	if (! latp) return 0;

    // grab longitude
      	char *longp = strtok(NULL, ",");
      	if (! longp) return 0;

      	*lat = atof(latp);
      	*lon = atof(longp);
	}else{
		HAL_NVIC_SystemReset();
	}
	return 1;
}

//Return Interval value
char getData(char* pV,daily_schedule_t* pSch,int* ns){\

	char buff[50];
	uint16_t n,j;

	uint16_t interval;

	char *tok = strtok(pV, ":");

	//DSN-Skip
	tok = strtok(NULL, ":");

	//SN-Skip
	tok = strtok(NULL, ":");

	//MODE-Skip
	char *mode = strtok(NULL, ":");

	//	char *mode1 = strtok(NULL, ",");

	//PWM
	char *p_pwm = strtok(NULL, ":");
	uint16_t pwm_val= atoi(p_pwm);

	//GPSLOCK
	char *p_gps = strtok(NULL, ":");
	uint16_t gps_lock= atoi(p_gps);

	//STP Daily
	char *p_stp = strtok(NULL, ":");

	//H1
	char *p_hour = strtok(NULL, ":");
	char *p_min = strtok(NULL, ":");
	char *p_sec = strtok(NULL, ":");

	pSch->pwm_schedule[0].hour = atoi(p_hour);
	pSch->pwm_schedule[0].min = atoi(p_min);
	pSch->pwm_schedule[0].sec = atoi(p_sec);

	char *V;
	//get &\ verify hour val
	for(int i=1; i<5; i++){
		p_hour = strtok(NULL, ":");
		pSch->pwm_schedule[i].hour = atoi(p_hour);

		if(pSch->pwm_schedule[i].hour == 0){
			j = 5-i;
			break;
		}

		p_min = strtok(NULL, ":");
		p_sec = strtok(NULL, ":");

		pSch->pwm_schedule[i].min = atoi(p_min);
		pSch->pwm_schedule[i].sec = atoi(p_sec);

		j=1;
		*ns=i+1;
	}

//	char *h;
	int i=0;
	while(i<j){
		V = strtok(NULL, ":");
		i++;
	}

	pSch->pwm_schedule[0].pwm_val = atoi(V);

	for(int i=1; i<5; i++){
		V = strtok(NULL, ":");
		pSch->pwm_schedule[i].pwm_val = atoi(V);
	}

	char *pInt = strtok(NULL, ":");
	interval = atoi(pInt);

	if(interval>60)interval=60;

	if(mode[0] == 'A' && mode[1] == 'U' && mode[2] == 'T')pSch->pwm_force = 0;
	else pSch->pwm_force = 1;

	pSch->pwm_force_val = pwm_val;

	return interval;
}

void updateTime(RTC_TimeTypeDef* pTime,RTC_DateTypeDef* pDate)
{
	char buff[60];
	uint16_t n;
	RTC_TimeTypeDef cTime;
	RTC_DateTypeDef cDate;

//	cekSendCmd("AT+CTZR=2","2");
//	cekSendCmd("AT+CTZU=3","3");
//
//	clrRXBuff();
//	HAL_Delay(500);
//	n=sprintf(buff,"AT+QIACT=1\r\n");
//	HAL_UART_Transmit(&huart1, (uint8_t *)buff, n, 100*n);
//	HAL_UART_Transmit(&huart3, (uint8_t *)buff, n, 100*n);
//	HAL_IWDG_Refresh(&hiwdg);
//	int i=0;
//	while(i<20){
//		if(strstr((char *)GSMRXbuff,"OK") != NULL)
//			break;
//		i++;
//		HAL_Delay(1000);
//	}
//
//	cekSendCmd("AT+QNTP=1,\"128.138.140.50\",28","OK");
//
//	HAL_IWDG_Refresh(&hiwdg);
//	clrRXBuff();
//	HAL_Delay(500);
//	n=sprintf(buff,"AT+QIDEACT=1");
//	if(cekSendCmd(buff,"OK")==NULL)gsmResetDev();


	uint16_t year;

	clrRXBuff();
	HAL_Delay(100);
	do{
		cekSendCmd("AT+CCLK?","OK");

		HAL_Delay(100);
		// skip GPS run status
		char *tok = strtok((char *)GSMRXbuff, "\"");

		// grab the year
		char *yearp = strtok(NULL, "/");
    	cDate.Year = atoi(yearp);

//    	if(cDate.Year == 80)goto CCLK;

		// grab the month
    	char *monthp = strtok(NULL, "/");
    	cDate.Month = atoi(monthp);

		// grab the date
    	char *dayp = strtok(NULL, ",");
		cDate.Date = atoi(dayp);

		cDate.WeekDay = RTC_WEEKDAY_MONDAY;

		// grab the hour
		char *hourp = strtok(NULL, ":");
    	cTime.Hours = atoi(hourp);

		// grab the minute
    	char *minp = strtok(NULL, ":");
		cTime.Minutes = atoi(minp);

		// grab the secon
		char *secp = strtok(NULL, "+");
    	cTime.Seconds = atoi(secp);

		pTime->Hours = cTime.Hours;
		pTime->Minutes = cTime.Minutes;
		pTime->Seconds = cTime.Seconds;
		//Update RTC
		HAL_RTC_SetTime(&hrtc,&cTime,RTC_FORMAT_BIN);

		pDate->WeekDay = RTC_WEEKDAY_MONDAY;
		pDate->Month = cDate.Month ;
		pDate->Date = cDate.Date;
		pDate->Year = cDate.Year;
		HAL_RTC_SetDate(&hrtc, &cDate, RTC_FORMAT_BIN);

		n=sprintf(buff,"DATE: %4d/%02d/%02d,%02d:%02d:%02d\r\n",cDate.Year,cDate.Month,cDate.Date,cTime.Hours,cTime.Minutes,cTime.Seconds);
		HAL_UART_Transmit(&huart1, (uint8_t*)buff, n, 100);

//		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
	}while(cDate.Year == 80);


}



void getIMSI(char *pRxbuff){
	clrRXBuff();
	HAL_Delay(100);
	if(cekSendCmd("AT+CIMI","OK"))
	{
		HAL_Delay(100);
		char *tok = strtok((char *)GSMRXbuff, "\n");
		char *IMI = strtok(NULL, "\r");
		strcpy(pRxbuff,IMI);
		HAL_Delay(100);
	}
}

void getIMEI(char *pRxbuff){
	clrRXBuff();
	HAL_Delay(100);
	if(cekSendCmd("AT+CGSN","OK"))
	{
		HAL_Delay(100);
		char *tok = strtok((char *)GSMRXbuff, "\n");
		char *IMEI = strtok(NULL, "\r");
		strcpy(pRxbuff,IMEI);
		HAL_Delay(100);
	}
}

int cekSignal(void)
{
	int signal;
	if(cekSendCmd("AT+CSQ", "OK"))
	{
		char *tok = strtok((char *)GSMRXbuff, ":");

		char *latp = strtok(NULL, ",");
		signal = atoi(latp);
		signal = 2*signal-114;
	}
	return signal;
}

