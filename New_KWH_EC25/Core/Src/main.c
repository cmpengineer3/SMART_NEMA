/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "mqtt.h"
#include "uart.h"
#include "config.h"
#include "modem_check.h"

#include "powermeter_ade7880.h"   /* driver ADE7880 (bit-bang)          */
#include "KWH.h"                  /* getElectricValue, WH, updatePwmVal */
#include "fram.h"                 /* simpan/baca WH persisten           */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static uint32_t last_read_tick    = 0;   /* timer baca ADE7880   */
static uint32_t last_publish_tick = 0;   /* timer publish MQTT   */

static int  retry_count_local = 0;
static int  total_retry       = 0;
static char last_topic[128]   = {0};
static char last_payload[512] = {0};

/* Buffer & struct sensor global (dipakai di loop) */
static pwr_value_t pwr_val;
static pwr_value_t last_pwr_val;
static WH_T        tWH;
static float       WH = 0.0f;
static uint16_t    ADEStuck_count = 0;

/* Status relay (DO1). 0 = OFF, 1 = ON. Dikendalikan via downlink H:S. */
static uint8_t relay_state = 0;

static ModemStatus mstat;

/* Versi firmware (untuk field "Ver" di payload) */
#define FW_VERSION "2026.07.01"

/* ──────────────────────────────────────────────────────────────────────────
 *  Kontrol relay DO1 (pin PA1 = RELAY_Pin).
 *
 *  POLARITAS active-low (sesuai referensi KWH_EC25_V1c/updatePwmVal):
 *    nilai 1 (ON)  → pin di-RESET (LOW)
 *    nilai 0 (OFF) → pin di-SET  (HIGH)
 *  Kalau modul relay board-mu active-high, tinggal balik SET/RESET di sini.
 * ────────────────────────────────────────────────────────────────────────── */
static void Relay_Set(uint8_t on)
{
    if (on)
        HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);  /* ON  */
    else
        HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);    /* OFF */
    relay_state = on ? 1 : 0;
}

/* ──────────────────────────────────────────────────────────────────────────
 *  Cek apakah DATA object berisi "H":"<c>" — TOLERAN SPASI.
 *  Cocok untuk "H":"S" maupun "H": "S" (server kadang kasih spasi).
 *  return true kalau ketemu.
 * ────────────────────────────────────────────────────────────────────────── */
static bool has_hcmd(const char *data_obj, char c)
{
    const char *p = strstr(data_obj, "\"H\"");   /* cari key "H"        */
    if (p == NULL) return false;
    p += 3;                                        /* lewati "H"          */
    while (*p == ' ') p++;                         /* skip spasi          */
    if (*p != ':') return false;
    p++;                                           /* skip ':'            */
    while (*p == ' ') p++;                         /* skip spasi          */
    if (*p != '\"') return false;
    p++;                                           /* skip quote pembuka  */
    return (*p == c);                              /* cocokkan hurufnya   */
}


/* ──────────────────────────────────────────────────────────────────────────
 *  Build payload JSON data KWH (TANPA enkripsi).
 *  Format DATA object: 9 parameter listrik + PF + FREQ + WH + metadata waktu.
 *  Dipublish via MQTT_PublishWithCRC → {"CRC":"XXXX","DATA":{...}}
 * ────────────────────────────────────────────────────────────────────────── */
static void Build_KWH_Payload(char *out, size_t out_size, pwr_value_t *pv, float wh)
{
    snprintf(out, out_size,
        "{\"UID\":\"%s\","
        "\"CTIME\":\"%04d%02d%02dT%02d%02d%02d\","
        "\"VR\":%.2f,\"VS\":%.2f,\"VT\":%.2f,"
        "\"IR\":%.2f,\"IS\":%.2f,\"IT\":%.2f,"
        "\"PWR\":%.2f,\"PWS\":%.2f,\"PWT\":%.2f,"
        "\"PFR\":%.2f,\"PFS\":%.2f,\"PFT\":%.2f,"
        "\"FREQ\":%.2f,\"WH\":%.2f}",
        uid,
        year, month, day, hour, minute, second,
        (double)pv->VRms_R, (double)pv->VRms_S, (double)pv->VRms_T,
        (double)pv->IRms_R, (double)pv->IRms_S, (double)pv->IRms_T,
        (double)pv->Pow_R,  (double)pv->Pow_S,  (double)pv->Pow_T,
        (double)pv->Pf_R,   (double)pv->Pf_S,   (double)pv->Pf_T,
        (double)pv->Freq_T, (double)wh);
}

/* ──────────────────────────────────────────────────────────────────────────
 *  Nama hari (Bahasa Indonesia) dari tanggal — algoritma Sakamoto.
 *  return 0=Minggu, 1=Senin, ... 6=Sabtu.
 * ────────────────────────────────────────────────────────────────────────── */
static const char *Nama_Hari(int y, int m, int d)
{
    static const char *hari[] = {
        "Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"
    };
    static const int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
    if (m < 3) y -= 1;
    if (m < 1 || m > 12) return "-";
    int idx = (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
    return hari[idx];
}

/* ──────────────────────────────────────────────────────────────────────────
 *  Susun payload LENGKAP (format server baru) lalu publish.
 *
 *  Bentuk:
 *  {"CRC":"0xXXXX","DATA":{...},"LU":{...},"Ver":..,"COM":..,"IMSI":..,"IMEI":..}
 *
 *  - CRC (Modbus CRC-16) dihitung dari string DATA object saja, format "0xXXXX".
 *  - VA per fasa = V x I (hitung manual).
 *  - PV per fasa = daya aktif dari ADE7880_getDataPOW() (murni dari ADE).
 *  - W           = total daya aktif (PV_R + PV_S + PV_T).
 *  - Sig (dBm)   = -113 + 2*CSQ  (konversi standar dari +CSQ).
 *  - Temp        = 0 (driver ADE7880 tidak menyediakan pembacaan suhu).
 *  - SEC         = 0 (placeholder — belum ada sumber).
 * ────────────────────────────────────────────────────────────────────────── */
static MQTT_StatusTypeDef Publish_Full_Payload(pwr_value_t *pv, float wh)
{
    /* Nilai daya sudah dipisahkan di getElectricValue():
     *   pv->Pow_R/S/T = daya AKTIF (Watt) murni dari ADE7880
     *   pv->VA_R/S/T  = daya SEMU  (VA)  = V x I
     * Jadi di sini tinggal pakai, tidak perlu panggil fungsi ADE lagi. */
    float PV_R = pv->Pow_R, PV_S = pv->Pow_S, PV_T = pv->Pow_T;   /* aktif (W)  */
    float VA_R = pv->VA_R,  VA_S = pv->VA_S,  VA_T = pv->VA_T;    /* semu (VA)  */

    /* Total daya aktif */
    float W_tot = PV_R + PV_S + PV_T;

    /* Sinyal dBm dari CSQ (99 = unknown → 0) */
    int sig_dbm = (mstat.csq == 99 || mstat.csq == 0) ? 0 : (-113 + 2 * mstat.csq);

    /* 1) Susun DATA object dulu (untuk hitung CRC-nya) */
    static char data_obj[768];
    snprintf(data_obj, sizeof(data_obj),
        "{\"H\":\"K\","
        "\"La\":%.6f,\"Lo\":%.6f,"
        "\"VR\":%.2f,\"VS\":%.2f,\"VT\":%.2f,"
        "\"IR\":%.3f,\"IS\":%.3f,\"IT\":%.3f,"
        "\"PFR\":%.2f,\"PFS\":%.2f,\"PFT\":%.2f,"
        "\"PVR\":%.2f,\"PVS\":%.2f,\"PVT\":%.2f,"
        "\"VAR\":%.2f,\"VAS\":%.2f,\"VAT\":%.2f,"
        "\"WH\":%.3f,\"W\":%.2f,"
        "\"Frek\":%.3f,\"Temp\":%.2f,\"Sig\":%d,"
        "\"DO1\":%u,\"SEC\":%d}",
        (double)LATITUDE, (double)LONGITUDE,
        (double)pv->VRms_R, (double)pv->VRms_S, (double)pv->VRms_T,
        (double)pv->IRms_R, (double)pv->IRms_S, (double)pv->IRms_T,
        (double)pv->Pf_R,   (double)pv->Pf_S,   (double)pv->Pf_T,
        (double)PV_R,       (double)PV_S,       (double)PV_T,
        (double)VA_R,       (double)VA_S,       (double)VA_T,
        (double)wh,         (double)W_tot,
        (double)pv->Freq_R, 0.0, sig_dbm,
        (unsigned)relay_state, 0);

    /* 2) Hitung CRC dari DATA object, format "0xXXXX" */
    uint16_t crc = Modbus_CRC16((const unsigned char *)data_obj, strlen(data_obj));
    char crc_str[8];
    snprintf(crc_str, sizeof(crc_str), "0x%04X", crc);

    /* 3) Rangkai payload penuh: CRC + DATA + LU + Ver + COM + IMSI + IMEI */
    static char full[1024];
    snprintf(full, sizeof(full),
        "{\"CRC\":\"%s\",\"DATA\":%s,"
        "\"LU\":{\"dy\":\"%s\",\"dt\":\"%04d:%02d:%02d\",\"tm\":\"%02d:%02d:%02d\",\"sc\":\"ntp\"},"
        "\"Ver\":\"%s\",\"COM\":\"GSM\",\"IMSI\":\"%s\",\"IMEI\":\"%s\"}",
        crc_str, data_obj,
        Nama_Hari(year, month, day),
        year, month, day, hour, minute, second,
        FW_VERSION, mstat.imsi, mstat.imei);

    /* 4) Publish langsung (BUKAN MQTT_PublishWithCRC, karena format beda) */
    MQTT_StatusTypeDef st = MQTT_Publish(mqtt_topic_pub, full, 1, 0);
    if (st == MQTT_OK)
        HAL_UART_Transmit(&huart1, (uint8_t*)"[PUBLISH] payload terkirim\r\n", 28, 500);
    else
        HAL_UART_Transmit(&huart1, (uint8_t*)"[PUBLISH] payload GAGAL\r\n", 25, 500);
    return st;
}

/* ──────────────────────────────────────────────────────────────────────────
 *  Cetak nilai sensor ke UART debug (huart1) — untuk verifikasi pembacaan.
 * ────────────────────────────────────────────────────────────────────────── */
static void Debug_Print_Sensor(pwr_value_t *pv, float wh)
{
    char buff[140];
    int n;
    /* PW = daya AKTIF (Watt, dari ADE) | VA = daya SEMU (V x I) */
    n = sprintf(buff, "R: V=%3.2f I=%2.3f PF=%1.2f PW=%2.2f VA=%2.2f\r\n",
                pv->VRms_R, pv->IRms_R, pv->Pf_R, pv->Pow_R, pv->VA_R);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, n, 1000);
    n = sprintf(buff, "S: V=%3.2f I=%2.3f PF=%1.2f PW=%2.2f VA=%2.2f\r\n",
                pv->VRms_S, pv->IRms_S, pv->Pf_S, pv->Pow_S, pv->VA_S);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, n, 1000);
    n = sprintf(buff, "T: V=%3.2f I=%2.3f PF=%1.2f PW=%2.2f VA=%2.2f\r\n",
                pv->VRms_T, pv->IRms_T, pv->Pf_T, pv->Pow_T, pv->VA_T);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, n, 1000);
    n = sprintf(buff, "Frek=%2.2f  WH=%3.5f\r\n", pv->Freq_R, wh);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, n, 1000);
}

static void DBG(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 500);
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
	  uint32_t millis(void) {
		return HAL_GetTick();
	  }
  sprintf(mqtt_topic_pub, "SIKLON/SMARTPJU/JKT/%s/UPLINK/CCO", uid);
  sprintf(mqtt_topic_sub, "SIKLON/SMARTPJU/JKT/%s/DOWNLINK", uid);

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
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* ── UART modem (huart3) ────────────────────────────────────────────────── */
   UART_Init_Buffer(&huart3);

   /* ── Power-on modem EC25 (sekuens dari project baru yang sudah jalan) ────── */
   HAL_GPIO_WritePin(LTE_RST_GPIO_Port, LTE_RST_Pin, GPIO_PIN_SET);
   HAL_GPIO_WritePin(EN3V8_GPIO_Port,   EN3V8_Pin,   GPIO_PIN_SET);
   HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, GPIO_PIN_SET);
   HAL_Delay(6000);

   /* ── Cek status modem (ala QNavigator): sync AT + SIM + sinyal + registrasi ─ */
   Modem_SyncAndCheck(&mstat);
   /* Kalau mau berhenti sampai modem benar-benar siap:
    * while (!Modem_SyncAndCheck(&mstat)) HAL_Delay(3000);              */

   for (int i = 0; i < 3; i++)
   {
       UART_SendATCommand("ATE0");
       if (UART_WaitForOK(2000)) break;
       HAL_Delay(500);
   }

   /* ── Inisialisasi ADE7880 (bit-bang) ────────────────────────────────────── */
   ADE7880_Config();

   /* ── Relay awal OFF ─────────────────────────────────────────────────────── */
   Relay_Set(0);

//   char dbg[64];
//   HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1, 0xA0, 3, 100);
//   if (st == HAL_OK) {
//       HAL_UART_Transmit(&huart1, (uint8_t*)"[FRAM] device ready\r\n", 21, 500);
//       WH = FRAM_Read_WH();
//       if (WH < 0 || WH > 1.0e9f) WH = 0;
//   } else {
//       int n = sprintf(dbg, "[FRAM] TIDAK terdeteksi, status=%d\r\n", st);
//       HAL_UART_Transmit(&huart1, (uint8_t*)dbg, n, 500);
//       WH = 0;
//   }
//   tWH.WH_R = WH / 3; tWH.WH_S = WH / 3; tWH.WH_T = WH / 3;
//   setWH(&tWH);
//   /* ── Ambil WH terakhir dari FRAM (persisten) ────────────────────────────── */
   WH = FRAM_Read_WH();
   if (WH < 0 || WH > 1.0e9f) WH = 0;   /* guard nilai sampah pertama kali */
   tWH.WH_R = WH / 3;
   tWH.WH_S = WH / 3;
   tWH.WH_T = WH / 3;
   setWH(&tWH);

   /* ── Bangun kredensial MQTT dari config.c ───────────────────────────────── */
   MQTT_Config mqtt_cfg = Config_GetMQTT();
   MQTT_Init(&mqtt_cfg);

   /* ── Sekuens koneksi MQTT (dari project baru yang sudah terbukti jalan) ──── */
   MQTT_START:
   retry_count_local = 0;
   total_retry        = 0;

   UART_SendATCommand("AT+QMTCFG=\"session\",0,1");
   UART_WaitForOK(3000);
   UART_SendATCommand("AT+QMTCFG=\"keepalive\",0,3600");
   UART_WaitForOK(3000);

   UART_SendATCommand("AT+QMTDISC=0");
   HAL_Delay(500);
   UART_SendATCommand("AT+QMTCLOSE=0");
   HAL_Delay(2000);

   DBG("[MQTT] Membuka koneksi ke broker...\r\n");
   while (MQTT_Open() != MQTT_OK)
   {
       DBG("[MQTT] QMTOPEN gagal, retry...\r\n");
       MQTT_CheckNetwork();
       UART_SendATCommand("AT+QMTCLOSE=0");
       HAL_Delay(3000);
       retry_count_local++;
       total_retry++;

       if (total_retry >= 10)
       {
           DBG("[MQTT] Gagal terus, reset device.\r\n");
           HAL_GPIO_WritePin(EN3V8_GPIO_Port, EN3V8_Pin, GPIO_PIN_RESET);
           HAL_Delay(3000);
           NVIC_SystemReset();
           while (1) {}
       }
       if (retry_count_local >= 5) { retry_count_local = 0; goto MQTT_START; }
   }
   DBG("[MQTT] Broker terbuka (QMTOPEN OK)\r\n");

   DBG("[MQTT] Menghubungkan (QMTCONN)...\r\n");
   while (MQTT_Connect() != MQTT_OK)
   {
       DBG("[MQTT] QMTCONN gagal, retry...\r\n");
       UART_SendATCommand("AT+QMTDISC=0");
       retry_count_local++;
       HAL_Delay(500);
       if (retry_count_local >= 3) { retry_count_local = 0; goto MQTT_START; }
       total_retry++;
       if (total_retry > 10) { NVIC_SystemReset(); while (1) {} }
   }
   DBG("[MQTT] Terhubung ke broker (QMTCONN OK)\r\n");

   retry_count_local = 0;
   while (MQTT_Subscribe(mqtt_topic_sub, 1) != MQTT_OK)
   {
       DBG("[MQTT] Subscribe gagal, retry...\r\n");
       HAL_Delay(500);
       retry_count_local++;
       if (retry_count_local >= 5) goto MQTT_START;
   }
   DBG("[MQTT] Subscribe topic downlink OK\r\n");

   char reg_paylaod [128];
   snprintf(reg_paylaod, sizeof(reg_paylaod),"{\"H\":\"K\",\"CITY\":\"JKT\",\"CCO\":\"%s\",\"STATUS\":\"REG\"}",uid);

   if (MQTT_PublishWithCRC(mqtt_regis_pub,reg_paylaod, 1, 0) == MQTT_OK)
       DBG("[MQTT] Registrasi terkirim\r\n");
   else
       DBG("[MQTT] Registrasi GAGAL\r\n");
   HAL_Delay(50);

   /* ── Ambil waktu dari jaringan (via AT+QLTS=2) → isi year/month/day dst ──── */
   mqtt_read_time();

   last_read_tick    = HAL_GetTick();
   last_publish_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    /* ── (1) Baca ADE7880 tiap read_interval ────────────────────────────── */
	  if ((HAL_GetTick() - last_read_tick) >= read_interval)
	      {
	          HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	          getElectricValue(&pwr_val);
	          WH = pwr_val.WH_R + pwr_val.WH_S + pwr_val.WH_T;

	          /* Deteksi ADE stuck: kalau VRms tidak berubah 10x → re-init */
	          if (pwr_val.VRms_R == last_pwr_val.VRms_R &&
	              pwr_val.VRms_S == last_pwr_val.VRms_S &&
	              pwr_val.VRms_T == last_pwr_val.VRms_T)
	              ADEStuck_count++;
	          else
	              ADEStuck_count = 0;

	          if (ADEStuck_count > 10)
	          {
	              ADE7880_Config();
	              ADEStuck_count = 0;
	          }

	          /* Simpan WH ke FRAM (persisten) */
	          WattH.m_float = WH;
	          WritemByte_FRAM(addr_energy, WattH.m_bytes);

	          /* Debug print ke huart1 */
	          Debug_Print_Sensor(&pwr_val, WH);

	          last_pwr_val.VRms_R = pwr_val.VRms_R;
	          last_pwr_val.VRms_S = pwr_val.VRms_S;
	          last_pwr_val.VRms_T = pwr_val.VRms_T;

	          HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	          last_read_tick = HAL_GetTick();
	      }

	      /* ── (2) Publish data ke server tiap send_interval ──────────────────── */
//	      if ((HAL_GetTick() - last_publish_tick) >= send_interval)
//	      {
//	          /* refresh waktu sebelum kirim */
//	          mqtt_read_time();
//
//	          /* susun payload JSON (tanpa enkripsi) & publish dengan CRC */
//	          char data_obj[512];
//	          Build_KWH_Payload(data_obj, sizeof(data_obj), &pwr_val, WH);
//
//	          if (MQTT_PublishWithCRC(mqtt_topic_pub, data_obj, 1, 0) != MQTT_OK)
//	          {
//	              resend_count++;
//	              /* kalau modem terputus, reconnect lalu subscribe ulang */
//	              if (mqtt_disconnected || resend_count >= 3)
//	              {
//	                  resend_count = 0;
//	                  if (MQTT_Reconnect() == MQTT_OK)
//	                      MQTT_Subscribe(mqtt_topic_sub, 1);
//	              }
//	          }
//	          else
//	          {
//	              resend_count = 0;
//	          }
//
//	          last_publish_tick = HAL_GetTick();
//	      }

	      /* ── (3) Handle downlink: request data (H:R) & set relay (H:S/DO1) ────── */
	      if (mqtt_data_ready)
	      {
	          if (MQTT_ProcessIncoming(last_topic,   sizeof(last_topic),
	                                   last_payload, sizeof(last_payload)))
	          {
	              /* Tampilkan downlink yang diterima */
	              DBG("[DOWNLINK] diterima: ");
	              DBG(last_payload);
	              DBG("\r\n");

	              /* Ekstrak DATA object langsung (TIDAK wajib CRC — server tes
	               * kirim tanpa field CRC). Kalau server-mu nanti pakai CRC,
	               * bungkus lagi blok ini dengan if (MQTT_VerifyPayloadCRC(...)). */
	              char data_obj_in[512];
	              if (MQTT_ExtractDataObject(last_payload, data_obj_in, sizeof(data_obj_in)))
	              {
	                  /* Request data (H:R) → device kirim payload lengkap sekarang */
	                  if (has_hcmd(data_obj_in, 'R'))
	                  {
	                      DBG("[DOWNLINK] H:R -> kirim data sekarang\r\n");
	                      Publish_Full_Payload(&pwr_val, WH);   /* status di-print di dalam fungsi */
	                  }
	                  /* Set output (H:S) → kendalikan relay DO1 (0 = OFF, 1 = ON) */
	                  else if (has_hcmd(data_obj_in, 'S'))
	                  {
	                      char *pdo = strstr(data_obj_in, "\"DO1\"");
	                      if (pdo != NULL)
	                      {
	                          pdo = strchr(pdo, ':');          /* ':' pemisah key:value */
	                          if (pdo != NULL)
	                          {
	                              pdo++;
	                              while (*pdo == ' ') pdo++;    /* toleran spasi        */
	                              int do1 = atoi(pdo);         /* 0 atau 1             */
	                              Relay_Set(do1 ? 1 : 0);

	                              char m[48];
	                              int mn = sprintf(m, "[DOWNLINK] H:S -> relay %s\r\n",
	                                               relay_state ? "ON" : "OFF");
	                              HAL_UART_Transmit(&huart1, (uint8_t*)m, mn, 500);

	                              /* Balas status relay terkini sebagai konfirmasi */
	                              char ack[64];
	                              snprintf(ack, sizeof(ack),
	                                       "{\"UID\":\"%s\",\"DO1\":%u}", uid, (unsigned)relay_state);
	                              MQTT_PublishWithCRC(mqtt_topic_pub, ack, 1, 0);
	                          }
	                      }
	                  }
	                  else
	                  {
	                      DBG("[DOWNLINK] perintah tidak dikenali\r\n");
	                  }
	              }

	              mqtt_data_ready = false;   /* reset flag setelah diproses */
	          }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
