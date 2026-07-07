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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ade7880_reader.h"
#include "mqtt.h"
#include "uart.h"
#include "config.h"
#include "ssd1306.h"
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define READ_INTERVAL_MS    1000        /* Baca sensor setiap 1 detik  */
#define PUBLISH_INTERVAL_MS 30000       /* Kirim MQTT setiap 30 detik  */
#define STUCK_THRESHOLD     10          /* Reinit jika nilai sama N kali */

static uint32_t last_read_tick    = 0;  /* timer pembacaan sensor  */
static uint32_t last_publish_tick = 0;  /* timer pengiriman MQTT   */
static float    last_VRms_R    = -1;
static float    last_VRms_S    = -1;
static float    last_VRms_T    = -1;
static uint8_t  stuck_count    = 0;

/* ─────────────────────────────────────────────────────────────────────────── */
/*  KONTROL RELAY                                                              */
/* ─────────────────────────────────────────────────────────────────────────── */
/* true  = DO2 mengikuti DI2 (normal)                                         */
/* false = DO2 selalu LOW, berapapun nilai DI2 (dinonaktifkan via MQTT R2=0)  */
static bool relay2_enabled = true;

/* ─────────────────────────────────────────────────────────────────────────── */
/*  PEMBACAAN MODEM QUECTEL                                                  */
/* ─────────────────────────────────────────────────────────────────────────── */
static int     retry_count    = 0;
static int     total_retry    = 0;
//static uint8_t mqtt_published = 0;
static char    last_topic[128]   = {0};
static char    last_payload[512] = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
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
  MX_SPI2_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  ADE7880_Config();
  SSD1309_Init(&hi2c2);
  HAL_IWDG_Refresh(&hiwdg);
  UART_Init_Buffer(&huart1);  /* modem terhubung ke USART1 di board ini */

  HAL_GPIO_WritePin(EN_3V8_GSM_GPIO_Port, EN_3V8_GSM_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_LTE_PWR_GPIO_Port, EN_LTE_PWR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LTE_RST_GPIO_Port,    LTE_RST_Pin,    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SELECT_ETH_GPIO_Port, SELECT_ETH_Pin, GPIO_PIN_RESET);

    SSD1309_Clear();
    HAL_Delay(50);
    SSD1309_DrawBitmap(siklon_logo, 6);
    SSD1309_ShowStringCenter(7,"HANDAL-TERANGMUDAH");
    HAL_Delay(4000);
    SSD1309_Clear();
    SSD1309_ShowString(0,0,"Mulai MQTT");
    SSD1309_ShowString(80,0,":...");
    SSD1309_ShowString(0,1,"Koneksi MQTT");
    SSD1309_ShowString(80,1,":...");
    SSD1309_ShowString(0,2,"Daftar Topik");
    SSD1309_ShowString(80,2,":...");
    SSD1309_ShowStringCenter(6,"== SMART KWH Ver.2 ==");
    SSD1309_ShowStringCenter(7,"== MEMUAT... ==");

    HAL_Delay(6000);
    UART_ClearBuffer();
    HAL_Delay(500);

    for (int i = 0; i < 3; i++)
    {
        UART_SendATCommand("ATE0");
        if (UART_WaitForOK(2000)) break;
        HAL_Delay(500);
    }

    /* ── Build MQTT credentials dari config.c ───────────────────────────────── */
    MQTT_Config mqtt_cfg = Config_GetMQTT();
    MQTT_Init(&mqtt_cfg);

    /* ── MQTT connection sequence ────────────────────────────────────────────── */
    MQTT_START:
    retry_count = 0;
    total_retry = 0;

    UART_SendATCommand("AT+QMTCFG=\"session\",0,1");
    UART_WaitForOK(3000);
    UART_SendATCommand("AT+QMTCFG=\"keepalive\",0,3600");
    UART_WaitForOK(3000);

    UART_SendATCommand("AT+QMTDISC=0");
    HAL_Delay(500);
    UART_SendATCommand("AT+QMTCLOSE=0");
    HAL_Delay(2000);

    while (MQTT_Open() != MQTT_OK)
    {
        HAL_IWDG_Refresh(&hiwdg);   /* cegah IWDG reset selama retry (~32s limit) */
        SSD1309_ClearArea(80, 0, 40);
  	    SSD1309_ShowString(80,0,":GAGAL");
        MQTT_CheckNetwork();
        UART_SendATCommand("AT+QMTCLOSE=0");
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(3000);
        retry_count++;
        total_retry++;

        if (total_retry >= 10)
        {
            HAL_GPIO_WritePin(EN_3V8_GSM_GPIO_Port, EN_3V8_GSM_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(EN_LTE_PWR_GPIO_Port, EN_LTE_PWR_Pin, GPIO_PIN_RESET);
            HAL_IWDG_Refresh(&hiwdg);
            HAL_Delay(3000);
            NVIC_SystemReset();
            while (1) {}
        }
        if (retry_count >= 5)
        {
            retry_count = 0;
            goto MQTT_START;
        }
    }
    SSD1309_ClearArea(80, 0, 40);
    SSD1309_ShowString(80,0,":SIAP");

    while (MQTT_Connect() != MQTT_OK)
    {
        HAL_IWDG_Refresh(&hiwdg);
  	    SSD1309_ClearArea(80, 1, 40);
  	    SSD1309_ShowString(80,1,":GAGAL");
        UART_SendATCommand("AT+QMTDISC=0");
        retry_count++;
        HAL_Delay(500);
        if (retry_count >= 3)
        {
            retry_count = 0;
            goto MQTT_START;
        }
        total_retry++;
        if (total_retry > 10)
        {
            NVIC_SystemReset();
            while (1) {}
        }
    }
    SSD1309_ClearArea(80, 1, 40);
    SSD1309_ShowString(80,1,":SIAP");

    retry_count = 0;
    while (MQTT_Subscribe(mqtt_topic_sub, 1) != MQTT_OK)
    {
        HAL_IWDG_Refresh(&hiwdg);
  	    SSD1309_ClearArea(80, 2, 40);
  	    SSD1309_ShowString(80,2,":GAGAL");
        HAL_Delay(2000);
        retry_count++;
        if (retry_count >= 5) goto MQTT_START;
    }
    SSD1309_ClearArea(80, 2, 40);
    SSD1309_ShowString(80,2,":SIAP");
    SSD1309_ClearArea(0, 7, 120);
    SSD1309_ShowStringCenter(7,"== TERHUBUNG ==");
    HAL_Delay(1000);

    const char *payload = "{Terhubung dengan Smart-KWV V.2}";
    MQTT_Publish(mqtt_topic_pub, payload, 1, 0);
    SSD1309_Clear();
    HAL_Delay(50);
    /* ── Setup tampilan sensor OLED setelah koneksi berhasil ─────────────── */
    SSD1309_ClearRect(0, 0, 128, 6);           /* bersihkan page 0-5        */
    SSD1309_ShowString(0, 0, "  V(V)   I(A)  P(W)");
    SSD1309_ShowString(0, 1, "R:  --    --     --");
    SSD1309_ShowString(0, 2, "S:  --    --     --");
    SSD1309_ShowString(0, 3, "T:  --    --     --");
    SSD1309_ShowStringCenter(6, "== SMART KWH Ver.2 ==");
    SSD1309_ShowStringCenter(7, "== BERJALAN ==");

    /* Set timer agar publish data sensor dimulai 30 detik setelah koneksi */
    last_read_tick    = HAL_GetTick();
    last_publish_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      /* ── Pembacaan sensor ADE7880 setiap 1 detik ──────────────────────────── */
      if ((HAL_GetTick() - last_read_tick) >= READ_INTERVAL_MS)
      {
          last_read_tick = HAL_GetTick();

          HAL_IWDG_Refresh(&hiwdg);
          ADE7880_ReadAll();
          HAL_IWDG_Refresh(&hiwdg);

          /* Stuck detection: jika VRms tidak berubah N kali berturut, reinit */
          if (ade_data.VRms_R == last_VRms_R &&
              ade_data.VRms_S == last_VRms_S &&
              ade_data.VRms_T == last_VRms_T)
          {
              stuck_count++;
              if (stuck_count > STUCK_THRESHOLD)
              {
                  ADE7880_Config();
                  stuck_count = 0;
              }
          }
          else
          {
              stuck_count = 0;
          }

          last_VRms_R = ade_data.VRms_R;
          last_VRms_S = ade_data.VRms_S;
          last_VRms_T = ade_data.VRms_T;

          /* ── Update nilai sensor ke OLED setiap 1 detik ──────────────── */
          char oled_buf[24];
          snprintf(oled_buf, sizeof(oled_buf), "R:%5.1f %5.3f %5.0f",
                   ade_data.VRms_R, ade_data.IRms_R, ade_data.Pow_R);
          SSD1309_ClearArea(0, 1, 128);
          SSD1309_ShowString(0, 1, oled_buf);

          snprintf(oled_buf, sizeof(oled_buf), "S:%5.1f %5.3f %5.0f",
                   ade_data.VRms_S, ade_data.IRms_S, ade_data.Pow_S);
          SSD1309_ClearArea(0, 2, 128);
          SSD1309_ShowString(0, 2, oled_buf);

          snprintf(oled_buf, sizeof(oled_buf), "T:%5.1f %5.3f %5.0f",
                   ade_data.VRms_T, ade_data.IRms_T, ade_data.Pow_T);
          SSD1309_ClearArea(0, 3, 128);
          SSD1309_ShowString(0, 3, oled_buf);
      }

      /* ── Pengiriman MQTT berdasarkan request (H:"R") dari downlink ────────
       * Timer 30 detik dinonaktifkan — data dikirim hanya saat ada permintaan.
       * Untuk mengaktifkan kembali: hapus komentar blok di bawah ini.        */
//      if ((HAL_GetTick() - last_publish_tick) >= PUBLISH_INTERVAL_MS)
//      {
//          last_publish_tick = HAL_GetTick();
//          /* ... snprintf + MQTT_Publish ... */
//      }
      if (mqtt_disconnected)
      {
          mqtt_disconnected = false;
          if (MQTT_Reconnect() == MQTT_OK)
          {
              MQTT_Subscribe(mqtt_topic_sub, 1);
          }
          else
          {
              goto MQTT_START;
          }
      }


      /* ── Proses pesan masuk dari MQTT downlink ──────────────────────────── */
      if (mqtt_data_ready)
      {
          if (MQTT_ProcessIncoming(last_topic,   sizeof(last_topic),
                                   last_payload, sizeof(last_payload)))
          {
              /* Buffer sudah disalin ke last_payload — langsung hapus rxBuffer
               * agar pesan MQTT berikutnya bisa masuk ke buffer yang bersih
               * (termasuk saat MQTT_Publish sedang berjalan di bawah).       */
              UART_ClearBuffer();

              /* ── Validasi CRC payload masuk ──────────────────────────────
               * Format yg diharapkan: {"CRC":"XXXX","DATA":{...}}
               * Jika CRC tidak cocok / tidak ada → abaikan payload.          */
              if (!MQTT_VerifyPayloadCRC(last_payload))
              {
                  /* CRC mismatch: payload korup atau format tidak sesuai.
                   * Abaikan dan tunggu pesan berikutnya. */
                  goto SKIP_PAYLOAD;
              }

              /* Ekstrak object DATA agar pencarian field hanya pada konten
               * yang relevan (lebih aman terhadap field di luar DATA).       */
              char data_obj[512];
              if (!MQTT_ExtractDataObject(last_payload, data_obj, sizeof(data_obj)))
                  goto SKIP_PAYLOAD;

              /* ── Command H:"R" — kirim data sensor ADE7880 on-demand ──── */
              if (strstr(data_obj, "\"H\":\"R\"") != NULL)
              {
                  /* Bangun hanya isi object DATA. Wrapper {"CRC":..,"DATA":..}
                   * akan ditambahkan otomatis oleh MQTT_PublishWithCRC().    */
                  char data_payload[800];
                  snprintf(data_payload, sizeof(data_payload),
                      "{"
                      "\"H\":\"K\","
                      "\"VR\":%.2f,\"VS\":%.2f,\"VT\":%.2f,"
                      "\"IR\":%.4f,\"IS\":%.4f,\"IT\":%.4f,"
                      "\"PR\":%.2f,\"PS\":%.2f,\"PT\":%.2f,"
                      "\"WHR\":%.4f,\"WHS\":%.4f,\"WHT\":%.4f,"
                      "\"PFR\":%.3f,\"PFS\":%.3f,\"PFT\":%.3f"
                      "}",
                      ade_data.VRms_R, ade_data.VRms_S, ade_data.VRms_T,
                      ade_data.IRms_R, ade_data.IRms_S, ade_data.IRms_T,
                      ade_data.Pow_R,  ade_data.Pow_S,  ade_data.Pow_T,
                      ade_data.WH_R,   ade_data.WH_S,   ade_data.WH_T,
                      ade_data.Pf_R,   ade_data.Pf_S,   ade_data.Pf_T);
                  HAL_IWDG_Refresh(&hiwdg);
                  MQTT_StatusTypeDef pub_result =
                      MQTT_PublishWithCRC(mqtt_topic_pub, data_payload, 1, 0);
                  HAL_IWDG_Refresh(&hiwdg);

                  /* Reset akumulasi KWH setelah publish berhasil (Pilihan A).
                   * WH mulai hitung ulang dari 0 untuk interval berikutnya.
                   *
                   * TODO: Setelah EEPROM terpasang, tambahkan di sini:
                   *   EEPROM_Save_WH_Offset();  ← simpan total kumulatif
                   *   sebelum reset, agar total tidak hilang saat power off. */
                  if (pub_result == MQTT_OK)
                  {
                      ADE7880_ResetWH();
                  }
              }
              /* ── Command H:"S" — set kontrol relay DO2 ──────────────── */
              else if (strstr(data_obj, "\"H\":\"S\"") != NULL)
              {
                  if      (strstr(data_obj, "\"R2\":1") != NULL)
                      relay2_enabled = true;   /* DO2 ikut DI2 */
                  else if (strstr(data_obj, "\"R2\":0") != NULL)
                      relay2_enabled = false;  /* DO2 paksa LOW */
              }

          SKIP_PAYLOAD:
              ;   /* label akhir; payload selesai diproses / dilewati */
          }
          else
          {
              /* Parsing gagal (format tidak dikenal) — reset manual */
              mqtt_data_ready = false;
              UART_ClearBuffer();
          }
      }

      HAL_Delay(10);
      /* ── Kontrol DO2 ─────────────────────────────────────────────────────
       * relay2_enabled=true  → DO2 mengikuti DI2 (normal)
       * relay2_enabled=false → DO2 selalu LOW (dikunci via MQTT R2=0)       */
      GPIO_PinState di2 = HAL_GPIO_ReadPin(DI_2_GPIO_Port, DI_2_Pin);
      HAL_GPIO_WritePin(DO_2_GPIO_Port, DO_2_Pin,
                        relay2_enabled ? di2 : GPIO_PIN_RESET);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SELECT_ETH_Pin|EN_3V8_GSM_Pin|LTE_RST_Pin|EN_LTE_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO_1_GPIO_Port, DO_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO_2_GPIO_Port, DO_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADE7880_CS_GPIO_Port, ADE7880_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ADE_RST_Pin|ADE_PM1_Pin|ADE_PM0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EEPROM_WP_GPIO_Port, EEPROM_WP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SELECT_ETH_Pin EN_3V8_GSM_Pin LTE_RST_Pin EN_LTE_PWR_Pin */
  GPIO_InitStruct.Pin = SELECT_ETH_Pin|EN_3V8_GSM_Pin|LTE_RST_Pin|EN_LTE_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_1_Pin DI_2_Pin */
  GPIO_InitStruct.Pin = DI_1_Pin|DI_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : DO_1_Pin */
  GPIO_InitStruct.Pin = DO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DO_2_Pin */
  GPIO_InitStruct.Pin = DO_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADE7880_CS_Pin */
  GPIO_InitStruct.Pin = ADE7880_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADE7880_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADE_RST_Pin ADE_PM1_Pin ADE_PM0_Pin */
  GPIO_InitStruct.Pin = ADE_RST_Pin|ADE_PM1_Pin|ADE_PM0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ADE_IRQ1_Pin ADE_IRQ0_Pin */
  GPIO_InitStruct.Pin = ADE_IRQ1_Pin|ADE_IRQ0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : EEPROM_WP_Pin */
  GPIO_InitStruct.Pin = EEPROM_WP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EEPROM_WP_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
