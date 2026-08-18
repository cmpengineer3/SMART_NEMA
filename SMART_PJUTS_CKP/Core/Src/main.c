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
#include "mqtt.h"
#include "uart.h"
#include "config.h"
#include "read_sensor.h"
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static uint32_t last_read_tick    = 0;  /* timer pembacaan sensor SRNE  */
static uint32_t last_publish_tick = 0;  /* timer publish MQTT           */

static int     retry_count_local = 0;
static int     total_retry       = 0;
static char    last_topic[128]   = {0};
static char    last_payload[512] = {0};

/* Bangun 1 object JSON "DATA" dari nilai sensor SRNE + waktu saat ini,
 * sesuai format payload yang diminta.                                       */
static void Build_Sensor_Payload(char *out, size_t out_size)
{
    snprintf(out, out_size,
        "{\"UID\":\"%s\",\"HEADER\":\"%s\","
        "\"CTIME\":\"%04d%02d%02dT%02d%02d%02d\","
        "\"VBAT\":%g,\"IBAT\":%g,\"SOCBAT\":%g,"
        "\"DEVTEMP\":%g,\"BATTEMP\":%g,"
        "\"VLOAD\":%g,\"ILOAD\":%g,\"PLOAD\":%g,"
        "\"VPV\":%g,\"IPV\":%g,\"PPV\":%g,"
        "\"CHRSTATE\":%u}",
        uid, header_x,
        year, month, day, hour, minute, second,
        (double)batteryVoltage, (double)batteryCurrent, (double)batterySOC,
        (double)deviceTemperature, (double)batteryTemperature,
        (double)loadVoltage, (double)loadCurrent, (double)loadPower,
        (double)pvVolt, (double)pvCurrent, (double)pvPower,
        (unsigned)chargeState);
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  UART_Init_Buffer(&huart2);  /* modem terhubung ke USART2 di board ini */
  HAL_GPIO_WritePin(EN_3V8_GSM_GPIO_Port, EN_3V8_GSM_Pin, GPIO_PIN_SET);
  HAL_Delay(6000);
  Read_All_Sensor();

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

    while (MQTT_Open() != MQTT_OK)
    {
        MQTT_CheckNetwork();
        UART_SendATCommand("AT+QMTCLOSE=0");
        HAL_Delay(3000);
        retry_count_local++;
        total_retry++;

        if (total_retry >= 10)
        {
            HAL_GPIO_WritePin(EN_3V8_GSM_GPIO_Port, EN_3V8_GSM_Pin, GPIO_PIN_RESET);
            HAL_Delay(3000);
            NVIC_SystemReset();
            while (1) {}
        }
        if (retry_count_local >= 5)
        {
            retry_count_local = 0;
            goto MQTT_START;
        }
    }

    while (MQTT_Connect() != MQTT_OK)
    {
        UART_SendATCommand("AT+QMTDISC=0");
        retry_count_local++;
        HAL_Delay(500);
        if (retry_count_local >= 3)
        {
            retry_count_local = 0;
            goto MQTT_START;
        }
        total_retry++;
        if (total_retry > 10)
        {
            NVIC_SystemReset();
            while (1) {}
        }
    }

    retry_count_local = 0;
    while (MQTT_Subscribe(mqtt_topic_sub, 1) != MQTT_OK)
    {
        HAL_Delay(500);
        retry_count_local++;
        if (retry_count_local >= 5) goto MQTT_START;
    }

    MQTT_Publish(mqtt_topic_pub, "{\"MSG\":\"Terhubung SRNE-MQTT\"}", 1, 0);
    HAL_Delay(50);

    /* Baca sensor pertama kali (setelah modem siap → mqtt_read_time bisa jalan) */
    Read_All_Sensor();
    HAL_Delay(100);
    char data_obj[512];
    Build_Sensor_Payload(data_obj, sizeof(data_obj));
    MQTT_PublishWithCRC(mqtt_topic_pub, data_obj, 1, 0);

    last_read_tick    = HAL_GetTick();
    last_publish_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t now = HAL_GetTick();

	       /* ── Baca sensor SRNE (via Modbus di huart3) tiap read_interval ────── */
	       if ((now - last_read_tick) >= read_interval)
	       {
	           last_read_tick = now;
	           Read_All_Sensor();   /* ini juga panggil mqtt_read_time() → update year..second */
	       }

	       /* ── Publish data ke MQTT tiap send_interval ──────────────────────── */
//	       if ((now - last_publish_tick) >= send_interval)
//	       {
//	           last_publish_tick = now;
//
//	           char data_obj[512];
//	           Build_Sensor_Payload(data_obj, sizeof(data_obj));
//
//	           if (MQTT_PublishWithCRC(mqtt_topic_pub, data_obj, 1, 0) != MQTT_OK)
//	           {
//	               resend_count++;
//	           }
//	       }

	       /* ── Reconnect otomatis kalau modem lapor +QMTSTAT disconnect ──────── */
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

	       /* ── Proses pesan downlink (mis. request data on-demand H:"R") ──────── */
	      	   if (mqtt_data_ready)
	      	   {
	      	       if (MQTT_ProcessIncoming(last_topic,   sizeof(last_topic),
	      	                                last_payload, sizeof(last_payload)))
	      	       {
	      	           UART_ClearBuffer();

	      	           /* Dua format payload didukung:
	      	            *   1) Dengan CRC : {"CRC":"XXXX","DATA":{...}}   → diverifikasi
	      	            *   2) Tanpa CRC  : {"DATA":{...}}  atau  {...} langsung
	      	            *      (mis. {"H":"R"} polos) → dipakai apa adanya, TANPA
	      	            *      pengecekan integritas.                                   */
	      	           char data_obj_in[512];
	      	           bool got_data = false;

	      	           if (strstr(last_payload, "\"CRC\":") != NULL)
	      	           {
	      	               /* Ada field CRC → wajib valid, kalau salah payload ditolak */
	      	               if (MQTT_VerifyPayloadCRC(last_payload))
	      	               {
	      	                   got_data = MQTT_ExtractDataObject(last_payload, data_obj_in,
	      	                                                     sizeof(data_obj_in));
	      	               }
	      	           }
	      	           else if (strstr(last_payload, "\"DATA\":") != NULL)
	      	           {
	      	               /* Tanpa CRC tapi masih dibungkus "DATA": {...} */
	      	               got_data = MQTT_ExtractDataObject(last_payload, data_obj_in,
	      	                                                 sizeof(data_obj_in));
	      	           }
	      	           else
	      	           {
	      	               /* Payload langsung berupa object command, mis. {"H":"R"} */
	      	               strncpy(data_obj_in, last_payload, sizeof(data_obj_in) - 1);
	      	               data_obj_in[sizeof(data_obj_in) - 1] = '\0';
	      	               got_data = true;
	      	           }

	      	           if (got_data)
	      	           {
	      	               /* Command H:"R" → kirim data sensor sekarang juga */
	      	               if (strstr(data_obj_in, "\"H\":\"R\"") != NULL)
	      	               {
	      	                   Read_All_Sensor();

	      	                   char data_obj_out[512];
	      	                   Build_Sensor_Payload(data_obj_out, sizeof(data_obj_out));
	      	                   MQTT_PublishWithCRC(mqtt_topic_pub, data_obj_out, 1, 0);
	      	               }
	      	           }
	      	       }
	      	       else
	      	       {
	      	           /* Parsing gagal — reset manual */
	      	           mqtt_data_ready = false;
	      	           UART_ClearBuffer();
	      	       }
	      	   }

	       HAL_Delay(10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|EN_3V8_GSM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA7 EN_3V8_GSM_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|EN_3V8_GSM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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
