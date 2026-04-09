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
#include "com_mcu1.h"
#include "ssd1306.h"
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int retry_count=0;
// Variable untuk monitoring (bisa dilihat di Live Expression)
uint8_t mqtt_connected = 0;
uint8_t mqtt_subscribed = 0;
char last_mqtt_message[512] = {0};
//char jml_node [10];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
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
	int total_retry = 0;
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
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  SSD1309_Init(&hi2c1);


  HAL_GPIO_WritePin(EN_3V8_GSM_GPIO_Port, EN_3V8_GSM_Pin, GPIO_PIN_SET); // ON GSM
  HAL_GPIO_WritePin(EN_LTE_PWR_GPIO_Port, EN_LTE_PWR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SELECT_GSM_GPIO_Port, SELECT_GSM_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SELECT_ETH_GPIO_Port, SELECT_ETH_Pin, GPIO_PIN_RESET);

  UART_Init_Buffer();
  MCU1_Request_Init();

  HAL_Delay(3000);  // Tunggu modem siap

   // Test AT
  UART_ClearBuffer();
//  for (int i = 0; i < 3; i++) {
//      UART_ClearBuffer();
//      UART_SendATCommand("ATE0");
//      if (UART_WaitForOK(2000)) break;
//      HAL_Delay(500);
//  }
//  UART_SendATCommand("AT+QCFG=\"ledmode\",2");
  HAL_Delay(500);

//  HAL_Delay(100);        // Tambahkan delay

  SSD1309_Clear();       // Clear layar
  HAL_Delay(50);         // Delay lagi

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
  SSD1309_ShowStringCenter(6,"== PJU NEMA ==");
  SSD1309_ShowStringCenter(7,"== MEMUAT... ==");
  START:
  retry_count = 0;
  total_retry = 0;

  for (int i = 0; i < 3; i++) {
      UART_ClearBuffer();
      UART_SendATCommand("ATE0");
      if (UART_WaitForOK(2000)) break;
      HAL_Delay(500);
  }
  UART_SendATCommand("AT+QMTCFG=\"session\",0,1");
  UART_WaitForOK(3000);
  UART_SendATCommand("AT+QMTCFG=\"keepalive\",0,3600");
  UART_WaitForOK(3000);
  // ✅ FIX 4: Tutup koneksi lama sebelum open baru
  UART_SendATCommand("AT+QMTDISC=0");
  HAL_Delay(500);
  UART_SendATCommand("AT+QMTCLOSE=0");
  HAL_Delay(2000);
  while (MQTT_Open() != MQTT_OK)
  {
	  SSD1309_ClearArea(80, 0, 40);  // Clear area "FAIL" saja
	  SSD1309_ShowString(80,0,":GAGAL");
	  MQTT_CheckNetwork();

	  UART_SendATCommand("AT+QMTCLOSE=0");
      HAL_Delay(3000);  // delay sebelum retry
      retry_count++;
      total_retry++;
      if(total_retry >= 10) {
          HAL_GPIO_WritePin(EN_3V8_GSM_GPIO_Port, EN_3V8_GSM_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(EN_LTE_PWR_GPIO_Port, EN_LTE_PWR_Pin, GPIO_PIN_RESET);
          HAL_Delay(3000);
          NVIC_SystemReset(); // reset kalau gagal terus
      }
      if(retry_count>=5){
    	  retry_count=0;
//    	  mqtt_check_internet_connection();
    	  goto START;
      }

  }
  HAL_Delay(100);
  SSD1309_ClearArea(80, 0, 40);  // Clear area "FAIL" saja
  SSD1309_ShowString(80,0,":SIAP");

  while (MQTT_Connect() != MQTT_OK)
  {
	  SSD1309_ClearArea(80, 1, 40);  // Clear area "FAIL" saja
	  SSD1309_ShowString(80,1,":GAGAL");
	  UART_SendATCommand("AT+QMTDISC=0");
	  retry_count++;
      HAL_Delay(500);  // delay sebelum retry
	  if (retry_count >=3){
		  retry_count=0;
		  goto START;
	  }
	  total_retry++;
      if(total_retry > 10) {
          NVIC_SystemReset(); // reset kalau gagal terus
      }
  }
  uint32_t node_request_timer = 0;

  SSD1309_ClearArea(80, 1, 40);  // Clear area "FAIL" saja
  SSD1309_ShowString(80,1,":SIAP");

  while (MQTT_Subscribe(mqtt_topic_sub, 0) != MQTT_OK)
  {
	  SSD1309_ClearArea(80, 2, 40);  // Clear area "FAIL" saja
	  SSD1309_ShowString(80,2,":GAGAL");

      HAL_Delay(2000);
      retry_count++;

      if (retry_count >= 5)
      {
          goto START;  // Continue anyway, subscribe might work later
      }
  }
  SSD1309_ClearArea(80, 2, 40);  // Clear area "FAIL" saja
  SSD1309_ShowString(80,2,":SIAP");
  node_request_timer = HAL_GetTick();
//  char Payload_pembuka[800];
//  snprintf(Payload_pembuka, sizeof(Payload_pembuka), "{\"H\":\"D\",\"STA\":\"FFFF00000005\",\"V\":222.05,\"I\":20.2,\"PWM\":12,\"R\":0}");
//  size_t lenY = strlen(Payload_pembuka);
//  uint16_t crcY = Modbus_CRC16((const unsigned char *)Payload_pembuka, lenY);
//  char Payload_pembuka_crc[1000];
//	  snprintf(Payload_pembuka_crc, sizeof(Payload_pembuka_crc),
//			  "{\"CRC\":\"%04X\",\"DATA\":%s}", crcY, Payload_pembuka);
//  MQTT_Publish(mqtt_topic_pub,Payload_pembuka_crc,1,0);
//  HAL_Delay(500);
//
  SSD1309_ClearArea(0, 7, 120);
  SSD1309_ShowStringCenter(7,"== TERHUBUNG ==");
  HAL_Delay(1000);
  SSD1309_Clear();
  SSD1309_ShowStringCenter(3,"== Mencari NODE ==");
//  MCU1_RequestNodeCount();
  MCU2_RequestMacCCO(mac_cco, sizeof(mac_cco));
  HAL_Delay(100);
  MCU2_RegisAllNodes();
  SSD1309_Clear();
  SSD1309_ShowString(0,0,"Jumlah Node:");
//  jml_node = atoi(node_total_count);
  snprintf(jml_node, sizeof(jml_node), "%d", node_total_count);
  SSD1309_ShowString(80,0,jml_node);

  uint32_t polling_timer  = HAL_GetTick();
//  MQTT_UART_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//      if (HAL_GetTick() - polling_timer > POLLING_INTERVAL_MS)
//      {
//	      char json_X_payload[800];
//
//
//          snprintf(json_X_payload, sizeof(json_X_payload), "{\"H\":\"D\",\"STA\":\"FFFF00000004\",\"V\":221.2,\"I\":20.2,\"PWM\":100,\"R\":1}");
//		  size_t lenX = strlen(json_X_payload);
//		  uint16_t crcX = Modbus_CRC16((const unsigned char *)json_X_payload, lenX);
//		  char json_X_with_crc[1000];
//			  snprintf(json_X_with_crc, sizeof(json_X_with_crc),
//					  "{\"CRC\":\"%04X\",\"DATA\":%s}", crcX, json_X_payload);
//	      MQTT_Publish(mqtt_topic_pub,json_X_with_crc,1,0);
//		  HAL_Delay(1000);
//
//
//
//          polling_timer = HAL_GetTick();
//      }
      if (mqtt_data_ready)
        {
            Command_Handler();  // Otomatis ekstrak dan proses payload
        }
      if (mqtt_disconnected)
      {
//          SSD1306_Clear();
//          SSD1306_Print("Reconnecting...");

          if (MQTT_Reconnect() == MQTT_OK) {
//              SSD1306_Print("Reconnected!");
          } else {
              HAL_Delay(5000); // tunggu sebelum coba lagi
          }
      }
//      if (mqtt_data_ready){
//    	  char topic[128];
//		  char payload[512];
//		  UART_GetMqttMessage(topic, sizeof(topic), payload, sizeof(payload));
//		  if (strcmp(payload, "{\"H\":\"R\"}") == 0)
//		  {
//			  char json_Y_payload[800];
//			  snprintf(json_Y_payload, sizeof(json_Y_payload), "{\"H\":\"D\",\"STA\":\"FFFF00000005\",\"V\":222.05,\"I\":20.2,\"PWM\":12,\"R\":0}");
//			  size_t lenY = strlen(json_Y_payload);
//			  uint16_t crcY = Modbus_CRC16((const unsigned char *)json_Y_payload, lenY);
//			  char json_Y_with_crc[1000];
//				  snprintf(json_Y_with_crc, sizeof(json_Y_with_crc),
//						  "{\"CRC\":\"%04X\",\"DATA\":%s}", crcY, json_Y_payload);
//			  MQTT_Publish(mqtt_topic_pub,json_Y_with_crc,1,0);
//			  HAL_Delay(500);
//		   }
//      }
//      if (mqtt_data_ready)
//      {
//          // Copy ke variable untuk monitoring
//          strncpy(last_mqtt_message, mqttRxBuffer, sizeof(last_mqtt_message) - 1);
//
//
//          // Parse data MQTT (contoh format):
//          // +QMTRECV: 0,0,"smartsiklon/001000000001/downlink","Hello from broker"
//
//          // Bisa tambahkan parsing di sini sesuai kebutuhan
//
//          // Clear flag dan buffer setelah diproses
//          //UART_ClearMqttBuffer();
//      }
      HAL_Delay(100);  // Delay polling
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hi2c3.Init.Timing = 0x00503D58;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_PWR_RTC_Pin|HeartBeat_Led_Pin|GPIO_OUT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_OUT1_GPIO_Port, GPIO_OUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_3V8_GSM_Pin|ADE_RESET_Pin|SELECT_ETH_Pin|SELECT_GSM_Pin
                          |SELECT_HLWIN1_Pin|LTE_RST_Pin|EN_LTE_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SELECT_HLWIN2_Pin|FRAM_WP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_PWR_RTC_Pin HeartBeat_Led_Pin GPIO_OUT2_Pin */
  GPIO_InitStruct.Pin = EN_PWR_RTC_Pin|HeartBeat_Led_Pin|GPIO_OUT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_OUT1_Pin */
  GPIO_InitStruct.Pin = GPIO_OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_OUT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GSM_ETH_SELECT_Pin */
  GPIO_InitStruct.Pin = GSM_ETH_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GSM_ETH_SELECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_3V8_GSM_Pin ADE_RESET_Pin SELECT_ETH_Pin SELECT_GSM_Pin
                           SELECT_HLWIN1_Pin LTE_RST_Pin EN_LTE_PWR_Pin */
  GPIO_InitStruct.Pin = EN_3V8_GSM_Pin|ADE_RESET_Pin|SELECT_ETH_Pin|SELECT_GSM_Pin
                          |SELECT_HLWIN1_Pin|LTE_RST_Pin|EN_LTE_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SELECT_HLWIN2_Pin FRAM_WP_Pin */
  GPIO_InitStruct.Pin = SELECT_HLWIN2_Pin|FRAM_WP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_PB_Pin GPIO_IN1_Pin GPIO_IN2_Pin */
  GPIO_InitStruct.Pin = GPIO_PB_Pin|GPIO_IN1_Pin|GPIO_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
