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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "mqtt.h"
#include "uart.h"
#include "config.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//static void MX_GPIO_Init(void);
//static void MX_UART4_Init(void);
//static void MX_USART2_UART_Init(void);
//static void MX_USART3_UART_Init(void);
//static void MX_USART6_UART_Init(void);
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  UART_Init_Buffer(&huart3);  /* modem terhubung ke USART2 di board ini */
  HAL_GPIO_WritePin(LTE_RST_GPIO_Port, LTE_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN3V8_GPIO_Port, EN3V8_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LTE_PWR_GPIO_Port, LTE_PWR_Pin, GPIO_PIN_SET);
  HAL_Delay(6000);

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
		  HAL_GPIO_WritePin(EN3V8_GPIO_Port, EN3V8_Pin, GPIO_PIN_RESET);
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
//  Read_All_Sensor();
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
