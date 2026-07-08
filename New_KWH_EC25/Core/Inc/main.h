/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN1_Pin GPIO_PIN_1
#define IN1_GPIO_Port GPIOC
#define IN2_Pin GPIO_PIN_2
#define IN2_GPIO_Port GPIOC
#define RELAY_Pin GPIO_PIN_1
#define RELAY_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define EN10_Pin GPIO_PIN_6
#define EN10_GPIO_Port GPIOA
#define EN3V8_Pin GPIO_PIN_7
#define EN3V8_GPIO_Port GPIOA
#define LTE_PWR_Pin GPIO_PIN_0
#define LTE_PWR_GPIO_Port GPIOB
#define LTE_RST_Pin GPIO_PIN_1
#define LTE_RST_GPIO_Port GPIOB
#define GSM_TX_Pin GPIO_PIN_10
#define GSM_TX_GPIO_Port GPIOB
#define GSM_RX_Pin GPIO_PIN_11
#define GSM_RX_GPIO_Port GPIOB
#define ADE7880_CS_Pin GPIO_PIN_12
#define ADE7880_CS_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define ADE_PM1_Pin GPIO_PIN_6
#define ADE_PM1_GPIO_Port GPIOC
#define ADE_PM0_Pin GPIO_PIN_7
#define ADE_PM0_GPIO_Port GPIOC
#define ADE_RST_Pin GPIO_PIN_8
#define ADE_RST_GPIO_Port GPIOC
#define ADE_IRQ_Pin GPIO_PIN_9
#define ADE_IRQ_GPIO_Port GPIOC
#define MEM_WP_Pin GPIO_PIN_5
#define MEM_WP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
