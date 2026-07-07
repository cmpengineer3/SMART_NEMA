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
#include "stm32f4xx_hal.h"

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
#define SELECT_ETH_Pin GPIO_PIN_2
#define SELECT_ETH_GPIO_Port GPIOE
#define DI_1_Pin GPIO_PIN_11
#define DI_1_GPIO_Port GPIOF
#define DI_2_Pin GPIO_PIN_12
#define DI_2_GPIO_Port GPIOF
#define DO_1_Pin GPIO_PIN_15
#define DO_1_GPIO_Port GPIOF
#define DO_2_Pin GPIO_PIN_0
#define DO_2_GPIO_Port GPIOG
#define EN_3V8_GSM_Pin GPIO_PIN_7
#define EN_3V8_GSM_GPIO_Port GPIOE
#define LTE_RST_Pin GPIO_PIN_8
#define LTE_RST_GPIO_Port GPIOE
#define EN_LTE_PWR_Pin GPIO_PIN_9
#define EN_LTE_PWR_GPIO_Port GPIOE
#define ADE7880_CS_Pin GPIO_PIN_12
#define ADE7880_CS_GPIO_Port GPIOB
#define ADE_RST_Pin GPIO_PIN_10
#define ADE_RST_GPIO_Port GPIOD
#define ADE_PM1_Pin GPIO_PIN_11
#define ADE_PM1_GPIO_Port GPIOD
#define ADE_PM0_Pin GPIO_PIN_12
#define ADE_PM0_GPIO_Port GPIOD
#define ADE_IRQ1_Pin GPIO_PIN_14
#define ADE_IRQ1_GPIO_Port GPIOD
#define ADE_IRQ0_Pin GPIO_PIN_15
#define ADE_IRQ0_GPIO_Port GPIOD
#define EEPROM_WP_Pin GPIO_PIN_9
#define EEPROM_WP_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
