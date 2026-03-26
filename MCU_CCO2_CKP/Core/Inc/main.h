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
#include "stm32g4xx_hal.h"

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
#define EN_PWR_RTC_Pin GPIO_PIN_13
#define EN_PWR_RTC_GPIO_Port GPIOC
#define HeartBeat_Led_Pin GPIO_PIN_14
#define HeartBeat_Led_GPIO_Port GPIOC
#define GPIO_OUT2_Pin GPIO_PIN_15
#define GPIO_OUT2_GPIO_Port GPIOC
#define GPIO_OUT1_Pin GPIO_PIN_0
#define GPIO_OUT1_GPIO_Port GPIOF
#define GSM_ETH_SELECT_Pin GPIO_PIN_0
#define GSM_ETH_SELECT_GPIO_Port GPIOA
#define EN_3V8_GSM_Pin GPIO_PIN_1
#define EN_3V8_GSM_GPIO_Port GPIOA
#define ADE_RESET_Pin GPIO_PIN_4
#define ADE_RESET_GPIO_Port GPIOA
#define SELECT_ETH_Pin GPIO_PIN_5
#define SELECT_ETH_GPIO_Port GPIOA
#define SELECT_GSM_Pin GPIO_PIN_6
#define SELECT_GSM_GPIO_Port GPIOA
#define SELECT_HLWIN1_Pin GPIO_PIN_7
#define SELECT_HLWIN1_GPIO_Port GPIOA
#define SELECT_HLWIN2_Pin GPIO_PIN_0
#define SELECT_HLWIN2_GPIO_Port GPIOB
#define GPIO_PB_Pin GPIO_PIN_12
#define GPIO_PB_GPIO_Port GPIOB
#define GPIO_IN1_Pin GPIO_PIN_13
#define GPIO_IN1_GPIO_Port GPIOB
#define GPIO_IN2_Pin GPIO_PIN_14
#define GPIO_IN2_GPIO_Port GPIOB
#define FRAM_I2C_SCL_Pin GPIO_PIN_8
#define FRAM_I2C_SCL_GPIO_Port GPIOA
#define LTE_RST_Pin GPIO_PIN_11
#define LTE_RST_GPIO_Port GPIOA
#define EN_LTE_PWR_Pin GPIO_PIN_12
#define EN_LTE_PWR_GPIO_Port GPIOA
#define FRAM_I2C_SDA_Pin GPIO_PIN_5
#define FRAM_I2C_SDA_GPIO_Port GPIOB
#define FRAM_WP_Pin GPIO_PIN_6
#define FRAM_WP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
