/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define CH0DIR_Pin GPIO_PIN_1
#define CH0DIR_GPIO_Port GPIOA
#define CH0CLK_Pin GPIO_PIN_2
#define CH0CLK_GPIO_Port GPIOA
#define CH0EN_Pin GPIO_PIN_3
#define CH0EN_GPIO_Port GPIOA
#define CH1DIR_Pin GPIO_PIN_4
#define CH1DIR_GPIO_Port GPIOA
#define CH1CLK_Pin GPIO_PIN_5
#define CH1CLK_GPIO_Port GPIOA
#define CH1EN_Pin GPIO_PIN_6
#define CH1EN_GPIO_Port GPIOA
#define CH2DIR_Pin GPIO_PIN_7
#define CH2DIR_GPIO_Port GPIOA
#define CH2CLK_Pin GPIO_PIN_0
#define CH2CLK_GPIO_Port GPIOB
#define CH2EN_Pin GPIO_PIN_1
#define CH2EN_GPIO_Port GPIOB
#define CH3DIR_Pin GPIO_PIN_2
#define CH3DIR_GPIO_Port GPIOB
#define CH3CLK_Pin GPIO_PIN_10
#define CH3CLK_GPIO_Port GPIOB
#define CH3EN_Pin GPIO_PIN_11
#define CH3EN_GPIO_Port GPIOB
#define BUTTON_Pin GPIO_PIN_10
#define BUTTON_GPIO_Port GPIOA
#define MARKER_Pin GPIO_PIN_11
#define MARKER_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
