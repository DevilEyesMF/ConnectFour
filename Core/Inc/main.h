/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define EN_YEL_Pin GPIO_PIN_15
#define EN_YEL_GPIO_Port GPIOC
#define BTN_LEFT_Pin GPIO_PIN_0
#define BTN_LEFT_GPIO_Port GPIOA
#define BTN_MID_Pin GPIO_PIN_1
#define BTN_MID_GPIO_Port GPIOA
#define BTN_RIGHT_Pin GPIO_PIN_2
#define BTN_RIGHT_GPIO_Port GPIOA
#define BTN_RESTART_Pin GPIO_PIN_3
#define BTN_RESTART_GPIO_Port GPIOA
#define ROW0_Pin GPIO_PIN_4
#define ROW0_GPIO_Port GPIOA
#define ROW1_Pin GPIO_PIN_5
#define ROW1_GPIO_Port GPIOA
#define ROW2_Pin GPIO_PIN_6
#define ROW2_GPIO_Port GPIOA
#define ROW3_Pin GPIO_PIN_7
#define ROW3_GPIO_Port GPIOA
#define ROW4_Pin GPIO_PIN_8
#define ROW4_GPIO_Port GPIOA
#define ROW5_Pin GPIO_PIN_9
#define ROW5_GPIO_Port GPIOA
#define ROW6_Pin GPIO_PIN_10
#define ROW6_GPIO_Port GPIOA
#define EN_RED_Pin GPIO_PIN_3
#define EN_RED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
