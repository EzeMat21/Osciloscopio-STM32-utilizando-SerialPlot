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
#define DEBUG_TIMER_INTERRUPT_Pin GPIO_PIN_1
#define DEBUG_TIMER_INTERRUPT_GPIO_Port GPIOA
#define ADC_TRIGGER_Pin GPIO_PIN_2
#define ADC_TRIGGER_GPIO_Port GPIOA
#define DEBUG_BUFFER_LLENO_Pin GPIO_PIN_3
#define DEBUG_BUFFER_LLENO_GPIO_Port GPIOA
#define BOTON_OK_Pin GPIO_PIN_5
#define BOTON_OK_GPIO_Port GPIOA
#define BOTON_OK_EXTI_IRQn EXTI9_5_IRQn
#define BOTON_FLECHA_Pin GPIO_PIN_6
#define BOTON_FLECHA_GPIO_Port GPIOA
#define BOTON_FLECHA_EXTI_IRQn EXTI9_5_IRQn
#define BOTON_ESC_Pin GPIO_PIN_7
#define BOTON_ESC_GPIO_Port GPIOA
#define BOTON_ESC_EXTI_IRQn EXTI9_5_IRQn
#define CS_Pin GPIO_PIN_0
#define CS_GPIO_Port GPIOB
#define DC_Pin GPIO_PIN_1
#define DC_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_6
#define RESET_GPIO_Port GPIOB
#define DEBUG_ADC1_Pin GPIO_PIN_7
#define DEBUG_ADC1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
