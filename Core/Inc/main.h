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
#include "stm32h7xx_hal.h"

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
#define POWER_OUT2_Pin GPIO_PIN_13
#define POWER_OUT2_GPIO_Port GPIOC
#define POWER_OUT1_Pin GPIO_PIN_14
#define POWER_OUT1_GPIO_Port GPIOC
#define ACCEL_CS_Pin GPIO_PIN_0
#define ACCEL_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC
#define pass_Pin GPIO_PIN_0
#define pass_GPIO_Port GPIOA
#define sensor_Pin GPIO_PIN_2
#define sensor_GPIO_Port GPIOA
#define sensor_EXTI_IRQn EXTI2_IRQn
#define push_Pin GPIO_PIN_9
#define push_GPIO_Port GPIOE
#define ACC_INT_Pin GPIO_PIN_10
#define ACC_INT_GPIO_Port GPIOE
#define GYRO_INT_Pin GPIO_PIN_12
#define GYRO_INT_GPIO_Port GPIOE
#define finger_Pin GPIO_PIN_13
#define finger_GPIO_Port GPIOE
#define PAT_CYLINDER_Pin GPIO_PIN_10
#define PAT_CYLINDER_GPIO_Port GPIOC
#define GIMBAL_CYLINDER_Pin GPIO_PIN_11
#define GIMBAL_CYLINDER_GPIO_Port GPIOC
#define SUCKER_Pin GPIO_PIN_8
#define SUCKER_GPIO_Port GPIOB
#define PASS_CYLINDER_Pin GPIO_PIN_9
#define PASS_CYLINDER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
