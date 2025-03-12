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
#include "stm32f0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Ecu_Signal_Pin GPIO_PIN_13
#define Ecu_Signal_GPIO_Port GPIOC
#define Altrnator_Sens_Pin GPIO_PIN_0
#define Altrnator_Sens_GPIO_Port GPIOA
#define WPL_Sens_Pin GPIO_PIN_1
#define WPL_Sens_GPIO_Port GPIOA
#define WPR_Sens_Pin GPIO_PIN_2
#define WPR_Sens_GPIO_Port GPIOA
#define F1R_Sens_Pin GPIO_PIN_3
#define F1R_Sens_GPIO_Port GPIOA
#define F2R_Sens_Pin GPIO_PIN_4
#define F2R_Sens_GPIO_Port GPIOA
#define F1L_Sens_Pin GPIO_PIN_5
#define F1L_Sens_GPIO_Port GPIOA
#define F2L_Sens_Pin GPIO_PIN_6
#define F2L_Sens_GPIO_Port GPIOA
#define V12_NP_Sens_Pin GPIO_PIN_7
#define V12_NP_Sens_GPIO_Port GPIOA
#define V12_P_Sens_Pin GPIO_PIN_0
#define V12_P_Sens_GPIO_Port GPIOB
#define Temp_Sens_Pin GPIO_PIN_1
#define Temp_Sens_GPIO_Port GPIOB
#define V12_NP_Signal_Pin GPIO_PIN_2
#define V12_NP_Signal_GPIO_Port GPIOB
#define F1L_pwm_Pin GPIO_PIN_10
#define F1L_pwm_GPIO_Port GPIOB
#define F2L_pwm_Pin GPIO_PIN_11
#define F2L_pwm_GPIO_Port GPIOB
#define WPR_Signal_Pin GPIO_PIN_8
#define WPR_Signal_GPIO_Port GPIOA
#define WPL_Signal_Pin GPIO_PIN_9
#define WPL_Signal_GPIO_Port GPIOA
#define Reset_Pin GPIO_PIN_10
#define Reset_GPIO_Port GPIOA
#define F2R_Signal_Pin GPIO_PIN_15
#define F2R_Signal_GPIO_Port GPIOA
#define F1R_Signal_Pin GPIO_PIN_3
#define F1R_Signal_GPIO_Port GPIOB
#define F1R_pwm_Pin GPIO_PIN_4
#define F1R_pwm_GPIO_Port GPIOB
#define F2R_pwm_Pin GPIO_PIN_5
#define F2R_pwm_GPIO_Port GPIOB
#define F2L_Signal_Pin GPIO_PIN_6
#define F2L_Signal_GPIO_Port GPIOB
#define F1L_Signal_Pin GPIO_PIN_7
#define F1L_Signal_GPIO_Port GPIOB
#define WPL_pwm_Pin GPIO_PIN_8
#define WPL_pwm_GPIO_Port GPIOB
#define WPR_pwm_Pin GPIO_PIN_9
#define WPR_pwm_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
