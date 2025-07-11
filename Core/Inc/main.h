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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "Raspberry_Pi.h"
#include "Motor_step_driver.h"
#include "Engine_conditions.h"
#include "Analog_digital_converter.h"
#include "Transfer_to_Raspberry_Pi.h"
#include "Encoder.h"
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
#define Trigger_position_Pin GPIO_PIN_0
#define Trigger_position_GPIO_Port GPIOC
#define Work_permit_Pin GPIO_PIN_1
#define Work_permit_GPIO_Port GPIOC
#define Encoder_AZ_A_Pin GPIO_PIN_0
#define Encoder_AZ_A_GPIO_Port GPIOA
#define Encoder_AZ_B_Pin GPIO_PIN_1
#define Encoder_AZ_B_GPIO_Port GPIOA
#define TIM_EL_Pin GPIO_PIN_5
#define TIM_EL_GPIO_Port GPIOA
#define TIM_AZ_Pin GPIO_PIN_6
#define TIM_AZ_GPIO_Port GPIOA
#define ADC_AZ_Pin GPIO_PIN_4
#define ADC_AZ_GPIO_Port GPIOC
#define ADC_EL_Pin GPIO_PIN_5
#define ADC_EL_GPIO_Port GPIOC
#define DIR_AZ_Pin GPIO_PIN_8
#define DIR_AZ_GPIO_Port GPIOE
#define DIR_EL_Pin GPIO_PIN_9
#define DIR_EL_GPIO_Port GPIOE
#define ENA_AZ_Pin GPIO_PIN_11
#define ENA_AZ_GPIO_Port GPIOE
#define ENA_EL_Pin GPIO_PIN_12
#define ENA_EL_GPIO_Port GPIOE
#define Trigger_Pin GPIO_PIN_13
#define Trigger_GPIO_Port GPIOE
#define LD4_green_Pin GPIO_PIN_12
#define LD4_green_GPIO_Port GPIOD
#define LD3_orange_Pin GPIO_PIN_13
#define LD3_orange_GPIO_Port GPIOD
#define LD5_red_Pin GPIO_PIN_14
#define LD5_red_GPIO_Port GPIOD
#define LD6_blue_Pin GPIO_PIN_15
#define LD6_blue_GPIO_Port GPIOD
#define Encoder_EL_A_Pin GPIO_PIN_6
#define Encoder_EL_A_GPIO_Port GPIOB
#define Encoder_EL_B_Pin GPIO_PIN_7
#define Encoder_EL_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
