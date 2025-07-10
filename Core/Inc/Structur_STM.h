/*
 * Structur_STM.h
 *
 *  Created on: Jul 8, 2025
 *      Author: u033008
 */

#ifndef INC_STRUCTUR_STM_H_
#define INC_STRUCTUR_STM_H_

#include "stm32f4xx_hal.h"

typedef struct // Config_GPIO
{
  GPIO_TypeDef *DIR_port, *ENA_port;
  uint32_t      DIR_pin,   ENA_pin;
} Config_GPIO;

typedef struct // Config_TIM
{
  TIM_HandleTypeDef *Timer;

  uint32_t Maximum_frequency, Minimum_frequency;

  uint32_t Increment_frequency;
} Config_TIM;

typedef struct // Config_ADC
{
  ADC_HandleTypeDef *Convertor;

  uint32_t Maximum_discrete_level, Middle_discrete_level, Minimum_discrete_level;

  float Inv_discrete_range;
} Config_ADC;

typedef struct // Config_Angular
{
  float Maximum_angular, Minimum_angular;

  float Guidance_accuracy; // Точность_прицелывания
  float Deviation; // Отступ_от_границ
} Config_Angular;

typedef struct // Config
{
  Config_GPIO GPIO;
  Config_TIM PWM;
  Config_ADC Convertor;
  Config_Angular Angular;

  float Alfa;
} Config;

typedef struct // Status
{
  uint32_t Frequency;
  uint32_t Last_freq_update_time;

  uint8_t Moving;

  uint32_t Discrete_level, filtered_Discrete_level;
  float Angular,        filter_Angular;
} Status;

typedef struct // Motor
{
  Status Status;
  Config Config;
} Motor;

extern Motor Motor_AZ;
extern Motor Motor_EL;

#endif /* INC_STRUCTUR_STM_H_ */
