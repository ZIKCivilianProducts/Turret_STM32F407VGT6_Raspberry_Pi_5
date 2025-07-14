/*
 * Analog_digital_converter.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Danil
 */

#include "Analog_digital_converter.h"

Analog_digital_converter ADC_AZ =
{
  .Maximum_discrete_level = 2150,
  .Minimum_discrete_level = 1550,

  .Discrete_level = 1850
};

Analog_digital_converter ADC_EL =
{
  .Maximum_discrete_level = 2100,
  .Minimum_discrete_level = 2000,

  .Discrete_level = 2100
};


void Read_AD_Conversion(Analog_digital_converter *ADC_xx, Motor *Motor_xx)
{
	HAL_ADC_Start(ADC_xx->Convertor);
	HAL_ADC_PollForConversion(ADC_xx->Convertor, 100);
	ADC_xx->Discrete_level = HAL_ADC_GetValue(ADC_xx->Convertor);
	HAL_ADC_Stop(ADC_xx->Convertor);
};
