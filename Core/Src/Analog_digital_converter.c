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
  .Middle_discrete_level = 1850,
  .Minimum_discrete_level = 1550,

  .Discrete_level = 1850
};

Analog_digital_converter ADC_EL =
{
  .Maximum_discrete_level = 2150,
  .Middle_discrete_level = 2100,
  .Minimum_discrete_level = 2050,

  .Discrete_level = 2100
};

/**
  * @brief Perform ADC conversion and process motor position data
  * @param Motor_xx Pointer to Motor structure containing configuration and status
  * @retval None
  *
  * @details
  * This function performs the following operations:
  * 1. Starts ADC conversion and reads raw value from the configured ADC converter
  * 2. Applies exponential smoothing filter to the raw discrete value
  * 3. Converts filtered discrete value to normalized range [0, 1]
  * 4. Maps normalized value to angular position within configured range
  * 5. Applies exponential smoothing filter to the angular position
  * 6. Updates motor status with filtered discrete and angular values
  *
  * The filtering uses the alpha factor (Motor_xx->Config.Alfa) for exponential smoothing:
  * filtered_value = alfa * new_value + (1 - alfa) * previous_value
  */
void Read_AD_Conversion(Analog_digital_converter *ADC_xx, Motor *Motor_xx)
{
  // Считывание значений
    HAL_ADC_Start(ADC_xx->Convertor);
    HAL_ADC_PollForConversion(ADC_xx->Convertor, 100);
    unsigned int raw_value = HAL_ADC_GetValue(ADC_xx->Convertor);
    HAL_ADC_Stop(ADC_xx->Convertor);
  // Обработка данных
    // Расчёт частоиспользуемых значений
      float alfa = Motor_xx->Config.Alfa;
      float unalfa = 1.0f - alfa;
      unsigned int discrete_range =
        ADC_xx->Maximum_discrete_level -
        ADC_xx->Minimum_discrete_level;
      float angular_range =
        Motor_xx->Config.Angular.Maximum_angular -
        Motor_xx->Config.Angular.Minimum_angular;
    // Расчёт выходных значений
      unsigned int filtered_discrete =
        (unsigned long int)(alfa * (float)raw_value +
        unalfa * (float)ADC_xx->Discrete_level);
      float norma =
        (filtered_discrete - (float)ADC_xx->Minimum_discrete_level) /
        (float)discrete_range;
      float angular =
        Motor_xx->Config.Angular.Minimum_angular +
        norma * angular_range;
      float filtered_angular =
        alfa * angular +
        unalfa * Motor_xx->Status.Angular;
  // Запись значений
    ADC_xx->Discrete_level = filtered_discrete;
    Motor_xx->Status.Angular = filtered_angular;
};
