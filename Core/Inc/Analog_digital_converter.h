/*
 * Analog_digital_converter.h
 *
 *  Created on: Jul 10, 2025
 *      Author: Danil
 */

#ifndef INC_ANALOG_DIGITAL_CONVERTER_H_
#define INC_ANALOG_DIGITAL_CONVERTER_H_

#include <Motor_step_driver.h>

typedef struct
{
  ADC_HandleTypeDef *Convertor;

  unsigned int Maximum_discrete_level, Middle_discrete_level, Minimum_discrete_level;
  unsigned int Discrete_level;
} Analog_digital_converter;

extern Analog_digital_converter ADC_AZ;
extern Analog_digital_converter ADC_EL;

void Read_AD_Conversion(Analog_digital_converter *ADC_xx, Motor *Motor_xx);

#endif /* INC_ANALOG_DIGITAL_CONVERTER_H_ */
