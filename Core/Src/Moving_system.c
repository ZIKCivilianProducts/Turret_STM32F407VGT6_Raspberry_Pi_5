/*
 * Moving_system.c
 *
 *  Created on: Jul 8, 2025
 *      Author: u033008
 */

#include "Moving_system.h"

void Read_AD_Conversion(Motor *Motor_xx)
{
	// Считывание значений
	HAL_ADC_Start(Motor_xx->Config.Convertor.Convertor);
	HAL_ADC_PollForConversion(Motor_xx->Config.Convertor.Convertor, 100);
	unsigned long int raw_value = HAL_ADC_GetValue(Motor_xx->Config.Convertor.Convertor);
	HAL_ADC_Stop(Motor_xx->Config.Convertor.Convertor);

	// Расчёт частоиспользуемых значений
	float alfa = Motor_xx->Config.Alfa;
	float unalfa = 1.0f - alfa;

	unsigned long int discrete_range = Motor_xx->Config.Convertor.Maximum_discrete_level - Motor_xx->Config.Convertor.Minimum_discrete_level;
	float angular_range = Motor_xx->Config.Angular.Maximum_angular - Motor_xx->Config.Angular.Minimum_angular;

	// Расчёт выходных значений
	unsigned long int filtered_discrete = (unsigned long int)(alfa * (float)raw_value + unalfa * (float)Motor_xx->Status.filtered_Discrete_level);
	float norma = (filtered_discrete - (float)Motor_xx->Config.Convertor.Minimum_discrete_level) / (float)discrete_range;

	float angular = Motor_xx->Config.Angular.Minimum_angular + norma * angular_range;
	float filtered_angular = alfa * angular + unalfa * Motor_xx->Status.filter_Angular;

	// Запись значений
	Motor_xx->Status.Discrete_level = filtered_discrete;
	Motor_xx->Status.filtered_Discrete_level = filtered_discrete;

	Motor_xx->Status.Angular = filtered_angular;
	Motor_xx->Status.filter_Angular = filtered_angular;
};

void Set_PWM_Frequency(Motor *Motor_xx, uint32_t freq)
{
	Motor_xx->Status.Frequency = freq;

	uint32_t prescaler = 0;
	uint32_t period = (HAL_RCC_GetPCLK1Freq() * 2 / freq) - 1;

	while (period > 0xFFFF)
	{
		prescaler++;
		period = (HAL_RCC_GetPCLK1Freq() * 2 / (freq * (prescaler + 1))) - 1;
	}

	HAL_TIM_PWM_Stop(Motor_xx->Config.PWM.Timer, TIM_CHANNEL_1);
	Motor_xx->Config.PWM.Timer->Instance->PSC = prescaler;
	Motor_xx->Config.PWM.Timer->Instance->ARR = period;
	Motor_xx->Config.PWM.Timer->Instance->CCR1 = period / 2;
	HAL_TIM_PWM_Start(Motor_xx->Config.PWM.Timer, TIM_CHANNEL_1);
};

char Working_area(Motor *Motor_xx)
{
    float current_angle = Motor_xx->Status.Angular;

    float lower_bound = Motor_xx->Config.Angular.Minimum_angular + Motor_xx->Config.Angular.Deviation;;
    float upper_bound = Motor_xx->Config.Angular.Maximum_angular - Motor_xx->Config.Angular.Deviation;;

    return (current_angle > lower_bound) && (current_angle < upper_bound);
};

void Start_motor(Motor *Motor_xx, GPIO_PinState roter)
{
	Set_PWM_Frequency(Motor_xx, Motor_xx->Config.PWM.Minimum_frequency);
	HAL_GPIO_WritePin(Motor_xx->Config.GPIO.DIR_port, Motor_xx->Config.GPIO.DIR_pin, roter);
	Motor_xx->Status.Moving = 1;
};

void Up_fequency(Motor *Motor_xx)
{
	if (Motor_xx->Status.Frequency < Motor_xx->Config.PWM.Maximum_frequency)
	{
		unsigned long int freq = Motor_xx->Status.Frequency + Motor_xx->Config.PWM.Increment_frequency;
		freq = MIN(freq, Motor_xx->Config.PWM.Maximum_frequency);
		Set_PWM_Frequency(Motor_xx, freq);
	};
};

void Stop_motor(Motor *Motor_xx)
{
	Set_PWM_Frequency(Motor_xx, 0);
	HAL_TIM_PWM_Stop(Motor_xx->Config.PWM.Timer, TIM_CHANNEL_1); // Требуется остановка, так как в конце установки частоты таймер запускается
	Motor_xx->Status.Moving = 0;
};

void Moving_away_from_borders(Motor *Motor_xx, unsigned long int TimeOut, GPIO_PinState roter)
{
	Stop_motor(Motor_xx->Config.PWM.Timer == Motor_AZ.Config.PWM.Timer ? &Motor_EL : &Motor_AZ);

	HAL_GPIO_WritePin(Motor_xx->Config.GPIO.DIR_port, Motor_xx->Config.GPIO.DIR_pin, roter);
	Set_PWM_Frequency(Motor_xx, 3000);
	HAL_Delay(TimeOut);

	Motor_xx->Status.Moving = 0;
};
