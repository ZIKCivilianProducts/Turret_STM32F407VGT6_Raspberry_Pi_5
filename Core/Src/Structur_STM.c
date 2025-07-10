/*
 * Structur_STM.c
 *
 *  Created on: Jul 8, 2025
 *      Author: u033008
 */

#include "Structur_STM.h"

Motor Motor_AZ =
{
		.Config =
		{
				.GPIO =
				{
						.DIR_port = GPIOE, .DIR_pin = GPIO_PIN_8,
						.ENA_port = GPIOE, .ENA_pin = GPIO_PIN_11
				},
				.PWM =
				{
//						.Timer = &htim3,
						.Maximum_frequency = 100000,
						.Minimum_frequency = 1000,
						.Increment_frequency = 500
				},
				.Convertor =
				{
//						Convertor = &hadc1,
						.Maximum_discrete_level = 2150,
						.Middle_discrete_level = 1850,
						.Minimum_discrete_level = 1550

				},
				.Angular =
				{
						.Maximum_angular = 270.0f,
						.Minimum_angular = -270.0f,
						.Deviation = 50.0f,
						.Guidance_accuracy = 5.0f
				},
				.Alfa = 0.01f
		},
		.Status =
		{
				.Angular = 0.0f,
				.filter_Angular = 0.0f,
				.Discrete_level = 1850,
				.filtered_Discrete_level = 1850,
				.Frequency = 5000,
				.Moving = 0
		}
};

Motor Motor_EL =
{
		.Config =
		{
				.GPIO =
				{
						.DIR_port = GPIOE, .DIR_pin = GPIO_PIN_9,
						.ENA_port = GPIOE, .ENA_pin = GPIO_PIN_12
				},
				.PWM =
				{
//						.Timer = &htim2,
						.Maximum_frequency = 100000,
						.Minimum_frequency = 1000,
						.Increment_frequency = 5000
				},
				.Convertor =
				{
//						Convertor = &hadc2,
						.Maximum_discrete_level = 2150,
						.Middle_discrete_level = 2100,
						.Minimum_discrete_level = 2050

				},
				.Angular =
				{
						.Maximum_angular = 90.0f,
						.Minimum_angular = -20.0f,
						.Deviation = 10.0f,
						.Guidance_accuracy = 5.0f
				},
				.Alfa = 0.01f
		},
		.Status =
		{
				.Angular = 0.0f,
				.filter_Angular = 0.0f,
				.Discrete_level = 2100,
				.filtered_Discrete_level = 2100,
				.Frequency = 5000,
				.Moving = 0
		}
};
