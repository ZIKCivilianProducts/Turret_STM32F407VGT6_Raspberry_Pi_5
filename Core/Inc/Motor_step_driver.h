/*
 * Engine_structure.h
 *
 *  Created on: Jul 8, 2025
 *      Author: u033008
 */

#ifndef INC_MOTOR_STEP_DRIVER_H_
#define INC_MOTOR_STEP_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "Motor_step_driver.h"
#include "Engine_conditions.h"
#include "Binary_macros.h"
#include "math.h"

typedef struct
{
  GPIO_TypeDef *DIR_port, *ENA_port;
  unsigned int DIR_pin, ENA_pin;
} Control_pins;

typedef struct
{
  TIM_HandleTypeDef *Timer;

  unsigned int Maximum_frequency, Minimum_frequency;
  unsigned int Increment_frequency;
} Pulse_width_modulation_пenerator;

typedef struct
{
  float Maximum_angular, Minimum_angular;

  float Guidance_accuracy; // Точность_прицелывания
  float Deviation; // Отступ_от_границ
} Angular_settings;

typedef struct // Config
{
  Control_pins GPIO;
  Pulse_width_modulation_пenerator PWM;
  Angular_settings Angular;

  float Alfa;
} System_settings;

typedef struct // Status
{
  uint32_t Frequency;

  uint8_t Moving;

  float Angular;
} System_status;

typedef struct // Motor
{
  System_settings Config;
  System_status Status;
} Motor;

extern Motor Motor_AZ;
extern Motor Motor_EL;

void First_mode (Motor *Motor_xx, float Angular);
void Moving_away_from_borders(Motor *Motor_xx, GPIO_PinState roter);
void Second_mode(float Angular, Motor *Motor_xx);
void Set_PWM_frequency(Motor *Motor_xx, uint32_t freq);
void Start_motor(Motor *Motor_xx, GPIO_PinState roter);
void Stop_motor(Motor *Motor_xx);
void Up_fequency(Motor *Motor_xx);
char Working_area(Motor *Motor_xx);

#endif /* INC_MOTOR_STEP_DRIVER_H_ */
