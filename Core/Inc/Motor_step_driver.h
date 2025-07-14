/*
 * Motor_step_driver.h
 *
 *  Created on: Jul 8, 2025
 *      Author: u033008
 */

#ifndef INC_MOTOR_STEP_DRIVER_H_
#define INC_MOTOR_STEP_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "Motor_step_driver.h"
#include "Binary_macros.h"
#include "math.h"
#include "Orientation_of_the_system.h"
#include "Raspberry_Pi.h"

#define Left GPIO_PIN_SET
#define Right GPIO_PIN_RESET

#define Up GPIO_PIN_SET
#define Down GPIO_PIN_RESET

#define Sleep GPIO_PIN_SET
#define Work GPIO_PIN_RESET

typedef struct
{
  GPIO_TypeDef *DIR_port, *ENA_port;
  unsigned int DIR_pin, ENA_pin;
} Control_pins;

typedef struct
{
  TIM_HandleTypeDef *Timer;
  unsigned int Channel;

  unsigned int Maximum_frequency, Minimum_frequency;
  unsigned int Increment_frequency;
} Pulse_width_modulation_пenerator;

typedef struct
{
  Control_pins Pins;
  Pulse_width_modulation_пenerator PWM;
} Motor;

extern Motor Motor_AZ;
extern Motor Motor_EL;

void Setting_the_pulse_frequency(Motor *Motor_xx, unsigned int frequency);
void Turning_on_the_engine(Motor *Motor_xx, GPIO_PinState direction_of_rotation);
void Engine_shutdown(Motor *Motor_xx);
void Return_to_the_workspace(void);
void Increasing_the_pulse_frequency(Motor *Motor_xx);

#endif /* INC_MOTOR_STEP_DRIVER_H_ */
