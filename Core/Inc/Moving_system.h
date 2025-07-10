/*
 * Moving_system.h
 *
 *  Created on: Jul 8, 2025
 *      Author: u033008
 */

#ifndef INC_MOVING_SYSTEM_H_
#define INC_MOVING_SYSTEM_H_

#include "Structur_STM.h"

#define Left GPIO_PIN_SET
#define Right GPIO_PIN_RESET

#define Up GPIO_PIN_SET
#define Down GPIO_PIN_RESET

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define CLAMP(x, min, max) MAX(min, MIN(x, max))

void Set_PWM_Frequency(Motor *Motor_xx, uint32_t freq);
char Working_area(Motor *Motor_xx);
void Start_motor(Motor *Motor_xx, GPIO_PinState roter);
void Up_fequency(Motor *Motor_xx);
void Stop_motor(Motor *Motor_xx);
void Moving_away_from_borders(Motor *Motor_xx, unsigned int TimeOut, GPIO_PinState roter);
void Read_AD_Conversion(Motor *Motor_xx);

#endif /* INC_MOVING_SYSTEM_H_ */
