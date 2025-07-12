/*
 * Encoder.h
 *
 *  Created on: Jul 12, 2025
 *      Author: Danil
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f4xx_hal.h"
#include "Orientation_of_the_system.h"
#include "Ternary_macros.h"

typedef struct {
	TIM_HandleTypeDef *Timer;
	short int pulses_per_revolution;
} Encoder;

extern Encoder Positional_encoder_AZ;
extern Encoder Positional_encoder_EL;

void Initialization_of_positioning_encoders(void);
void Reading_the_system_position(Encoder *Positional_encoder_xx, Systema *Systema_xx);

#endif /* INC_ENCODER_H_ */
