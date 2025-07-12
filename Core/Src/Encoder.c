/*
 * Encoder.c
 *
 *  Created on: Jul 12, 2025
 *      Author: Danil
 */

#include "Encoder.h"

Encoder Positional_encoder_AZ = {
  .pulses_per_revolution = 2000
};

Encoder Positional_encoder_EL = {
  .pulses_per_revolution = 2000
};

void Initialization_of_positioning_encoders(void) {
  HAL_StatusTypeDef Confirmation_of_initialization;
  do { // AZ
    Confirmation_of_initialization = HAL_TIM_Encoder_Start(Positional_encoder_AZ.Timer, TIM_CHANNEL_ALL);
  } while (!Confirmation_of_initialization);
  do { // EL
    Confirmation_of_initialization = HAL_TIM_Encoder_Start(Positional_encoder_EL.Timer, TIM_CHANNEL_ALL);
  } while (!Confirmation_of_initialization);
};

void Reading_the_system_position(Encoder *Positional_encoder_xx, Systema *Systema_xx) {
  int Counter_value = Positional_encoder_xx->Timer->Instance->CNT;
  float Angular = (float)Counter_value * 360.0f / (float)Positional_encoder_xx->pulses_per_revolution;
  Angular = CLAMP(Angular, -270.0f, 270.0);
  Systema_xx->Status.Angular = Angular;
};
